#pragma once
#include <math.h>
#include "../dlx_math.hpp"
#include "flight_control_params.hpp"

//===================================================================================================
// FlightController.hpp
// 无人机姿态 + 高度控制模块 (独立于单片机, 纯计算).
//
// 控制结构 (参考 PX4 mc_att_control / mc_rate_control / mc_pos_control):
//   1. 倾转分离: 期望姿态只取“倾转”分量, 偏航继承当前姿态 (单 IMU 偏航不准, 暂不控制);
//   2. 由倾转误差四元数计算旋转轴 / 旋转角, 得到期望角速度 (机体系);
//   3. 角速度环 (PID + 前馈) 得到期望角加速度, 乘转动惯量得到期望力矩 (机体系);
//   4. 高度环: 高度误差 -> 期望垂向速度 -> 期望垂向加速度 -> 油门 (0~1, 含倾角补偿).
//
//   提供两个相邻的入口, 共用底层姿态/角速度逻辑, 但接口各自独立、互不混入 throttle 分支:
//     updateAngle()         仅角度环(自稳): 输入 目标姿态+当前状态, 输出 期望力矩/角速度设定;
//                           不计算油门. 遥控手动油门由调用方写入 out.throttle 并直通混控.
//     updateAngleHeight()   角度环 + 高度环(定高): 在仅角度环基础上, 由高度环计算油门写入 out.throttle.
//
//   重要: 混控器 (flight_control_mixer.hpp) 围绕总油门做差动缩放, 推力优先, 保证无电机为负.
//         当油门=0 时, 差动缩放系数 s 会被压到 0 (T/(-d_min)=0), 四个电机输出全为 0.
//         因此自稳模式必须由遥控提供非零集体油门, 力矩才能被转换成差动转速.
//
// 垂向速度来源: 建议用 vertical_velocity_estimator.hpp 的估计器
//   (气压高度 getAltitude + 姿态四元数 + IMU 加速度 融合), 输出填入
//   FlightControlState.vertical_velocity_mps 并置 vertical_velocity_valid = true;
//   未提供时退回高度差分 + 低通估计 (精度较差).
//   仅角度环(updateAngle)不依赖垂向速度/高度, 因此高度估计缺失也不影响.
//
// 接口约定:
//   - 输入输出姿态均为世界系四元数 (w,x,y,z), 与 MadgwickAHRS 输出一致;
//   - 机体角速度使用 dlx_sensor_data.hpp 中的 GyroscopeRads, data[0..2] = (x,y,z) [rad/s];
//   - 高度 / 垂向速度向上为正 [m] / [m/s];
//   - updateAngle / updateAngleHeight 每控制周期调用一次, dt 为周期 [s].
//
// 使用示例:
//   dlx::FlightController fc;
//   while (1) {
//       dlx::FlightControlSetpoint sp;
//       sp.attitude = external_attitude_setpoint; // 期望姿态 (外部给定, 世界系)
//       dlx::FlightControlState state;
//       state.attitude = madgwick.getQuaternion(); // 当前姿态
//       state.body_rate = gyro_rads;               // 当前机体角速度
//
//       dlx::FlightControlOutput out;
//       out.throttle = pilot_throttle;    // 自稳模式: 遥控手动油门, 直接放在输出上
//
//       // 二选一 (也可由飞行模式切换):
//       fc.updateAngle(sp, state, dt, out);          // 仅角度环: 只填 torque/rate_setpoint, 不动油门
//       // fc.updateAngleHeight(sp, state, dt, out); // 角度+定高: 高度环覆写 out.throttle
//
//       // 混控: 输出 -> 四电机实际油门 (0~1), 见 flight_control_mixer.hpp
//       dlx::FlightControlMotorOutput motors = dlx::mixMotors(out, fc.params());
//       // motors.motor[0..3] 对应 M0..M3
//   }
//===================================================================================================

namespace dlx
{
    // 期望值 (外部给定, 以后可由位置控制输出生成)
    struct FlightControlSetpoint
    {
        Quaternion attitude;   // 期望姿态 (世界系, w,x,y,z)
        float height_m{0.0f};  // 期望高度 [m], 向上为正
    };

    // 当前状态 (估计值)
    struct FlightControlState
    {
        Quaternion attitude;               // 当前姿态 (世界系, w,x,y,z)
        float height_m{0.0f};              // 当前高度 [m], 向上为正
        GyroscopeRads body_rate;           // 当前机体角速度 [rad/s], data = (x,y,z)
        float vertical_velocity_mps{0.0f}; // 当前垂向速度 [m/s], 向上为正
        bool vertical_velocity_valid{false}; // 未提供时由高度差分估计
    };

    // 控制输出
    struct FlightControlOutput
    {
        float throttle{0.0f};     // 期望油门 (归一化 0~1)
        Vector3f torque;          // 期望机体力矩 [N*m]
        Vector3f rate_setpoint;   // 期望机体角速度 [rad/s]
    };

    // 调试信息 (最近一次 update 的中间量)
    struct FlightControlDebug
    {
        Vector3f attitude_error_axis; // 倾转误差旋转轴 (机体系, 单位向量)
        float attitude_error_angle{0.0f}; // 倾转误差角 [rad]
        Vector3f angular_accel;       // 期望角加速度 [rad/s^2]
        float vertical_accel{0.0f};   // 高度环输出的期望垂向加速度 [m/s^2]
    };

    class FlightController
    {
    public:
        FlightController()
        {
            _params.resetToDefaults();
            reset();
        }

        explicit FlightController(const FlightControlParams &params)
            : _params(params)
        {
            reset();
        }

        //===============================================================================================
        // 仅角度环(自稳): 输入 = 目标姿态 + 当前状态, 输出 = 期望力矩 + 角速度设定.
        // 不做高度/油门控制 —— out.throttle 由调用方预先放好遥控/手动油门, 本函数不改动它.
        //===============================================================================================
        void updateAngle(const FlightControlSetpoint &sp, const FlightControlState &state, float dt,
                         FlightControlOutput &out)
        {
            dt = sanitizeDt(dt);
            initIfFirstUpdate(state);

            Vector3f rate_sp;
            const Vector3f torque = computeTorque(sp, state, dt, rate_sp);

            out.torque = torque;
            out.rate_setpoint = rate_sp;
            // 不动 out.throttle —— 调用方已放入手动油门 (仅角度环模式)
            _debug.vertical_accel = 0.0f;
            _output = out;
        }

        //===============================================================================================
        // 角度 + 高度环(定高): 在仅角度环基础上加入高度/垂向速度环, 由高度环计算油门写入 out.throttle.
        //===============================================================================================
        void updateAngleHeight(const FlightControlSetpoint &sp, const FlightControlState &state, float dt,
                               FlightControlOutput &out)
        {
            dt = sanitizeDt(dt);
            initIfFirstUpdate(state);

            Vector3f rate_sp;
            const Vector3f torque = computeTorque(sp, state, dt, rate_sp);
            const float throttle = computeHeightThrottle(sp, state, dt);

            out.torque = torque;
            out.rate_setpoint = rate_sp;
            out.throttle = throttle;
            _output = out;
        }

        // 清除全部内部状态 (起飞/解锁前调用)
        void reset()
        {
            _first_update = true;
            _rate_int = Vector3f{};
            _rate_prev = Vector3f{};
            _ang_accel_lp = Vector3f{};
            _height_int = 0.0f;
            _height_prev = 0.0f;
            _vz_est = 0.0f;
            _output = FlightControlOutput{};
            _debug = FlightControlDebug{};
        }

        FlightControlParams &params()
        {
            return _params;
        }

        const FlightControlParams &params() const
        {
            return _params;
        }

        // 最近一次 update 的完整输出 (油门 / 力矩 / 角速度设定)
        const FlightControlOutput &lastOutput() const
        {
            return _output;
        }

        // 最近一次 update 的中间量 (调参/调试用)
        const FlightControlDebug &debug() const
        {
            return _debug;
        }

        // 角速度环积分状态 (调试用; 单位 rad/s, 被 RATE_INT_LIMIT 钳位)
        const Vector3f &rateIntegral() const
        {
            return _rate_int;
        }

    private:
        // 周期保护: 无效 dt -> 默认 500Hz, 并限幅
        static float sanitizeDt(float dt)
        {
            if (!(dt > 0.0f)) {
                dt = 0.002f; // 无效 dt 时使用默认 500Hz 周期, 避免除零
            }
            return constrain(dt, 0.0005f, 0.5f);
        }

        // 首次调用 (起飞/解锁后): 初始化全部内部状态
        void initIfFirstUpdate(const FlightControlState &state)
        {
            if (!_first_update) {
                return;
            }
            _first_update = false;
            _rate_prev = Vector3f{state.body_rate.data[0], state.body_rate.data[1], state.body_rate.data[2]};
            _height_prev = state.height_m;
            _vz_est = 0.0f;
            _ang_accel_lp = Vector3f{};
            _rate_int = Vector3f{};
            _height_int = 0.0f;
        }

        // 姿态环 + 角速度环 (自稳核心, 不含高度/油门): 计算期望力矩, 回填期望角速度设定
        Vector3f computeTorque(const FlightControlSetpoint &sp, const FlightControlState &state, float dt,
                               Vector3f &rate_sp_out)
        {
            const Quaternion q = quatNormalize(state.attitude);
            const Quaternion q_sp = quatNormalize(sp.attitude);

            // ---------- 1. 姿态环: 倾转分离 -> 旋转轴/旋转角 -> 期望角速度 ----------
            const Vector3f e_z = quatDcmZ(q);      // 当前机体 z 轴 (世界系)
            const Vector3f e_z_d = quatDcmZ(q_sp); // 期望机体 z 轴 (世界系)

            Quaternion qd_red = quatFromTwoVectors(e_z, e_z_d);
            if (fabsf(qd_red.data[1]) > (1.0f - 1e-5f) || fabsf(qd_red.data[2]) > (1.0f - 1e-5f)) {
                // 当前与期望 z 轴近似反向的退化情形, 直接使用完整期望姿态
                qd_red = q_sp;
            } else {
                qd_red = quatMul(qd_red, q);
            }

            // 姿态误差: 从当前姿态到降阶期望姿态的旋转 (机体系)
            Quaternion qe = quatMul(quatConjugate(q), qd_red);
            quatCanonicalize(qe);

            // 误差用 sin(angle/2) * 旋转轴 表示 (即 qe 虚部 * 2)
            Vector3f eq;
            eq.x() = 2.0f * qe.data[1];
            eq.y() = 2.0f * qe.data[2];
            eq.z() = 2.0f * qe.data[3];

            Vector3f rate_sp;
            rate_sp.x() = eq.x() * _params.get(FlightParamId::ATT_ROLL_P);
            rate_sp.y() = eq.y() * _params.get(FlightParamId::ATT_PITCH_P);
            rate_sp.z() = eq.z() * _params.get(FlightParamId::ATT_YAW_P);
            rate_sp.x() = constrain(rate_sp.x(), -_params.get(FlightParamId::RATE_ROLL_MAX), _params.get(FlightParamId::RATE_ROLL_MAX));
            rate_sp.y() = constrain(rate_sp.y(), -_params.get(FlightParamId::RATE_PITCH_MAX), _params.get(FlightParamId::RATE_PITCH_MAX));
            rate_sp.z() = constrain(rate_sp.z(), -_params.get(FlightParamId::RATE_YAW_MAX), _params.get(FlightParamId::RATE_YAW_MAX));

            // 调试: 倾转误差轴 / 角
            _debug.attitude_error_angle = 2.0f * acosf(constrain(qe.data[0], -1.0f, 1.0f));
            const float sin_half = sinf(0.5f * _debug.attitude_error_angle);
            if (sin_half > 1e-6f) {
                _debug.attitude_error_axis.x() = qe.data[1] / sin_half;
                _debug.attitude_error_axis.y() = qe.data[2] / sin_half;
                _debug.attitude_error_axis.z() = qe.data[3] / sin_half;
            } else {
                _debug.attitude_error_axis = Vector3f{};
            }

            // ---------- 2. 角速度环: 角速度误差 -> 期望角加速度 -> 期望力矩 ----------
            Vector3f rate;
            rate.x() = state.body_rate.data[0];
            rate.y() = state.body_rate.data[1];
            rate.z() = state.body_rate.data[2];
            const Vector3f rate_err = rate_sp - rate;

            // 积分 (限幅防饱和)
            const float int_lim = _params.get(FlightParamId::RATE_INT_LIMIT);
            _rate_int += rate_err * dt;
            _rate_int = constrain(_rate_int, -int_lim, int_lim);

            // 实测角加速度 (用于 D 项), 一阶低通
            const float ang_accel_alpha = dt / (_params.get(FlightParamId::RATE_D_TAU_S) + dt);
            const Vector3f ang_accel_meas = (rate - _rate_prev) / dt;
            _ang_accel_lp = _ang_accel_lp * (1.0f - ang_accel_alpha) + ang_accel_meas * ang_accel_alpha;
            _rate_prev = rate;

            Vector3f ang_acc_des;
            ang_acc_des.x() = _params.get(FlightParamId::RATE_ROLL_P) * rate_err.x()
                              + _params.get(FlightParamId::RATE_ROLL_I) * _rate_int.x()
                              - _params.get(FlightParamId::RATE_ROLL_D) * _ang_accel_lp.x()
                              + _params.get(FlightParamId::RATE_ROLL_FF) * rate_sp.x();
            ang_acc_des.y() = _params.get(FlightParamId::RATE_PITCH_P) * rate_err.y()
                              + _params.get(FlightParamId::RATE_PITCH_I) * _rate_int.y()
                              - _params.get(FlightParamId::RATE_PITCH_D) * _ang_accel_lp.y()
                              + _params.get(FlightParamId::RATE_PITCH_FF) * rate_sp.y();
            ang_acc_des.z() = _params.get(FlightParamId::RATE_YAW_P) * rate_err.z()
                              + _params.get(FlightParamId::RATE_YAW_I) * _rate_int.z()
                              - _params.get(FlightParamId::RATE_YAW_D) * _ang_accel_lp.z()
                              + _params.get(FlightParamId::RATE_YAW_FF) * rate_sp.z();
            const float acc_lim = _params.get(FlightParamId::ANGACC_MAX);
            ang_acc_des = constrain(ang_acc_des, -acc_lim, acc_lim);
            _debug.angular_accel = ang_acc_des;

            // 期望力矩 = 转动惯量 * 期望角加速度
            Vector3f torque;
            torque.x() = _params.get(FlightParamId::INERTIA_XX) * ang_acc_des.x();
            torque.y() = _params.get(FlightParamId::INERTIA_YY) * ang_acc_des.y();
            torque.z() = _params.get(FlightParamId::INERTIA_ZZ) * ang_acc_des.z();

            rate_sp_out = rate_sp;
            return torque;
        }

        // 高度状态估计 + 高度环: 高度误差 -> 期望垂向速度 -> 期望垂向加速度 -> 油门 (含倾角补偿)
        float computeHeightThrottle(const FlightControlSetpoint &sp, const FlightControlState &state, float dt)
        {
            // 高度状态估计 (有测量用测量值, 否则由高度差分 + 低通估计)
            if (!state.vertical_velocity_valid) {
                const float vz_alpha = dt / (_params.get(FlightParamId::VZ_EST_TAU_S) + dt);
                const float vz_meas = (state.height_m - _height_prev) / dt;
                _vz_est = _vz_est * (1.0f - vz_alpha) + vz_meas * vz_alpha;
            }
            _height_prev = state.height_m;

            const float thr_lo = _params.get(FlightParamId::THROTTLE_MIN);
            const float thr_hi = _params.get(FlightParamId::THROTTLE_MAX);
            const float h_err = sp.height_m - state.height_m;
            const float vz_sp = constrain(_params.get(FlightParamId::POS_Z_P) * h_err,
                                          -_params.get(FlightParamId::VZ_MAX_DOWN),
                                          _params.get(FlightParamId::VZ_MAX_UP));
            const float vz = state.vertical_velocity_valid ? state.vertical_velocity_mps : _vz_est;
            const float vel_err = vz_sp - vz;
            const float acc_z = _params.get(FlightParamId::VEL_Z_P) * vel_err
                                + _params.get(FlightParamId::VEL_Z_I) * _height_int;
            _debug.vertical_accel = acc_z;

            // 油门: 悬停油门 + 垂向加速度折算, 再按倾角补偿 (保持垂直分量不变)
            const float kGravity = 9.80665f;
            float throttle = _params.get(FlightParamId::HOVER_THROTTLE) * (1.0f + acc_z / kGravity);
            const float tilt_cos = constrain(quatDcmZ(quatNormalize(state.attitude)).z(), 0.2f, 1.0f); // 防除零
            throttle = throttle / tilt_cos;

            // 积分抗饱和: 油门已到上下限且误差同向时停止积分
            const bool saturated = (throttle >= thr_hi && vel_err > 0.0f)
                                   || (throttle <= thr_lo && vel_err < 0.0f);
            if (!saturated) {
                _height_int += vel_err * dt;
                _height_int = constrain(_height_int, -1.0f, 1.0f); // 积分限幅 [m], 折算加速度约 VEL_Z_I*1.0
            }
            return constrain(throttle, thr_lo, thr_hi);
        }

        FlightControlParams _params;
        bool _first_update{true};

        // 角速度环状态
        Vector3f _rate_int;     // 角速度误差积分
        Vector3f _rate_prev;    // 上一周期角速度
        Vector3f _ang_accel_lp; // 角加速度低通

        // 高度环状态
        float _height_int{0.0f}; // 垂向速度误差积分
        float _height_prev{0.0f}; // 上一周期高度
        float _vz_est{0.0f};     // 垂向速度估计 (低通)

        // 输出 / 调试
        FlightControlOutput _output;
        FlightControlDebug _debug;
    };

} // namespace dlx

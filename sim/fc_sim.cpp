//===================================================================================================
// fc_sim.cpp -- 把固件的"控制 + 滤波"链路原封不动搬到 PC 上, 编成 DLL 给 JSBSim 仿真用
//
// 这里不重写任何控制/滤波算法, 全部直接 include 固件源码:
//     DL_LIB/flight_control/flight_control_fliter.hpp        16 维 EKF (release.cpp 实际用的融合)
//     DL_LIB/DL_AHRS/MadgwickAHRS.hpp                        Madgwick AHRS (备选姿态融合)
//     DL_LIB/flight_control/vertical_velocity_estimator.hpp  垂速估计 (Madgwick 路线的定高用)
//     DL_LIB/flight_control/flight_controller.hpp            角度环 + 角速度环 (PX4 风格倾转分离)
//     DL_LIB/flight_control/flight_control_mixer.hpp         混控 (力矩 -> 四电机差动)
//     参数: flight_control_params.hpp / flight_config_struct.hpp (kConfig)
//
// 时序与 release.cpp 一致:
//     IMU          2000 Hz  integrateNominal
//     EKF 慢路径   250 Hz   propagateCovariance + updateAccelerometer   (ekfDecim = 8)
//     气压计       50 Hz    updateBarometer                             (baroPeriodMs = 20)
//     控制环       500 Hz   FlightController.updateAngle/AngleHeight + mixMotors + motorMap
//
// 对外只暴露一个 C ABI (POD 结构体), 由 sim/jsbsim_drone_sim.py 通过 ctypes 调用。
//
// 编译: sim/build_fc_sim.ps1   (g++ -shared -O2, 静态链接 libstdc++)
//===================================================================================================

#include <math.h>
#include <stdint.h>

#include "flight_control/flight_control_fliter.hpp"
#include "flight_control/flight_control_mixer.hpp"
#include "flight_control/flight_controller.hpp"
#include "flight_control/vertical_velocity_estimator.hpp"
#include "DL_AHRS/MadgwickAHRS.hpp"

namespace
{
    using namespace dlx;

    //===============================================================================================
    // 固件配置快照 (flight_config_struct.hpp 的 kConfig / release.cpp 的常量)
    //===============================================================================================
    constexpr float    kLoopHz      = 500.0f;              // kConfig.loopHz
    constexpr float    kImuHz       = 2000.0f;             // kConfig.imuRateHz
    constexpr float    kImuDt       = 1.0f / kImuHz;
    constexpr float    kLoopDt      = 1.0f / kLoopHz;
    constexpr int      kCtrlDecim   = (int)(kImuHz / kLoopHz);   // 4
    constexpr int      kEkfDecim    = 8;                   // kConfig.ekfDecim
    constexpr uint32_t kBaroPeriodMs = 20;                 // kConfig.baroPeriodMs
    constexpr float    kMinThrottle = 0.01f;               // kConfig.minThrottle
    constexpr int      kMotorMap[4] = {0, 2, 3, 1};        // kConfig.motorMap: 逻辑 -> 物理 DShot 通道
    constexpr float    kEkfBaroSigmaM       = 1.0f;        // kConfig.ekfBaroSigmaM
    constexpr float    kEkfAccelTiltGateMss = 2.0f;        // kConfig.ekfAccelTiltGateMss
    constexpr float    kMadgwickBeta        = 0.1f;        // main_att_ekf.cpp 用的 beta

    enum FilterMode : int
    {
        FILTER_EKF = 0,   // 16 维 EKF + 气压计 (release.cpp 在飞的那套)
        FILTER_MADGWICK = 1, // Madgwick AHRS + VerticalVelocityEstimator (备选路线)
    };

    struct FcSim
    {
        // ---- 配置 ----
        FilterMode filterMode = FILTER_EKF;

        // ---- 固件对象 ----
        FlightControlFilter ekf;
        MadgwickAHRS        ahrs{kImuHz, kMadgwickBeta};
        VerticalVelocityEstimator vzEst;
        FlightController    controller;
        FlightControlParams params;

        // ---- 状态 ----
        bool  first_step = true;
        int   imu_cnt = 0;
        int   ctrl_cnt = 0;
        float imu_dt_accum = 0.0f;
        float baro_accum = 0.0f;
        float baro_ref = 0.0f;
        bool  baro_ref_set = false;
        bool  baro_healthy = false;
        float baro_age_s = 0.0f;      // 距上次有效气压样本的时间 (baroFailLimit x baroPeriodMs = 200ms)

        GyroscopeRads lastGyro{{0.0f, 0.0f, 0.0f}};
        AccelerometerG lastAcc{{0.0f, 0.0f, 1.0f}};
        float last_baro_abs = 0.0f;

        Quaternion  attitude = quatIdentity();
        float       height_m = 0.0f;
        float       vz_mps = 0.0f;

        FlightControlOutput out{};
        FlightControlMotorOutput mixed{};
        float motor_phys[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float motor_logical[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        bool  motor_saturated = false;
        float fp_tilt_sigma = 0.05f;
        float tilt_angle_deg = 0.0f;
        int   last_armed = -1;      // 用于检测解锁瞬间

        void reset()
        {
            ekf.reset();
            ahrs = MadgwickAHRS(kImuHz, kMadgwickBeta);
            vzEst = VerticalVelocityEstimator();
            controller.reset();
            first_step = true;
            imu_cnt = 0;
            ctrl_cnt = 0;
            imu_dt_accum = 0.0f;
            baro_accum = 0.0f;
            baro_ref_set = false;
            baro_healthy = false;
            baro_age_s = 0.0f;
            lastGyro = GyroscopeRads{{0.0f, 0.0f, 0.0f}};
            lastAcc = AccelerometerG{{0.0f, 0.0f, 1.0f}};
            attitude = quatIdentity();
            height_m = 0.0f;
            vz_mps = 0.0f;
            out = FlightControlOutput{};
            for (int i = 0; i < 4; ++i)
            {
                motor_phys[i] = 0.0f;
                motor_logical[i] = 0.0f;
            }
            motor_saturated = false;

            // 与 release.cpp 的 EKF 参数一致 (kConfig.ekf + 两项实测整定)
            FlightControlFilterParams fp;
            fp.baro_sigma_m = kEkfBaroSigmaM;
            fp.accel_tilt_gate_mss = kEkfAccelTiltGateMss;
            fp.estimate_accel_bias = false;
            fp_tilt_sigma = fp.accel_tilt_sigma;
            ekf.set(fp);
        }

        // 首次解锁: 保留当前姿态估计, 高度按 init_height_m 定位 (与 release.cpp 的
        // g_ekf.reset(fcs.attitude, {0,0,0}, {0,0,0}) 等价, 只是允许给一个非零起点高度)
        void arm(const Quaternion &q, float init_height_m)
        {
            ekf.reset(q, Vector3f{0.0f, 0.0f, init_height_m}, Vector3f{});
            controller.reset();
            vzEst.reset(init_height_m);
            first_step = false;
        }
    };

    FcSim g;

    inline void run_control_500hz(const Quaternion &att, const GyroscopeRads &gyro,
                                  float height_m, float vz_mps, bool vz_valid,
                                  const FlightControlSetpoint &sp, float manual_throttle, bool height_mode)
    {
        FlightControlState fcs;
        fcs.attitude = att;
        fcs.body_rate = gyro;
        fcs.height_m = height_m;
        fcs.vertical_velocity_mps = vz_mps;
        fcs.vertical_velocity_valid = vz_valid;

        FlightControlOutput out = {};
        if (height_mode)
        {
            g.controller.updateAngleHeight(sp, fcs, kLoopDt, out);
        }
        else
        {
            g.controller.updateAngle(sp, fcs, kLoopDt, out);
            out.throttle = (manual_throttle > kMinThrottle) ? manual_throttle : kMinThrottle;
        }

        const FlightControlMotorOutput mo = mixMotors(out, g.controller.params());
        g.out = out;
        g.mixed = mo;
        g.motor_saturated = mo.mix_scale < 1.0f;
        for (int i = 0; i < 4; ++i)
        {
            g.motor_logical[i] = mo.motor[i];
            g.motor_phys[kMotorMap[i]] = mo.motor[i];
        }
    }
} // namespace

//===================================================================================================
// C ABI
//===================================================================================================
extern "C"
{
    struct FcSimInput
    {
        float gyro[3];            // 机体系角速度 [rad/s] (固件坐标: x 前, y 左, z 上)
        float accel[3];           // 机体系比力   [m/s^2]
        float baro_abs_m;         // 气压高度绝对值 [m] (地面对应 0)
        int   baro_valid;         // 本周期气压是否有新样本
        float target_yaw_rad;     // 期望姿态 (固件 quatFromEulerZYX 约定)
        float target_pitch_rad;
        float target_roll_rad;
        float target_height_m;    // 定高模式的期望高度 [m]
        float manual_throttle;    // 角度环模式的遥控油门 (0~1)
        int   height_mode;        // 0 = 仅角度环(手动油门), 1 = 角度 + 定高
        int   armed;              // 0 = 停机 (输出全 0), 1 = 正常运行
        int   reset;              // 1 = 复位滤波器/控制器
        int   filter_mode;        // 0 = EKF, 1 = Madgwick + 垂速估计
        float hover_throttle;     // <=0 表示用固件默认 (0.25)
        float init_height_m;      // 复位时的初始离地高度 [m] (气压基准 = 地面)
        float tilt_sigma;         // EKF 倾角修正量测噪声 (<=0 用固件默认 0.05)
        float tilt_gate_mss;      // EKF 倾角修正门限 (<=0 用固件默认 2.0)
        float tilt_inno_limit_rad;// EKF 倾角修正 innovation 限幅 (<=0 用固件默认 0 = 不限幅)
        int   freeze_gyro_bias;   // 1 = 只在通过倾角判定的那次更新里改陀螺零偏
        float int_limit;          // 角速度环积分限幅 RATE_INT_LIMIT 覆盖 (<=0 用固件默认 5)
        float rate_i_scale;       // 角速度环积分增益倍数 (RATE_{ROLL,PITCH,YAW}_I 同乘, <=0 用固件默认 1)
        float tilt_angle_gate_deg;// 倾角修正的"角度门控": 只有当估计倾角 < 该值(deg)时才做倾角修正; <=0 = 不门控
        int   accel_hold_enable;  // 1 = 打开"准静止判定"(要连续满足 | |a|-g |<=gate 且角速度<=rate 超过 hold_time)
        float accel_hold_gate_mss;  // <=0 用固件默认 0.15
        float accel_hold_rate_dps;  // <=0 用固件默认 150
        float accel_hold_time_s;    // <=0 用固件默认 0.05
        int   tilt_air_weak;      // 1 = 解锁后把倾角修正换成 tilt_sigma/tilt_gate/tilt_inno 这一组(更弱),
                                  //     解锁前(停在地面上)保持固件默认那组 —— 地面上加速度计是有效的,
                                  //     可以快速收敛陀螺零偏; 空中则必须弱, 否则会把姿态拉平
    };

    struct FcSimOutput
    {
        float motor_phys[4];      // 物理 DShot 通道油门 (0~1) —— 直接写 fcs/dlx/motorN-nd
        float motor_logical[4];   // 逻辑电机 M0..M3 油门, 便于和混控器对照
        float quat[4];            // 估计姿态 (w,x,y,z, 机体->世界)
        float height_m;
        float vz_mps;
        float torque[3];          // 期望机体力矩 [N*m]
        float rate_setpoint[3];   // 期望角速度 [rad/s]
        float att_err_angle;      // 倾转误差角 [rad]
        float mix_scale;          // 混控收缩系数 (1 = 未饱和)
        float gyro_bias[3];       // EKF 估出来的陀螺零偏 [rad/s] (Madgwick 路线恒为 0)
        float accel_norm;         // 本步比力模长 [m/s^2]
        float tilt_angle_deg;     // 本步估计出来的倾角 [deg] (倾角门控用)
        float rate_int[3];        // 角速度环积分状态 (单位 rad/s, 钳位 RATE_INT_LIMIT)
        float ang_acc[3];         // 期望角加速度 (钳位 ANGACC_MAX)
        int   motor_saturated;
        int   filter_mode;
    };

    int fc_sim_sizeof_input(void)
    {
        return (int)sizeof(FcSimInput);
    }

    int fc_sim_sizeof_output(void)
    {
        return (int)sizeof(FcSimOutput);
    }

    // 按"当前是否解锁"把 EKF 倾角修正参数切成地面组 / 空中组
    static void apply_filter_params(const FcSimInput *in)
    {
        FlightControlFilterParams fp;
        fp.baro_sigma_m = kEkfBaroSigmaM;
        fp.estimate_accel_bias = false;
        fp.accel_tilt_gate_mss = kEkfAccelTiltGateMss;
        fp.accel_tilt_sigma = g.fp_tilt_sigma;

        const bool air = (in->armed != 0) && (in->tilt_air_weak != 0);
        if (air || in->tilt_sigma > 0.0f || in->tilt_gate_mss > 0.0f || in->tilt_inno_limit_rad > 0.0f
            || in->freeze_gyro_bias != 0)
        {
            fp.accel_tilt_sigma = (in->tilt_sigma > 0.0f) ? in->tilt_sigma : 0.05f;
            fp.accel_tilt_gate_mss = (in->tilt_gate_mss > 0.0f) ? in->tilt_gate_mss : kEkfAccelTiltGateMss;
            fp.accel_tilt_inno_limit_rad = in->tilt_inno_limit_rad;
            fp.freeze_gyro_bias_when_dynamic = (in->freeze_gyro_bias != 0);
        }
        else
        {
            fp.accel_tilt_inno_limit_rad = 0.0f;
        }
        if (in->accel_hold_enable != 0)
        {
            fp.accel_hold_enable = true;
            if (in->accel_hold_gate_mss > 0.0f) fp.accel_hold_gate_mss = in->accel_hold_gate_mss;
            if (in->accel_hold_rate_dps > 0.0f) fp.accel_hold_rate_dps = in->accel_hold_rate_dps;
            if (in->accel_hold_time_s > 0.0f)  fp.accel_hold_time_s = in->accel_hold_time_s;
        }
        g.ekf.set(fp);
    }

    void fc_sim_reset(void)
    {
        g.reset();
        g.last_armed = -1;
    }

    void fc_sim_set_hover_throttle(float t)
    {
        FlightControlParams p;
        if (t > 0.0f)
        {
            p.set(FlightParamId::HOVER_THROTTLE, t);
        }
        g.controller.params() = p;
        g.params = p;
    }

    void fc_sim_step(const FcSimInput *in, FcSimOutput *outp)
    {
        if (in == nullptr || outp == nullptr)
        {
            return;
        }

        if (in->reset || g.first_step)
        {
            g.reset();
            {
                FlightControlParams p;
                if (in->hover_throttle > 0.0f)
                {
                    p.set(FlightParamId::HOVER_THROTTLE, in->hover_throttle);
                }
                if (in->int_limit > 0.0f)
                {
                    p.set(FlightParamId::RATE_INT_LIMIT, in->int_limit);
                }
                if (in->rate_i_scale > 0.0f)
                {
                    p.set(FlightParamId::RATE_ROLL_I,
                          p.get(FlightParamId::RATE_ROLL_I) * in->rate_i_scale);
                    p.set(FlightParamId::RATE_PITCH_I,
                          p.get(FlightParamId::RATE_PITCH_I) * in->rate_i_scale);
                    p.set(FlightParamId::RATE_YAW_I,
                          p.get(FlightParamId::RATE_YAW_I) * in->rate_i_scale);
                }
                g.controller.params() = p;
                g.params = p;
            }
            g.filterMode = (in->filter_mode == FILTER_MADGWICK) ? FILTER_MADGWICK : FILTER_EKF;
            apply_filter_params(in);

            if (in->baro_valid)
            {
                g.last_baro_abs = in->baro_abs_m;
                g.baro_ref = 0.0f;      // 驱动侧给的就是"离地高度", 基准已经在驱动侧扣掉
                g.baro_ref_set = true;
            }
            g.arm(quatIdentity(), in->init_height_m);
            g.baro_healthy = (in->baro_valid != 0);
        }

        if (g.last_armed != in->armed)
        {
            g.last_armed = in->armed;
            apply_filter_params(in);   // 解锁/上锁瞬间切换倾角修正强度
        }

        //===========================================================================================
        // 1. IMU 快路径 (2000 Hz): 姿态积分   —— 与固件一样, 上电后一直在跑, 解锁前也跑
        //===========================================================================================
        GyroscopeRads gyroR{{in->gyro[0], in->gyro[1], in->gyro[2]}};
        AccelerometerG accG{{in->accel[0] / 9.80665f,
                             in->accel[1] / 9.80665f,
                             in->accel[2] / 9.80665f}};
        g.lastGyro = gyroR;
        g.lastAcc = accG;

        if (g.filterMode == FILTER_EKF)
        {
            g.ekf.integrateNominal(gyroR, accG, kImuDt);
            g.imu_dt_accum += kImuDt;
            if (++g.imu_cnt >= kEkfDecim)
            {
                g.ekf.propagateCovariance(gyroR, accG, g.imu_dt_accum);
                g.ekf.updateAccelerometer(accG);
                g.imu_cnt = 0;
                g.imu_dt_accum = 0.0f;
            }
            g.attitude = g.ekf.getQuaternion();
            g.height_m = g.ekf.getHeight();
            g.vz_mps = g.ekf.getVerticalVelocity();
        }
        else
        {
            g.ahrs.MadgwickAHRSupdateIMU(gyroR.data[0], gyroR.data[1], gyroR.data[2],
                                         accG.data[0], accG.data[1], accG.data[2]);
            g.attitude = g.ahrs.getQuaternion();
            const float baro_rel = in->baro_valid ? (in->baro_abs_m - g.baro_ref) : g.height_m;
            const VerticalVelocityEstimate est =
                g.vzEst.update(baro_rel, g.attitude, accG, kImuDt, in->baro_valid != 0);
            if (est.valid)
            {
                g.height_m = est.height_m;
                g.vz_mps = est.vertical_velocity_mps;
            }
        }

        //===========================================================================================
        // 2. 气压计 (50 Hz): 高度观测 + 健康判定 (连续 baroFailLimit 次没有样本 -> 气压不可用)
        //===========================================================================================
        if (in->baro_valid)
        {
            if (!g.baro_ref_set)
            {
                g.baro_ref = in->baro_abs_m;
                g.baro_ref_set = true;
            }
            g.last_baro_abs = in->baro_abs_m;
            g.baro_age_s = 0.0f;
            if (g.filterMode == FILTER_EKF)
            {
                g.ekf.updateBarometer(in->baro_abs_m - g.baro_ref);
            }
        }
        else
        {
            g.baro_age_s += kImuDt;
        }
        g.baro_healthy = g.baro_ref_set && (g.baro_age_s < 0.2f);

        //===========================================================================================
        // 3. 控制环 (500 Hz): 角度环 + 角速度环 -> 混控 -> 物理电机通道
        //    与 release.cpp 一样: 只有解锁 (en && !stop) 才进这一段, 之前不调用控制器
        //===========================================================================================
        if (in->armed)
        {
            if (++g.ctrl_cnt >= kCtrlDecim)
            {
                g.ctrl_cnt = 0;
                FlightControlSetpoint sp;
                sp.attitude = quatFromEulerZYX(in->target_yaw_rad, in->target_pitch_rad, in->target_roll_rad);
                sp.height_m = in->target_height_m;
                run_control_500hz(g.attitude, g.lastGyro, g.height_m, g.vz_mps, g.baro_healthy,
                                  sp, in->manual_throttle, in->height_mode != 0);
            }
        }
        else
        {
            for (int i = 0; i < 4; ++i)
            {
                g.motor_phys[i] = 0.0f;
                g.motor_logical[i] = 0.0f;
            }
            g.out = FlightControlOutput{};
            g.motor_saturated = false;
            g.mixed.mix_scale = 1.0f;
        }

        // ---- 输出 ----
        for (int i = 0; i < 4; ++i)
        {
            outp->motor_phys[i] = g.motor_phys[i];
            outp->motor_logical[i] = g.motor_logical[i];
        }
        outp->quat[0] = g.attitude.data[0];
        outp->quat[1] = g.attitude.data[1];
        outp->quat[2] = g.attitude.data[2];
        outp->quat[3] = g.attitude.data[3];
        outp->height_m = g.height_m;
        outp->vz_mps = g.vz_mps;
        for (int i = 0; i < 3; ++i)
        {
            outp->torque[i] = g.out.torque.data[i];
            outp->rate_setpoint[i] = g.out.rate_setpoint.data[i];
        }
        outp->att_err_angle = g.controller.debug().attitude_error_angle;
        outp->mix_scale = g.mixed.mix_scale;
        {
            const Vector3f bg = (g.filterMode == FILTER_EKF) ? g.ekf.getGyroBias() : Vector3f{};
            outp->gyro_bias[0] = bg.x();
            outp->gyro_bias[1] = bg.y();
            outp->gyro_bias[2] = bg.z();
        }
        outp->accel_norm = norm(Vector3f{in->accel[0], in->accel[1], in->accel[2]});
        outp->tilt_angle_deg = g.tilt_angle_deg;
        {
            const Vector3f ri = g.controller.rateIntegral();
            const Vector3f aa = g.controller.debug().angular_accel;
            for (int i = 0; i < 3; ++i)
            {
                outp->rate_int[i] = ri.data[i];
                outp->ang_acc[i] = aa.data[i];
            }
        }
        outp->motor_saturated = g.motor_saturated ? 1 : 0;
        outp->filter_mode = (int)g.filterMode;
    }
} // extern "C"

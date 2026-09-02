#pragma once
#include <math.h>
#include "../dlx_math.hpp"
#include "../dlx_sensor_data.hpp"

//===================================================================================================
// vertical_velocity_estimator.hpp
// 垂向速度估计器 (气压高度 + 姿态四元数 + IMU 加速度 融合, 纯计算, 无动态内存).
//
// 目的: 为定高控制提供"当前高度方向 (世界系 z 轴) 的速度".
//
// 原理 (互补滤波 / 低阶观测器):
//   1. 用姿态四元数把机体系加速度(比力)转到世界系:  f_world = q * f_body * q^-1;
//   2. 扣除重力得到世界系垂向加速度:                 az = f_world.z - g;
//   3. 加速度积分给出高频的垂向速度/高度 (响应快, 但会漂移);
//   4. 气压高度 (getAltitude 输出) 作为低频基准, 按比例修正高度/速度,
//      并顺带估计加速度零偏 (消除长时间悬停时的速度漂移).
//
// 约定:
//   - 输入四元数与 MadgwickAHRS 输出一致, data[0..3] = (w,x,y,z);
//   - 加速度输入为机体系比力 [g] (BMI088::getAccelerationG 输出);
//   - 高度向上为正 [m], 垂向速度向上为正 [m/s];
//   - 纯浮点计算, 适合 200Hz 以上 IMU 循环调用, 气压更新慢时传入最近一次读数即可.
//
// 使用示例 (放在 IMU 循环里, 气压样本到了就更新):
//   dlx::VerticalVelocityEstimator vz_est;
//   vz_est.reset(bme.getAltitude(raw));   // 起飞前用当前气压高度锁存
//   while (1) {
//       ...
//       dlx::VerticalVelocityEstimate est =
//           vz_est.update(bme.getAltitude(raw), ahrs.getQuaternion(),
//                         BMI088::getAccelerationG(accel_raw), dt);
//
//       dlx::FlightControlState state;
//       state.attitude = ahrs.getQuaternion();
//       state.height_m = est.height_m;                       // 融合后高度(比原始气压平滑)
//       state.vertical_velocity_mps = est.vertical_velocity_mps;
//       state.vertical_velocity_valid = est.valid;
//       float throttle = fc.update(sp, state, dt);
//   }
//===================================================================================================

namespace dlx
{
    // 估计结果
    struct VerticalVelocityEstimate
    {
        float height_m{0.0f};              // 融合后的高度 [m], 向上为正
        float vertical_velocity_mps{0.0f}; // 融合后的垂向速度 [m/s], 向上为正
        float accel_z_world_mss{0.0f};     // 世界系垂向加速度(已扣除重力) [m/s^2], 低通后
        float accel_bias_mss{0.0f};        // 估计出的加速度零偏 [m/s^2]
        bool valid{false};                 // 是否已初始化 (收到过有效气压)
    };

    // 估计器参数 (按需调整)
    struct VerticalVelocityEstimatorParams
    {
        float gravity_mss{9.80665f};    // 重力加速度 [m/s^2]
        float accel_lp_tau_s{0.05f};    // 垂向加速度一阶低通时间常数 [s] (约 20Hz 截止, 抑制振动)
        float corr_height{1.0f};        // 气压高度误差 -> 高度修正增益 [1/s]
        float corr_velocity{0.5f};      // 气压高度误差 -> 速度修正增益 [1/s]
        float corr_accel_bias{0.03f};   // 气压高度误差 -> 加速度零偏修正增益 [1/s^2]
        float max_baro_err_m{0.5f};     // 单次气压高度误差超过该值视为毛刺, 丢弃该样本 [m]
        float baro_reacquire_time_s{1.0f}; // 气压丢失超过该时间后, 重新获得时直接对齐高度 [s]
        float max_accel_bias_mss{1.0f}; // 加速度零偏估计限幅 [m/s^2] (约 ±0.1g)
        float accel_norm_min_g{0.5f};   // 加速度模长有效范围下限 [g], 之外不积分(自由落体/异常)
        float accel_norm_max_g{1.5f};   // 加速度模长有效范围上限 [g]
    };

    class VerticalVelocityEstimator
    {
    public:
        VerticalVelocityEstimator()
        {
            reset();
        }

        explicit VerticalVelocityEstimator(const VerticalVelocityEstimatorParams &params)
            : _params(params)
        {
            reset();
        }

        /**
         * @brief 每周期调用: 四元数 + 加速度积分, 并用气压高度修正
         * @param height_baro_m getAltitude() 输出的气压高度 [m] (无新样本时传入最近一次读数)
         * @param attitude      当前姿态四元数 (w,x,y,z), 与 MadgwickAHRS 输出一致
         * @param accel_g       机体系加速度 (比力) [g]
         * @param dt            距上次调用的时间 [s]
         * @param height_valid  本次气压高度是否有效; false 时只做积分 (气压暂不可用)
         */
        VerticalVelocityEstimate update(float height_baro_m,
                                        const Quaternion &attitude,
                                        const AccelerometerG &accel_g,
                                        float dt,
                                        bool height_valid = true)
        {
            // ---------- dt 保护 ----------
            if (!(dt > 0.0f)) {
                dt = 0.002f;
            }
            dt = constrain(dt, 0.0005f, 0.5f);

            // ---------- 首次有效气压: 初始化 ----------
            if (!_initialized && height_valid) {
                _initialized = true;
                _height = height_baro_m;
                _vz = 0.0f;
                _bias = 0.0f;
                _az_lp = 0.0f;
                _time_since_baro_s = 0.0f;
            }

            if (_initialized) {
                //=================================================================================
                // 1. 机体系比力 -> 世界系垂向加速度 (扣除重力)
                //=================================================================================
                const Quaternion q = quatNormalize(attitude);
                const Vector3f accel_mss{accel_g.data[0] * _params.gravity_mss,
                                         accel_g.data[1] * _params.gravity_mss,
                                         accel_g.data[2] * _params.gravity_mss};
                const Vector3f f_world = quatRotate(q, accel_mss); // 机体系 -> 世界系
                const float az_raw = f_world.z() - _params.gravity_mss;

                // 比力模长不在有效区间 (自由落体/异常数据) 时不更新低通, 保持上次值
                const float norm_g = norm(accel_mss) / _params.gravity_mss;
                if (norm_g >= _params.accel_norm_min_g && norm_g <= _params.accel_norm_max_g) {
                    const float alpha = dt / (_params.accel_lp_tau_s + dt);
                    _az_lp += (az_raw - _az_lp) * alpha;
                }

                //=================================================================================
                // 2. 积分 (高频通道)
                //=================================================================================
                _height += _vz * dt;
                _vz += (_az_lp + _bias) * dt;

                //=================================================================================
                // 3. 气压修正 (低频基准)
                //=================================================================================
                if (height_valid) {
                    const bool reacquire = (_time_since_baro_s > _params.baro_reacquire_time_s);
                    _time_since_baro_s = 0.0f;

                    const float err = height_baro_m - _height;
                    if (fabsf(err) <= _params.max_baro_err_m) {
                        // 正常修正: 高度/速度/零偏 三个通道按各自增益拉回
                        _height += _params.corr_height * err * dt;
                        _vz += _params.corr_velocity * err * dt;
                        _bias += _params.corr_accel_bias * err * dt;
                        _bias = constrain(_bias, -_params.max_accel_bias_mss, _params.max_accel_bias_mss);
                    } else if (reacquire) {
                        // 气压长时间丢失后重新获得: 直接对齐高度, 速度保留 (避免持续错误积分)
                        _height = height_baro_m;
                    }
                    // 否则: 视为气压毛刺, 丢弃该样本
                } else {
                    _time_since_baro_s += dt;
                }
            }

            VerticalVelocityEstimate out;
            out.height_m = _height;
            out.vertical_velocity_mps = _vz;
            out.accel_z_world_mss = _az_lp;
            out.accel_bias_mss = _bias;
            out.valid = _initialized;
            return out;
        }

        // 完全清零; 下一次 update 首次收到有效气压时自动初始化
        void reset()
        {
            _initialized = false;
            _height = 0.0f;
            _vz = 0.0f;
            _bias = 0.0f;
            _az_lp = 0.0f;
            _time_since_baro_s = 0.0f;
        }

        // 直接给定初始状态 (起飞前用当前气压高度锁存, 可避免起飞瞬间的收敛过渡)
        void reset(float height_m, float vertical_velocity_mps = 0.0f)
        {
            _initialized = true;
            _height = height_m;
            _vz = vertical_velocity_mps;
            _bias = 0.0f;
            _az_lp = 0.0f;
            _time_since_baro_s = 0.0f;
        }

        VerticalVelocityEstimatorParams &params()
        {
            return _params;
        }

        const VerticalVelocityEstimatorParams &params() const
        {
            return _params;
        }

    private:
        VerticalVelocityEstimatorParams _params;
        bool _initialized{false};
        float _height{0.0f};             // 融合后高度 [m]
        float _vz{0.0f};                 // 融合后垂向速度 [m/s]
        float _bias{0.0f};               // 加速度零偏估计 [m/s^2]
        float _az_lp{0.0f};              // 垂向加速度低通 [m/s^2]
        float _time_since_baro_s{0.0f};  // 距上次气压修正的时间 [s]
    };

} // namespace dlx

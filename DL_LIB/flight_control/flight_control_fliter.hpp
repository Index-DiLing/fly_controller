#pragma once
#include "dlx_math.hpp"
#include "math.h"

//===================================================================================================
// flight_control_fliter.hpp
// 无人机传感器融合滤波器: 16 维状态扩展卡尔曼滤波 (EKF).
//
// 目的: 把 IMU(加速度计+陀螺仪) / 气压高度计 / GNSS / 磁力计 融合成一套自洽的
//       "姿态 + 速度 + 位置 + 陀螺零偏 + 加速度零偏" 状态, 代替分散的
//       MadgwickAHRS + VerticalVelocityEstimator 组合 (二者可作为本滤波器的退化/备份).
//
// 状态向量 (16 维, 名义状态, 单位为 SI):
//   idx  0.. 3 : 姿态四元数 q = (w,x,y,z)      机体系 -> 世界系
//   idx  4.. 6 : 世界系速度 v = (vx,vy,vz)     [m/s]
//   idx  7.. 9 : 世界系位置 p = (px,py,pz)     [m], z 向上为正
//   idx 10..12 : 陀螺零偏 bg = (bgx,bgy,bgz)   机体系 [rad/s]
//   idx 13..15 : 加速度零偏 ba = (bax,bay,baz) 机体系 [m/s^2]
//
// 坐标系约定 (与工程内 FlightController / VerticalVelocityEstimator 保持一致):
//   - 机体系: x 前(机头), y 左, z 上, 右手系;
//   - 世界系: 本地切平面 ENU, z 向上为正 (x 东, y 北, z 天);
//   - q 为 "机体 -> 世界" 旋转四元数, 即 v_world = R(q) * v_body.
//     这与 MadgwickAHRS 的 q 及 quatDcmZ / quatRotate / quatToEuler 的用法一致
//     (注意 dlx_math.hpp 顶部注释里的"世界->机体"字样为笔误, 实际代码均为机体->世界).
//
// 实现方式: 对四元数使用"误差态"表示 (姿态误差 δθ 为机体系小旋转向量),
//   因而协方差 P 是 15x15 (误差态 e = [δθ(3), δv(3), δp(3), δbg(3), δba(3)]),
//   避免直接对 4 元四元数做全 16 维协方差带来的冗余/奇异问题. 名义均值是 16 维.
//
// 可观测量测: 以下 H 矩阵均在"误差态"下给出 (δx 为求得的误差修正量):
//   H_accel (3x15) : 重力方向倾角修正 (仅可观测滚转/俯仰, 偏航不可观).
//                    机体系重力方向测量 h = R(q)^T * [0,0,1] (单位),
//                    inno = (a/norm(a)) - h,  H[0:3,0:3] = [h]x.
//   H_baro  (1x15) : 气压高度,  inno = alt_meas - pz,  H[0][8] = 1.
//   H_gpsP  (3x15) : GNSS 位置,  inno = pos - p,       H[0:3,6:9] = I.
//   H_gpsV  (3x15) : GNSS 速度,  inno = vel - v,       H[0:3,3:6] = I.
//   H_mag   (3x15) : 磁力计航向, 机体系磁测 h = R(q)^T * m_world,
//                    inno = mag - h,  H[0:3,0:3] = [h]x (含偏航).
//
// 高度源接口 (统一作为 z 位置观测, H[0][8]=1):
//   updateBarometer(alt_m)      气压高度 [m]
//   updateRangeHeight(h_m, gz)  超声波/激光: 离地高度 AGL + 地面高度 gz (默认 gz=0)
//   updateHeightSource(alt, s)  通用高度源, 自定义噪声 sigma
// 高度越准确, sigma 越小, 滤波器越信任 (近地面可用超声波/激光, 高空用气压).
//
// GNSS 接口:
//   updateGpsPosition(pos_m)   ENU 位置
//   updateGpsVelocity(vel_mps) ENU 速度
//   updateGps(pos, vel)        位置+速度合并
// 水平位置/速度与加速度零偏只有在可见 GNSS(或其它水平定位)时才可行, 否则退化.
//
// 使用示例 (放在 IMU 循环, 可按传感器到达频率分别调用):
//   dlx::FlightControlFilter filt;
//   filt.reset(bme.getAltitude(raw));                 // 起飞前用当前气压高度锁存
//   while (1) {
//       filt.predict(gyro_rads, accel_mss, dt);       // 或传 AccelerometerG([g])
//       filt.updateAccelerometer(accel_mss);          // 低动态时做倾斜修正(可关)
//       if (baro_new)   filt.updateBarometer(bme.getAltitude(raw));
//       if (range_new)  filt.updateRangeHeight(range_agl_m);       // 近地面用超声波/激光
//       if (gps_ok)     filt.updateGps(gps_pos, gps_vel);
//       if (mag_ok)     filt.updateMagnetometer(mag_meas, mag_world);
//
//       Quaternion q   = filt.getQuaternion();        // 机体->世界, 直接喂控制器
//       float      h   = filt.getHeight();            // 世界系 z [m]
//       float      vz  = filt.getVerticalVelocity();  // [m/s]
//   }
//
// 无 GNSS / 磁力计的退化情形:
//   - 滚转/俯仰: 由加速度计倾角修正约束 (与 Madgwick 同源), 稳定且不翻转;
//   - 高度/垂速: 由气压/超声波高度修正 (与 VerticalVelocityEstimator 同源);
//   - 偏航与水平位置/速度: 不可观测, 会漂移 (代码注释已说明单 IMU 偏航不准, 暂不控制).
//   此退化情形下它可作为 Madgwick+VES 的可选增强/备份; 接上 GNSS/磁力计后性能才真正优于二者.
//
// 零偏估计 (重要):
//   - 陀螺零偏的滚转/俯仰分量可被加速度计倾角修正微弱可观, 默认开启;
//   - 加速度零偏在无 GNSS/速度观测(或水平定位)时基本不可观测, 属正常现象. 此时应
//     将 params.estimate_accel_bias 置 false, 让加速度零偏保持为 reset/初始化给的常数,
//     避免被高度/位置量测通过弱耦合拖偏 (滤波器的"倾角-速度-零偏"退化自由度).
//   - 一旦接了 GNSS 速度/位置, 设 true 即可在线学习加速度零偏, 此时 16 维状态才算完整可用.
//===================================================================================================

namespace dlx
{
    // 名义状态向量索引 (16 维)
    enum FlightFilterStateIdx : int
    {
        FF_QW = 0, FF_QX, FF_QY, FF_QZ, // 0..3  姿态四元数 (w,x,y,z)
        FF_VX, FF_VY, FF_VZ,            // 4..6  世界系速度
        FF_PX, FF_PY, FF_PZ,            // 7..9  世界系位置 (z 向上)
        FF_BGX, FF_BGY, FF_BGZ,         // 10..12 陀螺零偏
        FF_BAX, FF_BAY, FF_BAZ,         // 13..15 加速度零偏
        FF_STATE_COUNT,
    };

    // 误差态向量索引 (15 维)
    enum FlightFilterErrorIdx : int
    {
        FF_E_DTH0 = 0, FF_E_DTH1, FF_E_DTH2,   // 0..2  δθ 机体系姿态误差
        FF_E_DV0, FF_E_DV1, FF_E_DV2,          // 3..5  δv 世界系速度
        FF_E_DP0, FF_E_DP1, FF_E_DP2,          // 6..8  δp 世界系位置
        FF_E_DBG0, FF_E_DBG1, FF_E_DBG2,       // 9..11 δbg
        FF_E_DBA0, FF_E_DBA1, FF_E_DBA2,       // 12..14 δba
        FF_ERROR_COUNT,
    };

    // 滤波器参数 (按传感器特性和激励程度整定)
    struct FlightControlFilterParams
    {
        // ---- 重力 / 过程噪声 ---- (Q = sigma^2 * dt)
        float gravity_mss{9.80665f};      // 重力加速度 [m/s^2]
        float gyro_noise_sigma{0.02f};    // 陀螺角速率噪声 [rad/s/√Hz]
        float accel_noise_sigma{0.15f};   // 加速度计比力噪声 [m/s^2/√Hz]
        float gyro_bias_walk_sigma{1e-5f}; // 陀螺零偏随机游走 [rad/s²/√Hz]
        float accel_bias_walk_sigma{1e-5f}; // 加速度零偏随机游走 [m/s²/√Hz]

        // ---- 量测噪声 ----
        float accel_tilt_gate_mss{1.0f};  // 倾角修正门限: |a|-g 超过则忽略 [m/s^2]
        float accel_tilt_sigma{0.05f};    // 倾角修正量测噪声 (无量纲方向)
        float baro_sigma_m{0.3f};         // 气压高度噪声 [m]
        float rangefinder_sigma_m{0.05f}; // 超声波/激光测距高度噪声 [m] (近地面精度高)
        float gps_pos_sigma_m{2.0f};      // GNSS 位置噪声 [m]
        float gps_vel_sigma_mps{0.3f};    // GNSS 速度噪声 [m/s]
        float mag_sigma{0.05f};           // 磁力计噪声 [uT]

        // ---- 零偏是否参与估计 ----
        // 陀螺零偏(滚转/俯仰分量)可由加速度计倾角修正微弱可观, 故默认开;
        // 加速度零偏在无 GNSS/速度观测时不可观, 关闭后即为常数(保持 reset 值), 避免被拖偏.
        bool estimate_gyro_bias{true};
        bool estimate_accel_bias{false};

        // ---- 零偏限幅 (抑制不可观测方向的漂移) ----
        float gyro_bias_limit{0.3f};      // [rad/s]
        float accel_bias_limit{2.0f};     // [m/s^2]

        // ---- 初始协方差 (对角线) ----
        float init_att_var{0.01f};        // (0.1 rad)^2
        float init_vel_var{0.25f};        // (0.5 m/s)^2
        float init_pos_var{1.0f};         // (1 m)^2
        float init_gyro_bias_var{1e-4f};  // (0.01 rad/s)^2
        float init_accel_bias_var{1e-3f}; // (0.03 m/s^2)^2
    };

    // 最近一次迭代的状态 (16 维原始数组快照, 便于日志/一致性检查)
    struct FlightFilterState16
    {
        float data[FF_STATE_COUNT];
    };

    class FlightControlFilter
    {
    public:
        FlightControlFilter()
        {
            reset();
        }

        explicit FlightControlFilter(const FlightControlFilterParams &params)
            : _params(params)
        {
            reset();
        }

        void set(const FlightControlFilterParams &params){
            _params = params;
            
        }

        //============================================================================================
        // 初始化 / 复位
        //============================================================================================
        // 完全清零; 下一次 predict 前需先 reset 指定初始姿态/位置/高度, 避免悬停收敛过渡
        void reset()
        {
            _q = quatIdentity();   // 机体->世界, 初始默认水平
            _v = Vector3f{};
            _p = Vector3f{};
            _bg = Vector3f{};
            _ba = Vector3f{};
            _valid = false;
            _P = MatF<FF_ERROR_COUNT, FF_ERROR_COUNT>{};
            _P.d[0][0] = _P.d[1][1] = _P.d[2][2] = _params.init_att_var;
            _P.d[3][3] = _P.d[4][4] = _P.d[5][5] = _params.init_vel_var;
            _P.d[6][6] = _P.d[7][7] = _P.d[8][8] = _params.init_pos_var;
            _P.d[9][9] = _P.d[10][10] = _P.d[11][11] = _params.init_gyro_bias_var;
            _P.d[12][12] = _P.d[13][13] = _P.d[14][14] = _params.init_accel_bias_var;
        }

        // 用初始姿态/位置直接定位 (起飞前调用; 高度=位置 z). 姿态默认水平.
        void reset(const Quaternion &attitude, const Vector3f &position,
                   const Vector3f &velocity = Vector3f{}, const Vector3f &gyro_bias = Vector3f{},
                   const Vector3f &accel_bias = Vector3f{})
        {
            reset();
            _q = quatNormalize(attitude);
            _p = position;
            _v = velocity;
            _bg = gyro_bias;
            _ba = accel_bias;
            _valid = true;
        }

        // 仅用气压高度锁存高度, 其余按默认 (起飞前常用)
        void reset(float baro_altitude_m)
        {
            reset();
            _p.z() = baro_altitude_m;
            _valid = true;
        }

        //============================================================================================
        // 预测 (IMU 输入)
        //  - gyro  机体系角速率 [rad/s]
        //  - accel 机体系比力 [m/s^2]
        //============================================================================================
        // 快路径: 每 IMU 样本只推进名义状态 q/v/p(用去偏后的测量), 不碰协方差 P。
        // 零偏 _bg/_ba 由低频量测更新估计, 这里直接扣除, 因此短间隔直接积分不会有偏差累积。
        //============================================================================================
        void integrateNominal(const GyroscopeRads &gyro, const Vector3f &accel_mss, float dt)
        {
            dt = sanitizeDt(dt);
            _valid = true;

            // ---- 去偏置后的测量 ----
            const Vector3f om(gyro.data[0] - _bg.x(),
                              gyro.data[1] - _bg.y(),
                              gyro.data[2] - _bg.z());
            const Vector3f fb(accel_mss.x() - _ba.x(),
                              accel_mss.y() - _ba.y(),
                              accel_mss.z() - _ba.z());

            // 四元数: q <- q * exp([0; om] dt)
            const float om_norm = norm(om);
            if (om_norm > 1e-9f) {
                _q = quatNormalize(quatMul(_q, quatFromAxisAngle(om, om_norm * dt)));
            }

            const Vector3f a_world = quatRotate(_q, fb) + Vector3f{0.0f, 0.0f, -_params.gravity_mss};
            const Vector3f v_new = _v + a_world * dt;
            const Vector3f p_new = _p + (_v + v_new) * (0.5f * dt); // 梯形积分
            _v = v_new;
            _p = p_new;
        }

        void integrateNominal(const GyroscopeRads &gyro, const AccelerometerG &accel_g, float dt)
        {
            Vector3f accel_mss(accel_g.data[0] * _params.gravity_mss,
                               accel_g.data[1] * _params.gravity_mss,
                               accel_g.data[2] * _params.gravity_mss);
            integrateNominal(gyro, accel_mss, dt);
        }

        //============================================================================================
        // 慢路径: 只推进误差态协方差 P(用当前状态构造 F/Q, 步长 = 累计 dt), 不改动名义状态。
        // 协方差是慢变量, 无需每样本刷新, 低频(如 250Hz)调用即可。
        //============================================================================================
        void propagateCovariance(const GyroscopeRads &gyro, const Vector3f &accel_mss, float dt)
        {
            dt = sanitizeDt(dt);

            const Vector3f om(gyro.data[0] - _bg.x(),
                              gyro.data[1] - _bg.y(),
                              gyro.data[2] - _bg.z());
            const Vector3f fb(accel_mss.x() - _ba.x(),
                              accel_mss.y() - _ba.y(),
                              accel_mss.z() - _ba.z());

            // ---- 误差态连续雅可比 F (15x15) -> 离散转移 Phi = I + F*dt ----
            const MatF<3, 3> Rq = matFromQuaternion(_q);
            const MatF<3, 3> sk_om = skew(om);
            const MatF<3, 3> sk_fb = skew(fb);
            const MatF<3, 3> R_sk_fb = matMul(Rq, sk_fb);

            MatF<FF_ERROR_COUNT, FF_ERROR_COUNT> F;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    F.d[r][c] = -sk_om.d[r][c];           // δθdot = -[om]x δθ
                    if (_params.estimate_gyro_bias) {
                        F.d[r][9 + c] = -(r == c ? 1.0f : 0.0f); // δθdot 随 δbg: -I
                    }
                    F.d[3 + r][c] = -R_sk_fb.d[r][c];     // δvdot 随 δθ: -R[fb]x
                    if (_params.estimate_accel_bias) {
                        F.d[3 + r][12 + c] = -Rq.d[r][c]; // δvdot 随 δba: -R
                    }
                    F.d[6 + r][3 + c] = (r == c ? 1.0f : 0.0f); // δpdot = δv
                }
            }
            const MatF<FF_ERROR_COUNT, FF_ERROR_COUNT> Phi =
                matAdd(matIdent<FF_ERROR_COUNT>(), matScale(F, dt));

            // ---- 过程噪声 Q (块对角, Q = sigma^2 * dt) ----
            MatF<FF_ERROR_COUNT, FF_ERROR_COUNT> Q;
            for (int i = 0; i < 3; ++i) {
                Q.d[i][i] = _params.gyro_noise_sigma * _params.gyro_noise_sigma * dt;
                Q.d[3 + i][3 + i] = _params.accel_noise_sigma * _params.accel_noise_sigma * dt;
                if (_params.estimate_gyro_bias) {
                    Q.d[9 + i][9 + i] = _params.gyro_bias_walk_sigma * _params.gyro_bias_walk_sigma * dt;
                }
                if (_params.estimate_accel_bias) {
                    Q.d[12 + i][12 + i] = _params.accel_bias_walk_sigma * _params.accel_bias_walk_sigma * dt;
                }
            }

            // ---- P = Phi * P * Phi^T + Q ----
            _P = matAdd(matMul(matMul(Phi, _P), matTranspose(Phi)), Q);

            // 数值保护: 协方差对称化 + 防负
            for (int i = 0; i < FF_ERROR_COUNT; ++i) {
                for (int j = 0; j < FF_ERROR_COUNT; ++j) {
                    float s = 0.5f * (_P.d[i][j] + _P.d[j][i]);
                    if (i == j && s < 0.0f) s = 0.0f;
                    _P.d[i][j] = _P.d[j][i] = s;
                }
            }
        }

        void propagateCovariance(const GyroscopeRads &gyro, const AccelerometerG &accel_g, float dt)
        {
            Vector3f accel_mss(accel_g.data[0] * _params.gravity_mss,
                               accel_g.data[1] * _params.gravity_mss,
                               accel_g.data[2] * _params.gravity_mss);
            propagateCovariance(gyro, accel_mss, dt);
        }

        //============================================================================================
        // 全速率 predict: 名义积分 + 协方差传播(等价于原 predict, 保留兼容/全速率使用)。
        // 主循环已拆成"快 integrateNominal + 慢 propagateCovariance"分别调用。
        //============================================================================================
        void predict(const GyroscopeRads &gyro, const Vector3f &accel_mss, float dt)
        {
            integrateNominal(gyro, accel_mss, dt);
            propagateCovariance(gyro, accel_mss, dt);
        }

        // 便捷: 加速度计以 g 为单位 (BMI088::getAccelerationG 输出)
        void predict(const GyroscopeRads &gyro, const AccelerometerG &accel_g, float dt)
        {
            Vector3f accel_mss(accel_g.data[0] * _params.gravity_mss,
                               accel_g.data[1] * _params.gravity_mss,
                               accel_g.data[2] * _params.gravity_mss);
            predict(gyro, accel_mss, dt);
        }

        //============================================================================================
        // 量测更新
        //============================================================================================
        // 加速度计倾角修正: 仅在近似静止/匀速时可信 (| |a|-g | 不超过门限).
        // 输入为机体系比力 [m/s^2] 或 [g].
        void updateAccelerometer(const Vector3f &accel_mss)
        {
            const float n = norm(accel_mss);
            if (fabsf(n - _params.gravity_mss) > _params.accel_tilt_gate_mss) {
                return; // 高动态, 丢弃 (该倾角修正只在低动态有效)
            }
            // 预测的机体系重力方向 (单位): h = R(q)^T * [0,0,1]
            const Vector3f h = quatRotate(quatConjugate(_q), Vector3f{0.0f, 0.0f, 1.0f});
            const Vector3f z = accel_mss / n;

            MatF<3, 1> innov;
            innov.d[0][0] = z.x() - h.x();
            innov.d[1][0] = z.y() - h.y();
            innov.d[2][0] = z.z() - h.z();

            MatF<3, FF_ERROR_COUNT> H;
            const MatF<3, 3> sh = skew(h); // [h]x
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    H.d[i][j] = sh.d[i][j];
                }
            }
            MatF<3, 3> Rz = matScale(matIdent<3>(), _params.accel_tilt_sigma * _params.accel_tilt_sigma);
            kalmanUpdate(H, Rz, innov);
        }

        void updateAccelerometer(const AccelerometerG &accel_g)
        {
            Vector3f accel_mss(accel_g.data[0] * _params.gravity_mss,
                               accel_g.data[1] * _params.gravity_mss,
                               accel_g.data[2] * _params.gravity_mss);
            updateAccelerometer(accel_mss);
        }

        // 气压高度 [m] (向上为正, 与状态 pz 同号; 以起飞点/海平面为参考系)
        void updateBarometer(float altitude_m)
        {
            applyHeightUpdate(altitude_m, _params.baro_sigma_m);
        }

        // 超声波 / 激光测距: 垂直方向到地面的高度 AGL [m].
        //   观测的世界系 z = ground_z + range. 默认 ground_z=0 表示 pz 以地面为原点;
        //   若 pz 以起飞点为原点且地面在起飞点下方 pGround_z, 则传 ground_z=pGround_z.
        void updateRangeHeight(float height_m, float ground_z = 0.0f)
        {
            applyHeightUpdate(ground_z + height_m, _params.rangefinder_sigma_m);
        }

        // 高度类量测的统一入口 (气压 / 超声波 / 其它高度源), 均作为 z 位置观测
        void updateHeightSource(float altitude_m, float sigma_m)
        {
            applyHeightUpdate(altitude_m, sigma_m);
        }

        // GNSS 本地切平面位置 (ENU [m]) 与地速 (ENU [m/s]). 可单独调用其一.
        void updateGpsPosition(const Vector3f &pos_m)
        {
            MatF<3, FF_ERROR_COUNT> H;
            for (int i = 0; i < 3; ++i) {
                H.d[i][FF_E_DP0 + i] = 1.0f;
            }
            MatF<3, 3> Rz = matScale(matIdent<3>(), _params.gps_pos_sigma_m * _params.gps_pos_sigma_m);
            MatF<3, 1> innov;
            innov.d[0][0] = pos_m.x() - _p.x();
            innov.d[1][0] = pos_m.y() - _p.y();
            innov.d[2][0] = pos_m.z() - _p.z();
            kalmanUpdate(H, Rz, innov);
        }

        void updateGpsVelocity(const Vector3f &vel_mps)
        {
            MatF<3, FF_ERROR_COUNT> H;
            for (int i = 0; i < 3; ++i) {
                H.d[i][FF_E_DV0 + i] = 1.0f;
            }
            MatF<3, 3> Rz = matScale(matIdent<3>(), _params.gps_vel_sigma_mps * _params.gps_vel_sigma_mps);
            MatF<3, 1> innov;
            innov.d[0][0] = vel_mps.x() - _v.x();
            innov.d[1][0] = vel_mps.y() - _v.y();
            innov.d[2][0] = vel_mps.z() - _v.z();
            kalmanUpdate(H, Rz, innov);
        }

        void updateGps(const GnssPosition &pos, const GnssVelocity &vel)
        {
            updateGpsPosition(Vector3f(pos.data[0], pos.data[1], pos.data[2]));
            updateGpsVelocity(Vector3f(vel.data[0], vel.data[1], vel.data[2]));
        }

        // 磁力计航向修正. mag_meas 为机体系磁场, mag_world 为当地地磁在世界系 (ENU) 的已知向量.
        void updateMagnetometer(const Vector3f &mag_meas, const Vector3f &mag_world)
        {
            const Vector3f h = quatRotate(quatConjugate(_q), mag_world); // R(q)^T * m_world
            MatF<3, 1> innov;
            innov.d[0][0] = mag_meas.x() - h.x();
            innov.d[1][0] = mag_meas.y() - h.y();
            innov.d[2][0] = mag_meas.z() - h.z();

            MatF<3, FF_ERROR_COUNT> H;
            const MatF<3, 3> sh = skew(h);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    H.d[i][j] = sh.d[i][j];
                }
            }
            MatF<3, 3> Rz = matScale(matIdent<3>(), _params.mag_sigma * _params.mag_sigma);
            kalmanUpdate(H, Rz, innov);
        }

        //============================================================================================
        // 输出 (上一次迭代)
        //============================================================================================
        Quaternion getQuaternion() const { return quatNormalize(_q); }
        Vector3f getVelocity() const { return _v; }
        Vector3f getPosition() const { return _p; }
        Vector3f getGyroBias() const { return _bg; }
        Vector3f getAccelBias() const { return _ba; }
        float getHeight() const { return _p.z(); }               // 世界系 z 高度 [m]
        float getVerticalVelocity() const { return _v.z(); }     // 垂向速度 [m/s]
        bool isValid() const { return _valid; }

        // 按 16 维状态向量顺序拷贝到外部 float[16], 便于日志/上位机
        void getState(float out[FF_STATE_COUNT]) const
        {
            const FlightFilterState16 s = getState16();
            for (int i = 0; i < FF_STATE_COUNT; ++i) {
                out[i] = s.data[i];
            }
        }

        // 按 16 维状态向量顺序取, 便于日志/一致性检查
        FlightFilterState16 getState16() const
        {
            FlightFilterState16 s;
            s.data[FF_QW] = _q.data[0]; s.data[FF_QX] = _q.data[1];
            s.data[FF_QY] = _q.data[2]; s.data[FF_QZ] = _q.data[3];
            s.data[FF_VX] = _v.x(); s.data[FF_VY] = _v.y(); s.data[FF_VZ] = _v.z();
            s.data[FF_PX] = _p.x(); s.data[FF_PY] = _p.y(); s.data[FF_PZ] = _p.z();
            s.data[FF_BGX] = _bg.x(); s.data[FF_BGY] = _bg.y(); s.data[FF_BGZ] = _bg.z();
            s.data[FF_BAX] = _ba.x(); s.data[FF_BAY] = _ba.y(); s.data[FF_BAZ] = _ba.z();
            return s;
        }

        FlightControlFilterParams &params() { return _params; }
        const FlightControlFilterParams &params() const { return _params; }

    private:
        // 高度(世界系 z 位置)量测的公共实现
        void applyHeightUpdate(float altitude_m, float sigma_m)
        {
            MatF<1, FF_ERROR_COUNT> H;
            H.d[0][FF_E_DP2] = 1.0f;
            MatF<1, 1> Rz;
            Rz.d[0][0] = sigma_m * sigma_m;
            MatF<1, 1> innov;
            innov.d[0][0] = altitude_m - _p.z();
            kalmanUpdate(H, Rz, innov);
        }

        static float sanitizeDt(float dt)
        {
            if (!(dt > 0.0f)) {
                dt = 0.002f; // 周期无效时按 500Hz 默认
            }
            return constrain(dt, 0.0005f, 0.5f);
        }

        // 通用卡尔曼更新 (误差态): inno = z - h, H 为 M x 15
        template <size_t M>
        void kalmanUpdate(const MatF<M, FF_ERROR_COUNT> &H, const MatF<M, M> &Rz,
                          const MatF<M, 1> &innov)
        {
            // S = H P H^T + R
            const MatF<M, M> S = matAdd(matMul(matMul(H, _P), matTranspose(H)), Rz);
            const MatF<M, M> Sinv = invSmall(S);

            // K = P H^T S^-1
            const MatF<FF_ERROR_COUNT, M> K =
                matMul(matMul(_P, matTranspose(H)), Sinv);

            // 误差修正 dx = K * inno
            const MatF<FF_ERROR_COUNT, 1> dx = matMul(K, innov);

            // 协方差: P = (I-KH) P (I-KH)^T + K R K^T
            const MatF<FF_ERROR_COUNT, FF_ERROR_COUNT> KH = matMul(K, H);
            const MatF<FF_ERROR_COUNT, FF_ERROR_COUNT> IKH =
                matSub(matIdent<FF_ERROR_COUNT>(), KH);
            const MatF<FF_ERROR_COUNT, FF_ERROR_COUNT> IKHt = matTranspose(IKH);
            const MatF<FF_ERROR_COUNT, FF_ERROR_COUNT> KRK =
                matMul(matMul(K, Rz), matTranspose(K));
            _P = matAdd(matMul(matMul(IKH, _P), IKHt), KRK);

            injectError(dx);
        }

        // 把误差态修正注入名义状态; 姿态用四元数右乘 exp(1/2 δθ) (机体系误差)
        void injectError(const MatF<FF_ERROR_COUNT, 1> &dx)
        {
            const Vector3f dth(dx.d[0][0], dx.d[1][0], dx.d[2][0]);
            const Vector3f dv(dx.d[3][0], dx.d[4][0], dx.d[5][0]);
            const Vector3f dp(dx.d[6][0], dx.d[7][0], dx.d[8][0]);
            const Vector3f dbg(dx.d[9][0], dx.d[10][0], dx.d[11][0]);
            const Vector3f dba(dx.d[12][0], dx.d[13][0], dx.d[14][0]);

            const float ang = norm(dth);
            if (ang > 1e-8f) {
                _q = quatNormalize(quatMul(_q, quatFromAxisAngle(dth, ang))); // exp(1/2 δθ)
            }
            _v += dv;
            _p += dp;
            if (_params.estimate_gyro_bias) {
                _bg += dbg;
            }
            if (_params.estimate_accel_bias) {
                _ba += dba;
            }

            // 零偏限幅 (抑制不可观测方向的漂移)
            _bg = constrain(_bg, -_params.gyro_bias_limit, _params.gyro_bias_limit);
            _ba = constrain(_ba, -_params.accel_bias_limit, _params.accel_bias_limit);
        }

        // 小矩阵求逆: M=1 或 M=3
        template <size_t M>
        static MatF<M, M> invSmall(const MatF<M, M> &m)
        {
            return MatrixInvert<M>::apply(m);
        }

        // 1x1 / 3x3 求逆的标签分发
        template <size_t M>
        struct MatrixInvert;

        FlightControlFilterParams _params;

        // 名义状态
        Quaternion _q;   // 机体 -> 世界
        Vector3f _v;     // 世界系速度
        Vector3f _p;     // 世界系位置
        Vector3f _bg;    // 陀螺零偏
        Vector3f _ba;    // 加速度零偏
        bool _valid;

        // 误差态协方差 (15 x 15)
        MatF<FF_ERROR_COUNT, FF_ERROR_COUNT> _P;
    };

    // 1x1 特化
    template <>
    struct FlightControlFilter::MatrixInvert<1>
    {
        static MatF<1, 1> apply(const MatF<1, 1> &m)
        {
            MatF<1, 1> r;
            if (fabsf(m.d[0][0]) > 1e-12f) {
                r.d[0][0] = 1.0f / m.d[0][0];
            }
            return r;
        }
    };

    // 3x3 特化
    template <>
    struct FlightControlFilter::MatrixInvert<3>
    {
        static MatF<3, 3> apply(const MatF<3, 3> &m)
        {
            return mat3Inverse(m);
        }
    };

} // namespace dlx

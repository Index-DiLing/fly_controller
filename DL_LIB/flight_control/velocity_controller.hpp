#pragma once
#include <float.h>
#include <math.h>
#include <stdint.h>
#include "../dlx_math.hpp"

//===================================================================================================
// velocity_controller.hpp —— 水平速度环 VEL_HOLD (纯逻辑层: 输入速度, 输出**期望姿态**)
//
// 规格: docs/2026-10-02_upgrade_plan_v3.md §6.1(级联位置) / §6.2(实现方式) / §6.5(新配置)。
// 级联: v_sp(世界系) → 速度 PID → a_sp(世界系 m/s²) → ẑ_b_des = normalize(ax, ay, g) → q_sp
//       → ★复用现成的 FlightController::updateAngle()/update()(倾转分离 + ATT_*_P + 角速度环)
//       → torque → ★复用 mixMotors() → motor[4]。**不改 FlightController**。
// 参数: 全部由调用方从 cfg::control::kVelXy* / kHoverThrottle / kThrottleMin / kThrottleMax /
//       kTiltMaxRad 填(本文件不 include 配置/驱动/profile 头, 只依赖 dlx_math.hpp)。
//
//---------------------------------------------------------------------------------------------------
// 一、坐标系与符号约定 (Lead 拿去和 EKF/B6 对齐; 与 flight_controller.hpp 完全一致)
//---------------------------------------------------------------------------------------------------
//   - 输入输出姿态四元数 = 世界系 (w,x,y,z), 与 MadgwickAHRS 输出同系, "机体系→世界系"。
//   - 输入/输出的水平速度 target_v*/current_v* 都是**世界系**水平两轴 [m/s] (ENU 的 x/y, z 向上)。
//     ⇒ 与姿态四元数必须是**同一个世界系**; 否则倾角指令会朝错误方向(甚至随偏航旋转)。
//       机体速度(光流估计器输出)必须先 v_world = quatRotate(q_ekf, v_body) 再进本控制器(§6.3)。
//   - 世界系 +x 方向加速度指令 ⇒ 期望姿态把机体 z 轴压向 +x_world: 本工程 ZYX 约定
//     (dlx_math.hpp: R = Rz·Ry·Rx, x 前 / y 左 / z 上) 下 quatToEuler 给出 **pitch = +atan(a/g)**
//     (注意: 这个约定里 +pitch 是**低头**), roll ≈ 0。
//   - 世界系 +y 方向加速度指令 ⇒ **roll = −atan(a/g)** (机体向左倾), pitch ≈ 0。
//   - 期望姿态的偏航 = 0 (只给倾转); FlightController 内部做倾转分离并继承当前偏航, 不控偏航。
//   - 期望推力方向(世界系) = normalize(ax, ay, g), 恒有 z 分量 > 0 ⇒ 不会出现 180° 退化。
//   - 倾角大小 = atan2(|a_xy|, g) [rad], 与 "1/cos(倾角)" 的集合油门补偿用的是同一个倾角。
//
//---------------------------------------------------------------------------------------------------
// 二、"为什么不是这样"
//---------------------------------------------------------------------------------------------------
// ① 为什么输出**期望姿态**而不是力矩:
//    §6.1 的裁决。力矩输出会绕过已经整定好的角度/角速度环: 调速度环参数就会回头改掉姿态手感,
//    而且 HOVER_THROTTLE 已经通过混控比例影响姿态环等效增益一次了, 不该再引入第二次耦合。
// ② 为什么用 ẑ_b_des = normalize(ax, ay, g) 而不是直接把加速度除个系数当倾角给:
//    水平加速度与倾角是 tan 关系, 不是线性关系(10° 时 tan 已经比角度大 2%); 直接给角度还要自己
//    管 roll/pitch 的正负号与组合, 而 quatFromTwoVectors 一次就把"任意方向的倾转"做对了。
// ③ 为什么加速度限幅夹**矢量模长**而不是每轴:
//    每轴独立夹会让对角方向拿到 √2 倍权限(1.5 → 2.12 m/s²), 倾角也就跟着超限。
// ④ 为什么倾角限幅是"精确"夹(缩放矢量)而不是"乘个 0.9 留余量":
//    留余量的软夹会让实际权限随目标大小漂移, 无法验收; 等比缩放不改变指令方向, 只改大小,
//    且缩放后直接把 tilt_cmd 记成 tilt_max —— 于是"限幅值"是可以逐位核对的。
// ⑤ 为什么 acc_max 先于 tilt_max (§6.2):
//    交付配置下 acc_max=1.5 m/s² ⇒ atan(1.5/g)=0.1518 rad (8.70°), 比 tilt_max=0.17 rad (9.74°) **更紧**
//    ⇒ 稳态大指令时真正咬合的是加速度限幅; tilt_max 是外层硬网(只有 acc_max 被调大、或命令来路不同
//    时才会咬合)。这条必须写清楚, 否则"10° 限幅"是句假话。
// ⑥ 为什么光流失效时输出**水平**姿态, 而不是保持上一次的姿态:
//    没有速度观测时速度误差没有意义(它衡量的是"估计值离目标多远", 而估计值已经不可信), 继续按旧
//    误差压着倾角 = 盲飞。输出水平 = "不动", 且状态机在 kVelFlowTimeoutS 之后应当已经降级 ALT_HOLD
//    (§6.2/§9), 所以这个窗口很短。**不清积分**(风阻配平是有用的历史), **也不继续积分**(没有观测的
//    积分就是发散) —— 这一条有单测。
// ⑦ 为什么 dt ≤ 0 时"跳过积分"而不是像 FlightController 那样换一个默认周期:
//    FlightController::sanitizeDt 的 0.002s 是个没有配置来源的魔数, 而且它会把一次调度异常
//    **伪装成一次正常周期**。速度环里唯一依赖 dt 的是 I 项和 D 项, 跳过它们、只按 P 输出,
//    既不会除零也不会发散, 而且异常在 debug.dt_valid 里看得见。
// ⑧ 为什么集合油门用**指令**倾角而不是实测倾角:
//    §6.2 —— 前馈。等实测倾角跟上来时高度已经掉了一截; 指令倾角这一周期就算得出来。
// ⑨ 为什么集合油门做成独立纯函数:
//    tan/cos 的边界(接近 90°)、NaN、inf 都能单独测; 而且混控饱和逻辑不该和它纠缠在一起。
// ⑩ 为什么不实现"mix_scale < 1"那个冻结条件:
//    mix_scale 是 mixMotors 内部按(可用推力/期望力矩)算出来的, 本控制器看不到。替代办法是给
//    update() 传 mixer_saturated(默认 false), 由混控侧回填 —— 冻结逻辑本身是同一套。
//
//---------------------------------------------------------------------------------------------------
// 三、接线契约 (给 main/状态机的人)
//---------------------------------------------------------------------------------------------------
//   - 进入 VEL_HOLD 时调 reset() (方案 §6.2: 进入模式时清零积分); 退出时可不管。
//   - 每控制周期:
//         dlx::Quaternion q_sp = vc.update(vx_sp, vy_sp, vx_world_est, vy_world_est, dt,
//                                          height_m, flow_valid, tick_ms);
//         sp.attitude = q_sp;                       // FlightControlSetpoint::attitude
//         out.throttle = vc.debug().throttle_cmd;   // 集合油门(§6.2: hover/cos(tilt_cmd) + 飞行员偏置)
//         fc.updateAngle(sp, state, dt, out);       // 复用现成的角度/角速度环
//   - flow_valid = 光流估计器 OpticalFlowEstimate::valid (§6.3) —— 不能传"有数据"而要传"有效"。
//   - 高度也要有效: height_m ≤ 0 或非有限一律按"光流不可用"处理(速度量纲本来就依赖高度)。
//   - tick_ms 用同一个时间基(与光流样本的时间基一致), 用于第二层超时兜底(§6.5)。
//===================================================================================================

namespace dlx
{
    /** 标准重力加速度 [m/s²]。物理常数, 不是可调参数 —— cfg 里没有这一项, 见交付说明"缺的配置项"。
     *  取值与 flight_controller.hpp 内部的 kGravity 一致(高度环也用同一个值)。 */
    constexpr float kVelGravityMss = 9.80665f;

    /** cos 下限(纯数值保护, 不是可调参数): 保证 1/cos ≤ 1000 且能把 NaN 吃掉。
     *  正常路径永远走不到这里 —— tilt_comp_max_rad ≤ kTiltMaxRad(0.35) ⇒ cos ≥ 0.939。 */
    constexpr float kVelThrottleCosFloor = 1.0e-3f;

    /** NaN/±inf 判定(与 optical_flow_estimator.hpp 里的 flowIsFinite 同源, 各自独立以免耦合)。 */
    inline bool velIsFinite(float v)
    {
        return (v == v) && (v <= FLT_MAX) && (v >= -FLT_MAX);
    }

    //================================================================================================
    // VEL_HOLD 集合油门 —— 纯函数 (§6.2: throttle = HOVER_THROTTLE / cos(tilt_cmd) + 飞行员偏置)
    //   倾角越大, 推力的垂直分量越小, 所以必须补 1/cos; 用**指令**倾角(前馈), 见"为什么不是这样 ⑧"。
    //   @param tilt_cmd_rad       本周期的期望倾角 [rad](取绝对值)
    //   @param hover_throttle     cfg::control::kHoverThrottle
    //   @param throttle_min/max   cfg::control::kThrottleMin / kThrottleMax(输出夹在这里)
    //   @param tilt_comp_max_rad  补偿角上限(cfg::control::kTiltMaxRad): 超过它按它算,
    //                             保证 cos 不会接近 0 ⇒ 不会出现 1/0 = inf
    //   @param pilot_bias         飞行员油门偏置(§6.2 的"+ 飞行员偏置"), 默认 0
    //   @return 归一化油门, 保证是有限数且落在 [throttle_min, throttle_max]
    //================================================================================================
    inline float velHoldCollectiveThrottle(float tilt_cmd_rad, float hover_throttle, float throttle_min,
                                           float throttle_max, float tilt_comp_max_rad,
                                           float pilot_bias = 0.0f)
    {
        // 输入兜底: 任何一个非有限都退回安全值 —— 这个函数的输出会被直接写进 out.throttle,
        // 一个 NaN 就能让四个电机全停, 所以它必须"吞掉"坏输入而不是把坏输入传下去。
        if (!velIsFinite(tilt_cmd_rad)) {
            tilt_cmd_rad = 0.0f;
        }
        if (!velIsFinite(hover_throttle)) {
            hover_throttle = 0.0f;
        }
        if (!velIsFinite(pilot_bias)) {
            pilot_bias = 0.0f;
        }
        if (!velIsFinite(throttle_min)) {
            throttle_min = 0.0f;
        }
        if (!velIsFinite(throttle_max) || throttle_max < throttle_min) {
            throttle_max = throttle_min;
        }
        if (!velIsFinite(tilt_comp_max_rad) || !(tilt_comp_max_rad > 0.0f)) {
            tilt_comp_max_rad = 0.0f; // 补偿角上限非法 ⇒ 不补偿(等于按 0° 算), 宁可少补也不出 inf
        }

        const float tilt = constrain(fabsf(tilt_cmd_rad), 0.0f, tilt_comp_max_rad);
        const float cos_tilt = max(cosf(tilt), kVelThrottleCosFloor); // max 同时吃掉 cos 的 NaN
        const float throttle = hover_throttle / cos_tilt + pilot_bias;
        return constrain(throttle, throttle_min, throttle_max);
    }

    //================================================================================================
    // 参数 POD (由调用方从 cfg 填; 故意不给"看起来合理"的默认值 —— 见交付说明)
    //   kp ← kVelXyP   ki ← kVelXyI   kd ← kVelXyD   int_limit ← kVelXyIntLimit
    //   vel_max_mps ← kVelXyMaxMps      acc_max_mss ← kVelAccMaxMss
    //   tilt_max_rad ← kVelTiltMaxRad   flow_timeout_s ← kVelFlowTimeoutS
    //   hover_throttle ← kHoverThrottle throttle_min/max ← kThrottleMin/kThrottleMax
    //   tilt_comp_max_rad ← kTiltMaxRad (集合油门补偿角上限)
    // 全 0 参数的退化行为: 增益 0 ⇒ 输出水平姿态; 不会发散、不会出 NaN —— 但仍然不算"可用参数",
    // paramsSane() 会报 false, 状态机应当据此拒绝进入 VEL_HOLD。
    //================================================================================================
    struct VelocityControllerParams
    {
        float kp{0.0f};               // [1/s]    速度误差 -> 加速度
        float ki{0.0f};               // [1/s²]   速度误差积分 -> 加速度
        float kd{0.0f};               // [1/s]    速度误差微分 -> 加速度 (cfg 默认 0, 不开)
        float int_limit{0.0f};        // [m/s]    积分限幅(积分状态本身的上限)
        float vel_max_mps{0.0f};      // [m/s]    目标速度限幅
        float acc_max_mss{0.0f};      // [m/s²]   水平加速度指令幅值限幅(先于倾角限幅)
        float tilt_max_rad{0.0f};     // [rad]    期望倾角硬限幅
        float flow_timeout_s{0.0f};   // [s]      无有效光流多久算"过期"(第二层兜底)
        float hover_throttle{0.0f};   // [-]      悬停油门
        float throttle_min{0.0f};     // [-]      油门下限
        float throttle_max{0.0f};     // [-]      油门上限
        float tilt_comp_max_rad{0.0f}; // [rad]   集合油门补偿角上限
    };

    //================================================================================================
    // 最近一次 update 的中间量 (只进日志/调试, 不占链路带宽, 不参与控制)
    //================================================================================================
    struct VelocityControllerDebug
    {
        // 输入
        float target_vx_mps{0.0f};
        float target_vy_mps{0.0f};
        float current_vx_mps{0.0f};
        float current_vy_mps{0.0f};
        float dt_s{0.0f};
        float height_m{0.0f};
        uint32_t tick_ms{0};
        // 误差与 PID 分项 [m/s²]
        float err_vx_mps{0.0f};
        float err_vy_mps{0.0f};
        float p_x_mss{0.0f};
        float p_y_mss{0.0f};
        float i_x_mss{0.0f};
        float i_y_mss{0.0f};
        float d_x_mss{0.0f};
        float d_y_mss{0.0f};
        float integral_x_mps{0.0f}; // 积分状态本身(未乘 ki), 用来核对限幅与冻结
        float integral_y_mps{0.0f};
        // 限幅后输出 [m/s²] / 期望推力方向(世界系)
        float acc_x_mss{0.0f};
        float acc_y_mss{0.0f};
        Vector3f z_body_des_world; // 期望机体 z 轴(世界系, 单位矢量, z > 0)
        float tilt_cmd_rad{0.0f};  // 期望倾角 = atan2(|a_xy|, g)
        float throttle_cmd{0.0f};  // VEL_HOLD 集合油门(不含状态机/飞行员偏置)
        // 标志位(位域留给日志打包, 这里先用 bool 便于单测)
        bool acc_limited{false};       // 加速度幅值限幅咬合
        bool tilt_limited{false};      // 倾角限幅咬合
        bool integral_frozen{false};   // 本周期**两轴**积分都没有累加(dt 非法 / 光流失效 / 抗饱和跳过)
        bool integral_x_frozen{false}; // x 轴本周期没有累加积分(抗饱和时只有一个轴被跳过是正常的)
        bool flow_ok{false};           // 本周期有有效速度观测
        bool flow_aged{false};         // 距上次有效观测已超过 flow_timeout_s(状态机据此降级)
        bool dt_valid{false};          // dt 合法(>0 且有限)
        bool mixer_saturated{false};   // 调用方回填的混控饱和标志
        bool params_sane{false};       // 参数自洽
        bool acc_sanitized{false};     // 非有限兜底触发过(说明上游有坏数, 应该去查)
    };

    //================================================================================================
    // 速度控制器
    //================================================================================================
    class VelocityController
    {
    public:
        VelocityController() = default;

        explicit VelocityController(const VelocityControllerParams &params)
            : _params(params)
        {
        }

        VelocityControllerParams &params()
        {
            return _params;
        }

        const VelocityControllerParams &params() const
        {
            return _params;
        }

        // 清零积分/微分状态 + 清空调试信息(进入 VEL_HOLD 前调用, §6.2)
        void reset()
        {
            _int_x = 0.0f;
            _int_y = 0.0f;
            _err_prev_x = 0.0f;
            _err_prev_y = 0.0f;
            _last_flow_ok_ms = 0u;
            _attitude = quatIdentity();
            _debug = VelocityControllerDebug{};
        }

        /**
         * 跑一个速度环周期, 返回**期望姿态**(可直接写进 FlightControlSetpoint::attitude)。
         * @param target_vx,target_vy   期望世界系水平速度 [m/s]
         * @param current_vx,current_vy 当前世界系水平速度估计 [m/s](光流经 EKF 后的世界系值)
         * @param dt_s                  控制周期 [s](≤0 或非有限 ⇒ 本周期不积分/不微分)
         * @param height_m              当前对地高度 [m](≤0 或非有限 ⇒ 按光流不可用处理)
         * @param flow_valid            本周期是否有有效光流速度观测(OpticalFlowEstimate::valid)
         * @param tick_ms               本周期时刻 [ms](与光流样本同一时间基)
         * @param mixer_saturated       混控饱和(§6.2 的 mix_scale < 1), 由混控侧回填, 默认 false
         */
        Quaternion update(float target_vx, float target_vy, float current_vx, float current_vy, float dt_s,
                          float height_m, bool flow_valid, uint32_t tick_ms,
                          bool mixer_saturated = false)
        {
            VelocityControllerDebug dbg;
            const bool params_sane = paramsSane();
            const bool dt_valid = velIsFinite(dt_s) && (dt_s > 0.0f);

            // ---- 输入归一化: 坏数一律当成 0, 不让 NaN 扩散 ----
            const float vx_sp = clampTarget(target_vx);
            const float vy_sp = clampTarget(target_vy);
            const float vx = velIsFinite(current_vx) ? current_vx : 0.0f;
            const float vy = velIsFinite(current_vy) ? current_vy : 0.0f;
            const bool height_ok = velIsFinite(height_m) && (height_m > 0.0f);

            // ---- 光流可用性: ① 本轮有效 ② 高度有效 ③ 距上次有效观测未超时 ----
            if (flow_valid && height_ok) {
                _last_flow_ok_ms = tick_ms;
            }
            const uint32_t age_ms = tick_ms - _last_flow_ok_ms;
            const bool flow_aged = age_ms > timeoutMs();
            const bool flow_ok = flow_valid && height_ok;

            const float err_x = vx_sp - vx;
            const float err_y = vy_sp - vy;

            dbg.target_vx_mps = vx_sp;
            dbg.target_vy_mps = vy_sp;
            dbg.current_vx_mps = vx;
            dbg.current_vy_mps = vy;
            dbg.dt_s = dt_s;
            dbg.height_m = height_m;
            dbg.tick_ms = tick_ms;
            dbg.err_vx_mps = err_x;
            dbg.err_vy_mps = err_y;
            dbg.integral_x_mps = _int_x;
            dbg.integral_y_mps = _int_y;
            dbg.flow_ok = flow_ok;
            dbg.flow_aged = flow_aged;
            dbg.dt_valid = dt_valid;
            dbg.mixer_saturated = mixer_saturated;
            dbg.params_sane = params_sane;

            if (!flow_ok) {
                // 失效: 冻结积分(不动、不清), 输出水平姿态。见"为什么不是这样 ⑥"。
                dbg.integral_frozen = true;
                dbg.integral_x_frozen = true;
                dbg.z_body_des_world = Vector3f{0.0f, 0.0f, 1.0f};
                dbg.tilt_cmd_rad = 0.0f;
                dbg.throttle_cmd = velHoldCollectiveThrottle(0.0f, _params.hover_throttle, _params.throttle_min,
                                                             _params.throttle_max, _params.tilt_comp_max_rad);
                // 记录本轮误差: 恢复后不要因为跨过失效窗口的一次差分产生微分尖峰
                _err_prev_x = err_x;
                _err_prev_y = err_y;
                _debug = dbg;
                _attitude = quatIdentity();
                return _attitude;
            }

            // ---- PID (世界系) ----
            const float p_x = _params.kp * err_x;
            const float p_y = _params.kp * err_y;
            const float i_x = _params.ki * _int_x;
            const float i_y = _params.ki * _int_y;
            float d_x = 0.0f;
            float d_y = 0.0f;
            if (dt_valid && _params.kd != 0.0f) {
                // D 项不做低通: cfg 里 kd = 0.0 (方案 §6.2 "不开——微分光流噪声没有意义"),
                // 加滤波器就得引入一个配置里没有的时间常数, 那是个没人验证过的参数。
                d_x = _params.kd * (err_x - _err_prev_x) / dt_s;
                d_y = _params.kd * (err_y - _err_prev_y) / dt_s;
            }

            float ax = p_x + i_x + d_x;
            float ay = p_y + i_y + d_y;

            // 非有限兜底: 无论如何都不把 NaN/inf 送进 atan2/quatFromTwoVectors (见单测 B10)
            if (!velIsFinite(ax) || !velIsFinite(ay)) {
                ax = 0.0f;
                ay = 0.0f;
                dbg.acc_sanitized = true;
            }

            // ---- 限幅 ①: 加速度幅值 (先于倾角限幅, §6.2) ----
            const float acc_max = (velIsFinite(_params.acc_max_mss) && _params.acc_max_mss > 0.0f)
                                      ? _params.acc_max_mss
                                      : 0.0f;
            const float tilt_max = (velIsFinite(_params.tilt_max_rad) && _params.tilt_max_rad > 0.0f)
                                       ? _params.tilt_max_rad
                                       : 0.0f;
            float a_mag = sqrtf(ax * ax + ay * ay);
            if (a_mag > acc_max && a_mag > 0.0f) {
                const float scale = acc_max / a_mag;
                ax *= scale;
                ay *= scale;
                a_mag = acc_max;
                dbg.acc_limited = true;
            }

            // ---- 限幅 ②: 倾角硬限幅 (等比缩放, 精确夹到 tilt_max) ----
            float tilt_cmd = atan2f(a_mag, kVelGravityMss);
            if (tilt_cmd > tilt_max && a_mag > 0.0f) {
                const float a_lim = kVelGravityMss * tanf(tilt_max);
                const float scale = a_lim / a_mag;
                ax *= scale;
                ay *= scale;
                a_mag = a_lim;
                tilt_cmd = tilt_max;
                dbg.tilt_limited = true;
            }

            // ---- 期望推力方向 -> 期望姿态 (世界系四元数, 无偏航) ----
            const Vector3f z_des = normalized(Vector3f{ax, ay, kVelGravityMss});
            _attitude = quatFromTwoVectors(Vector3f{0.0f, 0.0f, 1.0f}, z_des);

            // ---- 条件积分(照抄 computeHeightThrottle 的"同方向饱和就跳过"): 抗积分饱和 ----
            const bool saturated = dbg.acc_limited || dbg.tilt_limited || mixer_saturated;
            bool skip_x = false;
            bool skip_y = false;
            if (dt_valid) {
                if (saturated) {
                    // 只有"误差方向与已经饱和的方向相反"(能把指令拉回来)的分量才继续积分
                    skip_x = !(err_x * ax <= 0.0f);
                    skip_y = !(err_y * ay <= 0.0f);
                }
                const float int_limit =
                    (velIsFinite(_params.int_limit) && _params.int_limit > 0.0f) ? _params.int_limit : 0.0f;
                if (!skip_x) {
                    _int_x += err_x * dt_s;
                }
                if (!skip_y) {
                    _int_y += err_y * dt_s;
                }
                _int_x = constrain(_int_x, -int_limit, int_limit);
                _int_y = constrain(_int_y, -int_limit, int_limit);
            }
            dbg.integral_frozen = !dt_valid || (skip_x && skip_y);
            dbg.integral_x_frozen = !dt_valid || skip_x;

            _err_prev_x = err_x;
            _err_prev_y = err_y;

            // ---- 调试量 + 集合油门 ----
            dbg.p_x_mss = p_x;
            dbg.p_y_mss = p_y;
            dbg.i_x_mss = i_x;
            dbg.i_y_mss = i_y;
            dbg.d_x_mss = d_x;
            dbg.d_y_mss = d_y;
            dbg.integral_x_mps = _int_x;
            dbg.integral_y_mps = _int_y;
            dbg.acc_x_mss = ax;
            dbg.acc_y_mss = ay;
            dbg.z_body_des_world = z_des;
            dbg.tilt_cmd_rad = tilt_cmd;
            dbg.throttle_cmd = velHoldCollectiveThrottle(tilt_cmd, _params.hover_throttle, _params.throttle_min,
                                                         _params.throttle_max, _params.tilt_comp_max_rad);
            _debug = dbg;
            return _attitude;
        }

        /** 最近一次 update 的中间量(调参/日志用) */
        const VelocityControllerDebug &debug() const
        {
            return _debug;
        }

        /** 最近一次输出的期望姿态 */
        const Quaternion &lastAttitude() const
        {
            return _attitude;
        }

        /** 积分状态(调试/单测用, 单位 m/s) */
        float integralX() const
        {
            return _int_x;
        }

        float integralY() const
        {
            return _int_y;
        }

        /**
         * 参数是否自洽(用于"能不能进 VEL_HOLD")。全 0 的未填参数 ⇒ false。
         * 注意: 参数不合理时 update() 仍然安全(退化成水平姿态), 但状态机不该放行。
         */
        bool paramsSane() const
        {
            const VelocityControllerParams &p = _params;
            if (!velIsFinite(p.kp) || !velIsFinite(p.ki) || !velIsFinite(p.kd)) {
                return false;
            }
            if (!velIsFinite(p.int_limit) || !(p.int_limit > 0.0f)) {
                return false;
            }
            if (!velIsFinite(p.vel_max_mps) || !(p.vel_max_mps > 0.0f)) {
                return false;
            }
            if (!velIsFinite(p.acc_max_mss) || !(p.acc_max_mss > 0.0f)) {
                return false;
            }
            if (!velIsFinite(p.tilt_max_rad) || !(p.tilt_max_rad > 0.0f)) {
                return false;
            }
            if (!velIsFinite(p.flow_timeout_s) || !(p.flow_timeout_s > 0.0f)) {
                return false;
            }
            if (!velIsFinite(p.hover_throttle) || !(p.hover_throttle > 0.0f)) {
                return false;
            }
            if (!velIsFinite(p.throttle_min) || !velIsFinite(p.throttle_max) || !(p.throttle_max > p.throttle_min)) {
                return false;
            }
            if (!(p.hover_throttle <= p.throttle_max)) {
                return false;
            }
            if (!velIsFinite(p.tilt_comp_max_rad) || !(p.tilt_comp_max_rad > 0.0f)) {
                return false;
            }
            if (!(p.kp > 0.0f) && !(p.ki > 0.0f)) {
                return false; // 两个增益全 0: 输出恒为水平, 不算"可用参数"
            }
            return true;
        }

    private:
        // 目标速度限幅(非有限 ⇒ 0): §6.1 的 v_sp = 杆量 × kVelXyMaxMps 由调用方算,
        // 这里再夹一次是第二层保护(调用方算错时不会把飞机按 20 m/s 的目标压下去)。
        float clampTarget(float v) const
        {
            if (!velIsFinite(v)) {
                return 0.0f;
            }
            const float lim = (velIsFinite(_params.vel_max_mps) && _params.vel_max_mps > 0.0f)
                                  ? _params.vel_max_mps
                                  : 0.0f;
            return constrain(v, -lim, lim);
        }

        uint32_t timeoutMs() const
        {
            const float to_s = _params.flow_timeout_s;
            // 上限 1e6 s 只是"别让 float->uint32 的转换越界"的数值保护, 不是可调参数
            if (!velIsFinite(to_s) || !(to_s > 0.0f) || to_s > 1.0e6f) {
                return 0u;
            }
            return static_cast<uint32_t>(to_s * 1000.0f);
        }

        VelocityControllerParams _params;
        VelocityControllerDebug _debug;
        Quaternion _attitude{{1.0f, 0.0f, 0.0f, 0.0f}};
        float _int_x{0.0f};     // 速度误差积分 [m/s]
        float _int_y{0.0f};
        float _err_prev_x{0.0f}; // 上一周期误差(D 项)
        float _err_prev_y{0.0f};
        uint32_t _last_flow_ok_ms{0u};
    };

} // namespace dlx

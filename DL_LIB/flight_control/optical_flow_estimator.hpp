#pragma once
#include <float.h>
#include <math.h>
#include <stdint.h>
#include "../dlx_math.hpp"

//===================================================================================================
// optical_flow_estimator.hpp —— PMW3901 光流 → 机体水平速度估计 (纯逻辑层, 无驱动/无 RTOS/无硬件)
//
// 规格: docs/2026-10-02_upgrade_plan_v3.md §6.3(光流→EKF 链路) / §6.6(新文件)。
// 分工: 本文件负责 §6.3 链路的**前半段**(burst → 质量门限 → 轴重映射/符号 → Δθ → ω → 去掉机体旋转
//       → 尺度换算 → LPF → 有效性), 输出**机体系**水平速度; 后半段(v_world = R(q_ekf)·v_body →
//       g_ekf.updateHorizontalVelocity)由 EKF 侧接线完成, 见文件末"接线契约"。
//       参数全部由调用方从 cfg::sensor::flow::* 填(本文件不 include 配置/驱动/profile 头)。
//
//---------------------------------------------------------------------------------------------------
// 一、单位与坐标系约定 (Lead 与 EKF/B6 接线时按这一段对齐, 不要凭感觉)
//---------------------------------------------------------------------------------------------------
// 机体坐标系 (与 dlx_math.hpp / flight_control_params.hpp 一致):
//     x = 机头前, y = 左, z = 上, 右手系。
// 世界坐标系: 与 MadgwickAHRS / EKF 四元数同一个世界系 (四元数是"机体系→世界系", 即 v_world = R(q)·v_body)。
//
// 数据链, 一步一个单位:
//     ① 输入 dx_count / dy_count = PMW3901 Motion_Burst(0x16) 的 dX_L/dX_H, dY_L/dY_H,
//        **int16 二进制补码**原始 counts (§4.3), 本接口用 float 只为免去接线处的转换。
//     ② 器件坐标符号:   ω_cx = sign_x · rad_per_count · dx_count / dt_s      [rad/s]
//                       ω_cy = sign_y · rad_per_count · dy_count / dt_s      [rad/s]
//        rad_per_count = kRadPerCount [rad/count] (≈409.3 counts/rad, 必须标定);
//        dt_s = **两次读取之间的实测间隔** [s], 不是控制周期(§6.3)。
//     ③ 装角: ω_body = Rz(yaw_deg) · ω_chip, yaw_deg = 光流模块相对机体绕 +z 的偏转角
//        (cfg::sensor::flow::kYawDeg)。右手系、从上往下看逆时针为正:
//            ω_bx =  cosψ·ω_cx − sinψ·ω_cy
//            ω_by =  sinψ·ω_cx + cosψ·ω_cy
//     ④ 去掉机体旋转(§6.3 必须做的一步): ω_trans = ω_body − ω_gyro   [rad/s]
//        (ω_gyro = 同周期的机体角速度, 由调用方从 IMU 传入; 不传 = 传 0 时, 转弯/滚转期间的速度
//         估计是错的, 见"为什么不是这样 ③")。
//     ⑤ 角速率 → 线速度: 光流测的是**角速率**, 线速度 = 角速率 × 高度:
//            v_body_x = ω_trans_x · h        [m/s]
//            v_body_y = ω_trans_y · h        [m/s]
//            v_body_z = 0  (光流给不出垂向速度, 恒 0; 不是"没算", 是物理上没有)
//        h = 离地高度(z_agl) [m], 来自测距 (§6.3: 必须配对地用高度源)。
//     ⑥ 一阶低通 (τ = kLpfTauS): α = dt_s/(τ+dt_s), y += α·(x−y)  —— 在**前端**做, 不在 EKF 输出上做。
//     ⑦ 限幅/拒绝: 角速率 ±kMaxRateRads 饱和(器件规格); 速度幅值 > kMaxVelMps **拒绝**该样本。
//
// 与方案 §6.3 的一处写法差异 (必须知道, 因为它决定前飞会不会被算成侧移):
//     §6.3 写 v_body = ( ω_trans.y·h , −ω_trans.x·h , 0 )  —— PX4 口径, 含一次 90° 轴交叉;
//     本文件按任务书 A 的公式逐轴同号: v_body = ( ω_trans.x·h , ω_trans.y·h , 0 )。
//     两者只差一个绕 +z 的 −90° 旋转, 因此**不需要改代码**: 把 kYawDeg 设成 90(或 270, 取决于器件
//     原始符号) 就等价于 §6.3 的交叉写法。逐轴写法被选中的理由:
//       ① 与 cfg 文档里 kSignX 的字面定义 "v_x = −ω_x·h" 一致(§6.3 的交叉式里 v_x 其实来自 ω_y);
//       ② 符号只管镜像/反相, 装角只管装歪 —— 两件事分离, 台架标定时才能一次只改一个;
//       ③ 交叉写法会让"装歪 90°"和"器件反相"在代码里互相污染, 标定方程无解。
//     ⇒ 台架标定判据(必做, 见下): 悬停 1m, 手推机体**沿机头方向**匀速平移, 看 estimate().velocity_body_mps.x
//       是否 > 0 且大小 ≈ 真实速度。不对就按下表改配置, 永远不改代码。
//
// (yaw_deg, sign_x, sign_y) → 输出轴向真值表 (dx=+1, dy=0, h>0, 已收敛):
//     yaw=0   & sign_x=+1 : v = (+k·h/dt_s, 0)         yaw=0   & sign_x=−1 : v = (−k·h/dt_s, 0)
//     yaw=90  & sign_x=+1 : v = (0, +k·h/dt_s)         yaw=90  & sign_x=−1 : v = (0, −k·h/dt_s)
//     yaw=180 & sign_x=+1 : v = (−k·h/dt_s, 0)         yaw=180 & sign_x=−1 : v = (+k·h/dt_s, 0)
//     yaw=270 & sign_x=+1 : v = (0, −k·h/dt_s)         yaw=270 & sign_x=−1 : v = (0, +k·h/dt_s)
//   dy=+1 同理作用在"另一个轴"上: yaw=0/180 时进 v_y, yaw=90/270 时进 v_x (符号同理由 sign_y 定)。
//   现象 → 改哪个配置: 前飞算成侧移(轴向错) → 改 kYawDeg ±90; 单个轴反号(镜像/反相) → 改对应 kSign。
//
//---------------------------------------------------------------------------------------------------
// 二、"为什么不是这样"
//---------------------------------------------------------------------------------------------------
// ① 为什么失效时速度置 0 而不是保持上一次的值:
//    保持旧值 = 下游拿到一个"看起来完全合理"的近似零速(悬停时光流本来就读 ~0), 连门限都拦不住 ——
//    它会被当成一次真实的"没有动"的观测(进 EKF 就是一次量测, 进速度环就是"误差为零")。
//    置 0 + valid=false 把"这一轮没有观测"变成一个**必须显式检查的状态位**, 明确失败 > 静默正确。
//    (方案 §6.3 写"永不填 0"指的是"永远不要把 0 当成量测喂进 EKF" —— 本文件把它写成契约:
//      调用方**只有 valid==true 时**才允许把 velocity_body_mps 交给 EKF/速度环。)
// ② 为什么高度是必需的(不是"精度问题"):
//    光流给的是角速率; 线速度 = 角速率 × 高度。同一个 counts 在 1m 和 3m 高度下对应的速度是 1:3。
//    高度错了不是"差一点", 是速度量纲整个错。所以高度超出 [min_h, max_h]、或高度无效 ⇒ 直接 invalid。
// ③ 为什么必须减掉机体旋转:
//    2 rad/s 滚转在 1m 高度下会换算出 2 m/s 的**假平移速度** —— 比 ±kMaxVelMps(3.0) 预算的一半还多,
//    而且它长得完全像一个正常样本。不做这一步, 只要有角运动速度环就在追一个不存在的速度。
// ④ 为什么 LPF 在前端、不滤 EKF 的输出:
//    §6.3/kLpfTauS 的依据: 滤观测只改变观测本身的噪声特性; 滤 EKF 输出会破坏协方差一致性
//    (P 与实际误差不再匹配), 是"看起来平滑但估计发散"的经典来源。
// ⑤ 为什么不先滤 counts 再换算:
//    counts 是量化后的整数(12bit 有效), 先滤会把量化误差变成**相关**噪声; 而且速度 ∝ 高度,
//    "在速度上低通"才是物理量的低通 —— 高度变化时两种做法的传递函数不一样。
// ⑥ 为什么角速率"夹"而速度"拒":
//    kMaxRateRads 是**器件量程**(超出即饱和), 夹住是描述事实; 而换算出的速度超 kMaxVelMps 说明
//    "高度源或 counts 至少有一个是错的"(§4.3: 观测可以略大于控制上限, 但不接受明显离谱的值),
//    这种样本没有正确的值可以夹, 只能拒绝。
// ⑦ 为什么任何门限失败都清低通状态:
//    输出说"速度 0"、滤波器里却留着一个非零历史, 恢复后的头几个样本会带上一段不存在的"过去",
//    输出与内部状态必须一致。代价是一次坏样本会让本轮估计归零 —— 方向是安全的(速度环冻结)。
// ⑧ 为什么 param 不合理时整体 invalid 而不是"用 0 继续算":
//    忘了填参数(全 0)会得到"速度恒为 0 且 valid=true" —— 这是最坏的结果(喂给 EKF 一个假的静止)。
//    ⇒ 宁可整条链路显式失效。
//
//---------------------------------------------------------------------------------------------------
// 三、接线契约 (给 main/EKF 接线的人)
//---------------------------------------------------------------------------------------------------
//   - 100Hz 光流线程里: est = estimator.feed(dx, dy, squal, dt_flow, height_agl_m, tick_ms, gx, gy);
//   - if (est.valid) { v_world = quatRotate(q_ekf, est.velocity_body_mps);  // §6.3
//                      g_ekf.updateHorizontalVelocity(v_world, sigma_v, gate); }
//     else            { /* 本周期不给 EKF 任何水平速度量测 */ }
//   - 健康位(§7.1 health.flow): estimator.healthy() (连续拒绝 ≥ kFailLimit ⇒ 0)。
//   - 控制端还有第二层超时(kVelFlowTimeoutS), 见 velocity_controller.hpp —— 前后端各兜一层(§6.5)。
//===================================================================================================

namespace dlx
{
    /** π/180: 度→弧度。数学常数, 不是可调参数(配置项 kYawDeg 的单位是度)。 */
    constexpr float kFlowDegToRad = 0.017453292519943295f;

    /** NaN/±inf 判定: 输入与输出门限检查都靠它, 保证本文件任何路径都不会把非有限值传下去。 */
    inline bool flowIsFinite(float v)
    {
        // 不直接用 isfinite(): 它是 C99 宏, 在 C++/不同工具链下可能是宏也可能是函数。
        // v==v 捕 NaN; 与 ±FLT_MAX 比较捕 ±inf —— 两者在 IEEE-754 下都是严格定义的行为。
        return (v == v) && (v <= FLT_MAX) && (v >= -FLT_MAX);
    }

    /** 样本被拒绝的原因 (只进日志/调试, 不参与控制) */
    enum class FlowReject : uint8_t
    {
        None = 0,
        ParamsInvalid, // 参数不可用(未填/非有限/自相矛盾): 整条链路显式失效
        NotFinite,     // 输入里出现 NaN/inf
        BadDt,         // dt ≤ 0 或 dt > timeout(样本间隔异常/过期)
        LowSqual,      // squal < min_squal
        DeltaGate,     // |delta| > max_delta 或 ≥ sat_delta (SPI 垃圾/饱和)
        HeightRange,   // 高度不在 [min_h, max_h]
        VelOutOfRange, // 换算速度幅值 > max_vel
    };

    //================================================================================================
    // 参数 (全部由调用方从 cfg::sensor::flow::* 填; 这里是 POD, 故意**不给默认值**)
    //   rad_per_count ← kRadPerCount    lp_tau ← kLpfTauS       max_rate ← kMaxRateRads
    //   max_vel       ← kMaxVelMps      min_squal ← kMinSqual   sign_x/sign_y ← kSignX/kSignY
    //   yaw_deg       ← kYawDeg         min_h/max_h ← kMinHeightM/kMaxHeightM
    //   timeout       ← kTimeoutS       max_delta ← kMaxDelta   sat_delta ← kSatDelta
    //   fail_limit    ← kFailLimit
    // 为什么不给默认值: 给了默认值, 接线时忘记填就会静默拿到"一套看起来合理的参数";
    //   零初始化会让 paramsSane() 失败 ⇒ 显式 invalid, 比静默跑一套猜的参数安全。
    //================================================================================================
    struct OpticalFlowEstimatorParams
    {
        float rad_per_count{0.0f}; // [rad/count] 尺度, 决定速度量纲, 必须标定
        float lp_tau{0.0f};        // [s]   速度低通时间常数
        float max_rate{0.0f};      // [rad/s] 器件角速率量程, 超出饱和
        float max_vel{0.0f};       // [m/s] 换算速度上限, 超出拒绝该样本
        float min_h{0.0f};         // [m]   光流可用高度下限(低于即 invalid)
        float max_h{0.0f};         // [m]   光流可用高度上限(高于即 invalid)
        float timeout{0.0f};       // [s]   样本间隔上限, 超过视为过期样本
        float max_delta{0.0f};     // [count] 单帧位移门限(0 = 关闭该门限)
        float sat_delta{0.0f};     // [count] 饱和门限(0 = 关闭该门限)
        int8_t sign_x{0};          // 器件 x 轴符号(±1)
        int8_t sign_y{0};          // 器件 y 轴符号(±1)
        int32_t yaw_deg{0};        // [deg] 模块相对机体绕 +z 的装角(0/90/180/270)
        uint8_t min_squal{0};      // SQUAL 质量门限
        uint8_t fail_limit{0};     // 连续拒绝多少次算"光流不健康"(0 = 关闭)
    };

    //================================================================================================
    // 一次估计的结果 (POD, 只读回传; 调用方**必须**先看 valid 再看速度)
    //================================================================================================
    struct OpticalFlowEstimate
    {
        // --- 原始输入回传(日志用) ---
        float dx_count{0.0f};   // [count] 本次 burst 的原始 dX
        float dy_count{0.0f};   // [count] 本次 burst 的原始 dY
        uint8_t squal{0};       // 器件 SQUAL
        float dt_s{0.0f};       // [s] 本次样本的实测间隔
        uint32_t seq{0};        // 序号(每次 feed +1, 含被拒绝的)
        uint32_t tick_ms{0};    // 本次 feed 的时刻
        // --- 结果 ---
        bool valid{false};              // 唯一许可位: false ⇒ 下面三个字段全部无意义(且速度为 0)
        FlowReject reject{FlowReject::None}; // valid==false 时的原因
        Vector3f velocity_body_mps;     // [m/s] 机体系水平速度(x 前, y 左, z 恒 0), 已低通
        float omega_body_x_rads{0.0f};  // [rad/s] 已去机体旋转、已限幅的机体系角速率(调试用)
        float omega_body_y_rads{0.0f};
    };

    //================================================================================================
    // 光流估计器
    //================================================================================================
    class OpticalFlowEstimator
    {
    public:
        OpticalFlowEstimator()
        {
            reset();
        }

        explicit OpticalFlowEstimator(const OpticalFlowEstimatorParams &params)
            : _params(params)
        {
            reset();
        }

        OpticalFlowEstimatorParams &params()
        {
            return _params;
        }

        const OpticalFlowEstimatorParams &params() const
        {
            return _params;
        }

        // 清空全部内部状态(解锁前 / 光流线程重启 / 高度源切换时调用)
        void reset()
        {
            _estimate = OpticalFlowEstimate{};
            _lp_vx = 0.0f;
            _lp_vy = 0.0f;
            _seq = 0u;
            _drop_streak = 0u;
        }

        /** 不带陀螺前馈的入口: 等价于 gyro = 0 (只在机体确实没有角运动时才对, 见"为什么不是这样 ③") */
        const OpticalFlowEstimate &feed(float dx_count, float dy_count, uint8_t squal, float dt_s,
                                       float height_m, uint32_t tick_ms)
        {
            return feed(dx_count, dy_count, squal, dt_s, height_m, tick_ms, 0.0f, 0.0f);
        }

        /**
         * 喂一个光流样本。
         * @param dx_count,dy_count PMW3901 原始 counts (int16 补码, 这里用 float 免转换)
         * @param squal             器件 SQUAL
         * @param dt_s              两次读取之间的实测间隔 [s]
         * @param height_m          当前对地高度(z_agl) [m] —— 没有它换不出线速度
         * @param tick_ms           本样本时刻 [ms]
         * @param gyro_x/y_rads     同周期机体系角速度 [rad/s], 用于去掉机体旋转(§6.3)
         * @return 内部估计结果(引用, 无拷贝): 只有 valid==true 时才允许下游使用速度
         */
        const OpticalFlowEstimate &feed(float dx_count, float dy_count, uint8_t squal, float dt_s,
                                       float height_m, uint32_t tick_ms, float gyro_x_rads,
                                       float gyro_y_rads)
        {
            // 每轮先整体清空: 任何一条拒绝路径天然就是 valid=false + 速度 0 + 拒绝原因, 不会漏字段。
            _estimate = OpticalFlowEstimate{};
            _estimate.dx_count = dx_count;
            _estimate.dy_count = dy_count;
            _estimate.squal = squal;
            _estimate.dt_s = dt_s;
            _estimate.seq = ++_seq;
            _estimate.tick_ms = tick_ms;

            const OpticalFlowEstimatorParams &p = _params;

            FlowReject reject = FlowReject::None;
            if (!paramsSane(p)) {
                reject = FlowReject::ParamsInvalid;
            } else if (!flowIsFinite(dx_count) || !flowIsFinite(dy_count) || !flowIsFinite(dt_s) ||
                       !flowIsFinite(height_m) || !flowIsFinite(gyro_x_rads) || !flowIsFinite(gyro_y_rads)) {
                reject = FlowReject::NotFinite;
            } else if (!(dt_s > 0.0f) || dt_s > p.timeout) {
                // dt ≤ 0: 除零保护; dt > timeout: 样本已经过期(§4.3 的 kTimeoutS, 第一层)
                reject = FlowReject::BadDt;
            } else if (squal < p.min_squal) {
                reject = FlowReject::LowSqual;
            } else if (deltaGateTripped(p, dx_count, dy_count)) {
                reject = FlowReject::DeltaGate;
            } else if (!(height_m >= p.min_h) || !(height_m <= p.max_h)) {
                reject = FlowReject::HeightRange;
            }

            if (reject != FlowReject::None) {
                rejectSample(reject);
                return _estimate;
            }

            // ② 器件坐标 → 角速率
            const float omega_cx = static_cast<float>(p.sign_x) * p.rad_per_count * dx_count / dt_s;
            const float omega_cy = static_cast<float>(p.sign_y) * p.rad_per_count * dy_count / dt_s;

            // ③ 装角: ω_body = Rz(yaw)·ω_chip。每次现算 cos/sin: 100Hz 下开销可忽略,
            //    缓存它反而会在调用方改了 params().yaw_deg 之后静默用旧值。
            const float yaw_rad = static_cast<float>(p.yaw_deg) * kFlowDegToRad;
            const float c = cosf(yaw_rad);
            const float s = sinf(yaw_rad);

            // ④ 去掉机体旋转 (ω_trans = ω_flow,body − ω_gyro,body)
            float omega_bx = (c * omega_cx - s * omega_cy) - gyro_x_rads;
            float omega_by = (s * omega_cx + c * omega_cy) - gyro_y_rads;

            // ⑤' 角速率限幅: 器件量程饱和(夹, 不是拒绝)
            omega_bx = constrain(omega_bx, -p.max_rate, p.max_rate);
            omega_by = constrain(omega_by, -p.max_rate, p.max_rate);

            // ⑤ 角速率 → 线速度(逐轴同号, 见文件头"与 §6.3 的写法差异")
            const float vx_raw = omega_bx * height_m;
            const float vy_raw = omega_by * height_m;

            // ⑦ 速度幅值门限: 拒绝而不是夹(见"为什么不是这样 ⑥")
            if (sqrtf(vx_raw * vx_raw + vy_raw * vy_raw) > p.max_vel) {
                rejectSample(FlowReject::VelOutOfRange);
                return _estimate;
            }

            // ⑥ 一阶低通(在前端做, 见"为什么不是这样 ④")
            const float alpha = dt_s / (p.lp_tau + dt_s);
            _lp_vx += alpha * (vx_raw - _lp_vx);
            _lp_vy += alpha * (vy_raw - _lp_vy);

            _estimate.velocity_body_mps.x() = _lp_vx;
            _estimate.velocity_body_mps.y() = _lp_vy;
            _estimate.velocity_body_mps.z() = 0.0f;
            _estimate.omega_body_x_rads = omega_bx;
            _estimate.omega_body_y_rads = omega_by;
            _estimate.reject = FlowReject::None;
            _estimate.valid = true;
            _drop_streak = 0u;
            return _estimate;
        }

        /** 最近一次估计结果(未 feed 过时 valid=false) */
        const OpticalFlowEstimate &estimate() const
        {
            return _estimate;
        }

        /** 连续被拒绝的样本数(一次有效样本清零) */
        uint32_t dropStreak() const
        {
            return _drop_streak;
        }

        /** 健康位(§7.1 health.flow): 连续拒绝 ≥ fail_limit ⇒ false。fail_limit==0 表示不判健康。 */
        bool healthy() const
        {
            if (_params.fail_limit == 0u) {
                return true;
            }
            return _drop_streak < static_cast<uint32_t>(_params.fail_limit);
        }

    private:
        // 参数自洽性: 任何一个不合理都不算"可用参数"(见"为什么不是这样 ⑧")
        static bool paramsSane(const OpticalFlowEstimatorParams &p)
        {
            if (!flowIsFinite(p.rad_per_count) || !(p.rad_per_count > 0.0f)) {
                return false;
            }
            if (!flowIsFinite(p.lp_tau) || !(p.lp_tau > 0.0f)) {
                return false;
            }
            if (!flowIsFinite(p.max_rate) || !(p.max_rate > 0.0f)) {
                return false;
            }
            if (!flowIsFinite(p.max_vel) || !(p.max_vel > 0.0f)) {
                return false;
            }
            if (!flowIsFinite(p.min_h) || !flowIsFinite(p.max_h) || !(p.max_h > p.min_h)) {
                return false;
            }
            if (!flowIsFinite(p.timeout) || !(p.timeout > 0.0f)) {
                return false;
            }
            if (p.sign_x != 1 && p.sign_x != -1) {
                return false;
            }
            if (p.sign_y != 1 && p.sign_y != -1) {
                return false;
            }
            return true;
        }

        static bool deltaGateTripped(const OpticalFlowEstimatorParams &p, float dx, float dy)
        {
            const float ax = fabsf(dx);
            const float ay = fabsf(dy);
            if (p.max_delta > 0.0f && (ax > p.max_delta || ay > p.max_delta)) {
                return true; // §4.3: 一帧跳 240 counts 物理上不可能, 是 SPI 垃圾
            }
            if (p.sat_delta > 0.0f && (ax >= p.sat_delta || ay >= p.sat_delta)) {
                return true; // 接近饱和的样本没有线性度, 宁可丢弃
            }
            return false;
        }

        // 拒绝路径只有这一条, 保证"输出与内部状态一致"(见"为什么不是这样 ⑦")
        void rejectSample(FlowReject reason)
        {
            _lp_vx = 0.0f;
            _lp_vy = 0.0f;
            _estimate.velocity_body_mps = Vector3f{};
            _estimate.omega_body_x_rads = 0.0f;
            _estimate.omega_body_y_rads = 0.0f;
            _estimate.valid = false;
            _estimate.reject = reason;
            if (_drop_streak != 0xFFFFFFFFu) {
                ++_drop_streak;
            }
        }

        OpticalFlowEstimatorParams _params;
        OpticalFlowEstimate _estimate;
        float _lp_vx{0.0f}; // 低通状态 [m/s]
        float _lp_vy{0.0f};
        uint32_t _seq{0u};
        uint32_t _drop_streak{0u};
    };

} // namespace dlx

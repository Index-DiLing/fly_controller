#pragma once
//===================================================================================================
// DL_LIB/GNSS/dlx_geo.hpp —— WGS84 大地坐标(φ,λ,h) → 以"起飞点"为原点的局部切平面 ENU [m]
//                            header-only, 只做坐标学: 不碰 IO / 配置 / 线程 / 动态分配
//
// 为什么单独一个文件(dlx_gnss_nmea.hpp 的文件头明确写了它**不做**这件事):
//   ① 原点(起飞点)是**状态**, 解析器不该持有它(dlx_gnss_nmea.hpp 文件头 101~103 行原话: "不做 ENU
//      转换: 那需要起飞点原点 + 参考纬度这两个本文件拿不到的状态");
//   ② 位置/高度 §4.4 只回传不进控制, 但回传、日志、地理围栏、返航都需要"以起飞点为原点的东/北/天 [m]";
//   ③ 这件事有它自己的一组坑(float 量化、椭球高 vs 海拔高、帧间跳变), 混进解析器里没人会去核对。
//   本文件只依赖 <stdint.h>/<cmath>, 因此在 PC 上能原样跑单测(test/geo_check), 不需要硬件桩。
//
//===================================================================================================
// ① 投影选型: ECEF 差分(严格切线平面), **不用**等距圆柱近似
//===================================================================================================
//   做法: p = ECEF(φ,λ,h);  ENU = R(φ0,λ0)ᵀ·(p - p0)。R 的三行就是原点处的 东/北/天 单位矢量:
//       east  =                        -sinλ0·dx +               cosλ0·dy
//       north =  -sinφ0·cosλ0·dx - sinφ0·sinλ0·dy + cosφ0·dz
//       up    =   cosφ0·cosλ0·dx + cosφ0·sinλ0·dy + sinφ0·dz
//   为什么不选等距圆柱(east = N(φ0)cos(φ0)·Δλ, north = M(φ0)·Δφ, up = h-h0; 只要 2 次三角函数、
//   代码短一半): 1 km 范围内它的误差是 **mm~13.5 cm 级**, 而 ECEF 差分是 **μm 级**。数字如下
//   (由 test/geo_check/tools/gen_expected.py 第 ④ 节算出: 目标点先用 ECEF 逆变换放到"切平面上正好
//   1000 m 正东"处, 再分别用两种投影正算 ⇒ 期望 ENU 恰为 (1000,0,0), 差异全是投影引入的):
//
//     φ0      ECEF 差分 (e,n,u) [m]           等距圆柱 (n,u) [m]        Δnorth      Δup       Δeast
//     0°   (1000.000000, 0.000000, 0.000000)  ( 0.000000, 0.078392)    0.000 mm  +78.4 mm  -6.83 mm
//     45°  (1000.000000, 0.000000, 0.000000)  (-0.078260, 0.078261)  -78.3 mm   +78.3 mm  -6.83 mm
//     60°  (1000.000000, 0.000000, 0.000000)  (-0.135437, 0.078195) -135.4 mm   +78.2 mm  -6.84 mm
//
//   三项误差各自的来源(都能手算复核, 不是"实测个大概"):
//     * Δup = d²/(2a) = 1000²/(2×6378137) = 78.393 mm —— 切平面的 up 天然含地球曲率的"下沉"项,
//       等距圆柱完全没有这一项。注意这**不是**谁算错了, 而是"沿椭球面走的弧长"与"在起飞点水平面上
//       量的直角坐标"两种定义的差; 飞控要的是后者(机体姿态/加速度计/水平仪定义的那个水平面)。
//       ⚠ 后果: 沿等 MSL 高度平飞 1 km, 本函数的 up 会读到 -7.8 cm。这不是漂移, 是定义。
//     * Δnorth ≈ -(d²/(2a))·tanφ0 —— 45°/60° 实测比值 135.4/78.3 = 1.73 = tan60°/tan45° ✓。
//       来源: 切平面的东向单位矢量 (-sinλ0, cosλ0, 0) 的 z 分量恒为 0, 沿它走 d 米后目标点 z 不变,
//       而到自转轴的距离增加 √(ρ0²+d²)-ρ0 ≈ d²/(2ρ0), 于是大地纬度必然变小, 而等距圆柱直接把这个
//       纬度差当成 north。
//     * Δeast = -d·h0/N0 = -1000 × 43.5/6378137 = -6.82 mm —— 等距圆柱用 N(φ0)cos(φ0)·Δλ, 但目标
//       点实际在半径 (N+h)·cosφ 的平行圈上, 差一个 (1 + h/N) 因子。
//   ⇒ 1 km 上等距圆柱最坏 13.5 cm(north@60°), 而且是**随距离平方长**的: 5 km 时 up 差 1.96 m, 10 km
//     时 7.84 m(见 gen_expected.py 第 ④ 节)。飞控里气压计噪声才 ±0.1 m、速度观测 sigma 是 0.3 m/s,
//     没理由白送 13.5 cm 的系统差。ECEF 差分每帧多 4 次 sin/cos + 1 次 sqrt, 5 Hz 下连 0.1% CPU 都
//     不到; 这颗 MCU 有 FPU(虽然是单精度, 见 ②), 不必为省三角函数牺牲正确性 ⇒ **选 ECEF 差分**。
//
//   ECEF 差分自己的精度(1 km 内, 三项分开看):
//     * 数值(double)误差: ECEF 坐标 6.4e6 量级, double 的 ulp = 2^(22-52) = 9.3e-10 m, 两点相减
//       后误差 ~1e-9 m。gen_expected.py 用 60 位 Decimal 独立复算, 与本实现的差 < 1e-9 m。
//     * 与"沿椭球面弧长"的定义差: s³/(6a²) = 1000³/(6×6378137²) = 4.1e-6 m; 1113.2 m 处
//       5.65e-6 m —— 与脚本第 ① 节独立算出的残差 5.652e-06 m 逐位吻合(这不是巧合, 就是同一项)。
//     * 切线平面是**线性化**, 只在原点附近有意义: 114.6 km(北京→天津)处, 切平面弦长与椭球面测地线
//       已经差 6.09 m(脚本第 ③ 节)。本模块按"起飞点几百米~几公里"(多旋翼作业半径)设计。
//
//===================================================================================================
// ② 精度: 全程 double, 只在最后一步落 float —— 为什么
//===================================================================================================
//   出参是 float(与 DL_GPS 遥测帧 / 控制环的浮点习惯一致), 但**中间量一律 double**, 只在 return 前
//   落一次 float。量化数字(题面要求的口径: 按 2^-23 相对精度算, 即把 binary32 在 x 处的 ulp 记作
//   x·2^-23 = x·1.192093e-07):
//     * 经度量级 1e2: 116.4 × 2^-23 = 1.388e-05 °。1° 经度 = 111320·cosφ m ⇒
//         赤道 **1.54 m** / 45°N **1.09 m** / 60°N **0.77 m**。
//       (紧界: binary32 是 round-to-nearest, 实际误差 ≤ 半个 ulp = x·2^-24 ⇒ 0.77/0.55/0.39 m。
//        gen_expected.py 第 ⑤(b) 节用 struct 真量化出来的例子: lonDeg 116.39723 → 2.06e-06 ° →
//        45°N 上 **0.162 m**; latE7 = 1163972300(整数形式) → 5.2e+01 个 1e-7° → **0.409 m**。)
//     * 纬度: 1° = 110574 m, latDeg 39.9075 落 float 丢 1.22e-06 ° ⇒ **0.135 m**。
//     * 反过来把**米**落 float: 1 km 处 ulp = 2^(9-23) = 6.10e-05 m, 半 ulp 3.05e-05 m。
//     * 把 **ECEF 坐标**(6.4e6 ∈ [2²²,2²³))落 float: ulp = 2^(22-23) = **0.5 m**! 半 ulp 0.25 m/轴
//       ⇒ 3D 最坏 0.25·√3 = 0.43 m。
//   ⇒ 角度早早落 float = 白送 0.1~1.5 m 的位置误差(与 GPS 自身 2~3 m 噪声同阶, 但一分好处都没有);
//     米落 float = 3e-05 m。所以 double 进 double 出。单测第 ⑤ 节把这条界钉住了: 1 km 处的输出必须
//     落在 ±6e-05 m 内, 而"提前落 float"的反例会差 800 倍以上。
//     ⚠ 代价说清楚: F407 **没有**双精度 FPU, double 的 sin/cos/sqrt 走软件库, 每次约 1~3 μs。本函数
//     每帧 8 次三角函数 + 2 次 sqrt ⇒ 约 10~25 μs。5 Hz 更新率下占 0.05% CPU 以内。这是拿 CPU 换
//     正确性, 划算; 如果将来更新率提高到 50 Hz 再回头量这一项。
//
//===================================================================================================
// ③ 椭球高 vs 海拔高 —— **显式声明忽略**, 代价推导如下
//===================================================================================================
//   GGA 给的是"海拔(MSL, 大地水准面之上)", 而 ECEF 公式要的是"椭球高", 两者差大地水准面差距 N:
//   h_椭球 = H_MSL + N, 中国境内 N 大致 -10 ~ +30 m。
//   **本文件的选择**: 原点与目标点**都直接用 GGA 的 MSL 海拔**(把 MSL 当椭球高喂进公式)。
//   为什么不修正 —— 忽略的代价可以精确推, 不需要猜:
//     h 在 ECEF 公式里只以 (曲率半径 + h) 的线性形式出现 ⇒ 拿 H 代替 h = H+N, 得到的点恰好等于真点
//     沿该点椭球法线(up)下移 N:  P(φ,λ,H) = P(φ,λ,h) - N·n̂(φ,λ),  n̂ = (cosφcosλ, cosφsinλ, sinφ)。
//     于是 ENU 误差 = R0ᵀ(误差_t - 误差_0) = -(N_t - N_0)·(0,0,1)ᵀ - N_t·R0ᵀ(n̂_t - n̂_0)   (旋转不改变模)
//       * **up 轴**: 误差 = -(N_t - N_0) —— 只跟起飞点到目标点的 N **差值**有关, N 的绝对值(几米~
//         三十米)在差分里**自动抵消**。1 km 内大地水准面梯度取极端 1e-4(≈1 m/10 km)⇒ |ΔN| ≤ 0.1 m;
//         典型梯度(≤1e-5) ⇒ ≤0.01 m。⚠ 梯度量级是估计, 我手上没有 EGM96 网格可查(见"不确定的点")。
//       * **水平两轴**: 误差模长 = N_t·|n̂_t - n̂_0| ≈ N_t·(d/a) = 30 × 1000/6.378e6 = **4.7 mm**。
//     即 1 km 内最多 0.1 m 的 up 偏差 + 5 mm 的水平偏差。对照 GPS 自身的垂直噪声(5~10 m = 水平
//     2~3 m 的 2~3 倍)与气压计 ±0.1 m ⇒ 可以忽略。
//   为什么不去弄一张真的 N 表: EGM96 的 15′ 网格是几百 KB~MB 级数据 + 双线性插值, 换来的只是
//   ≤0.1 m 的 up 改善, 这颗 MCU 的 Flash/RAM 要留给控制律和日志。
//   ⚠ 必须记住的后果: 本函数的 up 是**相对起飞点**的量, 不能当绝对海拔用。要绝对高度请用
//   GnssSolution::altMslMm(MSL, mm)或气压计。
//   ⚠ 换 RTK(可能直接给椭球高)或换到 N 梯度大的区域时, 这一条必须重新评估。
//
//===================================================================================================
// ④ 帧间跳变拒绝(防多路径 / 定位重置)
//===================================================================================================
//   物理依据: 相邻两帧的真实位移 ≤ v_max / f_update。30 m/s @ 5 Hz = 6 m; 5 m/s @ 1 Hz = 5 m;
//   15 m/s @ 10 Hz = 1.5 m(gen_expected.py 第 ⑥ 节把这张表打全了)。门限取"最大速度/更新率 × 安全
//   系数"。
//   为什么门限是**参数**而不是 cfg:: 常量: 它是"机型 + 更新率"的知识, 不是坐标学常数; 而且
//   dlx_gnss_nmea.hpp 已经演示过硬绑 cfg:: 要拖进 flight_config_gen.hpp 再配一整套 #if 兜底。本文件
//   保持零依赖, 阈值由调用方**一处**给 ⇒ 改阈值只改一个地方, 单测也能构造各种门限做反例。
//   判据细节:
//     * 与**上一帧被接受**的位置比(不是上一帧原始位置 —— 否则连续两帧同向的多路径偏差会一步一步把
//       参考点拖走);
//     * 用 3D 距离而不是只看水平(高度跳变同样是坏数据);
//     * 第一帧永远接受, 它只负责立参考点;
//     * **被拒的帧不更新参考点, 也不改出参**(出参清零, 与失败一致)。
//   maxStepM <= 0 或非有限 ⇒ 门限关闭(直通)。这是本文件**唯一**的 fail-open 分支: 配置里写 0 的语义
//   必须是"不限制"而不是"拒绝一切"(后者会让整条链路哑掉); 单测第 ⑥ 节用它做反例对照。
//   ⚠ 连续被拒 != 永久锁死: 位置只要回到参考点附近就立刻重新被接受; 但如果是接收机**真的**换了位置
//     (重新定位), 后面每一帧都会离参考点很远 ⇒ 会一直拒。这时该由**调用方**决定怎么办
//     (geoJumpGateReset() 重新立参考点, 或连原点一起重设)。本模块不自动解锁 —— "连续拒 N 次之后
//     就信它"会把真正长时间的多路径偏差也放进来, 而 GNSS 位置在这套系统里**不进控制**(§4.4 只回传),
//     冻结的代价是遥测看着不动(可见、可诊断), 不是炸机。
//
// ---- 与 gnssGroundSpeedToEnu()(dlx_gnss_nmea.hpp)的关系 ----
//   那个函数做"地速 + 真航向 → ENU 水平速度", 本文件的 trackSpeedToEnu() 是同一件事(定义逐字相同:
//   真北 0° 顺时针为正 ⇒ vE = v·sin, vN = v·cos), 供不想 include NMEA 解析器的地方使用; 单测把两者
//   对拍到 1e-6 m/s。
//===================================================================================================

#include <stdint.h>
#include <cmath>

namespace dlx
{
    //===============================================================================================
    // WGS84 椭球(定义值; 与 test/geo_check/tools/gen_expected.py 里的 Decimal 常数逐位一致)
    //===============================================================================================
    constexpr double kGeoWgs84A    = 6378137.0;      ///< 长半轴 a [m](WGS84 定义值, 精确)
    constexpr double kGeoWgs84InvF = 298.257223563;  ///< 扁率倒数 1/f(定义值, 精确)
    constexpr double kGeoWgs84F    = 1.0 / kGeoWgs84InvF;
    constexpr double kGeoWgs84E2   = 2.0 * kGeoWgs84F - kGeoWgs84F * kGeoWgs84F; ///< e² = 2f - f²
    constexpr double kGeoWgs84B    = kGeoWgs84A * (1.0 - kGeoWgs84F);            ///< 短半轴 b [m]
    /** π/180。为什么自己写 34 位常数而不是 M_PI/180: M_PI 不是标准 C++(要 _USE_MATH_DEFINES),
     *  而两次除法(π/180)会比一次乘法多一次舍入 —— 常数只有一份, 直接写死最省事也最准。 */
    constexpr double kGeoDegToRad  = 0.0174532925199432957692369076848861271;

    //===============================================================================================
    // GeoOrigin —— 局部切平面的原点(就是"起飞点"), 由调用方在首次定位有效时钉一次
    //===============================================================================================
    /** ⚠ 这是 POD, **故意不给默认初始化**: `GeoOrigin o;` 里 o.set 是未定义的垃圾。请写
     *  `GeoOrigin o = {};`(→ set=false)或 `GeoOrigin o = makeOrigin(...)`。
     *  为什么不给 `bool set = false;` 这种成员初值(C++11 支持 NSDMI): 加了它这个结构在 C++11 里就
     *  不再是聚合类型, `GeoOrigin o = {false, 1.0, 2.0, 3.0};` 会编译不过 —— 而"默认有效"的原点
     *  (比如 (0,0))比未初始化更危险: 它会让"忘了设原点"变成一个悄悄算错几百万米的 bug, 而不是一个
     *  clean 的 valid=false。 */
    struct GeoOrigin
    {
        bool   set;     ///< true = 已经钉过原点; false = 未设置 ⇒ 一切转换返回 false + 出参清零
        double latDeg;  ///< 纬度 [度](南纬负)
        double lonDeg;  ///< 经度 [度](西经负)
        double altM;    ///< 高度 [m](语义见文件头 ③: 存的是 GGA 的 MSL 海拔)
    };

    /** 构造一个"已设置"的原点。参数越界(NaN / |lat|>90 / |lon|>180 / alt 非有限)时返回 set=false 的
     *  空原点 —— 为什么在入口就拦: 越界的原点会让后面每一帧都算出一个"看着合理"的假位置(把 116.397
     *  写成 1163.97 只是把点搬到 1000 km 外, 不会崩), 这里返回 false 能让调用方当场发现。
     *  ⚠ 允许 (0,0): "不许拿 (0,0) 当原点"指的是**不许在未设置时默认用 (0,0)**, 而不是禁止真的在
     *  (0°,0°) 起飞。 */
    inline GeoOrigin makeOrigin(double latDeg, double lonDeg, double altM)
    {
        GeoOrigin o;
        const bool okLat = (latDeg >= -90.0) && (latDeg <= 90.0);   // NaN 比较恒 false ⇒ 自动被拦
        const bool okLon = (lonDeg >= -180.0) && (lonDeg <= 180.0);
        const bool okAlt = std::isfinite(altM);
        o.set    = okLat && okLon && okAlt;
        o.latDeg = o.set ? latDeg : 0.0;
        o.lonDeg = o.set ? lonDeg : 0.0;
        o.altM   = o.set ? altM : 0.0;
        return o;
    }

    //===============================================================================================
    // 内部实现(不属于对外接口, 放在 detail 里表明"别的地方不要依赖它")
    //===============================================================================================
    namespace geo_detail
    {
        /** 大地坐标(φ,λ,h) → ECEF [m]。h 按**椭球高**用; 传 MSL 海拔进来时见文件头 ③。
         *  N(φ) = a/√(1-e²sin²φ) 是卯酉圈曲率半径(注意 z 里是 N(1-e²)+h, 不是 N+h)。 */
        inline void toEcef(double latDeg, double lonDeg, double altM,
                           double &x, double &y, double &z)
        {
            const double lat = latDeg * kGeoDegToRad;
            const double lon = lonDeg * kGeoDegToRad;
            const double sla = std::sin(lat), cla = std::cos(lat);
            const double slo = std::sin(lon), clo = std::cos(lon);
            const double n   = kGeoWgs84A / std::sqrt(1.0 - kGeoWgs84E2 * sla * sla);
            x = (n + altM) * cla * clo;
            y = (n + altM) * cla * slo;
            z = (n * (1.0 - kGeoWgs84E2) + altM) * sla;
        }
    } // namespace geo_detail

    //===============================================================================================
    // 对外接口 ①: 大地坐标 → ENU
    //===============================================================================================
    /** WGS84 经纬高 → 以 origin 为原点的 东/北/天 [m]。
     *
     *  @param latDeg  目标点纬度 [度](南纬负)
     *  @param lonDeg  目标点经度 [度](西经负)
     *  @param altM    目标点高度 [m](MSL 海拔, 见文件头 ③)
     *  @param o       原点(必须是 makeOrigin() 或已设置好的; o.set == false ⇒ 直接 false)
     *  @param eastM   出: 东向 [m](正 = 原点以东)
     *  @param northM  出: 北向 [m](正 = 原点以北)
     *  @param upM     出: 天向 [m](正 = 比原点高; 含 -d²/2R 的曲率项, 见文件头 ①)
     *  @return true = 出参有效; false = 原点未设置 / 输入越界或非有限 / 结果非有限
     *
     *  失败时把三个出参**清零**, 不保留上一次的值: 调用方忘了看返回值时, 一个"看起来正常的旧位置"
     *  比 0 危险得多(会被当成飞机还在那儿)。
     *  ⚠ 不抛异常、不分配内存、不做任何 IO; 可以从中断/控制环里直接调(纯函数, 除了 <cmath> 无状态)。 */
    inline bool geoToEnu(double latDeg, double lonDeg, double altM, const GeoOrigin &o,
                         float &eastM, float &northM, float &upM)
    {
        eastM  = 0.0f;
        northM = 0.0f;
        upM    = 0.0f;

        // ---- 门禁: 原点未设置(绝不拿 (0,0) 兜底)、原点手搓越界、目标点越界/NaN ----
        if (!o.set) { return false; }
        if (!std::isfinite(o.latDeg) || !std::isfinite(o.lonDeg) || !std::isfinite(o.altM)) { return false; }
        if ((o.latDeg < -90.0) || (o.latDeg > 90.0)) { return false; }
        if ((o.lonDeg < -180.0) || (o.lonDeg > 180.0)) { return false; }
        if (!std::isfinite(latDeg) || !std::isfinite(lonDeg) || !std::isfinite(altM)) { return false; }
        if ((latDeg < -90.0) || (latDeg > 90.0)) { return false; }
        if ((lonDeg < -180.0) || (lonDeg > 180.0)) { return false; }

        // ---- 投影(全程 double) ----
        double x0 = 0.0, y0 = 0.0, z0 = 0.0, x1 = 0.0, y1 = 0.0, z1 = 0.0;
        geo_detail::toEcef(o.latDeg, o.lonDeg, o.altM, x0, y0, z0);
        geo_detail::toEcef(latDeg, lonDeg, altM, x1, y1, z1);
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double dz = z1 - z0;

        const double la = o.latDeg * kGeoDegToRad;
        const double lo = o.lonDeg * kGeoDegToRad;
        const double sla = std::sin(la), cla = std::cos(la);
        const double slo = std::sin(lo), clo = std::cos(lo);

        const double e =                         -slo * dx +         clo * dy;
        const double n = -sla * clo * dx - sla * slo * dy + cla * dz;
        const double u =  cla * clo * dx + cla * slo * dy + sla * dz;

        if (!std::isfinite(e) || !std::isfinite(n) || !std::isfinite(u)) { return false; }

        eastM  = (float)e;   // ← 全程 double, 只在这里落一次 float(文件头 ②: 约 3e-05 m @1km)
        northM = (float)n;
        upM    = (float)u;
        return true;
    }

    //===============================================================================================
    // 对外接口 ②: 航向 + 对地速度 → ENU 水平速度
    //===============================================================================================
    /** @param trackDeg  真航向 [度](真北 0°, 顺时针为正; 取 GnssSolution::courseCdeg × 0.01f)
     *  @param speedMps  对地速度 [m/s](取 GnssSolution::gspeedCms × 0.01f, 非负)
     *  @param vEast     出: 东向速度 [m/s]
     *  @param vNorth    出: 北向速度 [m/s]
     *
     *  真北 = 0°、顺时针为正 ⇒ vE = v·sin(trk), vN = v·cos(trk)(与 dlx_gnss_nmea.hpp 的
     *  gnssGroundSpeedToEnu() 定义逐字相同, 单测对拍到 1e-6 m/s)。
     *  为什么不是 NED: EKF(flight_control_fliter.hpp)的 FF_VX/VY 是 ENU(世界系 z 向上); 给 NED 会
     *  同时错"轴交换"和"符号"两处(悬停不飘、一推杆往反方向加速), 是天上最难查的一类 bug。
     *  ⚠ 只有水平两轴, 没有竖直分量。塞进 3 轴 EKF 接口时 z 必须理解成"没有这个量测"(用只看水平的
     *    接口, 或把 z 的 sigma 放到无穷大), 不能填 0 当"垂速恒为 0"的强观测。
     *  ⚠ 这里用 double 只是为了不多引入一层 float 舍入(与参考实现对拍时干净); 真正决定精度的是输入
     *    trackDeg 本身的分辨率(0.01° = 1.7e-4 rad ⇒ 30 m/s 上 5 mm/s), 所以别指望这一层能救什么。 */
    inline void trackSpeedToEnu(float trackDeg, float speedMps, float &vEast, float &vNorth)
    {
        const double t = (double)trackDeg * kGeoDegToRad;
        const double v = (double)speedMps;
        vEast  = (float)(v * std::sin(t));
        vNorth = (float)(v * std::cos(t));
    }

    //===============================================================================================
    // 对外接口 ③: 从解析结果取原点(只在首次定位有效且质量达标时设置)
    //===============================================================================================
    /** 定位质量字段。与 dlx::GnssSolution 的同名字段**同名同型**(单位也一模一样), 但**不 include** 它。
     *  为什么要有这个结构: dlx_geo.hpp 的依赖方向必须是"坐标工具不知道 NMEA 存在"。反过来直接
     *  include dlx_gnss_nmea.hpp 会把 dlx_exception.h + generated/flight_config_gen.hpp 一起拖进来
     *  (主机单测里多两个依赖), 并且以后改解析器会牵动本文件。想直接喂 GnssSolution 的话, 用下面
     *  那个模板重载(setOriginFromFix(gnssSolution, origin, minSats, maxHdopX10))。 */
    struct GeoFix
    {
        int32_t latE7;     ///< 纬度 ×1e7 [度](南纬负)   ← GnssSolution::latE7
        int32_t lonE7;     ///< 经度 ×1e7 [度](西经负)   ← GnssSolution::lonE7
        int32_t altMslMm;  ///< 海拔(MSL)×1e3 [mm]       ← GnssSolution::altMslMm
        uint8_t fix;       ///< GGA 定位质量(0 = 没定位) ← GnssSolution::fix
        uint8_t sats;      ///< 参与解算卫星数           ← GnssSolution::sats
        uint8_t hdopX10;   ///< HDOP ×10(0 = 从没收到过) ← GnssSolution::hdopX10
        bool    posValid;  ///< 位置可用(含新鲜度)       ← GnssSolution::posValid
        bool    altValid;  ///< 海拔可用                 ← GnssSolution::altValid
        bool    rmcSeen;   ///< 收到过 RMC               ← GnssSolution::rmcSeen
        bool    rmcValid;  ///< 最后一条 RMC 状态是 'A'  ← GnssSolution::rmcValid
    };

    /** 从一份定位解里取原点: **只在首次定位有效且质量达标时**设置, 之后永不改动。
     *
     *  @param f           定位解(GeoFix, 或任何有这些同名字段的类型)
     *  @param o           被写入的原点; 只在本次真的钉上时才改
     *  @param minSats     卫星数下限(调用方给, 例: cfg::sensor::gnss::kMinSats)
     *  @param maxHdopX10  HDOP×10 上限(调用方给, 例: (uint16_t)(kMaxHdop*10.0f + 0.5f))
     *  @return true = **本次调用**把原点钉上了。false 有两种含义("还没达标"和"早就钉过了"),
     *          所以判断"原点到底设置没有"请看 o.set, 不要看返回值。
     *
     *  判据(全部 fail-closed, 与 gnssSpeedObservationUsable() 同一套精神):
     *    posValid && fix>=1 && sats>=minSats && hdopX10!=0 && hdopX10<=maxHdopX10
     *    && !(rmcSeen && !rmcValid) && altValid
     *  为什么阈值都是参数: 本文件不许硬编码 cfg::(见文件头 ④: 依赖 + 单测)。
     *  为什么要求 altValid: 原点高度定错 ⇒ up 轴整体偏一个常数(爬升率不受影响, 但绝对高度回传会错),
     *  而 GGA 的海拔与经纬度是同一句给的, 要求它并不会让原点更难钉上。
     *  为什么要 hdopX10 != 0: 0 是"从没收到过 HDOP"的哨兵(HDOP 物理上 ≥ 1.0), 收不到就 fail-closed,
     *  不能默认"几何很好"。 */
    inline bool setOriginFromFix(const GeoFix &f, GeoOrigin &o, uint8_t minSats, uint16_t maxHdopX10)
    {
        if (o.set) { return false; }                     // 只在首次: 之后再有"更好"的定位也不动原点
        if (!f.posValid) { return false; }               // 解析器说位置不可用(含过期)⇒ 不信
        if (f.fix < 1u) { return false; }                // 1 = kGnssFixGps(见 dlx_gnss_nmea.hpp)
        if (f.sats < minSats) { return false; }
        if (f.hdopX10 == 0u) { return false; }           // 没收到过 HDOP ⇒ fail-closed
        if ((uint16_t)f.hdopX10 > maxHdopX10) { return false; }
        if (f.rmcSeen && !f.rmcValid) { return false; }  // 接收机自己说这次的位置不算数
        if (!f.altValid) { return false; }

        const GeoOrigin cand = makeOrigin((double)f.latE7 * 1e-7,
                                          (double)f.lonE7 * 1e-7,
                                          (double)f.altMslMm * 1e-3);
        if (!cand.set) { return false; }                 // 越界的 latE7/lonE7 会在这里被拦(见 makeOrigin)
        o = cand;
        return true;
    }

    /** 直接从 dlx::GnssSolution(或任何有这些同名字段的类型)取原点 —— 鸭子类型, 本文件不需要
     *  include NMEA 解析器。GeoFix 本身会走上面那个非模板重载(精确匹配优先于模板)。
     *  ⚠ 字段名拼错/改名的代价是**编译错**, 不是静默走默认值 —— 这正是我们要的。 */
    template <typename FixLike>
    inline bool setOriginFromFix(const FixLike &f, GeoOrigin &o, uint8_t minSats, uint16_t maxHdopX10)
    {
        GeoFix g;
        g.latE7    = f.latE7;
        g.lonE7    = f.lonE7;
        g.altMslMm = f.altMslMm;
        g.fix      = f.fix;
        g.sats     = f.sats;
        g.hdopX10  = f.hdopX10;
        g.posValid = f.posValid;
        g.altValid = f.altValid;
        g.rmcSeen  = f.rmcSeen;
        g.rmcValid = f.rmcValid;
        return setOriginFromFix(g, o, minSats, maxHdopX10);
    }

    //===============================================================================================
    // 对外接口 ④: 带帧间跳变拒绝的 ENU 转换
    //===============================================================================================
    /** 跳变门限的状态机(每条 GNSS 链路一份)。纯数据, 默认构造即"空门限"。 */
    struct GeoJumpGate
    {
        bool  primed;    ///< 是否已有一帧被接受的观测(第一帧永远接受, 它只负责立参考点)
        bool  rejected;  ///< 最近一次 update 是否**因为跳变**被拒(诊断用, 不参与判据)
        float eastM;     ///< 最后一帧**被接受**的 ENU 位置 [m]
        float northM;
        float upM;
        float stepM;     ///< 最近一次 update 与参考点的 3D 距离 [m](诊断用)

        GeoJumpGate() : primed(false), rejected(false), eastM(0.0f), northM(0.0f), upM(0.0f), stepM(0.0f) {}
    };

    /** 重新立参考点(下一帧无条件被接受)。接收机重新定位/操作员确认位置可信之后由调用方调。 */
    inline void geoJumpGateReset(GeoJumpGate &g)
    {
        g.primed   = false;
        g.rejected = false;
        g.eastM    = 0.0f;
        g.northM   = 0.0f;
        g.upM      = 0.0f;
        g.stepM    = 0.0f;
    }

    /** 同 geoToEnu(), 但多一道帧间跳变门限。
     *
     *  @param maxStepM  门限 [m]: 与上一帧**被接受**位置的 3D 距离超过它就判本帧无效。
     *                   取"机型最大速度 / 更新率 × 安全系数"(例: 5 m/s @ 5 Hz ⇒ 1 m, 给 3~5 倍余量)。
     *                   **<= 0 或非有限 ⇒ 门限关闭(直通)**, 见文件头 ④。
     *  @param g         门限状态; 调用方持有, 上电时默认构造即可
     *  @return true = 本帧被采纳(出参有效, 参考点已更新); false = 未采纳(出参清零), 原因看:
     *            g.rejected == true  ⇒ 跳变被拒(位置离参考点太远)
     *            g.rejected == false ⇒ 前面 geoToEnu 的失败(原点没设/输入越界/非有限)
     *  ⚠ 被拒的那一帧**不更新参考点**: 否则连续两帧同向的多路径偏差会一步步把参考点拖走。
     *  ⚠ 对比用的是落 float 之后的位置(与调用方拿到的是同一个数), 量化误差在 1 km 处 3e-05 m、10 km 处
     *    5e-04 m, 对 1 m 量级的门限无关紧要。 */
    inline bool geoToEnuGated(double latDeg, double lonDeg, double altM, const GeoOrigin &o,
                              float maxStepM, GeoJumpGate &g,
                              float &eastM, float &northM, float &upM)
    {
        eastM  = 0.0f;
        northM = 0.0f;
        upM    = 0.0f;
        g.rejected = false;

        float e = 0.0f, n = 0.0f, u = 0.0f;
        if (!geoToEnu(latDeg, lonDeg, altM, o, e, n, u)) {
            g.stepM = 0.0f;                 // 没定位 ≠ 跳变: 不碰参考点, 也不置 rejected
            return false;
        }

        if (g.primed) {
            const double de = (double)e - (double)g.eastM;
            const double dn = (double)n - (double)g.northM;
            const double du = (double)u - (double)g.upM;
            const double step = std::sqrt(de * de + dn * dn + du * du);
            g.stepM = (float)step;
            const bool gateOn = std::isfinite(maxStepM) && (maxStepM > 0.0f);
            if (gateOn && (step > (double)maxStepM)) {
                g.rejected = true;          // 参考点与出参都不动
                return false;
            }
        } else {
            g.stepM = 0.0f;                 // 第一帧: 没有参照, 只立点
        }

        g.eastM  = e;
        g.northM = n;
        g.upM    = u;
        g.primed = true;
        eastM    = e;
        northM   = n;
        upM      = u;
        return true;
    }

} // namespace dlx

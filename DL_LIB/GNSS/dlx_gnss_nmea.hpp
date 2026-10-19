#pragma once
//===================================================================================================
// DL_LIB/GNSS/dlx_gnss_nmea.hpp —— NEO-7M(USART2 PD5/PD6) 的 NMEA 0183 增量解析器
//                               + GnssSolution POD(可以被控制环整体快照)
//
// 职责(只有这一件事):
//   把"不定长、会碎片到达的字节流"变成"一份可直接回传的定位解 + 一份可进 EKF 的水平速度 + 一个
//   '这份速度到底能不能信'的判据"。字节与时刻都由调用方给, 本文件**不碰** USART/DMA/中断/线程/日志,
//   因此它能在 PC 上原样跑单测(test/gnss_check), 不需要任何硬件桩。
//
// ---- 输入形态(为什么接口是"逐字节") ----
//   §4.4 的接收方式: **DMA1_Stream5_Ch4 循环缓冲 + USART2 IDLE 中断**。IDLE 一次给你的是"不止一句、
//   也不保证是整句"的一段字节(缓冲边界、丢字节、上电中点都会切开句子), 所以解析器必须是**纯增量**的:
//   喂 1 字节和喂 200 字节必须得到完全相同的结果(单测第 ③ 条就是钉这个)。
//   ⇒ feed(uint8_t, nowMs) / feed(const uint8_t*, len, nowMs) / feed(const char*, nowMs) 三个入口,
//     内部走同一条状态机。
//
// ---- 支持的语句(位开关 = cfg::sensor::gnss::kNmeaMask, 位值见下面 NmeaMaskBit) ----
//   语句      mask 位   提供什么                                    进 GnssSolution 的字段
//   $GxRMC    bit0=1    定位有效性(A/V)/经纬度/地速(节)/真航向      rmcValid, lat/lon, speed, course
//   $GxGGA    bit1=2    定位质量 fix/卫星数/HDOP/海拔(MSL)/经纬度   fix, sats, hdop, alt, lat/lon
//   $GxVTG    bit4=16   地速(节 / km/h / m/s) + 真航向(与 RMC 重复) speed, course
//   $GxGSA    bit2=4    定位类型(1/2/3) + PDOP/HDOP/VDOP            gsaFix, hdop
//   $GxGSV    bit3=8    **不实现**(只影响搜星调试, 运行期不需要)   ——
//   $GxGLL    bit5=32   **不实现**(与 RMC 完全重复)                ——
//   $GxZDA    bit6=64   **不实现**(不需要绝对时间戳)               ——
//   talker 前缀(GN/GP/GL/GA/GB/GQ)一律不看, 只认后 3 个字符的语句类型: 换模块/换固件(单 GPS 的 GP 与多系统
//   的 GN)不该改解析器; 反过来, 多认一个 talker 也不会把别的语句误认成 RMC(还要校验和与字段数兜底)。
//
// ---- NMEA mask 位定义(与 config/flight_config.schema.json → enums.NmeaMaskBits 逐位一致) ----
//   为什么"自己定一套位"是错的: kNmeaMask 是**配置项**给进来的数字(schema 已钉死 1/2/4/8/16/32/64),
//   代码里再排一套只会让"配置写 RMC=1 / 代码认 RMC=2"这种错永远对不上号, 而且没人会去核对第二份表。
//   本文件实现的位: RMC/GGA/VTG/GSA; 置了 GSV/GLL/ZDA 位**不会**报错, 但也不会解析(见上表)。
//
// ---- 有效性判据: "当前速度能不能进 EKF" (唯一真相 = gnssSpeedObservationUsable() 纯函数) ----
//   speedUsable == fix >= 1                    (GGA 定位质量; 0 = 没定位)
//               && (没收到过 RMC || RMC 状态 == 'A')   (§4.4 的"或 RMC 的 A": mask 关了 RMC 就不能要求它)
//               && sats >= cfg::sensor::gnss::kMinSats (默认 6)
//               && hdopX10 >= 1 && hdopX10 <= kMaxHdopX10 (默认 20 ⇒ HDOP <= 2.0; 0 = 没收到过 HDOP)
//               && ageMs <= cfg::sensor::gnss::kStaleMs (默认 500; age = 判据依赖量里**最旧**的那个的年龄)
//   外加一条前置条件(在类里, 不在纯函数里): **必须收到过速度句(RMC 或 VTG)**, 从没收到过就没有速度可谈。
//   ageMs 取"最旧的那个依赖量"而不是"速度句的年龄": fix/星数/HDOP 和速度是**不同语句**给的, 拿 5 秒前的
//   HDOP 去担保 100ms 前的速度是把两个不同时刻的东西拼在一起 —— 所以要一起过期。
//   为什么做成纯函数: 这条判据是"喂不喂 EKF"的唯一闸门, 它错了天上就会飘; 只有做成无状态的纯函数,
//   才能在 PC 上把 fix/星数/HDOP/新旧 的**全组合**穷举一遍(test/gnss_check 第 ⑨ 条)。
//
// ---- 静止 / 低质量时的行为(§4.4 的口径: 位置只回传, 只有水平速度进 EKF) ----
//   不可信时**绝不**输出"速度 = 0": EKF 会把 0 当成"真的没动"这个极强的观测, 比不喂观测糟得多
//   (同理 §6.3 光流那条链路的"永不填 0")。本文件的做法是:
//     - 数值字段**原样保留**(最后已知值), 用于回传/日志/排查;
//     - 有效性标志(speedValid / posValid / altValid / courseValid)置 false —— 调用方只许看标志,
//       看到 false 就**一次都不许**调 updateGpsVelocity/updateHorizontalVelocity。
//   NEO-7M 静止时给的是 0.0 节或几个 cm/s 的噪声, HDOP 大时速度根本不可信; 这两件事都由上面的判据挡住。
//
// ---- 单位换算表(全部换算只在本文件做一次, 调用方别抄第二份) ----
//   源(语句)          源单位            目标(GnssSolution)   公式(整数, 四舍五入)      举例
//   RMC 字段7 / VTG N 节  (1 位小数)     gspeedCms [cm/s]     (mknot*463 + 4500)/9000   022.4 kn → 1152
//   VTG K              km/h(1 位小数)    gspeedCms [cm/s]     (mkmh + 18)/36            010.2 km/h → 283
//   VTG 扩展 M         m/s (可选扩展)    gspeedCms [cm/s]     (mmps + 5)/10             2.83 m/s → 283
//   1 节 = 1852 m/h = 0.5144444… m/s; 1 km/h = 0.2777778… m/s ⇒ 换算成 cm/s 后 1 位小数源量的量化误差:
//   节 5.1 cm/s、km/h 2.8 cm/s、m/s 1 cm/s。**同一句 VTG 给了多个单位时取量化最细的那个**
//   (m/s > km/h > 节), 理由就是这三个数; 不取"字段靠后的赢", 否则结果会随固件的字段排列变。
//   经纬度 ddmm.mmmm → latE7/lonE7(度 ×1e7)  用 64 位整数算, 不经过 float:
//     1e-7 度 ≈ 1.1 cm, float 只有 24 位有效位(≈ ±16.7M 就满), 1e-7 度的经度(±1.8e9)直接溢出精度;
//     而 NMEA 本来就是 ddmm.mmmm 定点, 整除 + 四舍五入是精确的。
//   海拔 m → altMslMm [mm] ×1000; 航向 度 → courseCdeg [0.01°] ×100; HDOP → hdopX10 ×10(1 位小数)。
//
// ---- 组句规则 & "谁更新谁生效"(B10 要对齐的就是这一段) ----
//   ① 每一句只更新它**有权**更新的字段(见文件头的语句表); 字段写入的唯一入口是 put(), 规则是:
//        "时间戳(nowMs)更大的赢; 时间戳相同则按**字段权威顺序**赢"。
//      字段权威顺序(同刻 tie-break):
//        lat / lon : GGA > RMC      (同一接收机的同一解算, 二者应完全相等; 定个序只为了可判定)
//        hdop      : GSA > GGA      (schema 的 GSA 说明就是"DOP 用于 HDOP 门限判据")
//        speed/course: RMC > VTG    (RMC 是推荐最小定位信息, 通常必开)
//        alt / sats / fix : 只有 GGA 有权
//      为什么同刻不能用"后到的赢": 真实到达方式是"一次 IDLE 把整段突发喂进来", 这一批句子的 nowMs
//      **完全相同**; 那时谁赢就取决于模块固件的输出顺序(NEO-7M 默认 GGA→GLL→GSA→GSV→RMC→VTG),
//      换个固件版本、开关某一句, 结果就变了 —— 那就不可能写出一条稳定的单测断言, B10 也没法预期。
//      时间戳不同时**新的赢**(RMC 后到就能覆盖 GGA 的位置), 所以"权威顺序"只在同刻生效。
//   ② 空字段(`,,`)**不更新**该字段, 也不动它的时间戳 —— 空 ≠ 0。NEO-7M 未定位时 RMC 的经纬/速度是
//      空的, 若按 0 处理: 位置被抹成 0°N 0°E, 速度被抹成"真的没动"(最坏的那种)。
//   ③ 校验和错 / 半句 / 超长句: **整句丢弃**, 连时间戳都不动(宁可留着上一次的真值 + 让判据按"过期"失效)。
//   ④ 这里只做"组句", 不做"跨句投票": 不缓存历史、不做中值/一致性检查 —— 那是 B10 在 EKF 门限层该干的事,
//      放在解析器里会变成两处都有门限、谁都不知道最终生效的是哪一个。
//
// ---- 与时间有关的两条约定 ----
//   * nowMs 由调用方给(rt_tick/HAL_GetTick 都行), 解析器只做**无符号减法**求年龄 ⇒ 天然抗 32 位回绕
//     (49.7 天翻一次也不会突然"全部过期")。字段时间戳比较用 (int32)(new - old) > 0, 同样抗回绕。
//   * 有效性是**时间的函数**: 没有新句子时也要定期调 refresh(nowMs), 否则"过期"永远不会生效。
//
// ---- 为什么不是这样(几条被否掉的方案) ----
//   * 不用 sscanf/strtod 解字段: ① 拉进一大坨 libc(float 解析尤其重, 还依赖 locale); ② NMEA 的空字段
//     与超长垃圾字段在 sscanf 下行为不明确; ③ armclang 与 MinGW 的 printf 家族差异会让"PC 上过、
//     板上不过"。自己写整数解析(parseFixed/parseLatLon)总共不到 60 行, 而且能被穷举单测。
//   * 不解析 GSV: 它是"可见卫星列表", 一帧可能拆成 3~4 句、还带循环序号, 解析成本最高而控制上毫无用处
//     (只有地面站想看)。要调试搜星就临时开 GSV 看原始串口, 不要放进飞控的热路径。
//   * 不把 GSA 的定位类型(1/2/3)混进 fix: 它和 GGA 的质量(0..8)是两套语义, 混进同一个字段会让
//     "fix >= 1"这条判据在两条语句之间来回跳。GSA 的价值在 HDOP(见权威顺序表), 定位类型只记录在
//     gsaFix 里做交叉检查。副作用(写清楚): 如果 mask 只开 GSA 不开 GGA, fix 永远是 0 ⇒ 速度永远不可用
//     —— 这是**故意**的 fail-closed, 因为 GGA 的 fix 是判据里唯一有定义的那一个。
//   * 不做 ENU 转换(经纬度 → 本地切平面 GnssPosition): 那需要"起飞点原点 + 参考纬度"这两个本文件拿不到
//     的状态, 而且 §4.4 明确位置/高度**只回传不进控制**。本文件只提供 gnssGroundSpeedToEnu() 这一个
//     确实要进 EKF 的换算(速度+航向 → ENU), 原点相关的转换留给位置回传那条链路。
//   * 不做 GSA/GSV 的 PDOP/VDOP: 判据用的是 HDOP(水平速度只受水平几何影响), 多解析两个不用的数
//     只会多两处可能出错的代码。
//
// ---- 与方案 §4.4 的关系 / 需要 B10 决定的事 ----
//   * 一致: USART2 + DMA/IDLE 碎片到达 ⇒ 纯增量解析; 位置/高度只回传; 只有水平速度进 EKF;
//     RMC+GGA+GSA 足够; 9600@1Hz 或改模块到 5Hz 都不影响本文件(它不关心速率, 只关心字节)。
//   * 需要决定: ① §4.4 说"RMC + GGA + GSA 足够", 而 schema 的 GSA 说明写了"用于 HDOP 门限判据";
//     本文件两者都给 HDOP, 同刻以 GSA 为准(见权威顺序)。② §4.4 只写了速度进 EKF, 没写用哪个接口:
//     本文件给出 gnssGroundSpeedToEnu()(ENU 水平两轴, m/s), 但 **z 分量不是量测** —— 必须走
//     "只看水平"的接口(§6.4 的 updateHorizontalVelocity(vel_world, sigma, gate))或把 z 的 sigma
//     放到无穷大; 直接喂 updateGpsVelocity(3 轴)会把 vz = 0 当成"真的没有垂速"这个强观测。
//   * B10 的接线(伪代码, 在 gnss 线程里按周期跑):
//       parser.feed(rxBytes, n, nowMs);          // IDLE 回调搬过来的那一坨
//       const GnssSolution &s = parser.solution();
//       if (s.speedValid) {                       // ← 唯一的闸门, 不可信就一次都别喂
//           float ve = 0.0f, vn = 0.0f;
//           gnssGroundSpeedToEnu(s.gspeedCms, s.courseCdeg, ve, vn);
//           // sigma = cfg::sensor::gnss::kVelSigmaMps(0.3), gate = 3.0(§6.4)
//       }
//       parser.refresh(nowMs);                    // 没有新句子时也要跑: 让"过期"生效
//     位置/海拔(latE7/lonE7/altMslMm)**只回传**(DL_GPS 遥测帧, 字段名/单位完全一致, 直接拷)。
//   * GNSS_SPEED_REJECTED(0x0704) 不在这里报: 它是 EKF 门限拒绝速度观测的码, 属于 B10 的调用点。
//===================================================================================================

#include <stdint.h>
#include <math.h>

#include "dlx_exception.h"

// 阈值/位掩码的唯一真相是配置生成头(config/flight_config.schema.json → sensor.gnss / hw.gnss)。
// 只在 feature.kGnss 打开时存在; 关掉时用兜底常量, 好让本文件在 PC 上仍能被单测直接编译。
#include "generated/flight_config_gen.hpp"

#if defined(CFG_FEATURE_GNSS) && CFG_FEATURE_GNSS
#define DLX_GNSS_HAVE_CFG 1
#else
#define DLX_GNSS_HAVE_CFG 0
#endif

namespace dlx
{
    //===============================================================================================
    // 阈值(全部来自配置, 见文件头)
    //===============================================================================================
#if DLX_GNSS_HAVE_CFG
    constexpr uint8_t  kGnssMinSats    = cfg::sensor::gnss::kMinSats;
    constexpr uint16_t kGnssMaxHdopX10 = (uint16_t)(cfg::sensor::gnss::kMaxHdop * 10.0f + 0.5f);
    constexpr uint32_t kGnssStaleMs    = cfg::sensor::gnss::kStaleMs;
    constexpr uint8_t  kGnssDefaultMask = cfg::sensor::gnss::kNmeaMask;
#else
    // 兜底值 = schema 的默认值(sensor.gnss), 只为让本文件脱离工程也能编译/单测。
    constexpr uint8_t  kGnssMinSats    = 6;
    constexpr uint16_t kGnssMaxHdopX10 = 20;
    constexpr uint32_t kGnssStaleMs    = 500;
    constexpr uint8_t  kGnssDefaultMask = 0x07;
#endif

    //===============================================================================================
    // NMEA mask 位(逐位等于 schema.enums.NmeaMaskBits, 见文件头"为什么自己定一套位是错的")
    //===============================================================================================
    enum NmeaMaskBit : uint8_t
    {
        kNmeaMaskRmc = 0x01, ///< $GxRMC: 定位有效性 + 经纬度 + 地速(节) + 真航向  —— 本文件实现
        kNmeaMaskGga = 0x02, ///< $GxGGA: fix / 星数 / HDOP / 海拔 / 经纬度          —— 本文件实现
        kNmeaMaskGsa = 0x04, ///< $GxGSA: 定位类型(1/2/3) + PDOP/HDOP/VDOP          —— 本文件实现(只用 HDOP/类型)
        kNmeaMaskGsv = 0x08, ///< $GxGSV: 可见卫星列表 —— **本文件不实现**(运行期无用, 只影响搜星调试)
        kNmeaMaskVtg = 0x10, ///< $GxVTG: 地速(节/km/h/m/s) + 航向, 与 RMC 重复     —— 本文件实现
        kNmeaMaskGll = 0x20, ///< $GxGLL: 经纬度 + UTC —— **本文件不实现**(与 RMC 完全重复)
        kNmeaMaskZda = 0x40, ///< $GxZDA: UTC 日期时间 —— **本文件不实现**(不需要绝对时间戳)
    };

    //===============================================================================================
    // 语句分组(用于"这句属于哪一组 / 哪一组最新"以及逐字段的"谁更新谁生效")
    //   ⚠ 取值顺序 = 权威顺序的下标, 不要随手调整; 索引直接用作数组下标(见 GnssSolution::group)。
    //===============================================================================================
    enum class NmeaGroup : uint8_t
    {
        None = 0,
        Gga,   ///< 1: 定位质量/星数/HDOP/海拔/经纬度
        Rmc,   ///< 2: 有效性/经纬度/速度/航向
        Vtg,   ///< 3: 速度/航向(与 RMC 重复)
        Gsa,   ///< 4: 定位类型/HDOP
        Gll,   ///< 5: (mask 有位, 本文件不实现)
        Gsv,   ///< 6: (mask 有位, 本文件不实现)
        Count
    };

    /** 组名(诊断/日志用)。写成函数而不是表: 以后往 NmeaGroup 里加值忘了加 case, 单测第 ① 条会露出来。 */
    inline const char *nmeaGroupName(NmeaGroup g)
    {
        switch (g) {
        case NmeaGroup::None: return "None";
        case NmeaGroup::Gga:  return "GGA";
        case NmeaGroup::Rmc:  return "RMC";
        case NmeaGroup::Vtg:  return "VTG";
        case NmeaGroup::Gsa:  return "GSA";
        case NmeaGroup::Gll:  return "GLL";
        case NmeaGroup::Gsv:  return "GSV";
        default:              return "?";
        }
    }

    //===============================================================================================
    // 字段枚举(逐字段时间戳用; 值全部是 int32 能装下的定点量)
    //===============================================================================================
    enum NmeaField : uint8_t
    {
        kNfLat = 0, ///< 纬度 ×1e7 [度]      (int32, 带符号)
        kNfLon,     ///< 经度 ×1e7 [度]      (int32, 带符号)
        kNfAlt,     ///< 海拔 MSL [mm]       (int32, 带符号: 死海一带是负的)
        kNfSpeed,   ///< 地速 [cm/s]         (>= 0)
        kNfCourse,  ///< 真航向 [0.01°]      (0..35999)
        kNfSats,    ///< 卫星数              (0..63, 与 DL_GPS 遥测的 u6 对齐)
        kNfFix,     ///< GGA 定位质量原值     (0..8)
        kNfHdop,    ///< HDOP ×10            (1..255; 0 = 没收到过)
        kNfCount
    };

    /** GGA 定位质量原值。判据只用 "fix >= 1" 这一条(见 gnssSpeedObservationUsable)。
     *  为什么不在判据里排掉 6/7/8(估算/手动/模拟): 判据要能一眼看懂、能穷举, 而 NEO-7M 正常天空下只给
     *  0/1/2。要收紧就改那个纯函数并同步改单测, 而不是在各调用点各写一个 if。
     *  ⚠ DL_GPS 遥测帧的 fix 只有 u2 ⇒ 4(RTK)~8 上线后饱和成 3(编码层做的, 见 dlx_link_frame.hpp)。 */
    enum GnssFixQuality : uint8_t
    {
        kGnssFixNone       = 0,
        kGnssFixGps        = 1,
        kGnssFixDgps       = 2,
        kGnssFixPps        = 3,
        kGnssFixRtk        = 4,
        kGnssFixFloatRtk   = 5,
        kGnssFixEstimated  = 6,
        kGnssFixManual     = 7,
        kGnssFixSimulation = 8
    };

    //===============================================================================================
    // 解析统计 / 错误码
    //===============================================================================================
    /** 计数器(全部是"上电以来"的累计值, reset() 清零)。排查"收不到数据"时按这个顺序看:
     *  bytesIn == 0 ⇒ DMA/IDLE 没起来; noiseBytes 涨而 sentencesOk 不涨 ⇒ 波特率/线序/电平错(收到的是
     *  0x00/0xFF)或校验和范围写错; overlongDropped 涨 ⇒ 波特率错到把字节流切错了。 */
    struct GnssParserStats
    {
        uint32_t bytesIn;          ///< feed 进来的总字节数
        uint32_t noiseBytes;       ///< '$' 之外被丢掉的字节(句子之间的 CR/LF 也算)
        uint32_t checksumOk;       ///< 校验和通过的句子数(还没看 mask / 语句类型)
        uint32_t accepted;         ///< 真正被接受的句子数(= feed 返回 true 的次数)
        uint32_t checksumErrors;   ///< 校验和错 → 丢整句 → 报 GNSS_NMEA_CHECKSUM
        uint32_t overlongDropped;  ///< 超过 82 字符(正文 > 76)的句子 → 丢句并重新同步到下一个 '$'
        uint32_t framingDropped;   ///< 句内出现 CR/LF/'$'/非十六进制校验位 → 半句, 丢弃
        uint32_t unknown;          ///< 校验和正确但语句类型不认识(如 $GPXXX) → 忽略
        uint32_t maskedOut;        ///< 认识但被 kNmeaMask 关掉(或本文件没实现的 GSV/GLL/ZDA)
        uint32_t fieldErrors;      ///< 字段数不足/数值非法/空必填字段 → 该句不更新字段
        uint32_t errorsDropped;    ///< 待报异常环满时丢掉的条数(见 takeError)
        ExceptionCode lastError;   ///< 最近一次报出的异常码(不会被 takeError 清掉, 便于事后看)
    };

    //===============================================================================================
    // GnssSolution —— 一份可以被控制环整体快照的定位解(纯 POD, 无指针/无动态分配)
    //===============================================================================================
    /** 数值字段与 DL_GPS 遥测帧(DlGpsPayload)的 8 个字段一一对应, 单位完全相同 ⇒ 回传时直接拷贝即可。
     *  ⚠ 数值字段**不代表"可以用"**: 用之前必须看 posValid/altValid/speedValid/courseValid 与 fix。
     *  为什么要保留"无效时的旧值"而不是清零: ① 回传/日志想看最后已知位置; ② 清零会被下游误当成
     *  "真的在原点/真的没动" —— 这正是本文件反复强调的那类事故。 */
    struct GnssSolution
    {
        // ---- 8 个数据字段(单位见名字, 与 DL_GPS 遥测帧一致) ----
        int32_t  latE7;      ///< 纬度 ×1e7 [度](南纬为负)
        int32_t  lonE7;      ///< 经度 ×1e7 [度](西经为负)
        int32_t  altMslMm;   ///< 海拔(MSL) [mm](低于海平面为负)
        int16_t  gspeedCms;  ///< 地速(水平) [cm/s]
        uint16_t courseCdeg; ///< 真航向 [0.01°], 0..35999
        uint8_t  sats;       ///< 参与解算的卫星数(0..63)
        uint8_t  fix;        ///< GGA 定位质量原值(0..8, 见 GnssFixQuality)
        uint8_t  hdopX10;    ///< HDOP ×10(0 = 没收到过)

        // ---- 有效性(唯一的"能不能用"依据) ----
        bool posValid;    ///< 位置可用: 收到过经纬 + fix>=1 + RMC(若开)为 'A' + 位置不旧于 kStaleMs
        bool altValid;    ///< 海拔可用: 同上(但只看海拔自己的时间戳)
        bool speedValid;  ///< **速度可进 EKF**: == gnssSpeedObservationUsable(...), 唯一闸门
        bool courseValid; ///< 航向可用: 收到过航向 && speedValid(静止时接收机给的航向是噪声, 单独没有意义)

        // ---- 组句信息(回答"这句属于哪一组 / 哪一组最新") ----
        struct GroupInfo
        {
            uint32_t ms;    ///< 该组最后一句被接受的时刻(调用方给的 nowMs)
            uint32_t count; ///< 该组累计被接受的句数
            bool     seen;  ///< 收到过至少一句
            bool     valid; ///< 该组自己声明的有效性(RMC 的 A/V; GGA 的 fix>=1; VTG 恒 true; GSA 的 2D/3D)
        };
        GroupInfo group[(uint8_t)NmeaGroup::Count]; ///< 下标 = (uint8_t)NmeaGroup; 槽 0 恒空

        NmeaGroup latestGroup; ///< 最近被接受的那一句属于哪一组
        uint32_t  latestMs;    ///< 上面那一句的时刻
        uint32_t  seq;         ///< 每接受一句 +1(调用方用它判断"有没有新东西")

        // ---- RMC 状态(判据要用; V = 接收机自己说"这次的位置不算数") ----
        bool rmcSeen;  ///< 收到过 RMC(mask 关掉 RMC 时为 false ⇒ 判据不再要求 'A')
        bool rmcValid; ///< 最后一条 RMC 的状态是不是 'A'

        // ---- GSA 定位类型(只记录/交叉检查, **不进判据**, 见文件头) ----
        uint8_t gsaFix; ///< 0 = 没收到过; 1 = 无解; 2 = 2D; 3 = 3D

        // ---- 逐字段来源(诊断 + 单测断言"谁更新谁生效"用) ----
        uint32_t fieldMs[kNfCount];  ///< 该字段最后一次被写入的时刻
        uint8_t  fieldSrc[kNfCount]; ///< 该字段最后一次是谁写的((uint8_t)NmeaGroup, 0 = 从没写过)

        /** HDOP 的浮点视图(只给显示/日志用; 判据一律用整数 hdopX10, 免得边界比较随编译器飘) */
        float hdop() const { return (float)hdopX10 * 0.1f; }
        /** 地速的浮点视图 [m/s] */
        float speedMps() const { return (float)gspeedCms * 0.01f; }
        void  clear(); ///< 全清零(字段值也清), 见实现
    };

    //===============================================================================================
    // 有效性判据 —— 全系统唯一的一份, 纯函数(单测第 ⑨ 条穷举它)
    //===============================================================================================
    /** "这份 GNSS 地速能不能作为 EKF 的水平速度观测"。无状态: 给定这几个数, 答案唯一。
     *
     *  @param fix       GGA 定位质量原值(0..8); 0 = 没定位 ⇒ 拒绝
     *  @param sats      参与解算的卫星数
     *  @param hdopX10   HDOP ×10; **0 表示"从没收到过 HDOP" ⇒ 拒绝**(HDOP 物理上 >= 1.0, 真值不可能是 0,
     *                   所以 0 当哨兵是安全的 —— 收不到 HDOP 就 fail-closed, 不能默认"几何很好")
     *  @param rmcValid  最后一条 RMC 的状态是否为 'A'
     *  @param rmcSeen   是否收到过 RMC(mask 关了 RMC 时为 false, 此时不要求 'A')
     *  @param ageMs     判据依赖量中**最旧**那个的年龄(now - min(fix,sats,hdop,speed 的时间戳))
     *  @return true = 可以喂给 EKF 的水平速度观测
     *
     *  ⚠ 前置条件(在类里检查): 必须**收到过速度句**(RMC 或 VTG); 从没收到过就没有速度可谈, 此时
     *  ageMs 会传 0xFFFFFFFF 让这里必然拒绝。
     *  ⚠ 本函数只回答"能不能信", 不回答"是不是 0": 可信的 0 与不可信的 0 是两件事, 不可信时调用方
     *  必须**整条观测都不发**, 而不是发一个 0。 */
    inline bool gnssSpeedObservationUsable(uint8_t fix, uint8_t sats, uint8_t hdopX10,
                                           bool rmcValid, bool rmcSeen, uint32_t ageMs)
    {
        if (fix < (uint8_t)kGnssFixGps) { return false; }                  // 没定位
        if (sats < kGnssMinSats) { return false; }                         // 星太少: 速度误差可达数 m/s
        if (hdopX10 == 0u) { return false; }                               // 没收到过 HDOP ⇒ 不敢信
        if ((uint16_t)hdopX10 > kGnssMaxHdopX10) { return false; }         // 几何太差
        if (rmcSeen && !rmcValid) { return false; }                        // 接收机自己说这次不算数
        if (ageMs > kGnssStaleMs) { return false; }                        // 数据过期
        return true;
    }

    //===============================================================================================
    // 地速 + 航向 → ENU 水平速度(§4.4: 只有水平速度进 EKF)
    //===============================================================================================
    /** @param gspeedCms    地速 [cm/s](取 GnssSolution::gspeedCms)
     *  @param courseCdeg   真航向 [0.01°](取 GnssSolution::courseCdeg)
     *  @param vEastMps     出: 东向速度 [m/s]
     *  @param vNorthMps    出: 北向速度 [m/s]
     *
     *  真北 = 0°、顺时针为正 ⇒ vE = v·sin(course), vN = v·cos(course)。
     *  为什么不是 NED: EKF(flight_control_fliter.hpp)的 FF_VX/VY 是 ENU(世界系 z 向上)。给 NED 会同时
     *  错"轴交换"和"符号"两处, 是天上最难查的一类 bug(悬停不飘、一推杆就往反方向加速)。
     *  ⚠ 竖直分量: 本函数**只给水平两轴**。如果把它填进 3 轴接口, 那个 z = 0 必须理解成"没有这个
     *    量测"而不是"垂速为 0" —— 正确做法是用只看水平的接口(§6.4 的 updateHorizontalVelocity)
     *    或把 z 的 sigma 放到无穷大; 否则 EKF 会收到一个"飞机垂速恒为 0"的强观测。
     *  ⚠ 磁偏角: 我们按"真北"用。u-blox 会根据当前位置自己算磁偏角并把它加到磁航向上, 所以通常情况下
     *    这里就是真航向; 若模块没算(或没配 CFG-DAT), 会有几度的系统偏差 ⇒ 速度方向误差 = v·sin(3°)
     *    ≈ 0.3 × 0.05 = 0.016 m/s, 相对 kVelSigmaMps(0.3) 可以忽略。 */
    inline void gnssGroundSpeedToEnu(int16_t gspeedCms, uint16_t courseCdeg,
                                     float &vEastMps, float &vNorthMps)
    {
        const float v    = (float)gspeedCms * 0.01f;
        const float rad  = (float)courseCdeg * 0.01f * 0.01745329251994329577f; // ×(π/180)
        vEastMps  = v * sinf(rad);
        vNorthMps = v * cosf(rad);
    }

    //===============================================================================================
    // 解析器
    //===============================================================================================
    /** 增量 NMEA 解析器。全部状态在对象内部(定长缓冲, 无动态分配, 无异常, 无 RTTI)。
     *  线程约定: 只允许**一个**线程/中断上下文调用 feed(); 控制环读 solution() 前请自行做临界区快照
     *  (与工程里其它 POD 一致, 这里不加锁 —— 加锁会让"控制环不做任何阻塞"这条硬规则失效)。 */
    class GnssNmeaParser
    {
    public:
        explicit GnssNmeaParser(uint8_t mask = kGnssDefaultMask)
        {
            reset();
            _mask = mask;
        }

        /** 清空解析状态与解(统计也清零), **mask 不变**。 */
        void reset()
        {
            _len   = 0;
            _state = kRxIdle;
            _ckHi  = 0;
            _nf    = 0;
            _pendHead  = 0;
            _pendCount = 0;
            for (uint8_t i = 0; i < kPendingErrorCount; ++i) {
                // 这里的 0 不是"异常码表里的 OK", 而是"这个槽是空的" —— 异常码里没有 OK(见 dlx_exception.h),
                // EXCEPTION_NONE 就是它专门给"没有"留的那个值。
                _pending[i] = ExceptionCode::EXCEPTION_NONE;
            }
            for (uint16_t i = 0; i < (uint16_t)kNfCount; ++i) {
                _fv[i]   = 0;
                _fsrc[i] = (uint8_t)NmeaGroup::None;
                _fms[i]  = 0;
            }
            _sol.clear();
            _stats.bytesIn         = 0;
            _stats.noiseBytes      = 0;
            _stats.checksumOk      = 0;
            _stats.accepted        = 0;
            _stats.checksumErrors  = 0;
            _stats.overlongDropped = 0;
            _stats.framingDropped  = 0;
            _stats.unknown         = 0;
            _stats.maskedOut       = 0;
            _stats.fieldErrors     = 0;
            _stats.errorsDropped   = 0;
            _stats.lastError       = ExceptionCode::EXCEPTION_NONE; ///< "还没有报过异常"(状态, 不是 OK 码)
        }

        void    setMask(uint8_t mask) { _mask = mask; }
        uint8_t mask() const { return _mask; }

        //-------------------------------------------------------------------------------------------
        // 喂字节
        //-------------------------------------------------------------------------------------------
        /** 喂 1 个字节。@return true = 这 1 个字节凑成了一句**被接受**的语句。
         *  @param nowMs 当前时刻(ms), 只用来给字段打时间戳, 不做别的。 */
        bool feed(uint8_t b, uint32_t nowMs)
        {
            ++_stats.bytesIn;
            switch (_state) {
            case kRxIdle:
                if (b == (uint8_t)'$') {
                    _len   = 0;
                    _state = kRxBody;
                } else {
                    ++_stats.noiseBytes; // 句子之间的一切(CR/LF/0x00/0xFF/半句尾巴)都算噪声
                }
                return false;

            case kRxBody:
                if (b == (uint8_t)'$') {
                    // 上一句还没等到 '*' 就来了新的句首: 丢半句, 立刻从这句重新开始。
                    // 为什么可以这样"抢跑": NMEA 正文里不可能出现 '$'(它是唯一的句首标志), 所以
                    // 看到 '$' 就一定是新句子的开头 —— 比"一路丢到 CR/LF 再重新找"少丢一句。
                    ++_stats.framingDropped;
                    _len = 0;
                    return false;
                }
                if (b == (uint8_t)'*') {
                    _state = kRxCkHi;
                    return false;
                }
                if (b == (uint8_t)'\r' || b == (uint8_t)'\n') {
                    ++_stats.framingDropped; // 没有 '*' 就换行了 ⇒ 半句(丢字节/线序错时常见)
                    _state = kRxIdle;
                    return false;
                }
                if (_len >= kMaxBodyLen) {
                    // 正文超过 76 字符 ⇒ 整句必然超过 NMEA 的 82 字符上限。**先判长度再写缓冲**,
                    // 所以永远不会溢出(kMaxBodyLen 与缓冲大小由 static_assert 钉死)。
                    ++_stats.overlongDropped;
                    _state = kRxIdle; // 重新同步: 下一个 '$' 就是新句子
                    return false;
                }
                _buf[_len] = (char)b;
                ++_len;
                return false;

            case kRxCkHi: {
                const int8_t v = hexNibble(b);
                if (v < 0) {
                    ++_stats.framingDropped;
                    resync(b);
                    return false;
                }
                _ckHi  = (uint8_t)v;
                _state = kRxCkLo;
                return false;
            }

            case kRxCkLo: {
                const int8_t v = hexNibble(b);
                if (v < 0) {
                    ++_stats.framingDropped;
                    resync(b);
                    return false;
                }
                _state = kRxIdle;
                const uint8_t want = (uint8_t)((uint8_t)(_ckHi << 4) | (uint8_t)v);
                const uint8_t got  = bodyChecksum();
                if (want != got) {
                    // 校验和错 ⇒ **整句丢弃**: 不更新任何字段, 也不动任何时间戳。
                    // 宁可留着上一次的真值(由 kStaleMs 判过期), 也不要一个"看似新的"错值。
                    ++_stats.checksumErrors;
                    setError(ExceptionCode::GNSS_NMEA_CHECKSUM);
                    return false;
                }
                _buf[_len] = '\0'; // 让 lastSentence() 可以当 C 字符串用
                ++_stats.checksumOk;
                return dispatch(nowMs);
            }

            default:
                break;
            }
            return false;
        }

        /** 一次喂一段(DMA/IDLE 回调里搬过来的那一坨)。@return 被接受的句子数。 */
        uint16_t feed(const uint8_t *data, uint16_t len, uint32_t nowMs)
        {
            if (data == nullptr || len == 0u) { return 0u; }
            uint16_t n = 0;
            for (uint16_t i = 0; i < len; ++i) {
                if (feed(data[i], nowMs)) { ++n; }
            }
            return n;
        }

        /** 喂一串以 '\0' 结尾的字符(串口调试/单测方便; 飞控热路径请用上面两个)。 */
        uint16_t feed(const char *text, uint32_t nowMs)
        {
            if (text == nullptr) { return 0u; }
            uint16_t n = 0;
            for (const char *p = text; *p != '\0'; ++p) {
                if (feed((uint8_t)*p, nowMs)) { ++n; }
            }
            return n;
        }

        //-------------------------------------------------------------------------------------------
        // 读结果
        //-------------------------------------------------------------------------------------------
        const GnssSolution &solution() const { return _sol; }
        const GnssParserStats &stats() const { return _stats; }

        /** 最近**通过校验和**的那一句原始正文(不含 '$' 与 '*XX'), 供上板排查时打日志。
         *  只在解析过句子之后有意义(第一句之前是空串)。 */
        const char *lastSentence() const { return _buf; }

        /** 只重算"时间相关"的有效性(没有新字节时也要按周期调, 否则数据永远不会被判过期)。
         *  @param nowMs 当前时刻(ms) */
        void refresh(uint32_t nowMs) { syncSolution(nowMs); }

        /** 速度能不能进 EKF(等价于 solution().speedValid 的最新值)。 */
        bool speedUsable(uint32_t nowMs)
        {
            syncSolution(nowMs);
            return _sol.speedValid;
        }

        /** ISR/DMA 侧发现丢字节时调一下(环形缓冲被覆盖/溢出), 转成 GNSS_UART_OVERRUN 报出去。
         *  为什么放在这里而不是让 ISR 自己存一个错误码: 全工程只有一处"当前 GNSS 异常"的状态。 */
        void noteRxOverrun() { setError(ExceptionCode::GNSS_UART_OVERRUN); }

        /** 取走待报异常里**最早**的一条(FIFO; 无异常返回 false)。
         *  为什么用一个小环而不是只留一条: "没定位"(NO_FIX, 上电后每次 GGA 都会报)会一直占着单槽,
         *  于是同一窗口里更该看见的"校验和错/串口溢出"就永远报不出来。环深 4 条, 满了丢最新的并计数
         *  (errorsDropped), 与"计数不丢、明细可覆盖"同一种处理方式。 */
        bool takeError(ExceptionCode &out)
        {
            if (_pendCount == 0u) { return false; }
            out       = _pending[_pendHead];
            _pendHead = (uint8_t)((_pendHead + 1u) % kPendingErrorCount);
            --_pendCount;
            return true;
        }

        /** NMEA 一句的字符上限(含 '$'、'*XX'、CRLF), 方案/规范都是 82。 */
        static const uint16_t kMaxSentenceBytes = 82;
        /** 正文($ 与 * 之间)上限 = 82 - 1('$') - 3('*XX') - 2(CRLF) = 76。 */
        static const uint16_t kMaxBodyLen = 76;

    private:
        //-------------------------------------------------------------------------------------------
        // 状态机
        //-------------------------------------------------------------------------------------------
        enum RxState : uint8_t
        {
            kRxIdle = 0, ///< 等 '$'
            kRxBody,     ///< 收正文直到 '*' (或 CR/LF/'$' 异常结束)
            kRxCkHi,     ///< 校验和高位
            kRxCkLo      ///< 校验和低位
        };

        /** 非十六进制字节打断了校验和字段: 丢句; 若这一字节本身就是 '$', 直接把它当新句首(少丢一句)。 */
        void resync(uint8_t b)
        {
            _state = kRxIdle;
            if (b == (uint8_t)'$') {
                _len   = 0;
                _state = kRxBody;
            }
        }

        static int8_t hexNibble(uint8_t c)
        {
            if (c >= (uint8_t)'0' && c <= (uint8_t)'9') { return (int8_t)(c - (uint8_t)'0'); }
            if (c >= (uint8_t)'A' && c <= (uint8_t)'F') { return (int8_t)(c - (uint8_t)'A' + 10); }
            if (c >= (uint8_t)'a' && c <= (uint8_t)'f') { return (int8_t)(c - (uint8_t)'a' + 10); }
            return -1;
        }

        /** 异或范围 = '$' 与 '*' **之间**的每一个字符(不含这两个)。 */
        uint8_t bodyChecksum() const
        {
            uint8_t x = 0;
            for (uint16_t i = 0; i < _len; ++i) { x = (uint8_t)(x ^ (uint8_t)_buf[i]); }
            return x;
        }

        void setError(ExceptionCode code)
        {
            _stats.lastError = code;
            if (_pendCount >= kPendingErrorCount) {
                ++_stats.errorsDropped; // 环满: 丢最新的那条(计数里仍然看得见)
                return;
            }
            _pending[(uint8_t)((_pendHead + _pendCount) % kPendingErrorCount)] = code;
            ++_pendCount;
        }

        //-------------------------------------------------------------------------------------------
        // 字段切分(按逗号)
        //-------------------------------------------------------------------------------------------
        /** 为什么先整句切好再取字段: NMEA 的字段位置是协议的一部分(如 GGA 的 fix 恒在第 6 个字段),
         *  切好之后就是 fld(6) 这种一眼能核对的写法; 边扫边取会让"第几个字段"散落在各处,
         *  而字段错位恰恰是这类解析器最常见的 bug。定长数组 + 上限截断, 无动态分配。 */
        static const uint8_t kMaxFields = 24; ///< GSA 19 个、GSV 20 个, 24 足够; 多出来的直接忽略
        struct Field
        {
            const char *p;
            uint8_t     n;
        };

        void splitAll(const char *s)
        {
            _nf = 0;
            while (s != nullptr && *s != '\0' && _nf < kMaxFields) {
                const char *q = s;
                while (*q != '\0' && *q != ',') { ++q; }
                _f[_nf].p = s;
                _f[_nf].n = (uint8_t)(q - s);
                ++_nf;
                if (*q == ',') { s = q + 1; } else { break; }
            }
        }

        /** 第 i 个字段(0 = 语句名如 "GNRMC")。不存在返回 nullptr 且 len = 0。 */
        const char *fld(uint8_t i, uint8_t &len) const
        {
            if (i >= _nf) {
                len = 0;
                return nullptr;
            }
            len = _f[i].n;
            return _f[i].p;
        }

        //-------------------------------------------------------------------------------------------
        // 数值解析(不用 sscanf/strtod, 见文件头)
        //-------------------------------------------------------------------------------------------
        static int64_t pow10i(uint8_t n)
        {
            int64_t v = 1;
            while (n > 0u) {
                v *= 10;
                --n;
            }
            return v;
        }

        /** 定点十进制 → ×10^scale 的整数(四舍五入)。
         *  @param scale     目标小数位数(0~3)
         *  @param allowNeg  允许 '-' 开头(海拔需要; 速度/航向/星数/HDOP 传 false ⇒ 见到 '-' 直接判非法)
         *  @return false = 空字段 / 含非法字符 / 值大得不像真数据(> 1e8) ⇒ 调用方**不要更新**该字段 */
        static bool parseFixed(const char *p, uint8_t len, uint8_t scale, bool allowNeg, int32_t &out)
        {
            out = 0;
            if (p == nullptr || len == 0u || len > 20u) { return false; }
            uint8_t i     = 0;
            bool    neg   = false;
            if (p[0] == '-') {
                if (!allowNeg) { return false; }
                neg = true;
                i   = 1;
            } else if (p[0] == '+') {
                i = 1;
            }
            if (i >= len) { return false; }

            int64_t acc    = 0;
            uint8_t frac   = 0;   // 已收下的小数位数
            int8_t  roundD = -1;  // 小数第 scale+1 位(决定舍入)
            bool    dot    = false;
            bool    anyDig = false;
            for (; i < len; ++i) {
                const char c = p[i];
                if (c == '.') {
                    if (dot) { return false; }
                    dot = true;
                    continue;
                }
                if (c < '0' || c > '9') { return false; }
                anyDig = true;
                const int8_t d = (int8_t)(c - '0');
                if (!dot) {
                    acc = acc * 10 + d;
                    if (acc > 100000000LL) { return false; } // 整数部分超过 1e8: 不是真数据
                } else if (frac < scale) {
                    acc = acc * 10 + d;
                    ++frac;
                } else if (frac == scale) {
                    roundD = d; // 只留一位用于舍入, 不累加(避免无意义的长尾)
                    ++frac;
                } else {
                    ++frac; // 更远的位不影响"四舍五入到 scale 位"的结果
                }
            }
            if (!anyDig) { return false; }
            for (uint8_t k = frac; k < scale; ++k) { acc *= 10; }
            if (roundD >= 5) { ++acc; }
            if (neg) { acc = -acc; }
            if (acc > 100000000LL || acc < -100000000LL) { return false; }
            out = (int32_t)acc;
            return true;
        }

        /** "ddmm.mmmm" / "dddmm.mmmm" + 半球字母 → 度 ×1e7。
         *  算法: 小数点是唯一的分界; 整数部分的**最后两位**是"分", 再往前是"度";
         *        度×1e7 + 分×(1e7/60), 全程 64 位整除(见文件头"为什么不经过 float")。
         *  @param isLat true = 纬度(半球必须是 N/S 且 <= 90°), false = 经度(必须是 E/W 且 <= 180°) */
        static bool parseLatLon(const char *p, uint8_t len, char hemi, bool isLat, int32_t &outE7)
        {
            outE7 = 0;
            if (p == nullptr || len == 0u) { return false; }
            if (hemi >= 'a' && hemi <= 'z') { hemi = (char)(hemi - 32); }
            if (isLat) {
                if (hemi != 'N' && hemi != 'S') { return false; }
            } else {
                if (hemi != 'E' && hemi != 'W') { return false; }
            }

            int64_t mant      = 0;
            uint8_t intDigits = 0;
            uint8_t fracDigits = 0;
            bool    dot       = false;
            for (uint8_t i = 0; i < len; ++i) {
                const char c = p[i];
                if (c == '.') {
                    if (dot) { return false; }
                    dot = true;
                    continue;
                }
                if (c < '0' || c > '9') { return false; }
                if (dot) { ++fracDigits; } else { ++intDigits; }
                if ((uint16_t)(intDigits + fracDigits) > 12u) { return false; }
                mant = mant * 10 + (c - '0');
            }
            if (intDigits < 3u || fracDigits == 0u) { return false; } // 最少 "dmm.m"
            const int64_t fscale = pow10i(fracDigits);
            const int64_t whole  = mant / fscale; // "ddmm" 的整数部分
            const int64_t fracP  = mant % fscale;
            if ((whole % 100) >= 60) { return false; } // 分 >= 60.0: 只可能是字段错位/垃圾
            const int64_t deg   = whole / 100;
            const int64_t minSc = (whole % 100) * fscale + fracP; // 分 × 10^fracDigits
            const int64_t den   = 60 * fscale;
            const int64_t e7    = deg * 10000000LL + (minSc * 10000000LL + den / 2) / den;
            if (isLat) {
                if (e7 > 900000000LL) { return false; }
            } else {
                if (e7 > 1800000000LL) { return false; }
            }
            outE7 = (int32_t)((hemi == 'S' || hemi == 'W') ? -e7 : e7);
            return true;
        }

        static char upper(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; }

        //-------------------------------------------------------------------------------------------
        // 字段写入(唯一入口) —— "谁更新谁生效"就在这里
        //-------------------------------------------------------------------------------------------
        /** 该字段的权威顺序: 0 = 最权威; 0xFF = 这一组**无权**更新该字段。
         *  (只在时间戳相同时用来分胜负, 见文件头"组句规则") */
        static uint8_t ownerRank(NmeaField f, NmeaGroup g)
        {
            switch (f) {
            case kNfLat:
            case kNfLon:
                if (g == NmeaGroup::Gga) { return 0; }
                if (g == NmeaGroup::Rmc) { return 1; }
                return 0xFF;
            case kNfAlt:
            case kNfSats:
            case kNfFix:
                return (g == NmeaGroup::Gga) ? 0 : 0xFF; // 只有 GGA 有这三样
            case kNfHdop:
                if (g == NmeaGroup::Gsa) { return 0; }
                if (g == NmeaGroup::Gga) { return 1; }
                return 0xFF;
            case kNfSpeed:
            case kNfCourse:
                if (g == NmeaGroup::Rmc) { return 0; }
                if (g == NmeaGroup::Vtg) { return 1; }
                return 0xFF;
            default:
                return 0xFF;
            }
        }

        /** 这一组这一次能不能写这个字段(时间戳更大者赢; 同刻按权威顺序)。 */
        bool wins(NmeaField f, NmeaGroup g, uint32_t ms) const
        {
            const uint8_t rank = ownerRank(f, g);
            if (rank == 0xFF) { return false; }
            const uint8_t src = _fsrc[f];
            if (src == (uint8_t)NmeaGroup::None) { return true; } // 从没写过
            // 时间戳比较用无符号差的符号位 ⇒ 32 位回绕也成立(见文件头)
            const int32_t d = (int32_t)(ms - _fms[f]);
            if (d != 0) { return d > 0; }
            return rank < ownerRank(f, (NmeaGroup)src);
        }

        void put(NmeaField f, NmeaGroup g, uint32_t ms, int32_t v)
        {
            if (!wins(f, g, ms)) { return; }
            _fv[f]   = v;
            _fsrc[f] = (uint8_t)g;
            _fms[f]  = ms;
        }

        //-------------------------------------------------------------------------------------------
        // 语句分发
        //-------------------------------------------------------------------------------------------
        /** _buf 形如 "GNRMC,...."。前 2 字符 = talker(不看), 3~5 = 语句类型, 第 6 字符必须是 ','。 */
        bool dispatch(uint32_t ms)
        {
            if (_len < 6u || _buf[5] != ',') {
                ++_stats.fieldErrors; // 连语句名都不完整
                return false;
            }
            NmeaGroup   g    = NmeaGroup::None;
            NmeaMaskBit mask = kNmeaMaskRmc;
            if (typeIs("RMC")) {
                g    = NmeaGroup::Rmc;
                mask = kNmeaMaskRmc;
            } else if (typeIs("GGA")) {
                g    = NmeaGroup::Gga;
                mask = kNmeaMaskGga;
            } else if (typeIs("VTG")) {
                g    = NmeaGroup::Vtg;
                mask = kNmeaMaskVtg;
            } else if (typeIs("GSA")) {
                g    = NmeaGroup::Gsa;
                mask = kNmeaMaskGsa;
            } else if (typeIs("GSV")) {
                ++_stats.maskedOut; // 认识但不实现(见文件头)
                return false;
            } else if (typeIs("GLL")) {
                ++_stats.maskedOut;
                return false;
            } else if (typeIs("ZDA")) {
                ++_stats.maskedOut;
                return false;
            } else {
                ++_stats.unknown; // 校验和是对的, 但我们不解析它 —— 忽略即可, 不影响后面的句子
                return false;
            }
            if ((_mask & (uint8_t)mask) == 0u) {
                ++_stats.maskedOut;
                return false;
            }

            // 从整句开头切字段 ⇒ 字段 0 = 语句名("GNRMC"), 与 NMEA 0183 的标准编号一致
            // (下面各 handler 的注释写的都是标准编号, 差一位就是最容易犯的错)
            splitAll(_buf);

            bool ok = false;
            switch (g) {
            case NmeaGroup::Rmc: ok = handleRmc(ms); break;
            case NmeaGroup::Gga: ok = handleGga(ms); break;
            case NmeaGroup::Vtg: ok = handleVtg(ms); break;
            case NmeaGroup::Gsa: ok = handleGsa(ms); break;
            default: break;
            }
            if (ok) {
                // 本组信息 + "最新一组"(只有**被接受**的句子才算, 否则 group.count 会与 accepted 对不上,
                // 排查时会以为"收到了很多句", 其实全是废句)
                GnssSolution::GroupInfo &gi = _sol.group[(uint8_t)g];
                gi.seen = true;
                gi.ms   = ms;
                ++gi.count;
                _sol.latestGroup = g;
                _sol.latestMs    = ms;
                ++_stats.accepted;
                ++_sol.seq;
            }
            syncSolution(ms);
            return ok;
        }

        /** _buf[0..4] 是不是 "xxYYY" 里的 YYY(前两位是 talker, 一律不看)。 */
        bool typeIs(const char *t) const
        {
            return _buf[2] == t[0] && _buf[3] == t[1] && _buf[4] == t[2];
        }

        //-------------------------------------------------------------------------------------------
        // $GxRMC: 0=名 1=UTC 2=状态A/V 3=纬度 4=N/S 5=经度 6=E/W 7=地速(节) 8=真航向
        //          9=日期 10=磁偏角 11=E/W 12=mode
        //-------------------------------------------------------------------------------------------
        bool handleRmc(uint32_t ms)
        {
            // 整句只要求"能取到状态字段(第 2)" —— 其余字段**逐个**校验, 缺哪个就少更新哪个。
            // 为什么不做"必须给满 13 个字段"的硬检查: 校验和已经保证这句是完整的(少一个字节就不对了),
            // 剩下的"字段少"只可能来自不标准的固件; 那种时候宁可救回经纬度, 也不要整句扔掉。
            if (_nf < 3u) {
                ++_stats.fieldErrors; // 连状态字段都没有: 不是我们能用的 RMC
                return false;
            }
            uint8_t     n  = 0;
            const char *st = fld(2, n);
            const bool  isA = (n == 1u && st != nullptr && upper(st[0]) == 'A');
            const bool  isV = (n == 1u && st != nullptr && upper(st[0]) == 'V');

            _sol.rmcSeen       = true;
            _sol.rmcValid      = isA;
            _sol.group[(uint8_t)NmeaGroup::Rmc].valid = isA;
            if (!isA) { setError(ExceptionCode::GNSS_NO_FIX); } // V = 接收机自己说"这次不算数"
            if (!isA && !isV) {
                // 空/非法状态: **不当成 A**(fail-closed)。空状态在丢字节/波特率错时最常出现,
                // 把它当 A 会让"位置有效"靠一个空字段成立。
                ++_stats.fieldErrors;
            }
            if (!isA) {
                // V(或状态非法)时**一个字段都不更新**: 这时模块给的往往是"上次的值"或空,
                // 更新它们会把上一帧的真实位置/速度换成过期的或空的。
                return true; // 句子本身是有效的(我们用它更新了"不有效"这个事实)
            }

            // ---- 位置 ----
            uint8_t     ln = 0;
            const char *p  = fld(3, ln);
            uint8_t     hn = 0;
            const char *h  = fld(4, hn);
            int32_t     v  = 0;
            if (p != nullptr && h != nullptr && hn == 1u && parseLatLon(p, ln, h[0], true, v)) {
                put(kNfLat, NmeaGroup::Rmc, ms, v);
            }
            p = fld(5, ln);
            h = fld(6, hn);
            if (p != nullptr && h != nullptr && hn == 1u && parseLatLon(p, ln, h[0], false, v)) {
                put(kNfLon, NmeaGroup::Rmc, ms, v);
            }

            // ---- 地速(节 → cm/s)与真航向 ----
            // 空字段/非法值 ⇒ 不更新(可能是 mask 关了 VTG、也可能是模块这次没给), 由"过期"判据兜住
            // ⚠ 必须分两句写: 不能写成 f(p = fld(7, ln), ln, ...) —— 同一个实参列表里既读 ln 又让
            //   fld() 去写 ln, 求值顺序在 C++ 里是**未指定**的(g++ 与 armclang 的方向不同),
            //   在 PC 上过、在板上不过就是这么来的(本文件第一版踩过)。
            p = fld(7, ln);
            speedFromKnots(p, ln, ms, NmeaGroup::Rmc);
            p = fld(8, ln);
            courseFromDeg(p, ln, ms, NmeaGroup::Rmc);
            return true;
        }

        //-------------------------------------------------------------------------------------------
        // $GxGGA: 0=名 1=UTC 2=纬度 3=N/S 4=经度 5=E/W 6=fix 7=星数 8=HDOP 9=海拔 10=M ...
        //-------------------------------------------------------------------------------------------
        bool handleGga(uint32_t ms)
        {
            // 同上: 只要求"能取到 fix(第 6)"(nf >= 7), 其余字段逐个校验
            if (_nf < 7u) {
                ++_stats.fieldErrors;
                return false;
            }
            uint8_t     n = 0;
            const char *p = nullptr;
            int32_t     v = 0;

            // fix: 0..8 的单个数字。空值只可能是丢字节(它是 GGA 的必填字段) ⇒ 记一次 fieldErrors,
            // 但仍然**不更新**这个字段(空 ≠ 0), 由上一次的 fix 值与"过期"判据一起兜住。
            p = fld(6, n);
            if (p == nullptr || n == 0u) {
                ++_stats.fieldErrors;
            } else if (parseFixed(p, n, 0, false, v) && v <= (int32_t)kGnssFixSimulation) {
                put(kNfFix, NmeaGroup::Gga, ms, v);
                _sol.group[(uint8_t)NmeaGroup::Gga].valid = (v >= (int32_t)kGnssFixGps);
                if (v < (int32_t)kGnssFixGps) { setError(ExceptionCode::GNSS_NO_FIX); }
            } else {
                ++_stats.fieldErrors; // 不是合法的 fix 值
            }

            // 星数: 0..63(上限与 DL_GPS 的 u6 对齐; NEO-7M 在轨可见数远小于 63, 超了只可能是错位)
            p = fld(7, n);
            if (p != nullptr && n > 0u && parseFixed(p, n, 0, false, v)) {
                if (v >= 0 && v <= 63) {
                    put(kNfSats, NmeaGroup::Gga, ms, v);
                } else {
                    ++_stats.fieldErrors;
                }
            }

            // HDOP: ×10
            p = fld(8, n);
            if (p != nullptr && n > 0u && parseFixed(p, n, 1, false, v)) {
                if (v > 0 && v <= 255) {
                    put(kNfHdop, NmeaGroup::Gga, ms, v);
                } else {
                    ++_stats.fieldErrors;
                }
            }

            // 海拔(MSL): 第 10 个字段是单位 'M', 对不上说明字段错位 ⇒ 不更新海拔
            p = fld(9, n);
            uint8_t     un = 0;
            const char *u  = fld(10, un);
            if (p != nullptr && n > 0u && u != nullptr && un == 1u && upper(u[0]) == 'M' &&
                parseFixed(p, n, 3, true, v)) {
                if (v > -20000000 && v < 20000000) { // ±20 km: 超出只可能是垃圾
                    put(kNfAlt, NmeaGroup::Gga, ms, v);
                } else {
                    ++_stats.fieldErrors;
                }
            }

            // 位置(GGA 也有经纬度; 与 RMC 的重复见文件头权威顺序)
            uint8_t     ln = 0;
            uint8_t     hn = 0;
            p              = fld(2, ln);
            const char *h  = fld(3, hn);
            if (p != nullptr && h != nullptr && hn == 1u && parseLatLon(p, ln, h[0], true, v)) {
                put(kNfLat, NmeaGroup::Gga, ms, v);
            }
            p = fld(4, ln);
            h = fld(5, hn);
            if (p != nullptr && h != nullptr && hn == 1u && parseLatLon(p, ln, h[0], false, v)) {
                put(kNfLon, NmeaGroup::Gga, ms, v);
            }
            return true;
        }

        //-------------------------------------------------------------------------------------------
        // $GxVTG: 0=名 1=真航向 2='T' 3=磁航向 4='M' 5=速度(节) 6='N' 7=速度(km/h) 8='K'
        //          9/10 = 一些固件额外给的 m/s 扩展
        //   为什么按"位置 + 单位字母"双重核对: VTG 的字段位置是 NMEA 0183 定死的, 单位字母是它的自校验;
        //   只看位置会在固件少给一对字段时把 km/h 当节用(差 1.85 倍), 只看字母则 'M' 有二义
        //   (第 4 字段的 'M' 是磁航向, 第 10 字段的 'M' 才是 m/s)。
        //-------------------------------------------------------------------------------------------
        bool handleVtg(uint32_t ms)
        {
            if (_nf < 2u) {
                ++_stats.fieldErrors;
                return false;
            }
            uint8_t n = 0;

            // ---- 航向: 优先真航向(T), 没有才退回磁航向(M) ----
            const char *p = fld(1, n);
            uint8_t     un = 0;
            const char *u  = fld(2, un);
            int32_t     v  = 0;
            bool        courseOk = false;
            if (p != nullptr && un == 1u && u != nullptr && upper(u[0]) == 'T' &&
                parseFixed(p, n, 2, false, v) && v >= 0 && v < 36000) {
                courseOk = courseFromCdeg(v, ms, NmeaGroup::Vtg);
            }
            if (!courseOk) {
                p = fld(3, n);
                u = fld(4, un);
                if (p != nullptr && un == 1u && u != nullptr && upper(u[0]) == 'M' &&
                    parseFixed(p, n, 2, false, v) && v >= 0 && v < 36000) {
                    courseFromCdeg(v, ms, NmeaGroup::Vtg);
                }
            }

            // ---- 速度: 取量化最细的单位(m/s > km/h > 节, 见文件头换算表) ----
            uint8_t bestRank = 0xFF;
            int32_t bestKn   = 0;
            int32_t bestKmh  = 0;
            int32_t bestMps  = 0;
            p = fld(5, n);
            u = fld(6, un);
            if (p != nullptr && un == 1u && u != nullptr && upper(u[0]) == 'N' &&
                parseFixed(p, n, 3, false, v) && v >= 0 && v <= 1000000) {
                if (2u < bestRank) { bestRank = 2u; bestKn = v; }
            }
            p = fld(7, n);
            u = fld(8, un);
            if (p != nullptr && un == 1u && u != nullptr && upper(u[0]) == 'K' &&
                parseFixed(p, n, 3, false, v) && v >= 0 && v <= 1000000) {
                if (1u < bestRank) { bestRank = 1u; bestKmh = v; }
            }
            p = fld(9, n);
            u = fld(10, un);
            if (p != nullptr && un == 1u && u != nullptr && upper(u[0]) == 'M' &&
                parseFixed(p, n, 3, false, v) && v >= 0 && v <= 1000000) {
                if (0u < bestRank) { bestRank = 0u; bestMps = v; }
            }
            switch (bestRank) {
            case 0u: speedFromMps(bestMps, ms, NmeaGroup::Vtg); break;
            case 1u: speedFromKmh(bestKmh, ms, NmeaGroup::Vtg); break;
            case 2u: speedFromKnotsScaled(bestKn, ms, NmeaGroup::Vtg); break;
            default: break;
            }
            return true; // 就算一个字段都没用上, 句子本身是合法的 VTG
        }

        //-------------------------------------------------------------------------------------------
        // $GxGSA: 0=名 1=M/A 2=定位类型1/2/3 3..14=PRN 15=PDOP 16=HDOP 17=VDOP
        //-------------------------------------------------------------------------------------------
        bool handleGsa(uint32_t ms)
        {
            if (_nf < 3u) {
                ++_stats.fieldErrors;
                return false;
            }
            uint8_t     n = 0;
            const char *p = fld(2, n);
            int32_t     v = 0;
            if (p != nullptr && n > 0u && parseFixed(p, n, 0, false, v) && v >= 1 && v <= 3) {
                _sol.gsaFix = (uint8_t)v;
                // 只记录, **不进判据**(见文件头"为什么不把 GSA 的定位类型混进 fix")
                _sol.group[(uint8_t)NmeaGroup::Gsa].valid = (v >= 2);
            }
            p = fld(16, n);
            if (p != nullptr && n > 0u && parseFixed(p, n, 1, false, v)) {
                if (v > 0 && v <= 255) {
                    put(kNfHdop, NmeaGroup::Gsa, ms, v);
                }
            }
            return true;
        }

        //-------------------------------------------------------------------------------------------
        // 速度/航向的小包装(把单位换算集中在一处, 见文件头换算表)
        //-------------------------------------------------------------------------------------------
        bool courseFromDeg(const char *p, uint8_t n, uint32_t ms, NmeaGroup g)
        {
            int32_t v = 0;
            if (p == nullptr || !parseFixed(p, n, 2, false, v)) { return false; }
            return courseFromCdeg(v, ms, g);
        }

        bool courseFromCdeg(int32_t cdeg, uint32_t ms, NmeaGroup g)
        {
            if (cdeg < 0 || cdeg >= 36000) { return false; } // 航向必须落在 [0, 360.00)
            put(kNfCourse, g, ms, cdeg);
            return true;
        }

        /** 节(×1e3) → cm/s: 1 节 = 1852 m/h = 51.4444… cm/s ⇒ mknot × 463/9000(四舍五入)。 */
        bool speedFromKnotsScaled(int32_t mknot, uint32_t ms, NmeaGroup g)
        {
            const int64_t cms = ((int64_t)mknot * 463 + 4500) / 9000;
            return putSpeedCms(cms, ms, g);
        }

        bool speedFromKnots(const char *p, uint8_t n, uint32_t ms, NmeaGroup g)
        {
            int32_t v = 0;
            if (p == nullptr || !parseFixed(p, n, 3, false, v) || v < 0 || v > 1000000) { return false; }
            return speedFromKnotsScaled(v, ms, g);
        }

        /** km/h(×1e3) → cm/s: 1 km/h = 27.7778… cm/s ⇒ mkmh/36(四舍五入)。 */
        bool speedFromKmh(int32_t mkmh, uint32_t ms, NmeaGroup g)
        {
            const int64_t cms = ((int64_t)mkmh + 18) / 36;
            return putSpeedCms(cms, ms, g);
        }

        /** m/s(×1e3) → cm/s */
        bool speedFromMps(int32_t mmps, uint32_t ms, NmeaGroup g)
        {
            const int64_t cms = ((int64_t)mmps + 5) / 10;
            return putSpeedCms(cms, ms, g);
        }

        bool putSpeedCms(int64_t cms, uint32_t ms, NmeaGroup g)
        {
            // > 100 m/s 只可能是字段错位/垃圾(多旋翼不可能), 而且 int16 的 cm/s 上限是 ±327 m/s:
            // 与其饱和成一个巨大但"看起来合法"的速度, 不如整条丢掉(判据/门限那一层再兜一次)。
            if (cms < 0 || cms > 10000) { return false; }
            put(kNfSpeed, g, ms, (int32_t)cms);
            return true;
        }

        //-------------------------------------------------------------------------------------------
        // 合成对外的那份解(每次接受一句 / 每次 refresh 调一次)
        //-------------------------------------------------------------------------------------------
        /** 判据依赖量中最旧那个的年龄。没收到过任何一个 ⇒ 0xFFFFFFFF(必然过期)。 */
        uint32_t dependencyAge(uint32_t nowMs) const
        {
            static const NmeaField kDep[4] = {kNfFix, kNfSats, kNfHdop, kNfSpeed};
            uint32_t oldest = 0;
            bool     any    = false;
            for (uint8_t i = 0; i < 4u; ++i) {
                const NmeaField f = kDep[i];
                if (_fsrc[f] == (uint8_t)NmeaGroup::None) { continue; }
                if (!any || (int32_t)(_fms[f] - oldest) < 0) {
                    oldest = _fms[f];
                    any    = true;
                }
            }
            if (!any) { return 0xFFFFFFFFu; }
            return (uint32_t)(nowMs - oldest);
        }

        /** 更旧的那个时间戳(抗回绕)。 */
        static uint32_t olderOf(uint32_t a, uint32_t b)
        {
            return ((int32_t)(a - b) <= 0) ? a : b;
        }

        void syncSolution(uint32_t nowMs)
        {
            GnssSolution &s = _sol;
            s.latE7      = _fv[kNfLat];
            s.lonE7      = _fv[kNfLon];
            s.altMslMm   = _fv[kNfAlt];
            s.gspeedCms  = (int16_t)_fv[kNfSpeed];
            s.courseCdeg = (uint16_t)_fv[kNfCourse];
            s.sats       = (uint8_t)_fv[kNfSats];
            s.fix        = (uint8_t)_fv[kNfFix];
            s.hdopX10    = (uint8_t)_fv[kNfHdop];
            for (uint8_t i = 0; i < (uint8_t)kNfCount; ++i) {
                s.fieldMs[i]  = _fms[i];
                s.fieldSrc[i] = _fsrc[i];
            }

            const bool rmcOk = (!s.rmcSeen) || s.rmcValid; // mask 关了 RMC 就不能要求它
            const bool fixOk = (s.fix >= (uint8_t)kGnssFixGps) && rmcOk;
            const uint32_t age = dependencyAge(nowMs);

            const bool hasPos = (_fsrc[kNfLat] != (uint8_t)NmeaGroup::None) &&
                                (_fsrc[kNfLon] != (uint8_t)NmeaGroup::None);
            // 位置/海拔的"新旧"只看它们自己: 判据(能不能进 EKF)只对速度那一组量成立, 位置只回传。
            s.posValid = hasPos && fixOk &&
                         ((uint32_t)(nowMs - olderOf(_fms[kNfLat], _fms[kNfLon])) <= kGnssStaleMs);
            s.altValid = (_fsrc[kNfAlt] != (uint8_t)NmeaGroup::None) && fixOk &&
                         ((uint32_t)(nowMs - _fms[kNfAlt]) <= kGnssStaleMs);
            // 速度: 先要求"收到过速度句"(前置条件, 见文件头), 再问那个纯函数
            s.speedValid = (_fsrc[kNfSpeed] != (uint8_t)NmeaGroup::None) &&
                           gnssSpeedObservationUsable(s.fix, s.sats, s.hdopX10, s.rmcValid, s.rmcSeen, age);
            s.courseValid = (_fsrc[kNfCourse] != (uint8_t)NmeaGroup::None) && s.speedValid &&
                            static_cast<uint32_t>(nowMs - _fms[kNfCourse]) <= kGnssStaleMs;
        }

        //-------------------------------------------------------------------------------------------
        // 成员
        //-------------------------------------------------------------------------------------------
        char     _buf[kMaxBodyLen + 1]; ///< 正文缓冲(+1 给 '\0'): 超长在写入**之前**就被拦住
        uint16_t _len;                  ///< 正文长度
        uint8_t  _state;                ///< RxState
        uint8_t  _ckHi;                 ///< 校验和高 4 位
        uint8_t  _mask;                 ///< kNmeaMask 位或
        uint8_t  _nf;                   ///< 本次切出的字段数
        Field    _f[kMaxFields];        ///< 字段表(指向 _buf 内部)

        int32_t  _fv[kNfCount];   ///< 逐字段当前值
        uint8_t  _fsrc[kNfCount]; ///< 逐字段最后写入者((uint8_t)NmeaGroup)
        uint32_t _fms[kNfCount];  ///< 逐字段最后写入时刻

        GnssSolution    _sol;
        GnssParserStats _stats;

        /** 待报异常的小环(FIFO)。深 4 的理由见 takeError(): 让"没定位"这种每帧都来的码占不满整环。 */
        static const uint8_t kPendingErrorCount = 4;
        ExceptionCode        _pending[kPendingErrorCount];
        uint8_t              _pendHead;  ///< 最老那条的下标
        uint8_t              _pendCount; ///< 环里现有条数

        static_assert(kMaxSentenceBytes == 82u, "NMEA 一句最长 82 字符");
        static_assert(kMaxBodyLen == kMaxSentenceBytes - 1u - 3u - 2u, "正文上限 = 82 - '$' - '*XX' - CRLF");
    };

    inline void GnssSolution::clear()
    {
        latE7      = 0;
        lonE7      = 0;
        altMslMm   = 0;
        gspeedCms  = 0;
        courseCdeg = 0;
        sats       = 0;
        fix        = 0;
        hdopX10    = 0;
        posValid   = false;
        altValid   = false;
        speedValid = false;
        courseValid = false;
        for (uint8_t i = 0; i < (uint8_t)NmeaGroup::Count; ++i) {
            group[i].ms    = 0;
            group[i].count = 0;
            group[i].seen  = false;
            group[i].valid = false;
        }
        latestGroup = NmeaGroup::None;
        latestMs    = 0;
        seq         = 0;
        rmcSeen     = false;
        rmcValid    = false;
        gsaFix      = 0;
        for (uint8_t i = 0; i < (uint8_t)kNfCount; ++i) {
            fieldMs[i]  = 0;
            fieldSrc[i] = (uint8_t)NmeaGroup::None;
        }
    }

} // namespace dlx

#pragma once
//===================================================================================================
// DL_LIB/VL53L1X/dlx_vl53l1x_config.h —— VL53L1X 的"寄存器地址 + 91 字节 ULD 配置数组 + 查表"
//
// 职责: 只放数据与常量, 不放逻辑。驱动(dlx_vl53l1x.hpp)从这里取寄存器号、ULD 数组、
//       距离模式/时间预算查表值, 以及本驱动自己引入的少量本地常量(见文件末尾"缺配置项")。
//
// 【数据来源与可信度】—— 上板前请务必读这一段
//   1. 数据手册 DS12385 Rev8 **完全没有寄存器级信息**, 它把读者推给 UM2356
//      ("full register details are not exposed. The customer should refer to the VL53L1X API user
//       manual (UM2356)"), 而 UM2356 在本次侦察中拿不到(镜像站 403)。所以：
//      **寄存器地址 → 取自 ST 官方驱动 STSW-IMG007 的 vl53l1x_register_map.h(经 Pololu 移植版转录);
//        91 字节数组 → 逐字节取自 ST 官方 ULD 的 VL51L1X_DEFAULT_CONFIGURATION[]。**
//   2. 本文件里的 91 个字节与 docs/_research/vl53l1x.md §6.2 的全文**逐字节一致**, 并且该文档里每个
//      字节自带的地址注释(0x2D … 0x87, 索引 i ↔ 0x2D+i)已被脚本校验无错位。自检见
//      test/vl53l1x_check/check.cpp: 那里硬编码了从 **文档**(而不是从本文件)算出的
//      求和/FNV-1a 校验和, 数组被抄错一个字节就会 [FAIL]。
//      - 原样数组:  sum=0x00000A53  FNV-1a=0x34205C8C
//      - 打过 2V8 补丁(0x2E/0x2F 置 bit0)后: sum=0x00000A55  FNV-1a=0x3C867E42
//   3. **本数组未经上板验证**, 也没有第二份独立来源(手册里没有这张表)。这是本驱动最大的风险点。
//
// 【两个必须知道的坑】(细节见 dlx_vl53l1x.hpp 头部)
//   a) 0x2E/0x2F 在 ST 的表里是 0x00 = 1V8 I/O 模式; 3.3V 供电的板子必须在**读模型 ID 之前**
//      把它们置 bit0=1 切到 2V8, 而且**下载 91 字节块时必须带上这个补丁**, 否则块下载会把刚设好的
//      2V8 位又清回 0(块覆盖 0x2D..0x87, 正好包含这两个寄存器)。
//   b) ST 的这张表默认是 **Long 模式 + 100ms 时间预算**(0x4B=0x0A, 0x60=0x0F, 0x5E/0x5F=0x01CC),
//      与本项目 cfg::sensor::range 的 Short + 33ms 相反 ⇒ 下载完必须显式改写模式与预算寄存器。
//===================================================================================================
#include <stdint.h>
#include "generated/flight_config_gen.hpp" // cfg::hw::range_ / cfg::sensor::range / cfg::RangeDistanceMode

namespace dlx
{
    //===============================================================================================
    // 一、I2C 与身份
    //===============================================================================================
    /** 7 位从机地址: 出厂默认 0x29(8 位写法 0x52/0x53), 取自配置 hw.range_.kAddr */
    constexpr uint16_t VL53L1X_IIC_ADDRESS = cfg::hw::range_::kAddr;

    /** 身份寄存器期望值(DS Table 8: 复位后未加载驱动时的读数) */
    constexpr uint8_t VL53L1X_MODEL_ID    = 0xEA; ///< 0x010F
    constexpr uint8_t VL53L1X_MODULE_TYPE = 0xCC; ///< 0x0110
    constexpr uint8_t VL53L1X_REVISION_ID = 0x10; ///< 0x0111 = 掩模版本(mask revision)

    //===============================================================================================
    // 二、寄存器地址(16 位索引; 总线上的顺序是 **高字节在前**, 见 dlx_vl53l1x.hpp 的寄存器约定)
    //===============================================================================================
    enum class VL53L1X_Reg : uint16_t
    {
        SoftReset                    = 0x0000, ///< 写 0x00→100us→0x01 = 固件软复位(本驱动不用, 有 XSHUT)
        I2cSlaveDeviceAddress        = 0x0001, ///< 7 位从机地址(本驱动不用, 总线上只有一个器件)
        VhvConfigTimeoutMacropLoopBound = 0x0008, ///< ULD 收尾写 0x09("two bounds VHV"); 见 .hpp 差异#6
        VhvConfigInit                = 0x000B, ///< ULD 收尾写 0x00("从上次温度开始"); 见 .hpp 差异#6
        AlgoCrosstalkCompensationPlaneOffsetKcps = 0x0016, ///< 串扰标定结果(7.9 格式)
        AlgoPartToPartRangeOffsetMm  = 0x001E, ///< 零偏标定, 值 = 毫米 × 4
        MmConfigOuterOffsetMm        = 0x0022, ///< 出厂零偏原始值(Polulu 用它 ×4 回填 0x001E)
        DssConfigTargetTotalRateMcps = 0x0024, ///< 动态 SPAD 选择的目标回波率
        PadI2cHvConfig               = 0x002D, ///< ULD 块首地址(0x2D..0x87 全部"不可修改"区)
        PadI2cHvExtsupConfig         = 0x002E, ///< **bit0=1 → I2C pad/上拉参考 AVDD(2V8 模式)**
        GpioHvPadCtrl                = 0x002F, ///< **bit0=1 → GPIO pad/上拉参考 AVDD(2V8 模式)**
        GpioHvMuxCtrl                = 0x0030, ///< bit4 = 中断极性(0=高有效, 1=低有效); bits[3:0] 必须 0x1
        GpioTioHvStatus              = 0x0031, ///< bit0 = 中断状态(与极性比对 ⇒ 新样本就绪)
        SystemInterruptConfigGpio    = 0x0046, ///< bits[2:0] 中断模式, 0x20 = 新样本就绪
        PhasecalConfigTimeoutMacrop  = 0x004B, ///< 相位标定超时(也是距离模式的"指纹": 0x14=Short, 0x0A=Long)
        RangeConfigTimeoutMacropA    = 0x005E, ///< 时间预算(MACROP A), 16 位
        RangeConfigVcselPeriodA      = 0x0060, ///< 距离模式: VCSEL 周期 A
        RangeConfigTimeoutMacropB    = 0x0061, ///< 时间预算(MACROP B), 16 位
        RangeConfigVcselPeriodB      = 0x0063, ///< 距离模式: VCSEL 周期 B
        RangeConfigSigmaThresh       = 0x0064, ///< σ 门限(14.2 格式), 默认 360
        RangeConfigMinCountRateRtnLimitMcps = 0x0066, ///< 最小回波率门限(9.7 格式), ULD 表里 128
        RangeConfigValidPhaseLow     = 0x0068, ///< 有效相位下限(三种模式都是 0x08, 本驱动不改)
        RangeConfigValidPhaseHigh    = 0x0069, ///< 有效相位上限(随距离模式而变)
        SystemIntermeasurementPeriod = 0x006C, ///< 连续测距间隔, **32 位, MSB 在前**
        SdConfigWoiSd0               = 0x0078, ///< 距离模式: WOI SD0
        SdConfigWoiSd1               = 0x0079, ///< 距离模式: WOI SD1
        SdConfigInitialPhaseSd0      = 0x007A, ///< 距离模式: 初始相位 SD0
        SdConfigInitialPhaseSd1      = 0x007B, ///< 距离模式: 初始相位 SD1
        SystemSequenceConfig         = 0x0081, ///< 测距序列开关(ULD 表里 0x9B, 见 .hpp 差异#4)
        SystemGroupedParameterHold   = 0x0082, ///< **必须为 0**, 否则一次测量都跑不完
        SystemInterruptClear         = 0x0086, ///< 写 0x01 = 清测距中断(**每读一次都必须清**)
        SystemModeStart              = 0x0087, ///< 0x40=连续/自主启动, 0x10=单次, 0x00=停止, 0x80=中止
        ResultInterruptStatus        = 0x0088, ///< Pololu 用它判就绪(本驱动用 0x0031, 与 ST ULD 一致)
        ResultRangeStatus            = 0x0089, ///< **低 5 位 = 器件测距状态码**(17 字节结果块首字节)
        ResultRangeMm                = 0x0096, ///< **测距结果 mm, 16 位**(结果块偏移 +13)
        ResultOscCalibrateVal        = 0x00DE, ///< 振荡器标定值(低 10 位有效), 算连续测距间隔要用
        FirmwareSystemStatus         = 0x00E5, ///< **bit0 = 1 → 启动完成**(首次 I2C 访问前轮询它)
        IdentificationModelId        = 0x010F, ///< 模型 ID, 必须 0xEA
        IdentificationModuleType     = 0x0110, ///< 模块类型, 典型 0xCC
        IdentificationRevisionId     = 0x0111, ///< 掩模版本, 典型 0x10
    };

    /** 17 字节结果块(RESULT__RANGE_STATUS..): 一次突发读, 保证状态码与距离来自同一帧 */
    constexpr uint16_t VL53L1X_RESULT_BLOCK_BYTES = 17;
    constexpr uint16_t VL53L1X_RESULT_RANGE_OFFSET = 13; ///< 0x0089+13 = 0x0096 = 距离 mm(高字节在前)
    constexpr uint16_t VL53L1X_RESULT_PEAK_OFFSET  = 15; ///< 0x0089+15 = 0x0098 = 峰值回波率(串扰修正后)
    constexpr uint16_t VL53L1X_RESULT_AMBIENT_OFFSET = 7; ///< 0x0089+7 = 0x0090 = 环境光计数率

    /** 结果块里的状态码(低 5 位) → 含义。表来源: 侦察文档 §7(ST ULD 的 GetRangeStatus 映射) */
    enum class VL53L1X_RangeStatus : uint8_t
    {
        Valid          = 9,  ///< 量程有效 —— **唯一被本驱动接受的状态**
        SigmaFail      = 6,  ///< σ 超过 0x0064 门限(回波质量差)
        SignalFail     = 4,  ///< 回波率低于 0x0066 门限 / MSRC 无目标
        MinRangeClip   = 8,  ///< 量程有效但被最小距离裁剪(随后会被 kMinValidMm 拦掉)
        OutOfBounds    = 5,  ///< 相位/量程检查越界
        HardwareFail   = 3,  ///< 硬件故障(API 还把 1/2/17 归到这一类)
        WrapTargetFail = 7,  ///< 相位一致性(缠绕)失败
        XtalkSignalFail= 12, ///< 串扰(量程忽略门限)导致的信号失败
        MinRangeFail   = 13, ///< USERROICLIP 最小距离失败
        SyncInt        = 18, ///< 同步中断(GPH stream count 0 ready)
    };

    /** 状态码 → 短名(给日志/遥测用, 纯 ASCII, 不占 flash 字符串表之外的东西) */
    constexpr const char *vl53l1xStatusName(uint8_t raw)
    {
        return raw == 9 ? "valid"
               : raw == 6 ? "sigma_fail"
               : raw == 4 ? "signal_fail"
               : raw == 8 ? "min_range_clip"
               : raw == 5 ? "out_of_bounds"
               : raw == 3 ? "hardware_fail"
               : raw == 7 ? "wrap_target_fail"
               : raw == 12 ? "xtalk_signal_fail"
               : raw == 13 ? "min_range_fail"
               : raw == 18 ? "sync_int"
               : (raw == 22 || raw == 23) ? "unknown_22_23" : "unknown";
    }

    //===============================================================================================
    // 三、91 字节 ULD 配置块(0x2D..0x87)
    //    逐字节取自 ST ULD 的 VL51L1X_DEFAULT_CONFIGURATION[](见文件头"数据来源");
    //    每行注释里的地址就是该字节在器件里的寄存器地址, 索引 i ↔ 0x2D+i。
    //    **不要手改这张表**: 3.3V 板上唯二要改的 0x2E/0x2F 由驱动按 cfg::sensor::range::kIOMode2V8
    //    在下发前打补丁(见 loadUld), 改表反而会让"表=ST 原始表"这个可核对性丢掉。
    //===============================================================================================
    constexpr uint16_t VL53L1X_ULD_START = 0x2D;
    constexpr uint16_t VL53L1X_ULD_END   = 0x87;
    constexpr uint16_t VL53L1X_ULD_SIZE  = 91;
    static_assert(VL53L1X_ULD_END - VL53L1X_ULD_START + 1 == VL53L1X_ULD_SIZE,
                  "ULD 块长度与 0x2D..0x87 对不上(91 = 0x87-0x2D+1)");

    constexpr uint8_t VL51L1X_DEFAULT_CONFIGURATION[VL53L1X_ULD_SIZE] = {
        0x00, /* 0x2D: bit2/bit5=1 → fast-plus(1MHz I2C); 器件上限 400kHz, 保持 0 */
        0x00, /* 0x2E: bit0=1 → I2C 上拉参考 AVDD(2V8); **驱动会在下发前补成 1** */
        0x00, /* 0x2F: bit0=1 → GPIO 上拉参考 AVDD(2V8); **驱动会在下发前补成 1** */
        0x01, /* 0x30: bit4=0 高有效 / 1 低有效; bits[3:0] 必须 0x1 */
        0x02, /* 0x31: 就绪位, 与极性相关(CheckForDataReady 用) */
        0x00, /* 0x32: 不可修改 */
        0x02, /* 0x33: 不可修改 */
        0x08, /* 0x34: 不可修改 */
        0x00, /* 0x35: 不可修改 */
        0x08, /* 0x36: σ 估计器有效脉宽(ns), tuning 默认 8 */
        0x10, /* 0x37: σ 估计器有效环境宽度(ns), tuning 默认 16 */
        0x01, /* 0x38: 不可修改 */
        0x01, /* 0x39: 串扰补偿有效高度(mm) */
        0x00, /* 0x3A: 不可修改 */
        0x00, /* 0x3B: 不可修改 */
        0x00, /* 0x3C: 不可修改 */
        0x00, /* 0x3D: 不可修改 */
        0xFF, /* 0x3E: 量程忽略有效高度(mm) */
        0x00, /* 0x3F: ALGO__RANGE_MIN_CLIP, tuning 默认 0 */
        0x0F, /* 0x40: 表原值 0x0F; Pololu 在此写 ALGO__CONSISTENCY_CHECK__TOLERANCE=2(tuning 默认) */
        0x00, /* 0x41: 不可修改 */
        0x00, /* 0x42: 不可修改 */
        0x00, /* 0x43: 不可修改 */
        0x00, /* 0x44: 不可修改 */
        0x00, /* 0x45: 不可修改 */
        0x20, /* 0x46: 中断配置: 0x20 = 新样本就绪(bits[2:0]=窗口模式, 本驱动不用) */
        0x0B, /* 0x47: 不可修改(手动相位标定起点) */
        0x00, /* 0x48: 不可修改 */
        0x00, /* 0x49: 不可修改 */
        0x02, /* 0x4A: 不可修改 */
        0x0A, /* 0x4B: 相位标定超时(**0x0A = Long 模式的指纹**, Short 要改 0x14) */
        0x21, /* 0x4C: 不可修改 */
        0x00, /* 0x4D: 相位标定 override */
        0x00, /* 0x4E: 不可修改 */
        0x05, /* 0x4F: DSS ROI 模式控制(表原值 0x05; Pololu 用 2 = REQUESTED_EFFECTIVE_SPADS) */
        0x00, /* 0x50: 不可修改 */
        0x00, /* 0x51: 不可修改 */
        0x00, /* 0x52: 不可修改 */
        0x00, /* 0x53: 不可修改 */
        0xC8, /* 0x54: 动态 SPAD 目标(200 << 8 的高字节) */
        0x00, /* 0x55: 不可修改 */
        0x00, /* 0x56: 不可修改 */
        0x38, /* 0x57: 孔径衰减 */
        0xFF, /* 0x58: 不可修改 */
        0x01, /* 0x59: 不可修改 */
        0x00, /* 0x5A: MM 超时 A 高字节(低功耗自动模式下 MM 步进被禁用) */
        0x08, /* 0x5B: MM 超时 A 低字节 */
        0x00, /* 0x5C: MM 超时 B 高字节 */
        0x00, /* 0x5D: MM 超时 B 低字节 */
        0x01, /* 0x5E: 时间预算 A 高字节(**表里是 Long+100ms 的 0x01CC**) */
        0xCC, /* 0x5F: 时间预算 A 低字节 */
        0x0F, /* 0x60: VCSEL 周期 A(**0x0F = Long**) */
        0x01, /* 0x61: 时间预算 B 高字节(表里 0x01F1, 与固定表的 0x01EA 差一点, 见 .hpp 差异#5) */
        0xF1, /* 0x62: 时间预算 B 低字节 */
        0x0D, /* 0x63: VCSEL 周期 B(**0x0D = Long**) */
        0x01, /* 0x64: σ 门限高字节(默认 360 = 0x0168) */
        0x68, /* 0x65: σ 门限低字节 */
        0x00, /* 0x66: 最小回波率高字节(默认 128 = 0x0080; Pololu 用 192) */
        0x80, /* 0x67: 最小回波率低字节 */
        0x08, /* 0x68: 有效相位下限(三种模式都是 0x08) */
        0xB8, /* 0x69: 有效相位上限(**0xB8 = Long**) */
        0x00, /* 0x6A: 不可修改 */
        0x00, /* 0x6B: 不可修改 */
        0x00, /* 0x6C: 连续测距间隔 MSB(32 位; 驱动按 kInterMeasMs 重写) */
        0x00, /* 0x6D: 连续测距间隔 */
        0x0F, /* 0x6E: 连续测距间隔 */
        0x89, /* 0x6F: 连续测距间隔 LSB(表里 0x00000F89) */
        0x00, /* 0x70: 不可修改 */
        0x00, /* 0x71: 分组参数保持 0 */
        0x00, /* 0x72: 距离阈值高字节 */
        0x00, /* 0x73: 距离阈值 */
        0x00, /* 0x74: 距离阈值低字节 */
        0x00, /* 0x75: 距离阈值 */
        0x00, /* 0x76: 不可修改 */
        0x01, /* 0x77: SEED 配置, tuning 默认 1 */
        0x0F, /* 0x78: WOI SD0(**0x0F = Long**) */
        0x0D, /* 0x79: WOI SD1(**0x0D = Long**) */
        0x0E, /* 0x7A: 初始相位 SD0(14 = Long) */
        0x0E, /* 0x7B: 初始相位 SD1(14 = Long) */
        0x00, /* 0x7C: 分组参数保持 1 */
        0x00, /* 0x7D: 不可修改 */
        0x02, /* 0x7E: 量化器, tuning 默认 2 */
        0xC7, /* 0x7F: ROI 中心 SPAD */
        0xFF, /* 0x80: ROI XY 尺寸(0xFF = 16x16 全视场) */
        0x9B, /* 0x81: 测距序列(0x9B = ST 表原值; API 的 preset 写 0x8B, 见 .hpp 差异#4) */
        0x00, /* 0x82: 分组参数保持, **必须为 0** */
        0x00, /* 0x83: 不可修改 */
        0x00, /* 0x84: 不可修改 */
        0x01, /* 0x85: 不可修改 */
        0x00, /* 0x86: 清中断(初值 0) */
        0x00  /* 0x87: 启动测距(0x00=不启动; 驱动最后单独写 0x40) */
    };

    /** 需要在 2V8 模式下补齐 bit0 的两个字节在数组里的下标(编译期算出来, 不写死 1/2) */
    constexpr uint16_t VL53L1X_ULD_INDEX_2E =
        static_cast<uint16_t>(VL53L1X_Reg::PadI2cHvExtsupConfig) - VL53L1X_ULD_START;
    constexpr uint16_t VL53L1X_ULD_INDEX_2F =
        static_cast<uint16_t>(VL53L1X_Reg::GpioHvPadCtrl) - VL53L1X_ULD_START;
    static_assert(VL53L1X_ULD_INDEX_2E == 1 && VL53L1X_ULD_INDEX_2F == 2,
                  "0x2E/0x2F 必须落在 ULD 块的第 1/2 个字节上(0x2D 起址), 表被改过了");

    //===============================================================================================
    // 四、距离模式 + 时间预算查表
    //    ST ULD 的"固定表"路径只接受离散的时间预算; 表(a) 见侦察文档 §4。
    //    注意: ULD 的 shortcut **只实现 Short 与 Long**, 没有独立的 Medium 表 ⇒ Medium 复用 Long 表
    //    (它的距离模式寄存器块仍然是 Medium 自己的值)。
    //===============================================================================================
    struct VL53L1X_ModeRegs {
        uint8_t  vcselPeriodA;   ///< 0x0060
        uint8_t  vcselPeriodB;   ///< 0x0063
        uint8_t  validPhaseHigh; ///< 0x0069
        uint8_t  woiSd0;         ///< 0x0078
        uint8_t  woiSd1;         ///< 0x0079
        uint8_t  initialPhase;   ///< 0x007A/0x007B
        uint8_t  phasecalTimeout;///< 0x004B(0x14=Short, 0x0A=Medium/Long)
    };

    /** 距离模式 → 模式寄存器块(侦察 §4 的寄存器表) */
    constexpr VL53L1X_ModeRegs vl53l1xModeRegs(cfg::RangeDistanceMode mode)
    {
        return mode == cfg::RangeDistanceMode::Short
                   ? VL53L1X_ModeRegs{ 0x07, 0x05, 0x38, 0x07, 0x05, 6, 0x14 }
                   : mode == cfg::RangeDistanceMode::Medium
                         ? VL53L1X_ModeRegs{ 0x0B, 0x09, 0x78, 0x0B, 0x09, 10, 0x0A }
                         : VL53L1X_ModeRegs{ 0x0F, 0x0D, 0xB8, 0x0F, 0x0D, 14, 0x0A };
    }

    struct VL53L1X_TbEntry {
        uint16_t ms;     ///< 时间预算 [ms]
        uint16_t macroA; ///< 0x005E
        uint16_t macroB; ///< 0x0061
    };

    constexpr uint16_t VL53L1X_TB_TABLE_SIZE = 6;

    /** Short 模式的时间预算表(20ms 是 Short 的下限; 15ms 那档低于手册 20ms 下限, 不收) */
    constexpr VL53L1X_TbEntry VL53L1X_TB_SHORT[VL53L1X_TB_TABLE_SIZE] = {
        {  20, 0x0051, 0x006E },
        {  33, 0x00D6, 0x006E },
        {  50, 0x01AE, 0x01E8 },
        { 100, 0x02E1, 0x0388 },
        { 200, 0x03E1, 0x0496 },
        { 500, 0x0591, 0x05C1 },
    };

    /** Medium/Long 共用的时间预算表(ST ULD 的 long 表) */
    constexpr VL53L1X_TbEntry VL53L1X_TB_LONG[VL53L1X_TB_TABLE_SIZE] = {
        {  20, 0x001E, 0x0022 },
        {  33, 0x0060, 0x006E },
        {  50, 0x00AD, 0x00C6 },
        { 100, 0x01CC, 0x01EA },
        { 200, 0x02D9, 0x02F8 },
        { 500, 0x048F, 0x04A4 },
    };

    /** 第 i 档(调用方保证 i < VL53L1X_TB_TABLE_SIZE) */
    constexpr VL53L1X_TbEntry vl53l1xTbEntryAt(cfg::RangeDistanceMode mode, uint16_t i)
    {
        return mode == cfg::RangeDistanceMode::Short ? VL53L1X_TB_SHORT[i] : VL53L1X_TB_LONG[i];
    }

    /** 表里有没有 (mode, ms) 这一档; 给下面的 static_assert 用 */
    constexpr bool vl53l1xTbExists(cfg::RangeDistanceMode mode, uint16_t ms, uint16_t i = 0)
    {
        return i >= VL53L1X_TB_TABLE_SIZE
                   ? false
                   : (vl53l1xTbEntryAt(mode, i).ms == ms
                          ? true
                          : vl53l1xTbExists(mode, ms, static_cast<uint16_t>(i + 1)));
    }

    /** 结果换算: mm = (raw × 2011 + 0x400) / 0x800, ST tuning 默认的 ~98.2% 增益修正 */
    constexpr uint16_t VL53L1X_GAIN_NUM   = 2011; ///< 分子(≈2048×0.982)
    constexpr uint16_t VL53L1X_GAIN_DEN   = 2048; ///< 分母(0x800)
    constexpr uint16_t VL53L1X_GAIN_ROUND = 0x400;///< 四舍五入项

    //===============================================================================================
    // 五、本地常量(**配置里没有对应项** —— 报告里已列为缺项, 想改就从这里改)
    //===============================================================================================
    /**
     * XSHUT 释放后的启动等待窗口 = 轮询次数 × 间隔。
     * 依据: 手册只说 "boot duration 1.2ms maximum"(DS §3.6), 没给最小值; 这段时间里器件不响应
     *       I2C(NACK), 所以只能轮询 0x00E5 bit0。20 × 1ms = 20ms ≈ 1.2ms 上限的 16 倍余量,
     *       够覆盖电源爬升慢的情况; 再长就说明器件根本没起来, 不如早点报 RANGE_BOOT_TIMEOUT。
     */
    constexpr uint16_t VL53L1X_BOOT_POLL_ATTEMPTS = 20;
    constexpr uint16_t VL53L1X_BOOT_POLL_INTERVAL_MS = 1;

    /**
     * 连续测距间隔的 ×1.075 系数。
     * 依据: ST ULD 写 0x006C 时用 (OSC_CALIBRATE_VAL & 0x3FF) × InterMeasMs × 1.075
     *       (读回时除以 ClockPLL × 1.065); Pololu 不用这个系数(direct 乘积)。本驱动跟 ST。
     */
    constexpr uint16_t VL53L1X_INTERMEAS_NUM = 1075;
    constexpr uint16_t VL53L1X_INTERMEAS_DEN = 1000;

    /** 状态码无效时占位(结果块没读出来时调用方看到的 rangeStatus) */
    constexpr uint8_t VL53L1X_STATUS_NONE = 0xFF;

    //===============================================================================================
    // 六、编译期守卫(把"配置 ↔ 寄存器表"的矛盾挡在编译期, 别等上板)
    //===============================================================================================
    static_assert(cfg::sensor::range::kTimingBudgetMs >= 20 && cfg::sensor::range::kTimingBudgetMs <= 1000,
                  "sensor.range.kTimingBudgetMs 超出手册允许的 20..1000ms");
    static_assert(vl53l1xTbExists(cfg::sensor::range::kDistanceMode, cfg::sensor::range::kTimingBudgetMs),
                  "sensor.range.kTimingBudgetMs 不在该距离模式的固定表里(ST ULD 只接受离散值): "
                  "Short = 20/33/50/100/200/500ms, Medium/Long = 20/33/50/100/200/500ms");
    static_assert(cfg::sensor::range::kInterMeasMs >= cfg::sensor::range::kTimingBudgetMs,
                  "sensor.range.kInterMeasMs 必须 ≥ kTimingBudgetMs(手册: 连续测距间隔不得短于时间预算)");
    static_assert(cfg::sensor::range::kMinValidMm > 0 &&
                      cfg::sensor::range::kMaxValidMm > cfg::sensor::range::kMinValidMm,
                  "sensor.range 的有效距离窗口非法: 必须 0 < kMinValidMm < kMaxValidMm(0 = 无目标, 不能当贴地)");
    static_assert(cfg::sensor::range::kFailLimit >= 1,
                  "sensor.range.kFailLimit = 0 会让第一次失败就报 DATA_STALE");

} // namespace dlx

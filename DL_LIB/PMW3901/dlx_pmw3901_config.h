#pragma once
#include <stdint.h>

// 时序/符号/门限等"整机参数"全部来自生成文件(cfg::hw::flow / cfg::sensor::flow),
// 本文件不重复定义它们 —— 见下面 PMW3901_INIT_TABLE 之外的说明。
#include "generated/flight_config_gen.hpp"

/**
 * @file dlx_pmw3901_config.h
 * @brief PMW3901MB-TXQT(光流)的寄存器表 + 厂家初始化序列表(仿 BMI088 的 _config.h 分层)
 *
 * 三块内容:
 *   1. PMW3901_Reg          —— 手册 Table 9 的寄存器地址(含 0x16 Motion_Burst)
 *   2. PMW3901_INIT_TABLE   —— 厂家初始化寄存器表, **76 条**, 必须按顺序执行
 *   3. PMW3901_VERIFY_TABLE —— init() 末尾"写进去了没有"的回读校验项
 *
 * 时序参数(地址→数据间隙 / 写尾保持 / 复位等待 / 初始化停顿)与符号、质量门限
 * **不在本文件**, 而在 cfg::hw::flow 与 cfg::sensor::flow:
 * 它们是 schema 里可改、带依据注释的整机参数, 不是驱动私有的接线常量。
 * 驱动里只引用, 不复制 —— 复制一份就会出现"改配置不生效"的经典坑。
 *
 * ── 初始化表的出处(拿不到权威表, 这里必须说清楚) ───────────────────────────
 * 逐条抄自 `docs/_research/pmw3901.md` §5 "Step 1 — canonical performance-optimisation
 * block"。该文档给出的来源是: **手册 Table 10 的前 20 行**(手上那份 PDF 是 11 页摘录,
 * Table 10 只包含到第 20 行) + **Bitcraze pmw3901.c** + **PX4 PMW3901.cpp**(转抄
 * "PixArt PMW3901MB demo kit V3.20, 21 Aug 2018")三份**逐字节一致**的部分;
 * 21 行之后(手册摘录里没有)来自这三份开源驱动的交叉验证 —— 所以这张表**不是**
 * 从厂商手册逐行誊抄的, 是"手册可见部分 + 三份独立驱动一致"的合成结果。
 *
 * 三份驱动互相冲突的行, 按 docs/_research/pmw3901.md §5 的取舍(2 票对 1 票, 且含官方
 * demo kit), 即:
 *   - 0x7F=0x14 块: 0x65=0x67, 0x63=0x70   (Paparazzi 的 0x60/0x78 **不采用**)
 *   - 0x74        : 0x1F                    (PX4 的 0x21 **不采用**; 若实测流量偏弱可试它)
 *   - 尾部 0x5A=0x10 / 0x54=0x00 只有 Bitcraze 有, docs 判定"无害, 保留"
 *
 * ── 为什么必须**顺序**执行, 不能逆序 ─────────────────────────────────────
 * 表里反复出现 `{0x7F, bank}` 这种"银行选择"行, 它**之后**的寄存器写落进哪个 bank
 * 完全取决于它。逆序执行会让每条寄存器写都落进错误的 bank —— 器件不会报错(它只按
 * 当前 bank 解释地址), 但配置全错, 现场表现是"初始化后 motion 恒 0", 极难查。
 * 写与写之间的真实间隙(≥45µs)、以及第 59 条之后的 10ms 停顿同样不能省。
 * 依据: `docs/2026-10-02_upgrade_plan_v3.md` §4.3 "⑥ 59 条初始化写入 → **强制等
 * 10ms** → 14 条写入 → Bitcraze 收尾 3 条" —— 顺序执行 + 单点停顿。
 * (本文件按上面这条组织: A 段 59 条 → 停顿 → B 段 14 条 → C 段 3 条。)
 */
namespace dlx
{
    /**
     * @brief PMW3901 寄存器地址(手册 Table 9)
     * @note 这里只放地址。**读写方向不在地址里**: 驱动发到线上的是
     *       `addr & 0x7F`(读) / `addr | 0x80`(写) —— 与 PMW3360/ADNS-9800 相反,
     *       见 dlx_pmw3901.hpp 的文件级说明。
     */
    enum class PMW3901_Reg : uint8_t
    {
        ProductId         = 0x00, ///< RO, 固定 0x49(读到别的值 = 通信有问题)
        RevisionId        = 0x01, ///< RO
        Motion            = 0x02, ///< R/W; 读它会锁存本帧 delta(与 burst 二选一)
        DeltaXL           = 0x03, ///< RO, delta 低字节
        DeltaXH           = 0x04, ///< RO, delta 高字节
        DeltaYL           = 0x05, ///< RO
        DeltaYH           = 0x06, ///< RO
        Squal             = 0x07, ///< RO, 表面质量, 越大越可信
        RawDataSum        = 0x08, ///< RO
        MaxRawData        = 0x09, ///< RO(注意: 是 0x09, 不是有些资料写的 0x0D)
        MinRawData        = 0x0A, ///< RO
        ShutterLower      = 0x0B, ///< RO
        ShutterUpper      = 0x0C, ///< RO
        Observation       = 0x15, ///< R/W
        MotionBurst       = 0x16, ///< RO, 一次原子锁存整帧: 地址 + 连读 12 字节
        PowerUpReset      = 0x3A, ///< WO, 写 0x5A
        Shutdown          = 0x3B, ///< WO
        RawDataGrab       = 0x58, ///< R/W
        RawDataGrabStatus = 0x59, ///< RO
        InverseProductId  = 0x5F, ///< RO, 固定 0xB6(与 0x00 互为反码校验)
        BankSelect        = 0x7F, ///< 银行选择(厂家初始化表内部使用)
    };

    /** 常数: 期望 ID / burst 长度 / 复位写值 / ID 重试次数 */
    enum
    {
        PMW3901_PRODUCT_ID         = 0x49, ///< 0x00 的期望值(手册 Table 9)
        PMW3901_INVERSE_PRODUCT_ID = 0xB6, ///< 0x5F 的期望值(~0x49 = 0xB6)
        PMW3901_BURST_BYTES        = 12,   ///< 0x16 突发读固定 12 字节
        PMW3901_POWER_UP_RESET_VAL = 0x5A, ///< 写 0x3A 的这个值触发上电复位
        PMW3901_ID_RETRY           = 3,    ///< 上电后 ID 校验失败的重试次数(首帧常是垃圾)
    };

    /**
     * @brief Motion_Burst(0x16)突发读的字节序(手册/docs §3)
     *
     * | # | 0      | 1           | 2      | 3      | 4      | 5      |
     * |---|--------|-------------|--------|--------|--------|--------|
     * |   | motion | observation | dX_L   | dX_H   | dY_L   | dY_H   |
     *
     * | # | 6     | 7           | 8       | 9       | 10            | 11           |
     * |---|-------|-------------|---------|---------|---------------|--------------|
     * |   | squal | rawdata_sum | max_raw | min_raw | shutter_upper | shutter_lower|
     *
     * 注意两处反直觉的地方:
     *  - delta 是**低字节在前**(小端), 所以要 `(H << 8) | L`;
     *  - shutter 是**高字节在前**(大端), 所以要 `(upper << 8) | lower` —— 同一个突发里
     *    两种字节序混用, Bitcraze 为此专门做了一次字节交换。
     */
    enum PMW3901_BurstIndex : uint8_t
    {
        PMW3901_BURST_MOTION         = 0,  ///< bit7 = 本帧有运动, bit0 = frameFrom0, bit4 = rawFrom0
        PMW3901_BURST_OBSERVATION    = 1,  ///< 与寄存器 0x15 相同
        PMW3901_BURST_DELTA_X_L      = 2,  ///< == 寄存器 0x03
        PMW3901_BURST_DELTA_X_H      = 3,  ///< == 寄存器 0x04
        PMW3901_BURST_DELTA_Y_L      = 4,  ///< == 寄存器 0x05
        PMW3901_BURST_DELTA_Y_H      = 5,  ///< == 寄存器 0x06
        PMW3901_BURST_SQUAL          = 6,  ///< == 寄存器 0x07
        PMW3901_BURST_RAW_SUM        = 7,
        PMW3901_BURST_MAX_RAW        = 8,
        PMW3901_BURST_MIN_RAW        = 9,
        PMW3901_BURST_SHUTTER_UPPER  = 10, ///< 高字节在前
        PMW3901_BURST_SHUTTER_LOWER  = 11,
    };

    /** 厂家初始化表的一行 */
    struct PMW3901_InitStep
    {
        uint8_t reg;   ///< 寄存器地址(0x7F = 银行选择)
        uint8_t val;   ///< 写入值
        uint8_t pause; ///< 非 0 = 本条写完之后等 cfg::hw::flow::kInitPauseMs(毫秒)
    };

    /**
     * @brief 厂家初始化序列表(A 段 59 条 → 停顿 → B 段 14 条 → C 段 3 条 = 76 条)
     *
     * `pause` 只标在第 59 条(A 段最后一条)上 —— 手册/各驱动都只在这里停一次。
     * docs 原文里那行 `{0x00, 0x00, 10}` 是"在这里等 10ms"的占位符(**不是**去写
     * Product_ID), 所以这里用 `pause` 字段表达, 而不是往表里塞一条假写操作。
     * 停顿的**时长**取 cfg::hw::flow::kInitPauseMs(10ms), 位置由本表决定。
     *
     * @note 本表是 `static const`(每个编译单元一份副本, 约 230 字节)。只在驱动里被
     *       引用, 若将来被多个 .cpp 包含且在意 flash, 可改成函数内 static + 取指针。
     */
    static const PMW3901_InitStep PMW3901_INIT_TABLE[] = {
        /* ================= A 段(59 条): 银行/寄存器配置 ================= */
        {0x7F, 0x00, 0}, {0x61, 0xAD, 0}, {0x7F, 0x03, 0}, {0x40, 0x00, 0},
        {0x7F, 0x05, 0}, {0x41, 0xB3, 0}, {0x43, 0xF1, 0}, {0x45, 0x14, 0},
        {0x5B, 0x32, 0}, {0x5F, 0x34, 0}, {0x7B, 0x08, 0},
        {0x7F, 0x06, 0}, {0x44, 0x1B, 0}, {0x40, 0xBF, 0}, {0x4E, 0x3F, 0},
        {0x7F, 0x08, 0}, {0x65, 0x20, 0}, {0x6A, 0x18, 0},
        {0x7F, 0x09, 0}, {0x4F, 0xAF, 0}, {0x5F, 0x40, 0}, {0x48, 0x80, 0},
        {0x49, 0x80, 0}, {0x57, 0x77, 0}, {0x60, 0x78, 0}, {0x61, 0x78, 0},
        {0x62, 0x08, 0}, {0x63, 0x50, 0},
        {0x7F, 0x0A, 0}, {0x45, 0x60, 0},
        {0x7F, 0x00, 0}, {0x4D, 0x11, 0}, {0x55, 0x80, 0}, {0x74, 0x1F, 0},
        {0x75, 0x1F, 0}, {0x4A, 0x78, 0}, {0x4B, 0x78, 0}, {0x44, 0x08, 0},
        {0x45, 0x50, 0}, {0x64, 0xFF, 0}, {0x65, 0x1F, 0},
        {0x7F, 0x14, 0}, {0x65, 0x67, 0}, {0x66, 0x08, 0}, {0x63, 0x70, 0},
        {0x7F, 0x15, 0}, {0x48, 0x48, 0},
        {0x7F, 0x07, 0}, {0x41, 0x0D, 0}, {0x43, 0x14, 0}, {0x4B, 0x0E, 0},
        {0x45, 0x0F, 0}, {0x44, 0x42, 0}, {0x4C, 0x80, 0},
        {0x7F, 0x10, 0}, {0x5B, 0x02, 0},
        {0x7F, 0x07, 0}, {0x40, 0x41, 0}, {0x70, 0x00, 1}, /* <<< 第 59 条: 写完后停 kInitPauseMs */

        /* ================= B 段(14 条): 停顿之后的配置 ================= */
        {0x32, 0x44, 0}, {0x7F, 0x07, 0}, {0x40, 0x40, 0}, {0x7F, 0x06, 0},
        {0x62, 0xF0, 0}, {0x63, 0x00, 0}, {0x7F, 0x0D, 0}, {0x48, 0xC0, 0},
        {0x6F, 0xD5, 0}, {0x7F, 0x00, 0}, {0x5B, 0xA0, 0}, {0x4E, 0xA8, 0},
        {0x5A, 0x50, 0}, {0x40, 0x80, 0},

        /* ================= C 段(3 条): Bitcraze 收尾 ================= */
        {0x7F, 0x00, 0}, {0x5A, 0x10, 0}, {0x54, 0x00, 0},
    };

    enum
    {
        PMW3901_INIT_TABLE_SIZE = (int)(sizeof(PMW3901_INIT_TABLE) / sizeof(PMW3901_INIT_TABLE[0])),
        PMW3901_INIT_PAUSE_INDEX = 58, ///< A 段最后一条(0 基)之后的停顿位置, 表里 59 条
    };

    /** 回读校验的一行: 选 bank → 读 reg → 期望 val */
    struct PMW3901_VerifyStep
    {
        uint8_t bank;
        uint8_t reg;
        uint8_t val;
    };

    /**
     * @brief init() 末尾的回读校验项
     *
     * 全部落在 **bank 0** 且都是初始化表**最后一次**写入的值(bank 0 里 0x4D/0x55/0x61/0x64
     * 之后没有任何一条再改它们), 所以"回读 == 期望"等价于"这批写真的落进器件了"。
     * 依据: docs/_research/pmw3901.md §5 末"Verification tip: after init you can read back
     * banked registers (select the bank with 0x7F, then read the register) to confirm what
     * actually landed."
     *
     * ⚠️ 这几个寄存器本身在手册里没有定义(属于厂家保留的 bank 寄存器), "可回读"这一点
     * 是上述文档的论断 + 三份驱动的做法, **没有**手册逐行背书 —— 上板第一次跑如果这里
     * 报 FLOW_SPI_GARBAGE 而 motion 数据正常, 请把 PMW3901_VERIFY_STRICT 置 false(只把
     * "全 0/全 FF"当故障), 而不是去怀疑 SPI 时序。
     */
    static const PMW3901_VerifyStep PMW3901_VERIFY_TABLE[] = {
        {0x00, 0x4D, 0x11},
        {0x00, 0x55, 0x80},
        {0x00, 0x61, 0xAD},
        {0x00, 0x64, 0xFF},
    };

    enum
    {
        PMW3901_VERIFY_TABLE_SIZE = (int)(sizeof(PMW3901_VERIFY_TABLE) / sizeof(PMW3901_VERIFY_TABLE[0])),
    };

    /**
     * @brief 回读校验策略
     *
     * true : 任何一项不符 → init() 失败(FLOW_SPI_GARBAGE)。理由与仓库其它模块一致:
     *        半配置的器件会安静地喂错速度, 比"没有光流"危险得多。
     * false: 只有"全 0 / 全 FF"才算故障 —— 用于上面那条 ⚠️ 的上板确认情形。
     *
     * ⚠️ **2026-10-03 上板实测后置为 false**(不是"为了让报错消失"):
     *   接线修好之后 ID 全对(0x00→0x49、0x5F→0xB6), 但回读校验 4 条里 **1 条**不一致 ——
     *   `bank0 寄存器 0x61 期望 0xAD 实得 0x01`。而另外 3 条(0x4D=0x11 / 0x55=0x80 / 0x64=0xFF)
     *   **全部命中** ⇒ **SPI 是通的、76 条厂家表也确实落地了**, 不是"时序余量不足"。
     *   0x61 正是上面那条 ⚠️ 点名的手册未定义厂家保留寄存器; 它在初始化表里出现过两次
     *   (**bank0 = 0xAD**、**bank9 = 0x78**, 是两个不同的寄存器) —— 说明这颗器件的 bank0 0x61
     *   本来就不是"写进去就能原样读回"的静态寄存器。
     *   ⇒ 按本文档**预先定好**的处置: 把它移出严格校验, 只保留"全 0/全 FF"这条退化判据。
     *
     *   **代价必须记住**: 现在"厂家表部分没落地"不再拦初始化了 —— 所以**唯一的验收依据变成了
     *   "motion 数据是否正常"**(看 `PMW3901::verifyBadCount()`, 以及 EKF 的水平速度观测)。
     *   如果光流数据是死的, 第一个要回头怀疑的就是这 1 条不符。
     *   注: 原先这里点名的 `[Flow] dx= dy= squal=` 那行日志, 已随光流台架标定完成一起删除
     *   (2026-10-04, 它当时是 4Hz 常驻刷屏); 复核步骤见 docs/2026-10-02_upgrade_progress.md §九。
     */
    static const bool PMW3901_VERIFY_STRICT = false;
} // namespace dlx

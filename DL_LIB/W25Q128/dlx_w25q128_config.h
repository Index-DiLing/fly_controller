#pragma once
#include <stdint.h>

namespace dlx
{
    // ==================================================================
    // W25Q128JV 指令码(数据手册 8.1.2 Instruction Set Table 1, 标准 SPI)
    // 本驱动只使用标准 SPI(1-1-1), 不涉及 Dual/Quad.
    // ==================================================================
    enum class W25Q128_Cmd : uint8_t
    {
        WriteEnable                = 0x06, ///< 置位 WEL, 每次写/擦除/写状态前必须执行
        VolatileSrWriteEnable      = 0x50, ///< 只对易失状态寄存器置位 WEL
        WriteDisable               = 0x04, ///< 清除 WEL
        ReleasePowerDown           = 0xAB, ///< 退出 Power-down(可跟 3 个 dummy 读设备 ID)
        ManufacturerDeviceID       = 0x90, ///< Dummy, Dummy, 00h, 然后读 (MF7-MF0)(ID7-ID0)
        JEDECID                    = 0x9F, ///< 读 3 字节 JEDEC ID
        ReadData                   = 0x03, ///< 标准读: 24bit 地址, 无 dummy(时钟最高 50MHz)
        FastRead                   = 0x0B, ///< 快速读: 24bit 地址 + 1 dummy(时钟最高 133MHz)
        PageProgram                = 0x02, ///< 页编程: 24bit 地址 + 1~256 字节(不可跨页, 超页回绕覆盖)
        SectorErase                = 0x20, ///< 4KB 扇区擦除
        BlockErase32K              = 0x52, ///< 32KB 块擦除
        BlockErase64K              = 0xD8, ///< 64KB 块擦除
        ChipErase                  = 0xC7, ///< 整片擦除(60h 同义)
        EraseProgramSuspend        = 0x75,
        EraseProgramResume         = 0x7A,
        PowerDown                  = 0xB9, ///< 进入 Power-down(之后只认 ReleasePowerDown)
        ReadStatus1                = 0x05, ///< 状态寄存器-1: BUSY/WEL/BP[2:0]/TB/SEC/SRP0
        WriteStatus1               = 0x01,
        ReadStatus2                = 0x35, ///< 状态寄存器-2: SRL/QE/LB[3:1]/CMP/SUS
        WriteStatus2               = 0x31,
        ReadStatus3                = 0x15, ///< 状态寄存器-3: WPS/DRV[1:0]/HOLD-RST
        WriteStatus3               = 0x11,
        EnableReset                = 0x66, ///< 软件复位必须 66h + 99h 连续两条
        ResetDevice                = 0x99,
    };

    // 状态寄存器-1 位定义(S0~S7)
    enum
    {
        W25Q128_SR1_BUSY = 0x01, ///< 擦/写/写状态进行中, 只读
        W25Q128_SR1_WEL  = 0x02, ///< 写使能锁存, 只读
        W25Q128_SR1_BP0  = 0x04, ///< 块保护位(与 TB/SEC/CMP 组合)
        W25Q128_SR1_BP1  = 0x08,
        W25Q128_SR1_BP2  = 0x10,
        W25Q128_SR1_TB   = 0x20, ///< 顶部/底部保护方向
        W25Q128_SR1_SEC  = 0x40, ///< 4KB 扇区/64KB 块粒度
        W25Q128_SR1_SRP0 = 0x80, ///< 状态寄存器保护(配合 /WP)
    };

    // 状态寄存器-2 位定义(S8~S15)
    enum
    {
        W25Q128_SR2_SRL  = 0x01, ///< 状态寄存器锁
        W25Q128_SR2_QE   = 0x02, ///< Quad 使能: -IQ/-JQ 出厂固定为 1, /HOLD、/WP 功能关闭
        W25Q128_SR2_LB1  = 0x04,
        W25Q128_SR2_LB2  = 0x08,
        W25Q128_SR2_LB3  = 0x10,
        W25Q128_SR2_CMP  = 0x40, ///< 互补保护
        W25Q128_SR2_SUS  = 0x80, ///< 擦/写挂起状态
    };

    // 状态寄存器-3 位定义(S16~S23)
    enum
    {
        W25Q128_SR3_WPS    = 0x04, ///< 写保护方案选择: 0=BP 位方案(默认), 1=块/扇区独立锁
        W25Q128_SR3_DRV1   = 0x40, ///< 输出驱动强度
        W25Q128_SR3_DRV0   = 0x20,
        W25Q128_SR3_HOLD_RST = 0x80, ///< /HOLD 或 /RESET 引脚功能选择
    };

    // ==================================================================
    // 容量/组织(数据手册 1. GENERAL DESCRIPTIONS)
    // ==================================================================
    constexpr uint32_t W25Q128_CAPACITY       = 16u * 1024u * 1024u; ///< 16MB
    constexpr uint32_t W25Q128_PAGE_SIZE      = 256u;                ///< 页大小, 一次最多编程 256B
    constexpr uint32_t W25Q128_SECTOR_SIZE    = 4u * 1024u;          ///< 扇区 4KB, 共 4096 个
    constexpr uint32_t W25Q128_BLOCK32K_SIZE  = 32u * 1024u;         ///< 32KB 块
    constexpr uint32_t W25Q128_BLOCK64K_SIZE  = 64u * 1024u;         ///< 64KB 块, 共 256 个
    constexpr uint32_t W25Q128_ADDR_MASK      = 0x00FFFFFFu;         ///< 24bit 地址

    // ==================================================================
    // 器件识别(数据手册 8.1.1): W25Q128JV-IN/IQ/JQ
    // ==================================================================
    constexpr uint8_t W25Q128_MANUFACTURER_ID = 0xEF; ///< Winbond
    constexpr uint8_t W25Q128_MEMORY_TYPE     = 0x40;
    constexpr uint8_t W25Q128_CAPACITY_ID     = 0x18;

    // ==================================================================
    // 等待超时(ms), 按数据手册 9.6 AC 特性最大值留余量
    // ==================================================================
    constexpr uint32_t W25Q128_PAGE_PROGRAM_TIMEOUT_MS = 10u;    ///< tPP 最大 3ms
    constexpr uint32_t W25Q128_SECTOR_ERASE_TIMEOUT_MS = 500u;   ///< tSE 最大 400ms
    constexpr uint32_t W25Q128_BLOCK32K_ERASE_TIMEOUT_MS = 2000u;  ///< tBE1 最大 1.6s
    constexpr uint32_t W25Q128_BLOCK64K_ERASE_TIMEOUT_MS = 2500u;  ///< tBE2 最大 2s
    constexpr uint32_t W25Q128_CHIP_ERASE_TIMEOUT_MS = 210000u;   ///< tCE 最大 200s
    constexpr uint32_t W25Q128_STATUS_WRITE_TIMEOUT_MS = 20u;    ///< tW 最大 15ms
    constexpr uint32_t W25Q128_POWER_DOWN_RELEASE_US = 20u;       ///< tRES1 最大 3us
    constexpr uint32_t W25Q128_POWER_UP_MS = 10u;                 ///< tPUW 上电后到能收指令(最大 5ms)
    constexpr uint32_t W25Q128_BUSY_WAIT_MS = 100u;              ///< 初始化时等 BUSY 清零的上限

    // ==================================================================
    // 识别重试(现场问题 2026-10-18: "flash 经常读不出 ID, 但并非总是")
    //
    // 现场表现: 上电/复位后偶发 JEDEC=0x171717, 同时 SR1=0x17 —— 这不是"另一个型号",
    // 而是**芯片根本没在驱动 DO**(MISO 悬空, 5.25MHz 下采出来就是 0x17), 一秒后又恢复.
    // 原因: 本板 W25Q128 的 /CS(PD1) **没有外部上拉**, MCU 复位期间该脚是浮空输入,
    // 实测默认电平为**低** —— 芯片在"被选中"状态下经历上电/复位, 只要 SCK(PB10) 上有一点
    // 噪声被当成时钟, 就可能被灌进半条/一整条杂散指令(写状态/擦除...), 于是接下来一段时间
    // 只忙自己的事、不理 JEDEC. 这是硬件隐患(建议 /CS 加 10k 上拉).
    //
    // 固件侧的对策(不依赖改板): ① 识别失败按 66h/99h 软复位 + Release Power-Down 重试;
    // ② 重试之间留出 tW/tPUW 量级的间隔, 让可能残留的擦写周期结束.
    // ==================================================================
    // 实测(2026-10-18 台架): 复位后芯片"不理人"的持续时间不是几十 ms 而是**约 0.1 ~ 1 s**
    // (5 轮 × 8ms = 91ms 全部落空, 1 秒后的只读探测已能读到 0xEF4018). 这个量级和
    // "杂散指令被当成扇区/块擦除" 吻合, 所以重试总预算按 ~1 秒给: 正常启动只花第一轮的
    // 20ms(tPUW), 只有真的出错才会等到 1 秒; 等不到就按"flash 不可用"降级, 不假装没事.
    constexpr uint8_t  W25Q128_ID_ATTEMPTS      = 9u;   ///< JEDEC 识别最多试几次(含首次)
    constexpr uint32_t W25Q128_ID_RETRY_GAP_MS  = 100u; ///< 两次识别之间的间隔(等残留擦写结束)
    constexpr uint32_t W25Q128_ID_RESET_GAP_US  = 50u;  ///< 66h/99h 软复位后的 tRST 等待(最大 30us)
} // namespace dlx

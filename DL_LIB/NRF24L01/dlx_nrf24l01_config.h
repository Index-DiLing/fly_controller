#pragma once
#include <stdint.h>

namespace dlx
{
    // ==================================================================
    // 运行模式(三选一, 驱动内条件编译):
    //   NRF24L01_MODE_FULL_IRQ  纯中断:   IRQ 中断里直接读 STATUS/收发数据
    //   NRF24L01_MODE_HALF_IRQ  半中断:   IRQ 中断只置标志, 主程序处理 SPI
    //   NRF24L01_MODE_POLLING   纯轮询:   不使用外部中断, 主程序轮询 STATUS
    //
    // 当前测试启用 HALF_IRQ: ISR 只置标志, SPI 全部在主循环里串行执行,
    // 避免中断与主循环在同一个 SPI 上重入. 切换方式: 注释/取消注释下面的宏,
    // 或编译器预定义 -DNRF24L01_MODE_POLLING 之类(与文件内重复定义会报错,
    // 因此命令行方式需先注释掉文件内的 HALF_IRQ).
    // ==================================================================
#if !defined(NRF24L01_MODE_FULL_IRQ) && !defined(NRF24L01_MODE_HALF_IRQ) && !defined(NRF24L01_MODE_POLLING)
#define NRF24L01_MODE_HALF_IRQ
#endif

#if (((defined(NRF24L01_MODE_FULL_IRQ) ? 1 : 0)) + \
     ((defined(NRF24L01_MODE_HALF_IRQ) ? 1 : 0)) + \
     ((defined(NRF24L01_MODE_POLLING) ? 1 : 0))) != 1
#error "NRF24L01 运行模式必须且只能启用一种: NRF24L01_MODE_FULL_IRQ / NRF24L01_MODE_HALF_IRQ / NRF24L01_MODE_POLLING"
#endif

    // ==================================================================
    // 寄存器地址(数据手册 9.1 Register map table)
    // ==================================================================
    enum class NRF24L01_Reg : uint8_t
    {
        Config     = 0x00, ///< 配置: MASK_RX_DR/TX_DS/MAX_RT, EN_CRC, CRCO, PWR_UP, PRIM_RX
        EnAa       = 0x01, ///< 自动应答使能(每数据管道 1 位)
        EnRxAddr   = 0x02, ///< 接收数据管道使能
        SetupAw    = 0x03, ///< 地址宽度: 01=3B, 10=4B, 11=5B
        SetupRetr  = 0x04, ///< ARD[7:4](250us 步进), ARC[3:0](重发次数)
        RfCh       = 0x05, ///< 射频信道 0~125, F = 2400 + RF_CH [MHz]
        RfSetup    = 0x06, ///< RF_DR(bit3: 1=2Mbps), RF_PWR[2:1], LNA_HCURR(bit0)
        Status     = 0x07, ///< RX_DR(bit6), TX_DS(bit5), MAX_RT(bit4), RX_P_NO[3:1], TX_FULL(bit0)
        ObserveTx  = 0x08, ///< PLOS_CNT[7:4], ARC_CNT[3:0]
        Rpd        = 0x09, ///< 载波检测(仅 RX)
        RxAddrP0   = 0x0A, ///< 接收地址管道 0(5B, LSByte 先写)
        RxAddrP1   = 0x0B,
        RxAddrP2   = 0x0C, ///< 管道 2 只有 LSB, 高 4 字节沿用 P1
        RxAddrP3   = 0x0D,
        RxAddrP4   = 0x0E,
        RxAddrP5   = 0x0F,
        TxAddr     = 0x10, ///< 发送地址(LSByte 先写)
        RxPwP0     = 0x11, ///< 管道 0 载荷宽度 1~32(0=不使用)
        RxPwP1     = 0x12,
        RxPwP2     = 0x13,
        RxPwP3     = 0x14,
        RxPwP4     = 0x15,
        RxPwP5     = 0x16,
        FifoStatus = 0x17, ///< TX_REUSE(6), TX_FULL(5), TX_EMPTY(4), RX_FULL(1), RX_EMPTY(0)
    };

    // ==================================================================
    // SPI 指令(数据手册 8.3.1 Table 16.)
    // ==================================================================
    enum class NRF24L01_Cmd : uint8_t
    {
        R_REGISTER    = 0x00, ///< 读寄存器, 命令字低 5 位是寄存器地址
        W_REGISTER    = 0x20, ///< 写寄存器(仅 Power down / Standby 模式可执行)
        R_RX_PAYLOAD  = 0x61, ///< 读 RX 载荷, 读完从 FIFO 删除
        W_TX_PAYLOAD  = 0xA0, ///< 写 TX 载荷(1~32B)
        FLUSH_TX      = 0xE1, ///< 清空 TX FIFO
        FLUSH_RX      = 0xE2, ///< 清空 RX FIFO
        NOP           = 0xFF, ///< 空操作, 可用于读 STATUS
    };

    // STATUS 位标志
    enum
    {
        NRF24L01_STATUS_RX_DR   = 0x40, ///< RX FIFO 有新数据, 写 1 清除
        NRF24L01_STATUS_TX_DS   = 0x20, ///< 数据发送完成(EN_AA 开时收到 ACK 才置位), 写 1 清除
        NRF24L01_STATUS_MAX_RT  = 0x10, ///< 达到最大重发次数, 写 1 清除(不清除无法继续通信)
    };

    // FIFO_STATUS 位标志
    enum
    {
        NRF24L01_FIFO_TX_FULL  = 0x20,
        NRF24L01_FIFO_TX_EMPTY = 0x10,
        NRF24L01_FIFO_RX_FULL  = 0x02,
        NRF24L01_FIFO_RX_EMPTY = 0x01,
    };

    // ==================================================================
    // 模块配置: 直接改这里的数值, init() 会自动应用
    // (参数与旧驱动保持一致, 寄存器位含义以数据手册 9.1 为准)
    // ==================================================================

    /** CONFIG: EN_CRC | PWR_UP | PRIM_RX, 即 0x0B; 三路中断均未屏蔽(默认反射到 IRQ 引脚) */
    constexpr uint8_t NRF24L01_CONFIG_RX = 0x0B;

    /** CONFIG: EN_CRC | PWR_UP(PRIM_RX=0 即 PTX), 即 0x0A */
    constexpr uint8_t NRF24L01_CONFIG_TX = 0x0A;

    /** EN_AA: 通道0 自动应答(与遥控器端对齐, 增强冲激) */
    constexpr uint8_t NRF24L01_EN_AA = 0x01;

    /** EN_RXADDR: 只使能数据管道 0(仅配置了 P0) */
    constexpr uint8_t NRF24L01_EN_RXADDR = 0x01;

    /** SETUP_AW: 5 字节地址(与 5 字节 TX/RX 地址配套) */
    constexpr uint8_t NRF24L01_SETUP_AW = 0x03;

    /** SETUP_RETR: 500us 间隔 + 5 次重发(与遥控器端对齐) */
    constexpr uint8_t NRF24L01_SETUP_RETR = 0x15;

    /** RF_CH: 信道 40 -> 2.440GHz(收发双方必须一致) */
    constexpr uint8_t NRF24L01_RF_CH = 40;

    /** RF_SETUP: 2Mbps(RF_DR=1) + 0dBm(RF_PWR=11) + LNA 高增益, 与旧驱动 0x0F 一致 */
    constexpr uint8_t NRF24L01_RF_SETUP = 0x0F;

    /** RX 载荷宽度(RX_PW_P0), 1~32; read() 默认按这个长度读一整包 */
    constexpr uint8_t NRF24L01_RX_PACKET_SIZE = 32;

    /** TX 载荷最大长度, 1~32 */
    constexpr uint8_t NRF24L01_TX_PACKET_SIZE = 32;

    /** read()/write() 等待中断/标志的超时(ms), 与遥控器端 4ms 对齐 */
    constexpr uint16_t NRF24L01_WAIT_TIMEOUT_MS = 4;

    /** 收发地址(LSByte 先发送; 收发双方一致才能通信) */
    constexpr uint8_t NRF24L01_TX_ADDR[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    constexpr uint8_t NRF24L01_RX_ADDR[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
} // namespace dlx

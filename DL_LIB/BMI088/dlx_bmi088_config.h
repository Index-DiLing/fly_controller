#pragma once
#include <stdint.h>

namespace dlx
{
    /**
     * @brief BMI088 寄存器地址
     *
     * 加速度计与陀螺仪各自拥有独立的地址空间(从 0x00 开始),
     * 由 CSB1/CSB2 两个片选区分访问对象.
     */
    enum class BMI088_Reg : uint8_t
    {
        /* ================= 加速度计 ================= */
        AccChipId       = 0x00,
        AccErrReg       = 0x02,
        AccStatus       = 0x03,
        AccXLsb         = 0x12,
        AccSensortime0  = 0x18,
        AccTempMsb      = 0x22,
        AccFifoLength0  = 0x24,
        AccFifoData     = 0x26,
        AccConf         = 0x40,
        AccRange        = 0x41,
        AccFifoDowns    = 0x45,
        AccFifoConfig0  = 0x48,
        AccFifoConfig1  = 0x49,
        AccInt1Io       = 0x53,
        AccInt2Io       = 0x54,
        AccIntMapData   = 0x58,
        AccSelfTest     = 0x6D,
        AccPwrConf      = 0x7C,
        AccPwrCtrl      = 0x7D,
        AccSoftreset    = 0x7E,

        /* ================= 陀螺仪 ================= */
        GyroChipId      = 0x00,
        GyroRateXLsb    = 0x02,
        GyroFifoStatus  = 0x0E,
        GyroRange       = 0x0F,
        GyroBandwidth   = 0x10,
        GyroSoftreset   = 0x14,
        GyroIntCtrl     = 0x15,
        GyroIntIoConf   = 0x16,
        GyroIntIoMap    = 0x18,
        GyroSelfTest    = 0x3C,
        GyroFifoConfig0 = 0x3D,
        GyroFifoConfig1 = 0x3E,
        GyroFifoData    = 0x3F,
    };

    /**
     * @brief BMI088 模块配置
     *
     * 与内部外设(USART/SPI/TIM 等)的 Profile 结构不同:
     * 模块外设每种基本只用一个, 因此不需要"外设实例 + 模式枚举"的组合,
     * 而是直接把配置写在本枚举里: 每个成员对应一个寄存器, 枚举值 = 写入该寄存器的值.
     * 需要调整工作参数时, 直接修改这里的数值即可, 驱动在 init() 中自动应用.
     */
    enum class BMI088_Config : uint8_t
    {
        /* ================= 加速度计 ================= */
        /**
         * ACC_CONF (0x40):
         *  高4位 acc_bwp: 0x8=OSR4, 0x9=OSR2, 0xA=Normal
         *  低4位 acc_odr: 0x5=12.5Hz, 0x6=25Hz, 0x7=50Hz, 0x8=100Hz,
         *                 0x9=200Hz, 0xA=400Hz, 0xB=800Hz, 0xC=1.6kHz
         */
        AccConf      = 0xA9, ///< 100Hz + Normal 滤波(默认)

        /** ACC_RANGE (0x41): 0x0=±3g, 0x1=±6g, 0x2=±12g, 0x3=±24g */
        AccRange     = 0x01, ///< ±6g(默认)

        /** FIFO_DOWNS (0x45): bit7 恒为 1, [6:4]=2^k 降采样系数 */
        AccFifoDowns = 0x80, ///< 不分频(默认)

        /** FIFO_CONFIG_0 (0x48): bit1 恒为 1; bit0: 0=STREAM, 1=FIFO(满则停) */
        AccFifoMode  = 0x02, ///< STREAM 模式(默认)

        /** FIFO_CONFIG_1 (0x49): bit6=acc_en(使能FIFO), bit4 恒为 1 */
        AccFifoConf  = 0x50, ///< 使能加速度计 FIFO(默认)

        /** INT1_IO_CTRL (0x53): int1_in/int1_out 均关闭 */
        AccInt1Io    = 0x00, ///< 禁用 INT1(默认)

        /** INT2_IO_CTRL (0x54): int2_in/int2_out 均关闭 */
        AccInt2Io    = 0x00, ///< 禁用 INT2(默认)

        /** INT1_INT2_MAP_DATA (0x58): 不把任何中断映射到 INT1/INT2 */
        AccIntMap    = 0x00, ///< 中断全部禁用(默认)

        /* ================= 陀螺仪 ================= */
        /** GYRO_RANGE (0x0F): 0x0=±2000, 0x1=±1000, 0x2=±500, 0x3=±250, 0x4=±125 dps */
        GyroRange    = 0x00, ///< ±2000dps(默认)

        /** GYRO_BANDWIDTH (0x10): ODR + 滤波带宽
         *  0x0=2000Hz/532Hz  0x1=2000Hz/230Hz  0x2=1000Hz/116Hz
         *  0x3=400Hz/47Hz    0x4=200Hz/23Hz    0x5=100Hz/12Hz
         *  0x6=200Hz/64Hz    0x7=100Hz/32Hz */
        GyroBw       = 0x06, ///< 100Hz ODR / 12Hz 带宽(默认)

        /** GYRO_INT_CTRL (0x15): bit7=data_en, bit6=fifo_en */
        GyroIntCtrl  = 0x00, ///< 数据/中断全部禁用(默认)

        /** INT3_INT4_IO_CONF (0x16): int3/int4 输出关闭 */
        GyroIntIo    = 0x00, ///< 禁用 INT3/INT4(默认)

        /** INT3_INT4_IO_MAP (0x18): 不把任何中断映射到 INT3/INT4 */
        GyroIntMap   = 0x00, ///< 中断全部禁用(默认)

        /** FIFO_CONFIG_0 (0x3D): watermark 水位(仅轮询 FIFO, 不使用水位中断) */
        GyroFifoWm   = 0x00, ///< 水位 0(默认)

        /** FIFO_CONFIG_1 (0x3E): 0x40=FIFO(满则停), 0x80=STREAM */
        GyroFifoMode = 0x80, ///< STREAM 模式(默认)
    };

    /**
     * @brief FIFO 分块读取参数(避免大数组占用栈)
     */
    enum
    {
        BMI088_GyroFrameBytes  = 6,   // 陀螺仪 FIFO 一帧: 3x16bit 角速度 = 6 字节(FIFO_DATA 突发读不含中断状态字)
        BMI088_GyroChunkFrames = 16,  // 每次最多读 16 帧
        BMI088_AccChunkBytes   = 112, // 加速度计 FIFO 每次最多读 16 个数据帧(7 字节/帧)
    };
} // namespace dlx

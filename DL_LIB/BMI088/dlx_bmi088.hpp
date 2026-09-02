#pragma once
#include <stdint.h>
#include "dlx_spi.hpp"
#include "dlx_gpio.hpp"
#include "dlx_delay.hpp"
#include "dlx_queue.hpp"
#include "dlx_bmi088_config.h"
#include "dlx_sensor_data.hpp"

/**
 * 加速度计部分以I2C模式启动。它将一直保持在I2C模式，直到检测到CSB1引脚（加
 * 速度计的片选）上的上升沿，此时加速度计部分会切换到SPI模式，并一直保持该模式，直至下一次上电复位。
 * 要在初始化阶段将加速度计更改为SPI模式，用户可以执行一次虚拟的SPI读操作，例如读取寄存器ACC_CHIP_ID（获得的值将无效）。POR之后，陀螺仪处于正常模式，而加速度计则处于暂停模式。要将加速度计切换到正常模式，用户必须执行以下步骤：
 * a. 为传感器通电
 * b. 等待 1 毫秒
 * c. 通过向ACC_PWR_CTRL写入'4'进入正常模式
 * d. 等待450微秒
 *
 * 陀螺仪和加速度计传感器数据的宽度为16位（温度传感器为11位）
 * BMI088的加速度计部分内置了一个24位宽的计数器。该计数器以39.0625µs的分辨率定期递增
 *
 * 自检:
 * 推荐的自检程序如下：
 * 1)  通过向寄存器ACC_RANGE (0x41)写入0x03，设置±24g量程；2）通过向寄存器ACC_CONF
 * (0x40)写入0xAC，设置ODR=1.6kHz、连续采样模式、"正常模式"（norm_avg4）。
 *     注: 手册 4.6.1 正文写的 0xA7 与寄存器表(5.3.10, ODR=0x0C 才是 1.6kHz)矛盾,
 *     以寄存器表为准用 0xAC。
 * 3）等待超过2毫秒 4）启用正向自检极性（即向寄存器ACC_SELF_TEST (0x6D)写入0x0D）
 * 5）等待超过50毫秒 6）读取每个轴的加速度计偏移值（正向自检响应）
 * 7）启用负向自检极性（即向寄存器ACC_SELF_TEST (0x6D)写入0x09）
 * 8）等待超过50毫秒 9）读取每个轴的加速度计偏移值（负向自检响应）
 * 10）禁用自检（即向寄存器ACC_SELF_TEST (0x6D)写入0x00）
 * 11）计算正向和负向自检响应的差值，并与预期值进行比较
 * 12）等待超过50毫秒
 *
 * 陀螺仪内置的自检功能不会使机械MEMS结构发生偏转（如加速度计自检那样），但此
 * 测试也提供了一种快速方法，以确定陀螺仪是否在规定的条件下正常工作。
 * 要触发自检，必须设置地址 GYRO_SELF_TEST 中的位 #0（'bite_trig'）。测试完成后，
 * 陀螺仪将置位位 #1（'bist_rdy'），测试结果随后可在位 #2（'bist_fail'）中找到。值为
 * '0'表示测试顺利通过。如果出现故障，位 'bist_fail' 将被置为'1'。
 *
 * 复位
 * - 对于加速度计部分，通过向寄存器ACC_SOFTRESET写入命令软复位（0xB6）
 * - 对于陀螺仪部分，通过向寄存器GYRO_SOFTRESET写入命令软复位（0xB6）
 * 软复位会对设备执行基本复位，这在很大程度上等同于电源循环。
 *
 * 寄存器0x00：ACC_CHIP_ID
 * 寄存器 0x02：ACC_ERR_REG
 * 寄存器 0x03：ACC_STATUS
 * 寄存器 0x12 – 0x17：ACC 数据
 * Accel_X_int16 = ACC_X_MSB * 256 + ACC_X_LSB
 * 寄存器 0x18 – 0x1A：传感器时间数据
 * 0x22 – 0x23：温度传感器数据
 *
 * BMI088的陀螺仪部分配备了一个集成的FIFO存储器，可在FIFO模式下存储多达100帧的数据。每帧内部由三个16位的rate_x、y、z数据字以及在同一时间采样的16位中断数据组成，但通过FIFO_DATA(0x3F)突发读取出时每帧只有6字节(3x16bit角速度)，中断状态字不会出现在数据流中。
 *
 * FIFO 帧边界说明(数据手册 4.9.3.2.6 / 4.9.4.2):
 *  - 加速度计 FIFO 的帧是不定长的, 若某次突发读在帧中间结束, 该不完整帧会在
 *    下一次访问时被传感器完整重发, 因此读取端应停在帧边界处, 把尾巴留给下次;
 *  - 陀螺仪 FIFO 帧长固定 6 字节(FIFO_DATA 突发读输出), 代码总是按整帧读取, 不会出现不完整帧.
 */
namespace dlx
{
    /**
     * @brief BMI088 六轴惯性传感器驱动(SPI 模式)
     *
     * 硬件要点:
     *  - 加速度计与陀螺仪各有一个片选(CSB1/CSB2), 本类直接控制两个片选,
     *    传入的 SPI 仅用于 SCK/MOSI/MISO 收发, SPI 自身的 NSS 不参与通信;
     *  - SPI 时钟不能超过 10MHz, 连续两次写访问之间至少间隔 2us;
 *  - 加速度计 SPI 读操作: 地址字节之后必须先发送一个 dummy 字节, 有效数据从
 *    第 3 拍开始 (数据手册 6.1.2: 单字节读要按 2 字节突发读, 丢弃第 1 个字节,
 *    第 2 个才是寄存器内容; 即 addr + dummy + data, Linux/RT-Thread 驱动一致);
 *  - 陀螺仪读操作没有 dummy 字节 (16bit 协议: addr + data);
     *  - 数据都是小端序(先 LSB 后 MSB).
     *
     * 配置方式见 dlx_bmi088_config.h: 直接在 BMI088_Config 枚举里修改数值,
     * init() 会自动应用; 自检会临时改变量程/ODR, 结束后会自动恢复配置.
     */
    class BMI088
    {
    private:
        SPI &spi;
        GPIO accelCs; // 加速度计片选 CSB1
        GPIO gyroCs;  // 陀螺仪片选 CSB2

        /* ==================== SPI 寄存器访问 ====================
         * 加速度计/陀螺仪拆成两份, 去掉 isAccel 分支, 用少量代码冗余换运行开销.
         */

        /** 加速度计写寄存器: 地址字节(bit7=0 写) + 数据字节, 写后保持 2us 间隔(数据手册 tIDLE_wacc) */
        inline void accelWriteReg(BMI088_Reg reg, uint8_t value)
        {
            const uint8_t addr = static_cast<uint8_t>(reg) & 0x7F; // 写: bit7=0
            accelCs = 0;
            spi.swap(addr);
            spi.swap(value);
            accelCs = 1;
            delay_us(2);
        }

        /** 陀螺仪写寄存器: 地址字节(bit7=0 写) + 数据字节 */
        inline void gyroWriteReg(BMI088_Reg reg, uint8_t value)
        {
            const uint8_t addr = static_cast<uint8_t>(reg) & 0x7F; // 写: bit7=0
            gyroCs = 0;
            spi.swap(addr);
            spi.swap(value);
            gyroCs = 1;
            delay_us(2);
        }

        /** 加速度计突发读: 地址字节(bit7=1 读) + 1 个 dummy 字节 + n 个数据字节 */
        inline void accelReadRegs(BMI088_Reg reg, uint8_t *out, uint16_t n)
        {
            const uint8_t addr = static_cast<uint8_t>(reg) | 0x80; // 读: bit7=1, 地址在 bit6:0
            accelCs = 0;
            spi.swap(addr);
            spi.swap(static_cast<uint8_t>(0x00)); // 必须的 dummy 字节, 加速度计读必带
            for (uint16_t i = 0; i < n; ++i) {
                out[i] = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            }
            accelCs = 1;
        }

        /** 陀螺仪突发读: 地址字节(bit7=1 读) + n 个数据字节(无 dummy) */
        inline void gyroReadRegs(BMI088_Reg reg, uint8_t *out, uint16_t n)
        {
            const uint8_t addr = static_cast<uint8_t>(reg) | 0x80; // 读: bit7=1, 地址在 bit6:0
            gyroCs = 0;
            spi.swap(addr);
            for (uint16_t i = 0; i < n; ++i) {
                out[i] = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            }
            gyroCs = 1;
        }

        inline uint8_t accelReadReg(BMI088_Reg reg)
        {
            uint8_t v;
            accelReadRegs(reg, &v, 1);
            return v;
        }

        inline uint8_t gyroReadReg(BMI088_Reg reg)
        {
            uint8_t v;
            gyroReadRegs(reg, &v, 1);
            return v;
        }

        /* ==================== 初始化辅助 ==================== */
        /** 加速度计上电: 虚拟读切到 SPI 模式 -> 等 1ms -> 开电 -> 等 450us */
        void accelPowerOn()
        {
            accelReadReg(BMI088_Reg::AccChipId); // 第一次访问: 从 I2C 切到 SPI(读值无效)
            delay_ms(1);
            accelWriteReg(BMI088_Reg::AccPwrConf, 0x00); // 退出 suspend
            // 0x7C 复位值是 0x03(suspend), suspend 模式下写间隔必须 >= 1000us
            // (数据手册 6: 2us 仅适用于 normal mode), 否则下一次写可能不被接受
            delay_ms(1);
            accelWriteReg(BMI088_Reg::AccPwrCtrl, 0x04); // accel 开
            delay_us(450);
        }

        /** 按 BMI088_Config 枚举应用全部配置(中断禁用, FIFO 使能, 写 FIFO 配置会清空 FIFO) */
        void applyConfig()
        {
            // 加速度计: 量程/滤波 -> 中断禁用 -> FIFO
            accelWriteReg(BMI088_Reg::AccConf, static_cast<uint8_t>(BMI088_Config::AccConf));
            accelWriteReg(BMI088_Reg::AccRange, static_cast<uint8_t>(BMI088_Config::AccRange));
            accelWriteReg(BMI088_Reg::AccInt1Io, static_cast<uint8_t>(BMI088_Config::AccInt1Io));
            accelWriteReg(BMI088_Reg::AccInt2Io, static_cast<uint8_t>(BMI088_Config::AccInt2Io));
            accelWriteReg(BMI088_Reg::AccIntMapData, static_cast<uint8_t>(BMI088_Config::AccIntMap));
            accelWriteReg(BMI088_Reg::AccFifoDowns, static_cast<uint8_t>(BMI088_Config::AccFifoDowns));
            accelWriteReg(BMI088_Reg::AccFifoConfig0, static_cast<uint8_t>(BMI088_Config::AccFifoMode));
            accelWriteReg(BMI088_Reg::AccFifoConfig1, static_cast<uint8_t>(BMI088_Config::AccFifoConf));

            // 陀螺仪: 量程/带宽 -> 中断禁用 -> FIFO
            gyroWriteReg(BMI088_Reg::GyroBandwidth, static_cast<uint8_t>(BMI088_Config::GyroBw));
            gyroWriteReg(BMI088_Reg::GyroRange, static_cast<uint8_t>(BMI088_Config::GyroRange));
            gyroWriteReg(BMI088_Reg::GyroIntCtrl, static_cast<uint8_t>(BMI088_Config::GyroIntCtrl));
            gyroWriteReg(BMI088_Reg::GyroIntIoConf, static_cast<uint8_t>(BMI088_Config::GyroIntIo));
            gyroWriteReg(BMI088_Reg::GyroIntIoMap, static_cast<uint8_t>(BMI088_Config::GyroIntMap));
            gyroWriteReg(BMI088_Reg::GyroFifoConfig0, static_cast<uint8_t>(BMI088_Config::GyroFifoWm));
            gyroWriteReg(BMI088_Reg::GyroFifoConfig1, static_cast<uint8_t>(BMI088_Config::GyroFifoMode));
        }

        /**
         * @brief 加速度计自检(数据手册 4.6.1)
         * 会临时把量程改为 ±24g、ODR 改为 1.6kHz, 结束后由 init() 负责恢复.
         * @return 三轴正负响应差满足: x/y >= 1000mg, z >= 500mg
         */
        bool accelSelfTest()
        {
            accelWriteReg(BMI088_Reg::AccRange, 0x03); // ±24g
            accelWriteReg(BMI088_Reg::AccConf, 0xAC);  // 1.6kHz(0x0C) + 连续滤波(bit7) + norm_avg4(bit5)
            delay_ms(2);

            accelWriteReg(BMI088_Reg::AccSelfTest, 0x0D); // 正极性
            delay_ms(50);
            AccelerometerRaw pos = getAcceleration();

            accelWriteReg(BMI088_Reg::AccSelfTest, 0x09); // 负极性
            delay_ms(50);
            AccelerometerRaw neg = getAcceleration();

            accelWriteReg(BMI088_Reg::AccSelfTest, 0x00); // 关闭自检
            delay_ms(50);                                 // 等待回到正常稳态

            const float lsb2mg = 24000.0f / 32768.0f; // ±24g 量程下 1 LSB = 0.732mg
            for (int i = 0; i < 3; ++i) {
                float diff = lsb2mg * (float)((int16_t)pos.data[i] - (int16_t)neg.data[i]);
                if (diff < 0) {
                    diff = -diff;
                }
                const float need = (i == 2) ? 500.0f : 1000.0f;
                if (diff < need) {
                    return false;
                }
            }
            return true;
        }

        /**
         * @brief 陀螺仪内建自检(数据手册 4.6.2)
         * 写 trig_bist 后轮询 bist_rdy(bit1), 检查 bist_fail(bit2) 是否为 0.
         */
        bool gyroSelfTest()
        {
            gyroWriteReg(BMI088_Reg::GyroSelfTest, 0x01); // 触发自检
            for (uint16_t i = 0; i < 100; ++i) {          // 最多等 100ms(手册未给时长)
                delay_ms(1);
                const uint8_t st = gyroReadReg(BMI088_Reg::GyroSelfTest);
                if (st & 0x02) {                          // bist_rdy
                    return (st & 0x04) == 0;              // bist_fail 必须为 0
                }
            }
            return false; // 超时
        }

    public:
        /**
         * @param interface 已按 ≤10MHz 配置好的 SPI(SCK/MOSI/MISO), 由 dlx_spi 工厂创建
         * @param accelCsProfile 加速度计片选 CSB1
         * @param gyroCsProfile  陀螺仪片选 CSB2
         * @note SPI 自带的 NSS 引脚不参与 BMI088 通信, 片选由本类直接控制
         */
        BMI088(SPI &interface, GPIOProfile accelCsProfile, GPIOProfile gyroCsProfile)
            : spi(interface), accelCs(accelCsProfile), gyroCs(gyroCsProfile)
        {
        }

        ~BMI088()
        {
        }

        /**
         * @brief 将加速度计设为SPI模式,从config中读取配置并设置好传感器,尝试自检并检查ID,最后恢复到传感器产生数据+可以读取数据的状态.
         *
         * 流程: 切 SPI -> 软复位 -> 上电 -> 校验 ID(0x1E/0x0F) -> 自检 ->
         *       软复位并重新应用 BMI088_Config 中的配置.
         *
         * @return true 表示 ID 正确且自检通过; false 表示传感器异常
         */
        bool init()
        {
            accelCs.init(GPIOModeProfile::OUT_PP_UP_50MHz);
            accelCs = 1;
            gyroCs.init(GPIOModeProfile::OUT_PP_UP_50MHz);
            gyroCs = 1;

            /* ---------- 加速度计: 切 SPI -> 软复位 -> 上电 -> 检查 ID ---------- */
            accelReadReg(BMI088_Reg::AccChipId); // 第一次访问: 从 I2C 切到 SPI(读值无效)
            delay_ms(1);
            accelWriteReg(BMI088_Reg::AccSoftreset, 0xB6);
            accelPowerOn();
            if (accelReadReg(BMI088_Reg::AccChipId) != 0x1E) {
                return false;
            }

            /* ---------- 陀螺仪: 软复位 -> 等 30ms -> 检查 ID ---------- */
            gyroWriteReg(BMI088_Reg::GyroSoftreset, 0xB6);
            delay_ms(30);
            if (gyroReadReg(BMI088_Reg::GyroChipId) != 0x0F) {
                return false;
            }

            /* ---------- 自检(会临时改动量程/ODR) ---------- */
            const bool accelOk = accelSelfTest();
            const bool gyroOk  = gyroSelfTest();

            /* ---------- 恢复到"产生数据 + 可读取"状态 ---------- */
            // 数据手册建议自检后做一次复位, 复位会清掉所有配置, 因此重新上电并应用配置
            accelWriteReg(BMI088_Reg::AccSoftreset, 0xB6);
            accelPowerOn();
            gyroWriteReg(BMI088_Reg::GyroSoftreset, 0xB6);
            delay_ms(30);
            applyConfig();
            // 刚改完 ODR/滤波带宽, 数据管线需要时间产出并稳定到新配置
            // (100Hz 下首样本约 10ms, 低通滤波稳定约 50ms, 手册 4.6.1 也要求自检后等 >50ms)
            delay_ms(50);
            
            return accelOk && gyroOk;
        }

        /**
         * @brief 读取两路 FIFO 数据并填入队列(加速度计 + 陀螺仪)
         *
         * 每次调用读取当前 FIFO 中全部(或队列能装下)的数据:
         *  - 陀螺仪 FIFO: 帧计数来自 0x0E, 每帧 6 字节(3x16bit 角速度), 无帧头;
         *  - 加速度计 FIFO: 字节计数来自 0x24/0x25, 数据带帧头
         *    (0x84 数据帧 / 0x40 跳过帧 / 0x44 时间帧 / 0x48 配置帧 / 0x50 丢帧),
         *    非数据帧直接跳过.
         *
         * @param accelQueue 加速度计数据队列(元素为 AccelerometerRaw)
         * @param gyroQueue  陀螺仪数据队列(元素为 GyroscopeRaw)
         */
        void fifoRead(Queue<AccelerometerRaw> &accelQueue, Queue<GyroscopeRaw> &gyroQueue)
        {
            /* ================= 陀螺仪 FIFO ================= */
            uint16_t frames = gyroReadReg(BMI088_Reg::GyroFifoStatus) & 0x7F; // bit6:0 帧计数
            const uint16_t gyroSlots = gyroQueue.remaining();
            if (frames > gyroSlots) {
                frames = gyroSlots;
            }
            while (frames > 0) {
                const uint16_t chunk = (frames > (uint16_t)BMI088_GyroChunkFrames)
                                           ? (uint16_t)BMI088_GyroChunkFrames
                                           : frames;
                uint8_t buf[BMI088_GyroChunkFrames * BMI088_GyroFrameBytes];
                gyroReadRegs(BMI088_Reg::GyroFifoData, buf, chunk * BMI088_GyroFrameBytes);
                for (uint16_t i = 0; i < chunk; ++i) {
                    const uint8_t *f = buf + i * BMI088_GyroFrameBytes;
                    GyroscopeRaw raw;
                    raw.data[0] = (uint16_t)((uint16_t)f[1] << 8 | f[0]);
                    raw.data[1] = (uint16_t)((uint16_t)f[3] << 8 | f[2]);
                    raw.data[2] = (uint16_t)((uint16_t)f[5] << 8 | f[4]);
                    gyroQueue.push(raw); // 一定成功: chunk <= 队列剩余
                }
                frames -= chunk;
            }

            /* ================= 加速度计 FIFO ================= */
            uint8_t lenBuf[2]; // FIFO_LENGTH_0(0x24) + FIFO_LENGTH_1(0x25)
            accelReadRegs(BMI088_Reg::AccFifoLength0, lenBuf, 2);
            const uint16_t len16 = (uint16_t)((uint16_t)lenBuf[1] << 8 | lenBuf[0]);
            uint16_t accelBytes  = (len16 & 0x8000) ? 0 : (len16 & 0x3FFF); // 0x8000=空
            const uint16_t accelSlots = accelQueue.remaining();
            uint16_t toRead = accelBytes;
            const uint16_t maxRead = accelSlots * 7u; // 数据帧最长 7 字节, 保证不溢出
            if (toRead > maxRead) {
                toRead = maxRead;
            }

            while (toRead > 0) {
                const uint16_t chunk = (toRead > (uint16_t)BMI088_AccChunkBytes)
                                           ? (uint16_t)BMI088_AccChunkBytes
                                           : toRead;
                uint8_t buf[BMI088_AccChunkBytes];
                accelReadRegs(BMI088_Reg::AccFifoData, buf, chunk);

                uint16_t pos = 0;
                while (pos < chunk) {
                    const uint8_t header = buf[pos] & 0xFC; // 高6位为帧类型
                    switch (header) {
                    case 0x84: { // 加速度计数据帧: 1 头 + 6 字节
                        if (pos + 7 > chunk) {
                            // 不完整帧: 数据手册 4.9.3.2.6 规定传感器会在下次访问时
                            // 完整重发该帧, 因此停在这里, 把尾巴留给下次 fifoRead.
                            pos = chunk;
                            break;
                        }
                        AccelerometerRaw raw;
                        raw.data[0] = (uint16_t)((uint16_t)buf[pos + 2] << 8 | buf[pos + 1]);
                        raw.data[1] = (uint16_t)((uint16_t)buf[pos + 4] << 8 | buf[pos + 3]);
                        raw.data[2] = (uint16_t)((uint16_t)buf[pos + 6] << 8 | buf[pos + 5]);
                        accelQueue.push(raw);
                        pos += 7;
                        break;
                    }
                    case 0x40: // 跳过帧: 2 字节
                    case 0x48: // 输入配置帧: 2 字节
                    case 0x50: // 采样丢弃帧: 2 字节
                        if (pos + 2 > chunk) {
                            pos = chunk;
                            break;
                        }
                        pos += 2;
                        break;
                    case 0x44: // 传感器时间帧: 4 字节
                        if (pos + 4 > chunk) {
                            pos = chunk;
                            break;
                        }
                        pos += 4;
                        break;
                    default: // 未知帧头, 停止本次解析
                        pos = chunk;
                        break;
                    }
                }
                toRead -= chunk;
            }
        }

        /**
         * @brief 直接读取传感器寄存器数据,阻塞返回,配套静态函数用于转换数值,转换函数允许在外部调用
         *
         * @return AccelerometerRaw 三轴原始值(LSB)
         */
        AccelerometerRaw getAcceleration()
        {
            uint8_t buf[6];
            accelReadRegs(BMI088_Reg::AccXLsb, buf, 6); // 寄存器 0x12~0x17, LSB 在前
            AccelerometerRaw raw;
            raw.data[0] = (uint16_t)((uint16_t)buf[1] << 8 | buf[0]);
            raw.data[1] = (uint16_t)((uint16_t)buf[3] << 8 | buf[2]);
            raw.data[2] = (uint16_t)((uint16_t)buf[5] << 8 | buf[4]);
            return raw;
        }

        /**
         * @brief 原始值 -> 加速度(g), 量程取 BMI088_Config::AccRange
         */
        static AccelerometerG getAccelerationG(AccelerometerRaw raw)
        {
            const uint8_t range = static_cast<uint8_t>(BMI088_Config::AccRange);
            const float fullScaleG = 3.0f * (float)(1u << range); // 3g/6g/12g/24g
            AccelerometerG g;
            for (int i = 0; i < 3; ++i) {
                g.data[i] = (float)(int16_t)raw.data[i] / 32768.0f * fullScaleG;
            }
            return g;
        }

        /**
         * @brief 直接读取传感器寄存器数据,阻塞返回,配套静态函数用于转换数值,转换函数允许在外部调用
         *
         * @return GyroscopeRaw 三轴原始值(LSB)
         */
        GyroscopeRaw getAngularVelocity()
        {
            uint8_t buf[6];
            gyroReadRegs(BMI088_Reg::GyroRateXLsb, buf, 6); // 寄存器 0x02~0x07, LSB 在前
            GyroscopeRaw raw;
            raw.data[0] = (uint16_t)((uint16_t)buf[1] << 8 | buf[0]);
            raw.data[1] = (uint16_t)((uint16_t)buf[3] << 8 | buf[2]);
            raw.data[2] = (uint16_t)((uint16_t)buf[5] << 8 | buf[4]);
            return raw;
        }

        /**
         * @brief 原始值 -> 角速度(dps), 量程取 BMI088_Config::GyroRange
         */
        static GyroscopeDps getAngularVelocityDps(GyroscopeRaw raw)
        {
            const uint8_t range = static_cast<uint8_t>(BMI088_Config::GyroRange);
            const float fullScaleDps = 2000.0f / (float)(1u << range); // 2000/1000/500/250/125
            GyroscopeDps g;
            for (int i = 0; i < 3; ++i) {
                g.data[i] = (float)(int16_t)raw.data[i] / 32768.0f * fullScaleDps;
            }
            return g;
        }

        /**
         * @brief 原始值 -> 角速度(dps), 量程取 BMI088_Config::GyroRange
         */
        static GyroscopeRads getAngularVelocityRads(GyroscopeRaw raw)
        {
            const uint8_t range = static_cast<uint8_t>(BMI088_Config::GyroRange);
            const float fullScaleDps = 2000.0f / (float)(1u << range); // 2000/1000/500/250/125
            GyroscopeRads g;
            for (int i = 0; i < 3; ++i) {
                g.data[i] = (float)(int16_t)raw.data[i] / 32768.0f * fullScaleDps /180 * 2 * 3.14159;
            }
            return g;
        }

        /**
         * @brief 返回传感器温度数据,立即转换为摄氏度
         * @note 温度传感器挂在加速度计部分, 更新周期 1.28s
         */
        float getTemperature()
        {
            uint8_t buf[2]; // TEMP_MSB(0x22) + TEMP_LSB(0x23)
            accelReadRegs(BMI088_Reg::AccTempMsb, buf, 2);
            int16_t t = (int16_t)(((uint16_t)buf[0] << 3) | (buf[1] >> 5)); // 11bit 补码
            if (t > 1023) {
                t -= 2048;
            }
            return (float)t * 0.125f + 23.0f; // 0.125°C/LSB, 23°C 偏移
        }

        /**
         * @brief 检查加速度计时间戳,转换为 us
         * @note 24bit 计数器, 分辨率 39.0625us, 每 655.36s 回绕
         */
        uint32_t getAccelTimeStamp()
        {
            uint8_t buf[3]; // SENSORTIME_0(0x18) + _1 + _2
            accelReadRegs(BMI088_Reg::AccSensortime0, buf, 3);
            const uint32_t t = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
            return t * 39u + (t >> 4); // 39.0625us = 39us + 1/16us
        }

    };

} // namespace dlx

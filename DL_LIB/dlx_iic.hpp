#pragma once
#include <stdint.h>
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_iic_profile.hpp"
#include "dlx_bytebuffer.hpp"
namespace dlx
{
    /**
     * @brief 一条 IIC 总线: 管理 I2C 外设的初始化与通用时序原语(起始/停止/等待事件/字节收发)
     * @note 同步&阻塞设计, 不做中断/DMA, 不处理错误与超时(与旧 dl_iic 行为一致)
     */
    class IICBus
    {
    private:
        IICBusProfile profile;

        inline I2C_TypeDef *getI2C_TypeDef()
        {
            switch (profile) {
                case IICBusProfile::I2C1_Profile:
                    return I2C1;
                case IICBusProfile::I2C2_Profile:
                    return I2C2;
                case IICBusProfile::I2C3_Profile:
                    return I2C3;
            }
            return I2C1;
        }

        inline uint32_t getRCC_APB1Periph()
        {
            switch (profile) {
                case IICBusProfile::I2C1_Profile:
                    return RCC_APB1Periph_I2C1;
                case IICBusProfile::I2C2_Profile:
                    return RCC_APB1Periph_I2C2;
                case IICBusProfile::I2C3_Profile:
                    return RCC_APB1Periph_I2C3;
                default:
                    return 0;
            }
        }

    public:
        IICBus(IICBusProfile profile) : profile(profile)
        {
#warning [Experimental]
        }

        // 本总线的事件中断号, 与 USART/SPI 的 profile 风格一致, 同步模式下暂不使用
        inline uint8_t getIRQn()
        {
            return static_cast<uint8_t>(profile);
        }

        // SCL/SDA 均为 I2C 复用(AF4)开漏模式, 上拉由板级外部电阻提供
        inline GPIOModeProfile getGPIOAFProfile()
        {
            return GPIOModeProfile::AF4_OD_NOPULL_50MHz;
        }

        // ------------------------------------------------------------------
        // 工厂函数: 配置 SCL/SDA 引脚(AF4 开漏), 返回总线对象
        // ------------------------------------------------------------------
        static inline IICBus make(GPIOProfile sclPin, GPIOProfile sdaPin, IICBusProfile profile)
        {
            IICBus bus(profile);
            GPIO scl(sclPin);
            scl.init(GPIOModeProfile::AF4_OD_NOPULL_50MHz);
            GPIO sda(sdaPin);
            sda.init(GPIOModeProfile::AF4_OD_NOPULL_50MHz);
            return bus;
        }

        // I2C1: SCL=B6, SDA=B7 (对应旧 dIIC1_cA6_dA7)
        static inline IICBus IIC1_SB6_DB7()
        { return make(GPIOProfile::B6, GPIOProfile::B7, IICBusProfile::I2C1_Profile); }
        // I2C1: SCL=B8, SDA=B9 (对应旧 dIIC1_cA8_dA9)
        static inline IICBus IIC1_SB8_DB9()
        { return make(GPIOProfile::B8, GPIOProfile::B9, IICBusProfile::I2C1_Profile); }
        // I2C2: SCL=B10, SDA=B11 (对应旧 dIIC2_cB10_dB11)
        static inline IICBus IIC2_SB10_DB11()
        { return make(GPIOProfile::BA, GPIOProfile::BB, IICBusProfile::I2C2_Profile); }
        // I2C3: SCL=A8, SDA=C9
        static inline IICBus IIC3_SA8_DC9()
        { return make(GPIOProfile::A8, GPIOProfile::C9, IICBusProfile::I2C3_Profile); }

        /**
         * @brief 初始化 I2C 外设并开启使能
         * @param mode IICBusModeProfile, 按位拼装的模式配置, 见 dlx_iic_profile.hpp
         * @param speed I2C 时钟频率, 一般 100000(标准) 或 400000(快速)
         * @param selfAddress 本机地址(7位), 主机模式下无实际意义, 给个非0值即可
         * @note SCL/SDA 引脚由工厂方法配置(见 getGPIOAFProfile)
         */
        void init(IICBusModeProfile mode, uint32_t speed, uint16_t selfAddress)
        {
            RCC_APB1PeriphClockCmd(getRCC_APB1Periph(), ENABLE);

            uint16_t modeVal = static_cast<uint16_t>(mode);

            I2C_InitTypeDef i2cInit;
            i2cInit.I2C_ClockSpeed          = speed;
            i2cInit.I2C_Mode                = I2C_Mode_I2C; // 固定 IIC, 不使用 SMBUS
            i2cInit.I2C_DutyCycle           = (modeVal & 0x4000) ? I2C_DutyCycle_16_9 : I2C_DutyCycle_2;
            i2cInit.I2C_OwnAddress1         = selfAddress;
            i2cInit.I2C_Ack                 = modeVal & 0x0400; // 0x0400=Enable, 0x0000=Disable
            i2cInit.I2C_AcknowledgedAddress = (modeVal & 0x8000) ? I2C_AcknowledgedAddress_10bit
                                                                 : I2C_AcknowledgedAddress_7bit;
            I2C_Init(getI2C_TypeDef(), &i2cInit);
            I2C_Cmd(getI2C_TypeDef(), ENABLE);
        }

        /**
         * @brief 阻塞等待 I2C 事件标志
         * @note 不处理错误与超时, 从机无应答等异常下会卡死(与旧 dl_iic 行为一致)
         */
        void waitEvent(uint32_t event)
        {
            while (I2C_CheckEvent(getI2C_TypeDef(), event) != SUCCESS);
        }

        /**
         * @brief 发送起始信号(首次为 START, 总线已占用时为重复 START), 并等待主机模式选定
         */
        void start()
        {
            I2C_GenerateSTART(getI2C_TypeDef(), ENABLE);
            waitEvent(I2C_EVENT_MASTER_MODE_SELECT);
        }

        /**
         * @brief 发送停止信号
         */
        void stop()
        {
            I2C_GenerateSTOP(getI2C_TypeDef(), ENABLE);
        }

        /**
         * @brief 发送 7 位从机地址 + 读写位, 并等待对应模式选定
         * @param slaveAddress7 7位地址, 不含读写位
         * @param direction I2C_Direction_Transmitter / I2C_Direction_Receiver
         */
        void sendAddress(uint8_t slaveAddress7, uint8_t direction)
        {
            I2C_Send7bitAddress(getI2C_TypeDef(), slaveAddress7 << 1, direction);
            if (direction == I2C_Direction_Transmitter) {
                waitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
            } else {
                waitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
            }
        }

        /**
         * @brief 阻塞发送一个字节, 等待发送完成(BTF)
         */
        void sendByte(uint8_t data)
        {
            I2C_SendData(getI2C_TypeDef(), data);
            waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        }

        /**
         * @brief 将 buffer 中已写入的数据(从 src 开始的 used() 字节)依次发出, 不修改 buffer 指针
         */
        void sendBuffer(ByteBuffer &buffer)
        {
            uint16_t length = buffer.used();
            for (uint16_t i = 0; i < length; ++i) {
                sendByte(buffer.src[i]);
            }
        }

        /**
         * @brief 接收一个字节
         * @param ack true 回 ACK(非最后一个字节), false 回 NACK(最后一个字节)
         * @return 接收到的字节
         */
        uint8_t receiveByte(bool ack)
        {
            I2C_TypeDef *i2cx = getI2C_TypeDef();
            I2C_AcknowledgeConfig(i2cx, ack ? ENABLE : DISABLE);
            waitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);
            return static_cast<uint8_t>(I2C_ReceiveData(i2cx));
        }

        /**
         * @brief 设置 ACK 开关(默认应答状态), 读操作结束恢复用
         */
        void setAck(bool enable)
        {
            I2C_AcknowledgeConfig(getI2C_TypeDef(), enable ? ENABLE : DISABLE);
        }
    };

    /**
     * @brief IIC 设备: 保存总线引用与从机地址的薄封装, 提供 write/read 方便外部使用
     */
    class IICDevice
    {
    private:
        IICBus &bus;
        uint16_t address; // 7位从机地址, 不含读写位

    public:
        IICDevice(IICBus &bus, uint16_t address) : bus(bus), address(address)
        {
        }

        /**
         * @brief 写操作: 发送起始+地址, 将 buffer 中已写入的数据依次发出, 最后停止
         * @note 外部需要写寄存器时自行拼接: 把 寄存器地址+数据 一起放进 buffer
         * @note 不保证不溢出, 数据长度由调用方保证
         */
        void write(ByteBuffer &buffer)
        {
            bus.start();
            bus.sendAddress(address, I2C_Direction_Transmitter);
            bus.sendBuffer(buffer);
            bus.stop();
        }

        /**
         * @brief 读操作: 先发送 writeBuffer 中的内容(如寄存器地址), 再重复起始并读取 length 字节到 readBuffer
         * @note 符合大多数 IIC 设备的读时序: 发寄存器地址 -> 重复起始 -> 连续读
         * @note readBuffer 是追加写入的(从当前 cur 开始, 用其自带 write 接口),
         *       外部使用前需自行 reset(), 否则数据会接在已有内容之后
         * @note 不保证不溢出, readBuffer 需从 cur 起预留至少 length 字节
         */
        void read(ByteBuffer &readBuffer, ByteBuffer &writeBuffer, uint16_t length)
        {
            bus.start();
            bus.sendAddress(address, I2C_Direction_Transmitter);
            bus.sendBuffer(writeBuffer);

            // 重复起始, 切换为接收方向
            bus.start();
            bus.sendAddress(address, I2C_Direction_Receiver);

            for (uint16_t i = 0; i < length; ++i) {
                bool isLast = (i == length - 1);
                readBuffer.write(bus.receiveByte(!isLast));
            }

            bus.stop();
            bus.setAck(true); // 恢复默认应答, 便于下次多字节读
        }
    };
} // namespace dlx

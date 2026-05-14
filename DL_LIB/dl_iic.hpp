#pragma once
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"

#include "dl_delay.hpp"

#include "dl_log.h"

namespace dl
{

    struct __IIC_Config {
        I2C_TypeDef *i2cx;
        uint32_t I2C_RCC_Periph;
        uint32_t GPIO_RCC_Periph;

        uint8_t I2C_AF_Source;
        uint8_t SCL_Pin_Source;
        uint8_t SDA_Pin_Source;

        GPIO_TypeDef *I2C_IO_Group;

        uint16_t scl_pin;
        uint16_t sda_pin;
    };

#define dIIC1_cA6_dA7                                                                                                                  \
    dl::__IIC_Config                                                                                                                   \
    {                                                                                                                                  \
        I2C1, RCC_APB1Periph_I2C1, RCC_AHB1Periph_GPIOB, GPIO_AF_I2C1, GPIO_PinSource6, GPIO_PinSource7, GPIOB, GPIO_Pin_6, GPIO_Pin_7 \
    }
#define dIIC1_cA8_dA9                                                                                                                  \
    dl::__IIC_Config                                                                                                                   \
    {                                                                                                                                  \
        I2C1, RCC_APB1Periph_I2C1, RCC_AHB1Periph_GPIOB, GPIO_AF_I2C1, GPIO_PinSource8, GPIO_PinSource9, GPIOB, GPIO_Pin_8, GPIO_Pin_9 \
    }
#define dIIC2_cB10_dB11                                                                                                                     \
    dl::__IIC_Config                                                                                                                        \
    {                                                                                                                                       \
        I2C2, RCC_APB1Periph_I2C2, RCC_AHB1Periph_GPIOB, GPIO_AF_I2C2, GPIO_PinSource10, GPIO_PinSource11, GPIOB, GPIO_Pin_10, GPIO_Pin_11 \
    }

    /**
     * @brief IIC类
     * IIC时序:
     * <S> <SLAVEADDRESS> <R/W> <A> <DATA+A>*n <S> <R/W> A ..... <P>
     * <S>:起始信号
     * <SLAVEADDRESS>:从机地址
     * <R/W> 读或写 0是主机写入从机
     * <DATA+A> 每一个字节后面跟一个ACK
     */

    class IIC
    {
    public:
        I2C_TypeDef *I2Cx = 0;
        IIC()             = default;

        /**
         * @brief Construct a new IIC object
         * @param I2Cx 选择I2C外设,I2C1/I2C2/I2C3
         * @param _I2C_ClockSpeed  时钟频率,此值必须低于 400,000
         * @param _I2C_Mode 选I2C即可
         * @param _I2C_DutyCycle 占空比,随便选
         * @param _I2C_OwnAddress1 自身IIC地址
         * @param _I2C_Ack 是否应答 一般开启
         * @param _I2C_AcknowledgedAddress 指定地址长度,可选7位或10位
         */
        IIC(
            __IIC_Config config,

            uint32_t _I2C_ClockSpeed = 400000,

            uint16_t _I2C_Mode = I2C_Mode_I2C,

            uint16_t _I2C_DutyCycle = I2C_DutyCycle_16_9,

            uint16_t _I2C_OwnAddress1 = 0x51,

            uint16_t _I2C_Ack = I2C_Ack_Enable,

            uint16_t _I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit)
        {
            I2Cx = config.i2cx;

            RCC_APB1PeriphClockCmd(config.I2C_RCC_Periph, ENABLE); // TODO: 支持其他I2C外设,目前实际只支持I2C1

            I2C_InitTypeDef init{
                .I2C_ClockSpeed          = _I2C_ClockSpeed,
                .I2C_Mode                = _I2C_Mode,
                .I2C_DutyCycle           = _I2C_DutyCycle,
                .I2C_OwnAddress1         = _I2C_OwnAddress1,
                .I2C_Ack                 = _I2C_Ack,
                .I2C_AcknowledgedAddress = _I2C_AcknowledgedAddress};
            I2C_Init(I2Cx, &init); // TODO: 支持其他I2C外设,目前实际只支持I2C1
            // 默认复用PB6 SCL PB7 SDA
            // 开启GPIOB
            RCC_AHB1PeriphClockCmd(config.GPIO_RCC_Periph, ENABLE);
            // 配置PB6 PB7
            GPIO_InitTypeDef GPIO_InitStruct{
                .GPIO_Pin   = config.scl_pin,
                .GPIO_Mode  = GPIO_Mode_AF,
                .GPIO_Speed = GPIO_Speed_50MHz,
                .GPIO_OType = GPIO_OType_OD,
                .GPIO_PuPd  = GPIO_PuPd_NOPULL};
            GPIO_Init(config.I2C_IO_Group, &GPIO_InitStruct);
            GPIO_InitStruct.GPIO_Pin = config.sda_pin;
            GPIO_Init(config.I2C_IO_Group,&GPIO_InitStruct);
            GPIO_PinAFConfig(config.I2C_IO_Group, config.SCL_Pin_Source, config.I2C_AF_Source);
            GPIO_PinAFConfig(config.I2C_IO_Group, config.SDA_Pin_Source, config.I2C_AF_Source);

            // I2C_ITConfig(I2Cx,I2C_IT_EVT,ENABLE);//默认只开启了EVT的中断
            // 默认无中断,串行通信(数据量不大)
            I2C_Cmd(I2Cx, ENABLE);
            
        }
        /**
         * @brief 传输七位地址+读写位,注意,传入的地址是7位,会自动偏移并且传入读/写
         *
         * @param slaveAddress
         * @param I2C_Direction 可选值
         *      @arg I2C_Direction_Transmitter
         *      @arg I2C_Direction_Recive
         */
        void send7bitSlaveAddressRW(uint8_t slaveAddress, uint8_t I2C_Direction)
        {
            I2C_Send7bitAddress(I2Cx, (slaveAddress << 1), I2C_Direction);
        }

        // 向指定地址写入指定数量的数据
        void write(uint8_t slaveAddress, uint8_t address, uint8_t *data, uint16_t size)
        {
            I2C_GenerateSTART(I2Cx, ENABLE);
            while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) {
                // 等待,TODO:超时机制/等待指示
            }
            send7bitSlaveAddressRW(slaveAddress, I2C_Direction_Transmitter);
            while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS) {
                // 同上
            }
            // 发送数据

            // 我不到啊我先看看

            I2C_SendData(I2Cx, address++);
            while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) {
            }
            while (size--) {
                I2C_SendData(I2Cx, *data++);
                while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) {
                }
            }
            // 产生停止信号
            I2C_GenerateSTOP(I2Cx, ENABLE);
        }

        void read(uint8_t slaveAddress, uint8_t address, uint8_t *data, uint16_t size)
        {
            I2C_GenerateSTART(I2Cx, ENABLE);
            while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) {
                // 等待,TODO:超时机制/等待指示
            }
            send7bitSlaveAddressRW(slaveAddress, I2C_Direction_Transmitter);
            while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS) {
                // 同上
            }
            // 写入地址
            I2C_SendData(I2Cx, address);
            while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) {
            }
            // 接下来反复读取数据

            // 产生重复开始信号
            I2C_GenerateSTART(I2Cx, ENABLE);
            while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) {
            }
            send7bitSlaveAddressRW(slaveAddress, I2C_Direction_Receiver);
            while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS) {
            }

            if (size == 1) {
                // 只有一个数据，不产生应答
                I2C_AcknowledgeConfig(I2Cx, DISABLE); // 不应答
            }

            while (size--) {
                while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS) {
                    // 这里已经产生应答了
                }
                // 接收到数据
                *data++ = I2C_ReceiveData(I2Cx);
                if (size == 1) {
                    // 最后一个字节,产生停止信号
                    I2C_AcknowledgeConfig(I2Cx, DISABLE); // 不应答
                }
            }
            // 收尾,关闭
            I2C_GenerateSTOP(I2Cx, ENABLE);
            // 开启回应
            I2C_AcknowledgeConfig(I2Cx, ENABLE);
        }
    };
    /**
     * @brief IIC设备,包含设备地址.多个设备可用一个I2C类
     *
     */
    class IICDevice
    {
    public:
        IIC iicBus;
        uint16_t slaveAddress = 0;
        uint8_t buffer[128]; // 预留了128字节的缓冲区,每次写入均从0开始,read时一次性读出
        IICDevice() = default;
        IICDevice(IIC &iic, uint16_t _slaveAddress) : iicBus(iic), slaveAddress(_slaveAddress)
        {
        }
        /**
         * @brief 写入模式,向目标地址写入一个字节,理论上可以写入多个字节，但是这里仅写入两个
         *
         * @param address 目标寄存器地址
         * @param data
         */
        void write(uint8_t address, uint8_t data)
        {
            iicBus.write(slaveAddress, address, &data, 1);
        }

        uint8_t read(uint8_t address, uint8_t size)
        {
            iicBus.read(slaveAddress, address, buffer, size);
            return size;
        }
    };

} // namespace dl

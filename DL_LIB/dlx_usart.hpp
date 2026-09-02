#pragma once

#include <stdint.h>
#include "stm32f4xx.h"
#include "dlx_usart_profile.hpp"
#include "dlx_bytebuffer.hpp"
#include "dlx_nvic_it.h"
#include "dlx_gpio.hpp"

#include "dlx_dma.hpp"

namespace dlx
{

    class USART
    {

        struct USART_IT_CallbackContext {
            void (*handler)(USART*,void*) = nullptr;
            void *context = nullptr;
        };


        USARTProfile profile;

        RingByteBuffer *rxBuffer = nullptr;
        
        USART_IT_CallbackContext ITContext;

        inline USART_TypeDef *getUSART_TypeDef()
        {
            switch (profile) {
                case USARTProfile::USART1_Profile:
                    return USART1;
                case USARTProfile::USART2_Profile:
                    return USART2;
                case USARTProfile::USART3_Profile:
                    return USART3;
                case USARTProfile::USART4_Profile:
                    return UART4;
                case USARTProfile::USART5_Profile:
                    return UART5;
                case USARTProfile::USART6_Profile:
                    return USART6;
            }
            return USART1;
        }

        inline bool isAPB2Periph()
        {
            return profile == USARTProfile::USART1_Profile || profile == USARTProfile::USART6_Profile;
        }

        inline uint32_t getRCC_APB2Periph()
        {
            switch (profile) {
                case USARTProfile::USART1_Profile:
                    return RCC_APB2Periph_USART1;
                case USARTProfile::USART6_Profile:
                    return RCC_APB2Periph_USART6;
                default:
                    return 0;
            }
        }

        inline uint32_t getRCC_APB1Periph()
        {
            switch (profile) {
                case USARTProfile::USART2_Profile:
                    return RCC_APB1Periph_USART2;
                case USARTProfile::USART3_Profile:
                    return RCC_APB1Periph_USART3;
                case USARTProfile::USART4_Profile:
                    return RCC_APB1Periph_UART4;
                case USARTProfile::USART5_Profile:
                    return RCC_APB1Periph_UART5;
                default:
                    return 0;
            }
        }

        // 校验 USART+DMA 场景下 DMAModeProfile 是否合理:
        //   方向必须与收发一致(P2M/M2P); 外设地址固定(USART->DR, 不可递增);
        //   内存端必须连续递增; 外设数据宽度必须是 Byte.
        // 不通过时直接进入死循环(fail-stop, 调试期即可暴露配置错误).
        /**
         * @brief 检查DMA模式是否合理
         * @warning 失败会导致卡死
         *
         * @param mode
         * @param isRx
         */
        inline void checkDMAMode(DMAModeProfile mode, bool isRx)
        {
            uint32_t m   = static_cast<uint32_t>(mode);
            uint32_t dir = m & 0x3;
            bool ok      = ((m >> 2) & 0x1) == 0       // PeripheralInc  关闭
                           && ((m >> 3) & 0x1) == 1    // MemoryInc      开启
                           && ((m >> 4) & 0x3) == 0    // PeripheralDataSize == Byte
                           && dir == (isRx ? 0u : 1u); // P2M / M2P
            if (!ok) {
                while (true);
            }
        }

    public:
        USART(USARTProfile profile)
            : profile(profile)
        {
#warning [Experimental]
        }

        inline uint8_t getIRQn()
        {
            return static_cast<uint8_t>(profile);
        }

        // 根据 USARTProfile 返回对应的 AF 模式 GPIO 枚举, 直接用于引脚初始化
        inline GPIOModeProfile getGPIOAFProfile()
        {
            switch (profile) {
                case USARTProfile::USART1_Profile:
                case USARTProfile::USART2_Profile:
                case USARTProfile::USART3_Profile:
                    return GPIOModeProfile::AF7_PP_NOPULL_50MHz;
                default:
                    return GPIOModeProfile::AF8_PP_NOPULL_50MHz;
            }
        }

        // 串口数据寄存器地址, 供 DMA init 作为外设地址使用
        inline uint32_t getDR()
        {
            return reinterpret_cast<uint32_t>(&getUSART_TypeDef()->DR);
        }

        // 本串口默认的 DMA 接收配置(数据流+通道), 供 setDMAReceive / 自行创建接收 DMA 使用
        inline DMAProfile getDefaultDMARxProfile()
        {
            switch (profile) {
                case USARTProfile::USART1_Profile:
                    return DMAProfile::DMA2_Stream2_Channel4; // USART1_RX
                case USARTProfile::USART2_Profile:
                    return DMAProfile::DMA1_Stream5_Channel4; // USART2_RX
                case USARTProfile::USART3_Profile:
                    return DMAProfile::DMA1_Stream1_Channel4; // USART3_RX
                case USARTProfile::USART4_Profile:
                    return DMAProfile::DMA1_Stream2_Channel4; // UART4_RX
                case USARTProfile::USART5_Profile:
                    return DMAProfile::DMA1_Stream0_Channel4; // UART5_RX
                case USARTProfile::USART6_Profile:
                    return DMAProfile::DMA2_Stream1_Channel5; // USART6_RX
            }
            return DMAProfile::DMA2_Stream2_Channel4;
        }

        // 本串口默认的 DMA 发送配置(数据流+通道), 供自行创建发送 DMA 使用
        inline DMAProfile getDefaultDMATxProfile()
        {
            switch (profile) {
                case USARTProfile::USART1_Profile:
                    return DMAProfile::DMA2_Stream7_Channel4; // USART1_TX
                case USARTProfile::USART2_Profile:
                    return DMAProfile::DMA1_Stream6_Channel4; // USART2_TX
                case USARTProfile::USART3_Profile:
                    return DMAProfile::DMA1_Stream3_Channel4; // USART3_TX
                case USARTProfile::USART4_Profile:
                    return DMAProfile::DMA1_Stream4_Channel4; // UART4_TX
                case USARTProfile::USART5_Profile:
                    return DMAProfile::DMA1_Stream7_Channel4; // UART5_TX
                case USARTProfile::USART6_Profile:
                    return DMAProfile::DMA2_Stream6_Channel5; // USART6_TX
            }
            return DMAProfile::DMA2_Stream7_Channel4;
        }

        // 工厂: 自动配置 TX/RX 引脚(AF 模式), 返回 USART 对象
        // 注意: 接收回调需要在稳定对象上调用 init(..., rxBuffer) 注册
        static inline USART make(GPIOProfile TXPin, GPIOProfile RXPin, USARTProfile profile)
        {
            USART usart(profile);
            GPIO tx(TXPin);
            tx.init(usart.getGPIOAFProfile());
            GPIO rx(RXPin);
            rx.init(usart.getGPIOAFProfile());
            return usart;
        }

        // 常用引脚组合工厂(对应旧 dl_usart.hpp 中的 dUSARTx_tX_rY 宏, 引脚号为十六进制)
        static inline USART USART1_TA9_RAA()
        { return make(GPIOProfile::A9, GPIOProfile::AA, USARTProfile::USART1_Profile); }
        static inline USART USART1_TB6_RB7()
        { return make(GPIOProfile::B6, GPIOProfile::B7, USARTProfile::USART1_Profile); }
        static inline USART USART2_TA2_RA3()
        { return make(GPIOProfile::A2, GPIOProfile::A3, USARTProfile::USART2_Profile); }
        static inline USART USART2_TD5_RD6()
        { return make(GPIOProfile::D5, GPIOProfile::D6, USARTProfile::USART2_Profile); }
        static inline USART USART3_TBA_RBB()
        { return make(GPIOProfile::BA, GPIOProfile::BB, USARTProfile::USART3_Profile); }
        static inline USART USART3_TCA_RCB()
        { return make(GPIOProfile::CA, GPIOProfile::CB, USARTProfile::USART3_Profile); }
        static inline USART USART3_TD8_RD9()
        { return make(GPIOProfile::D8, GPIOProfile::D9, USARTProfile::USART3_Profile); }
        static inline USART USART6_TC6_RC7()
        { return make(GPIOProfile::C6, GPIOProfile::C7, USARTProfile::USART6_Profile); }
        static inline USART USART6_TGE_RG9()
        { return make(GPIOProfile::GE, GPIOProfile::G9, USARTProfile::USART6_Profile); }

        /**
         * @brief 自动初始化,init()后串口可用
         *
         * @param modeProfile 模式配置,参见枚举
         * @param baudRate 目标波特率
         */
        void init(USARTModeProfile modeProfile, uint32_t baudRate)
        {
            if (isAPB2Periph()) {
                RCC_APB2PeriphClockCmd(getRCC_APB2Periph(), ENABLE);
            } else {
                RCC_APB1PeriphClockCmd(getRCC_APB1Periph(), ENABLE);
            }

            uint16_t mode = static_cast<uint16_t>(modeProfile);

            USART_InitTypeDef usartInit;
            usartInit.USART_BaudRate            = baudRate;
            usartInit.USART_WordLength          = (mode & 0x0100) ? USART_WordLength_9b : USART_WordLength_8b;
            usartInit.USART_StopBits            = static_cast<uint16_t>(((mode >> 6) & 0x3) << 12);
            usartInit.USART_Parity              = (((mode >> 4) & 0x3) == 0x2)   ? USART_Parity_Even
                                                  : (((mode >> 4) & 0x3) == 0x3) ? USART_Parity_Odd
                                                                                 : USART_Parity_No;
            usartInit.USART_Mode                = static_cast<uint16_t>(((mode >> 2) & 0x3) << 2);
            usartInit.USART_HardwareFlowControl = static_cast<uint16_t>((mode & 0x3) << 8);
            USART_Init(getUSART_TypeDef(), &usartInit);
            USART_Cmd(getUSART_TypeDef(), ENABLE);
        }

        void setITRequest(USARTITProfile it, FunctionalState newState = ENABLE)
        {
            USART_ITConfig(getUSART_TypeDef(), static_cast<uint16_t>(it), newState);
        }

        /**
         * @brief 初始化NVIC
         * @todo 要能关
         * @param priority
         * @param newState
         */
        void initUSART_NVIC(NVICPriorityProfile priority)
        {
            DLX_NVIC_Init(static_cast<NVICITProfile>(getIRQn()), priority);
        }

        /**
         * @brief 设置中断回调函数
         * 
         * @note 先初始化并启用ITRequest后调用
         * @note 生命周期由本对象保证,本对象销毁后中断丢失.
         * 
         * @param handler 
         * @param ctx 
         */
        void setITCallback(void(*handler)(USART*,void*),void* ctx)
        {
            ITContext.context = ctx;
            ITContext.handler = handler;
            DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn()),+[](void* s){
                auto self = static_cast<USART*>(s);
                self->ITContext.handler(self,self->ITContext.context);
            }, this);
        }

        void init(USARTModeProfile modeProfile, uint32_t baudRate, RingByteBuffer &rxBuffer, bool defaultIT = true)
        {
            init(modeProfile, baudRate);
            this->rxBuffer = &rxBuffer;
            if (defaultIT) {
                setITRequest(USARTITProfile::RXNE);
                ITContext.handler = [](USART *usart, void *) {
                    if (USART_GetFlagStatus(usart->getUSART_TypeDef(), USART_FLAG_RXNE)) {
                        usart->rxBuffer->write(USART_ReceiveData(usart->getUSART_TypeDef()));
                    }
                };
                ITContext.context = nullptr;
                initUSART_NVIC(NVICPriorityProfile::P1_S1);
                setITCallback(ITContext.handler, ITContext.context);
            }
        }

        /**
         * @brief 用于设置中断接收的缓冲区,会初始化并开启中断,使用默认中断策略
         * @param rxBuffer 环形缓冲区
         */
        void enableITReceive(RingByteBuffer &rxBuffer, NVICPriorityProfile priority, bool defaultITcallback = true)
        {
            this->rxBuffer = &rxBuffer;
            setITRequest(USARTITProfile::RXNE);

            if (defaultITcallback) {
                ITContext.handler = [](USART *usart, void *) {
                    if (USART_GetITStatus(usart->getUSART_TypeDef(), USART_IT_RXNE)) {
                        usart->rxBuffer->write(USART_ReceiveData(usart->getUSART_TypeDef()));
                    }
                };
                ITContext.context = nullptr;
                initUSART_NVIC(priority);
                setITCallback(ITContext.handler, ITContext.context);
                return;
            }
        }

        void disableITReceive()
        {
            this->rxBuffer = nullptr;
            setITRequest(USARTITProfile::RXNE, DISABLE);
        }
        /**
         * @brief 设置接收缓冲区
         * @see setSendBuffer()
         */
        void setReceiveBuffer(RingByteBuffer &buffer)
        {
            rxBuffer = &buffer;
        }

        // 同步&阻塞
        void send(const uint8_t *data, uint16_t len)
        {
            while (len--) {
                USART_SendData(getUSART_TypeDef(), *data++);
                while (USART_GetFlagStatus(getUSART_TypeDef(), USART_FLAG_TXE) == RESET);
            }
        }

        /**
         * @brief 会消耗buffer
         *
         * @param buffer
         */
        void send(ByteBuffer &buffer)
        {
            const uint8_t *p   = buffer.toArray<uint8_t>(nullptr);
            const uint16_t len = buffer.used();
            send(p, len);
            buffer.reset();
        }

        // 同步&阻塞
        uint8_t receive()
        {
            while (USART_GetFlagStatus(getUSART_TypeDef(), USART_FLAG_RXNE));
            return USART_ReceiveData(getUSART_TypeDef());
        }
        /**
         * @brief 轮询接收指定字节长度,写入缓冲区
         *
         * @param len 实际接收的字节数. 看环形缓冲区的覆写策略.
         * @return uint16_t
         */
        uint16_t receive(uint16_t len)
        {
            uint16_t l = len;
            if (rxBuffer == nullptr || len == 0) {
                return 0;
            }
            do {
                while (USART_GetFlagStatus(getUSART_TypeDef(), USART_FLAG_RXNE));
            } while (
                len-- && rxBuffer->write(USART_ReceiveData(getUSART_TypeDef())));
            return l - len;
        }

        /**
         * @brief 开启DMA接收数据.全默认实现(两个stream中选择其一)
         *
         * 关闭接收中断->设置DMARequest->创建DMA对象并init->返回DMA对象.
         * @param dmaBuffer 设置给DMA的接收缓冲区.DMA接收线性缓冲
         */
        DMA setDMAReceive(ByteBuffer &dmaBuffer)
        {
            // 默认策略: 外设→内存 / 外设地址不递增 / 内存递增 / 字节宽度 /
            // 单次模式 / 中优先级 / Direct FIFO; 需要其它策略请使用三参版本
            return setDMAReceive(dmaBuffer,
                                 DMAModeProfile::P2M_PID_MIE_PByte_MByte_Nor_Med,
                                 DMAFIFOProfile::Direct);
        }

        /**
         * @brief 开启DMA接收数据,使用指定的模式
         *
         * @param dmaBuffer
         * @param mode
         * @return DMA
         * @warning 缓冲区内存由外部管理, 调用方必须保证其在 DMA 使用期间有效;
         *          mode 会做部分合法性检查(方向/外设不递增/内存递增/外设字节宽度),
         *          不通过时进入死循环
         */
        DMA setDMAReceive(ByteBuffer &dmaBuffer, DMAModeProfile mode, DMAFIFOProfile fifo)
        {
            checkDMAMode(mode, true);                                  // 接收: P2M + 外设不递增 + 内存递增 + Byte
            disableITReceive();                                        // 1. 关闭接收中断(轮询/中断接收与 DMA 接收互斥)
            USART_DMACmd(getUSART_TypeDef(), USART_DMAReq_Rx, ENABLE); // 2. 使能 DMA 接收请求
            DMA dma(dmaBuffer, getDefaultDMARxProfile());              // 3. 创建 DMA 对象
            dma.init(mode, getDR(), 0, fifo);                          // 并按指定模式 init, 0 = 收满整个缓冲
            return dma;                                                // 4. 返回 init 好的 DMA 对象(未启动, 由调用方 start())
        }

        /**
         * @brief 开启 DMA 发送数据, 使用默认策略.
         *
         * 使能DMARequest->创建DMA对象并init->返回DMA对象. 发送缓冲就是传入的 buffer,
         * 发送长度取 buffer 的有效长度(used()), 完全依靠 DMA, 不内置缓冲.
         * @param buffer 待发送的缓冲区
         * @warning 缓冲区内存由外部管理, 调用方必须保证其在 DMA 使用期间有效
         */
        DMA setDMASend(ByteBuffer &buffer)
        {
            // 默认策略: 内存→外设 / 外设地址不递增 / 内存递增 / 字节宽度 /
            // Normal 模式(发完即停) / 中优先级 / Direct FIFO; 需要其它策略请使用三参版本
            return setDMASend(buffer,
                              DMAModeProfile::M2P_PID_MIE_PByte_MByte_Nor_Med,
                              DMAFIFOProfile::Direct);
        }

        /**
         * @brief 开启 DMA 发送数据, 使用指定的模式
         *
         * @param buffer 待发送的缓冲区(发送缓冲就是该 buffer)
         * @param mode
         * @return DMA
         * @warning 缓冲区内存由外部管理, 调用方必须保证其在 DMA 使用期间有效;
         *          mode 会做部分合法性检查(方向/外设不递增/内存递增/外设字节宽度),
         *          不通过时进入死循环
         */
        DMA setDMASend(ByteBuffer &buffer, DMAModeProfile mode, DMAFIFOProfile fifo)
        {
            checkDMAMode(mode, false);                                 // 发送: M2P + 外设不递增 + 内存递增 + Byte
            USART_DMACmd(getUSART_TypeDef(), USART_DMAReq_Tx, ENABLE); // 使能 DMA 发送请求
            DMA dma(buffer, getDefaultDMATxProfile());                 // 创建 DMA 对象
            dma.init(mode, getDR(), buffer.used(), fifo);              // 按指定模式 init, 传输长度为有效数据
            return dma;                                                // 返回 init 好的 DMA 对象(未启动, 由调用方 start())
        }
        ~USART(){
            DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn()),nullptr,nullptr);
        }
    };

} // namespace dlx

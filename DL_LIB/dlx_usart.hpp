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

        /**
         * @brief 底层外设指针
         *
         * 为什么要把这个私有成员暴露出来: 异常标志(IDLE/ORE/FE/NE)的清除必须"读 SR 再读 DR",
         * 而这件事只有中断处理方知道该不该做。配套类(见 USARTDmaReceiver)需要拿到寄存器,
         * 与其把整个外设操作复制一份, 不如开一个口子 —— 但**别拿它绕过本类去配串口**。
         */
        inline USART_TypeDef *peripheral() { return getUSART_TypeDef(); }

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
         * @brief 只开/关 UART 的 DMA 接收请求位, 不创建 DMA 对象
         *
         * 与 setDMAReceive() 的分工: 那个函数会顺手建一个 **Normal 模式**的 DMA 对象;
         * 而 IDLE 切帧要的是 **Circular** 模式(硬件自己绕回, 不需要每帧重启),
         * 所以由调用方自己建 DMA, 这里只负责把请求位打开(见 USARTDmaReceiver)。
         */
        void setDmaRxRequest(bool on)
        {
            USART_DMACmd(getUSART_TypeDef(), USART_DMAReq_Rx, on ? ENABLE : DISABLE);
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

    /**
     * @brief USART + DMA 循环接收 + IDLE 切帧
     *
     * 要解决的问题
     *   链路层要的是"一次突发"的边界, 而 RXNE 中断逐字节收只能给你字节流:
     *     - 57600 下一帧 8 字节就是 8 次进中断, 每字节一次上下文切换;
     *     - 更糟的是**边界信息丢了**, 上层只能靠"超时没过"猜一帧结束了没有 ——
     *       这在 20ms 一个槽的 TDMA 里根本猜不准。
     *   IDLE 是硬件给的边界: 总线上连续一个字符时间没有新数据, 它就置位。用它切帧,
     *   拿到的天然是"这一段连续到达的字节"。
     *
     * 工作方式
     *   - DMA **循环模式**连续把 DR 搬进 _buf, CPU 全程不参与;
     *   - IDLE 中断里: 读 SR/DR 清标志 → `pos = 容量 - NDTR` → 把 [_lastPos, pos) 交给回调;
     *   - 一次突发跨过缓冲末尾(DMA 绕回)时分两段交付, 上层拿到的仍是完整一段;
     *   - DMA 的 TC 标志置位 = 至少在取走之前绕回过一次 = 有数据被覆盖, 记 overruns。
     *
     * 三条使用约束(不满足会静默丢数据)
     *   1. **缓冲必须 ≥ 一帧长 + 余量**。突发比缓冲还长时, 覆盖发生在两次 IDLE 之间,
     *      只能靠 TC 标志事后发现 —— 那时数据已经没了。
     *   2. 回调在**中断上下文**里跑: 只许拷贝/置标志, 不许做 I/O、不许阻塞。
     *   3. 与 RXNE 中断接收**互斥**(start() 里会关掉 RXNE)。
     */
    class USARTDmaReceiver
    {
    public:
        /** 收到一段突发。data 只在回调期间有效(指向 DMA 缓冲), 需要留存请自己拷走。 */
        typedef void (*BurstHandler)(const uint8_t *data, uint16_t len, void *ctx);

        USARTDmaReceiver(USART &usart, uint8_t *buffer, uint16_t capacity)
            : _usart(usart),
              _buf(buffer),
              _cap(capacity),
              _view(buffer, capacity),
              _dma(_view, usart.getDefaultDMARxProfile()),
              _handler(nullptr),
              _ctx(nullptr),
              _lastPos(0),
              _bursts(0),
              _bytes(0),
              _overruns(0)
        {
        }

        /**
         * @brief 启动接收
         * @param prio USART 中断优先级 —— 这个中断每次突发只进一次(不是每字节),
         *             所以给常规优先级即可, 不需要抢在控制环前面。
         */
        void start(USARTModeProfile mode, uint32_t baud, NVICPriorityProfile prio)
        {
            _usart.init(mode, baud);          // 外设本身(不动中断)
            _usart.disableITReceive();        // 与 RXNE 逐字节接收互斥
            _lastPos = 0;
            _bursts = _bytes = _overruns = 0;

            _dma.init(DMAModeProfile::P2M_PID_MIE_PByte_MByte_Cir_Med, _usart.getDR(), _cap,
                      DMAFIFOProfile::Direct);
            _dma.clearTransferCompleteFlag();
            _usart.setDmaRxRequest(true);
            _dma.start(); // 先让 DMA 跑起来再开 IDLE: 反过来的话第一段可能在中断里还没准备好缓冲

            // ⚠ 顺序必须是"先装回调 → 清旧的 IDLE 挂起 → 再开 IDLE/NVIC"。
            //   之前是"开 NVIC 再装回调", 上板表现为**卡死在启动**:
            //   E220 的 AT 应答/上电期流量会把 USART3 的 IDLE 标志置起来; 一旦 NVIC 使能,
            //   IDLE 中断立刻进, 但此时 DLX_IT 注册表里还没有 handler(注册要等下面一行),
            //   分发器只能"什么都不做"就返回 —— 而 IDLE 标志只有"读 SR 再读 DR"才清得掉,
            //   于是中断级别一直挂着、反复重进, CPU 永远回不到 start() 之后的代码。
            //   现象就是"打印完启动 DMA+IDLE 接收后没有任何日志"(见 2026-10-18 上板复盘)。
            _usart.setITCallback(&USARTDmaReceiver::trampoline, this);
            // 清 IDLE 的标准序列: 先读 SR 再读 DR
            (void)_usart.peripheral()->SR;
            (void)_usart.peripheral()->DR;

            _usart.setITRequest(USARTITProfile::IDLE, ENABLE);
            _usart.initUSART_NVIC(prio);
        }

        void stop()
        {
            _usart.setITRequest(USARTITProfile::IDLE, DISABLE);
            _usart.setDmaRxRequest(false);
            _dma.stop();
            _usart.setITCallback(nullptr, nullptr);
        }

        void setHandler(BurstHandler handler, void *ctx)
        {
            _handler = handler;
            _ctx     = ctx;
        }

        uint32_t burstCount() const { return _bursts; }   ///< 收到多少段突发
        uint32_t byteCount() const { return _bytes; }     ///< 累计字节
        uint32_t overruns() const { return _overruns; }   ///< 被覆盖过几次(>0 就说明缓冲太小或上层太慢)

        uint16_t capacity() const { return _cap; }
        /** 当前已写位置(调试/排查用) */
        uint16_t writePos() { return static_cast<uint16_t>(_cap - _dma.remaining()); }

    private:
        /** USART 中断入口: 只认 IDLE, 其余标志一概不管(ORE/NE 由上层按需处理) */
        static void trampoline(USART *usart, void *ctx)
        {
            static_cast<USARTDmaReceiver *>(ctx)->onIdle(usart);
        }

        void onIdle(USART *usart)
        {
            if (USART_GetITStatus(usart->peripheral(), USART_IT_IDLE) == RESET) {
                return;
            }
            // 清 IDLE 必须"先读 SR 再读 DR" —— GetITStatus 已经读了 SR, 这里补一次 DR。
            // 少读 DR 的话标志永远清不掉, 中断会一直重进(而且是**看不到**的那种:
            // 每进一次就交付一段 0 长度, 表现成 CPU 跑满但功能好像还正常)。
            (void)USART_ReceiveData(usart->peripheral());

            // 溢出判定必须在取 pos **之前**做: TC 置位表示指针已经绕过起点一次,
            // 那意味着 [_lastPos, ...) 的前半段已经被新数据盖掉了。
            if (_dma.transferCompleteFlag()) {
                ++_overruns;
                _dma.clearTransferCompleteFlag();
                _lastPos = 0; // 只能从"当前能保证没被覆盖的"位置重新开始
            }

            const uint16_t pos = static_cast<uint16_t>(_cap - _dma.remaining());
            deliver(pos);
            _lastPos = pos;
        }

        /** 交付 [_lastPos, pos): 跨过缓冲末尾时分成两段 */
        void deliver(uint16_t pos)
        {
            if (_handler == nullptr || pos == _lastPos) {
                return; // 没有新数据(杂散 IDLE) —— 不要交付 0 长度, 上层会以为收到一个空帧
            }
            if (pos > _lastPos) {
                const uint16_t len = static_cast<uint16_t>(pos - _lastPos);
                ++_bursts;
                _bytes += len;
                _handler(_buf + _lastPos, len, _ctx);
                return;
            }
            // 绕回了: 先交付尾段, 再交付头段
            const uint16_t tail = static_cast<uint16_t>(_cap - _lastPos);
            if (tail > 0) {
                ++_bursts;
                _bytes += tail;
                _handler(_buf + _lastPos, tail, _ctx);
            }
            if (pos > 0) {
                ++_bursts;
                _bytes += pos;
                _handler(_buf, pos, _ctx);
            }
        }

        USART    &_usart;
        uint8_t  *_buf;
        uint16_t  _cap;
        ByteBuffer _view; ///< 给 DMA 的内存视图(必须在 _dma 之前构造)
        DMA        _dma;
        BurstHandler _handler;
        void        *_ctx;
        uint16_t     _lastPos;
        uint32_t     _bursts;
        uint32_t     _bytes;
        uint32_t     _overruns;
    };

} // namespace dlx

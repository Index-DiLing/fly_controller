#pragma once

#include "dlx_dma_profile.hpp"
#include "stm32f4xx.h"
#include "dlx_bytebuffer.hpp"
#include "dlx_nvic_it.h"
namespace dlx
{

    /**
     * @brief DMA类,用于DMA控制
     * 
     * 基本只使用外设到内存/内存到外设模式,内存到内存模式基本不使用.
     * 支持内存端双缓冲(setDoubleBuffer + isTransferingFirstBuffer),
     * 配合 Circular 模式由硬件自动在 M0/M1 之间切换.
     */
    class DMA
    {
    private:
        struct DMA_IT_CallbackContext {
            void (*handler)(DMA*, void*) = nullptr;
            void *context = nullptr;
        };

        DMAProfile profile;
        // 仅持有 ByteBuffer 视图副本(src/cur/len), 不持有数据内存本身;
        // 数据内存始终由调用方管理(传入的 buffer).
        ByteBuffer buffer;
        ByteBuffer* secondBuffer = nullptr;
        // 内存侧数据宽度(字节), init/setBuffer 时从 mode 解析并记录, reset() 换算用
        uint8_t memItemSize = 1;
        // 最近一次 init/setBuffer 设定的目标字节数(相当于游标, 指示当前用了缓冲区
        // 的多少), 供 reset(0) 恢复该长度; 上限始终是 buffer.len(容量)
        uint16_t transferBytes = 0;

        DMA_IT_CallbackContext ITContext;

        inline bool isDMA2()
        {
            return ((static_cast<uint8_t>(profile) >> 6) & 0x1) != 0;
        }
        inline uint8_t getStream()
        {
            return (static_cast<uint8_t>(profile) >> 3) & 0x7;
        }
        inline uint8_t getChannel()
        {
            return static_cast<uint8_t>(profile) & 0x7;
        }

        // DMA1/DMA2 的 8 条数据流从基地址 +0x10 起, 每条间隔 0x18
        inline DMA_Stream_TypeDef *getDMA_Stream()
        {
            uint32_t base = isDMA2() ? DMA2_BASE : DMA1_BASE;
            return (DMA_Stream_TypeDef *)(base + 0x10 + getStream() * 0x18);
        }

        // DMA_Channel_n = n << 25
        inline uint32_t getDMA_Channel()
        {
            return static_cast<uint32_t>(getChannel()) << 25;
        }

        inline uint32_t getDMA_FLAG_TC()
        {
            static const uint32_t tcif[8] = {
                DMA_FLAG_TCIF0, DMA_FLAG_TCIF1, DMA_FLAG_TCIF2, DMA_FLAG_TCIF3,
                DMA_FLAG_TCIF4, DMA_FLAG_TCIF5, DMA_FLAG_TCIF6, DMA_FLAG_TCIF7,
            };
            return tcif[getStream()];
        }

        // 中断号不连续(如 DMA1_Stream7 = 47, DMA2_Stream5 = 68), 查表
        inline IRQn_Type getDMA_IRQn()
        {
            static const IRQn_Type irqn[16] = {
                DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
                DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
                DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
                DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn,
            };
            return irqn[(isDMA2() ? 8 : 0) + getStream()];
        }

        // DMA_IT_TC/HT/TE/FE/DME 宏查表(仅与中断类型相关, 对应 DMAITProfile 位[2:0])
        inline uint32_t getDMA_IT(DMAITProfile it)
        {
            static const uint32_t itMap[5] = {
                DMA_IT_TC, DMA_IT_HT, DMA_IT_TE, DMA_IT_FE, DMA_IT_DME,
            };
            return itMap[static_cast<uint8_t>(it) & 0x7];
        }

        // 校验目标字节数不超过缓冲区容量(len, 单位字节); 0 表示"使用全容量", 恒合法.
        // 不通过时进入死循环(fail-stop, 调试期即可暴露配置错误).
        inline void checkTransferSize(uint16_t transferSize, uint16_t capacity)
        {
            if (transferSize > capacity) {
                while (true);
            }
        }

        // 内存侧数据宽度字节数: DMAModeProfile bit[7:6] = MemoryDataSize
        // (0=Byte, 1=HalfWord, 2=Word)
        inline uint8_t itemSizeFromMode(uint32_t m)
        {
            return static_cast<uint8_t>(1u << ((m >> 6) & 0x3));
        }

        // 把目标字节数按内存侧数据宽度换算成 NDTR 项数.
        // 字节数必须正好是宽度的整数倍, 否则 fail-stop(调试期即可暴露配置错误).
        inline uint16_t bytesToItems(uint16_t bytes, uint8_t itemSize)
        {
            if ((bytes % itemSize) != 0) {
                while (true);
            }
            return bytes / itemSize;
        }

    public:
    /**
     * @brief 根据内存缓冲区和profile创建对象
     * 
     * @param buffer @warning 必须至少4字节对齐!
     * @param profile 
     */
        DMA(ByteBuffer& buffer, DMAProfile profile)
            : profile(profile), buffer(buffer)
        {
#warning [Experimental]
        }
        ~DMA()
        {
            // 仅在本对象自己注册过回调时才解绑; 从未注册(或由副本注册)的对象
            // 不清理, 避免误删副本刚注册的中断转发器
            if (ITContext.handler != nullptr) {
                DLX_IT_set_callback(static_cast<NVICITProfile>(getDMA_IRQn()), nullptr, nullptr);
            }
        }

        /**
         * @brief 按 DMAModeProfile + 外设地址配置 DMA, 内存侧使用构造时传入的 ByteBuffer
         * @param mode 模式配置, 见 dlx_dma_profile.hpp
         * @param peripheralBaseAddr 外设寄存器地址(如 (uint32_t)&USART1->DR)
         * @param transferSize 本次传输的目标字节数, 可以小于 buffer.len
         *        (如接收缓冲开得较大但只需收满 N 字节); 0 表示使用 buffer 全容量;
         *        必须是内存数据宽度(Byte/HalfWord/Word)的整数倍
         * @param fifo FIFO/突发配置, 见 dlx_dma_profile.hpp 中的 DMAFIFOProfile,
         *        默认 Direct(直接模式, 单次突发), 合法组合已按表 41 列全
         * @note NDTR 的单位是"数据项"而不是字节: buffer.len/transferSize 一律按字节
         *       计(ByteBuffer 的本质语义), 内部按 mode 的内存侧数据宽度自动换算,
         *       例如 HalfWord 宽度下 128 字节 -> NDTR = 64
         */
        void init(DMAModeProfile mode, uint32_t peripheralBaseAddr,
                  uint16_t transferSize = 0,
                  DMAFIFOProfile fifo = DMAFIFOProfile::Direct)
        {
            if (isDMA2()) {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
            } else {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
            }

            uint32_t m = static_cast<uint32_t>(mode);
            uint16_t f = static_cast<uint16_t>(fifo);
            checkTransferSize(transferSize, buffer.len);

            // Direct 模式(FIFO 关闭)要求外设与内存数据宽度一致(RM0090), 否则 fail-stop
            if (((f & 0x0100) == 0) && (((m >> 4) & 0x3) != ((m >> 6) & 0x3))) {
                while (true);
            }

            // NDTR 是向下计数器, 传输完毕归零且不会自动恢复(Normal 模式还会自动关断 EN).
            // buffer.len 保持字节长度不变, 这里按 mode 的内存侧数据宽度换算项数,
            // 并把目标字节数记录进 transferBytes, 供 reset() 恢复计数器使用.
            memItemSize              = itemSizeFromMode(m);
            uint16_t bytes           = (transferSize == 0) ? buffer.len : transferSize;
            transferBytes            = bytes;
            uint16_t size            = bytesToItems(bytes, memItemSize);

            // 从按位拼装的配置中取出各字段, 移位结果即 stm32f4xx_dma.h 宏
            DMA_InitTypeDef dmaInit;
            dmaInit.DMA_Channel            = getDMA_Channel();
            dmaInit.DMA_PeripheralBaseAddr = peripheralBaseAddr;
            dmaInit.DMA_Memory0BaseAddr    = reinterpret_cast<uint32_t>(buffer.src);
            dmaInit.DMA_DIR                = (m & 0x3) << 6;         // bit[1:0]
            dmaInit.DMA_BufferSize         = size;
            dmaInit.DMA_PeripheralInc      = ((m >> 2) & 0x1) << 9;  // bit[2]
            dmaInit.DMA_MemoryInc          = ((m >> 3) & 0x1) << 10; // bit[3]
            dmaInit.DMA_PeripheralDataSize = ((m >> 4) & 0x3) << 11; // bit[5:4]
            dmaInit.DMA_MemoryDataSize     = ((m >> 6) & 0x3) << 13; // bit[7:6]
            dmaInit.DMA_Mode               = ((m >> 8) & 0x1) << 8;  // bit[8]
            dmaInit.DMA_Priority           = ((m >> 9) & 0x3) << 16; // bit[10:9]

            // 从 DMAFIFOProfile 按位取出 FIFO/突发字段:
            //   位[1:0]  外设突发   -> DMA_PeripheralBurst (移位结果即标准库宏)
            //   位[3:2]  存储器突发 -> DMA_MemoryBurst
            //   位[5:4]  FIFO 阈值  -> 值即 DMA_FIFOThreshold 宏
            //   位[8]    FIFO 模式  -> DMA_FIFOMode
            dmaInit.DMA_FIFOMode           = (f & 0x0100) ? DMA_FIFOMode_Enable : DMA_FIFOMode_Disable;
            dmaInit.DMA_FIFOThreshold      = (f >> 4) & 0x3;
            dmaInit.DMA_MemoryBurst        = ((f >> 2) & 0x3) << 23;
            dmaInit.DMA_PeripheralBurst    = (f & 0x3) << 21;
            DMA_Init(getDMA_Stream(), &dmaInit);
            DMA_ClearFlag(getDMA_Stream(), getDMA_FLAG_TC());
        }

        /**
         * @brief 返回本 DMA 数据流对应的标准库 DMA_IT_xxxIFx 状态/清除标志宏(查表)
         *        可用于 DMA_GetITStatus / DMA_ClearITPendingBit
         * @param it 中断类型, 见 DMAITProfile(TC/HT/TE/FE/DME)
         * @note 流号取自对象持有的 DMAProfile, 无需在参数中重复指定
         */
        inline uint32_t getITFlag(DMAITProfile it)
        {
            static const uint32_t itif[8][5] = {
                { DMA_IT_TCIF0, DMA_IT_HTIF0, DMA_IT_TEIF0, DMA_IT_FEIF0, DMA_IT_DMEIF0 },
                { DMA_IT_TCIF1, DMA_IT_HTIF1, DMA_IT_TEIF1, DMA_IT_FEIF1, DMA_IT_DMEIF1 },
                { DMA_IT_TCIF2, DMA_IT_HTIF2, DMA_IT_TEIF2, DMA_IT_FEIF2, DMA_IT_DMEIF2 },
                { DMA_IT_TCIF3, DMA_IT_HTIF3, DMA_IT_TEIF3, DMA_IT_FEIF3, DMA_IT_DMEIF3 },
                { DMA_IT_TCIF4, DMA_IT_HTIF4, DMA_IT_TEIF4, DMA_IT_FEIF4, DMA_IT_DMEIF4 },
                { DMA_IT_TCIF5, DMA_IT_HTIF5, DMA_IT_TEIF5, DMA_IT_FEIF5, DMA_IT_DMEIF5 },
                { DMA_IT_TCIF6, DMA_IT_HTIF6, DMA_IT_TEIF6, DMA_IT_FEIF6, DMA_IT_DMEIF6 },
                { DMA_IT_TCIF7, DMA_IT_HTIF7, DMA_IT_TEIF7, DMA_IT_FEIF7, DMA_IT_DMEIF7 },
            };
            return itif[getStream()][static_cast<uint8_t>(it) & 0x7];
        }

        /**
         * @brief 使能/禁止 DMA 中断, 风格与串口 setITRequest 一致
         * @param it 中断类型, 见 DMAITProfile(TC/HT/TE/FE/DME)
         * @param newState ENABLE / DISABLE
         * @note 流号取自对象持有的 DMAProfile; 仅配置 DMA 中断源,
         *       NVIC 使能需另用 initNVIC()
         */
        void setITRequest(DMAITProfile it, FunctionalState newState = ENABLE)
        {
            DMA_ITConfig(getDMA_Stream(), getDMA_IT(it), newState);
        }

        /**
         * @brief 设置DMA中断回调函数
         *
         * 与串口 USART::setITCallback 同款: 把用户回调连同上下文存进本对象,
         * 再向中断通道注册一个转发器; 中断到来时以本对象为参数调用用户回调.
         *
         * @note 先调用 init() 建立 DMA 配置, 再 setITRequest() 使能所需的中断源
         *       (如 TC/HT/TE), 再 initNVIC() 使能 NVIC 通道, 最后 setITCallback()
         *       注册回调; 回调内请用 isITPending()/clearITPending() 区分并清除中断.
         * @note 生命周期由本对象保证, 本对象销毁后中断丢失.
         *
         * @param handler 回调函数, 参数为本 DMA 对象与用户上下文
         * @param ctx 用户上下文
         */
        void setITCallback(void(*handler)(DMA*, void*), void* ctx)
        {
            ITContext.handler = handler;
            ITContext.context = ctx;
            DLX_IT_set_callback(static_cast<NVICITProfile>(getDMA_IRQn()), +[](void *s) {
                auto self = static_cast<DMA*>(s);
                if (self->ITContext.handler != nullptr) {
                    self->ITContext.handler(self, self->ITContext.context);
                }
            }, this);
        }

        /**
         * @brief 两次 DMA 传输之间更换内存侧缓冲区, 保持外设地址及其余配置不变.
         *
         * 外设地址寄存器(PAR)在 init 后保持不变, 因此只需恢复内存起始地址(M0AR)
         * 与传输计数器(NDTR). 先等待上一次传输完毕(TC 标志), 再关断并更新,
         * 使对象回到可 start() 的状态; 适用于发送数据量每次变化的场景.
         * @param buf 新的内存缓冲区(内存由外部管理)
         * @param transferSize 本次传输的目标字节数, 0 表示使用 buf 全容量
         * @note 数据宽度沿用 init() 设定的 mode, 字节数按其内存侧宽度自动换算 NDTR
         * @note 流未使能(从未启动, 或 Normal 模式已停止)时跳过等待, 直接换缓冲;
         *       正在传输时等待本轮 TC 完成后再换
         */
        void setBuffer(ByteBuffer &buf, uint16_t transferSize = 0)
        {
            // 流未使能(从未启动, 或 Normal 模式已停止)时无需等待 TC;
            // 正在传输(Circular 运行中)则等本轮完成
            if (getDMA_Stream()->CR & DMA_SxCR_EN) {
                wait();
            }
            checkTransferSize(transferSize, buf.len);
            DMA_Stream_TypeDef *stream = getDMA_Stream();
            DMA_Cmd(stream, DISABLE);
            while (stream->CR & DMA_SxCR_EN);
            DMA_ClearFlag(stream, getDMA_FLAG_TC());

            transferBytes = (transferSize == 0) ? buf.len : transferSize;
            uint16_t size = bytesToItems(transferBytes, memItemSize);
            buffer = buf; // 同步内部副本(src/cur/len 均按字节), 之后 reset() 也以新缓冲区为准
            DMA_SetCurrDataCounter(stream, size);
            stream->M0AR = reinterpret_cast<uint32_t>(buf.src);
        }

        /**
         * @brief 初始化本 DMA 数据流对应的 NVIC 中断通道, 风格与串口 initUSART_NVIC 一致
         * @param priority 优先级配置, 见 NVICPriorityProfile
         * @note 只使能 NVIC 通道; DMA 中断源本身请用 setITRequest() 配置
         */
        void initNVIC(NVICPriorityProfile priority)
        {
            DLX_NVIC_Init(static_cast<NVICITProfile>(getDMA_IRQn()), priority);
        }

        /**
         * @brief 查询本 DMA 数据流指定中断是否处于挂起状态
         *
         * 等价于标准库 DMA_GetITStatus(stream, getITFlag(it)), 用于在中断回调中
         * 区分 TC/HT/TE/FE/DME 等事件来源.
         * @param it 中断类型, 见 DMAITProfile
         */
        inline bool isITPending(DMAITProfile it)
        {
            return DMA_GetITStatus(getDMA_Stream(), getITFlag(it)) != RESET;
        }

        /**
         * @brief 清除本 DMA 数据流指定中断的挂起标志
         *
         * 等价于标准库 DMA_ClearITPendingBit(stream, getITFlag(it)).
         * 中断回调中处理完对应事件后请调用, 否则该中断会一直触发.
         * @param it 中断类型, 见 DMAITProfile
         */
        inline void clearITPending(DMAITProfile it)
        {
            DMA_ClearITPendingBit(getDMA_Stream(), getITFlag(it));
        }

        void start()
        {
            DMA_Cmd(getDMA_Stream(), ENABLE);
        }

        /**
         * @brief Normal 模式下 wait() 完成后重启 DMA 的复位函数.
         *
         * NDTR 向下计数传输完毕后归零, M0AR 也会随传输递增, 均不会自动恢复,
         * 因此本函数保持 init() 的其余配置不变, 仅恢复传输计数器与内存起始地址,
         * 并清除完成标志, 使对象回到可 start() 的状态.
         * @param transferSize 重置后的传输字节数, 0 表示使用最近一次 init/setBuffer
         *        设定的字节数(transferBytes 游标); 可大于该游标但不能大于
         *        buffer.len(缓冲区容量), 即在容量范围内可自由扩/缩
         * @note Circular 模式由硬件自动重载计数器, 无需调用本函数
         */
        void reset(uint16_t transferSize = 0)
        {
            checkTransferSize(transferSize, buffer.len);
            DMA_Stream_TypeDef *stream = getDMA_Stream();
            DMA_Cmd(stream, DISABLE);
            while (stream->CR & DMA_SxCR_EN);
            DMA_ClearFlag(stream, getDMA_FLAG_TC());
            uint16_t bytes = (transferSize == 0) ? transferBytes : transferSize;
            DMA_SetCurrDataCounter(stream, bytesToItems(bytes, memItemSize));
            stream->M0AR = reinterpret_cast<uint32_t>(buffer.src);
        }

        void wait()
        {
            while (DMA_GetFlagStatus(getDMA_Stream(), getDMA_FLAG_TC()) == RESET);
        }

        /**
         * @brief 使能内存侧双缓冲, 第二缓冲区配置与主缓冲完全一致
         *
         * STM32F4 双缓冲模式下 M1 与 M0 共享方向/数据宽度/递增/循环/突发等
         * 全部配置(RM0090), 只有内存地址不同, NDTR 也共用, 因此这里只需写入
         * 第二缓冲区地址并打开 DBM 位, 其余保持主缓冲的 init/setBuffer 配置.
         *
         * @param second 第二内存缓冲区; 容量(ByteBuffer.len)必须不小于当前传输
         *               字节数(transferBytes), 否则 fail-stop
         * @note 应在 init()(或 setBuffer())之后、start() 之前调用;
         *       初始目标为 M0, 每完成一块缓冲后硬件自动切换 CT,
         *       Circular 模式下无需软件干预
         */
        void setDoubleBuffer(ByteBuffer &second)
        {
            checkTransferSize(transferBytes, second.len);
            secondBuffer = &second;
            DMA_DoubleBufferModeConfig(getDMA_Stream(), reinterpret_cast<uint32_t>(second.src), DMA_Memory_0);
            DMA_DoubleBufferModeCmd(getDMA_Stream(), ENABLE);
        }

        /**
         * @brief 双缓冲模式下查询 DMA 当前目标(CT 位)
         * @return true  = 正在使用主缓冲(M0), 第二缓冲(M1)空闲可写
         *         false = 正在使用第二缓冲(M1), 主缓冲(M0)空闲可写
         * @note 在 TC 中断里调用即可得知刚完成的是哪一块(空闲块);
         *       未开启双缓冲时恒为 true
         */
        inline bool isTransferingFirstBuffer()
        {
            return (getDMA_Stream()->CR & DMA_SxCR_CT) == 0;
        }

    };

} // namespace dlx

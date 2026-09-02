#pragma once

#include <stdint.h>
#include "stm32f4xx.h"
#include "dlx_timer_profile.hpp"
#include "dlx_nvic_it.h"
#include "dlx_dma.hpp"
#include "dlx_gpio.hpp"
#include "dlx_dshot.hpp"

/**
 * @note 来自野火教程:
 * @note TIMxCLK为总线时钟的两倍， 使得表 各个定时器特性 中可选的最大定时器时钟为84MHz，即基本定时器的内部时钟(CK_INT)频率为84MHz。
 * @note 预分频系数 和 计数器实际都要+1
 * @note 基本定时器只能向上计时,CK_INT一般不动,因此可以认为只有两个参数-预分频系数和定时器周期，比较简单
 *
 * @note 高级计时器可选外部时钟
 * @note 高级控制定时器和部分通用定时器(TIM2至TIM5)可以设置为主模式或从模式，TIM9和TIM10可设置为从模式
 * @note 高级控制定时器的计数器有三种计数模式，分别为递增计数模式、递减计数模式和递增/递减(中心对齐)计数模式。
 * @note 高级计时器具有重复计数器 当计数器CNT的值跟比较寄存器CCR的值相等的时候，输出参考信号OCxREF的信号的极性就会改变
 * @note 优先实现PWM输出模式功能.
 * @note 高级计时器比较输出需要使用 TIM_TimeBaseInitTypeDef 初始化,再初始化TIM_OCInitTypeDef
 *       TIM_ClockDivision：时钟分频， 设置定时器时钟CK_INT频率与死区发生器以及数字滤波器采样时钟频率分频比
 *       高级控制定时器有四个定时器通道，使用时都必须单独设置
 *       TIM_OCMode：比较输出模式选择，总共有八种， 常用的为PWM1/PWM2。
 *       TIM_Pulse：比较输出脉冲宽度， 实际设定比较寄存器CCR的值  TIM_OCPolarity：比较输出极性 TIM_OCIdleState：空闲状态时通道输出电平设置
 *       不同的定时器可能对应不同的APB总线，在使能定时器时钟是必须特别注意。高级控制定时器属于APB2，定时器内部时钟是168MHz。
 */

namespace dlx
{
    /**
     * @brief 基本计时器(TIM6 / TIM7)
     *
     * 只有 预分频系数 和 定时器周期 两个数值参数可调:
     * 只能向上计数、CK_INT 固定、无输出通道、无 DMA(硬件本身就没有), 因此不需要工厂函数.
     * 中断只有 Update, 通过 dlx_nvic_it 注册回调.
     */
    class BasicTimer
    {
    private:
        struct IT_CallbackContext {
            void (*handler)(BasicTimer *, void *) = nullptr;
            void *context                        = nullptr;
        };

        BasicTimerProfile profile;
        IT_CallbackContext ITContext;

        inline TIM_TypeDef *getTIM_TypeDef() const
        {
            switch (profile) {
                case BasicTimerProfile::TIM6_Profile:
                    return TIM6;
                case BasicTimerProfile::TIM7_Profile:
                    return TIM7;
            }
            return TIM6;
        }

        inline uint32_t getRCC_APB1Periph()
        {
            switch (profile) {
                case BasicTimerProfile::TIM6_Profile:
                    return RCC_APB1Periph_TIM6;
                case BasicTimerProfile::TIM7_Profile:
                    return RCC_APB1Periph_TIM7;
            }
            return RCC_APB1Periph_TIM6;
        }

    public:
        BasicTimer(BasicTimerProfile profile)
            : profile(profile)
        {
#warning [Experimental]
        }

        ~BasicTimer()
        {
            DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn()), nullptr, nullptr);
        }

        // 枚举值即 IRQn 号(同 USARTProfile 的取值风格), 可直接用于 NVIC
        inline uint8_t getIRQn()
        {
            return static_cast<uint8_t>(profile);
        }

        /**
         * @brief 初始化时基并启动. 仅有的两个数值参数: 预分频系数与定时器周期
         *
         * @param prescaler TIM_Prescaler, 实际分频 = prescaler + 1
         * @param period    TIM_Period, 实际计数范围 = 0 ~ period
         * @note 使能 ARR 预装载, 之后 setAutoreload() 在更新事件生效
         */
        void init(uint16_t prescaler, uint32_t period)
        {
            RCC_APB1PeriphClockCmd(getRCC_APB1Periph(), ENABLE);

            TIM_TimeBaseInitTypeDef base;
            base.TIM_ClockDivision     = TIM_CKD_DIV1; // TIM6/TIM7 无 CKD 位, 填默认值即可
            base.TIM_CounterMode       = TIM_CounterMode_Up;
            base.TIM_Period            = period;
            base.TIM_Prescaler         = prescaler;
            base.TIM_RepetitionCounter = 0;
            TIM_TimeBaseInit(getTIM_TypeDef(), &base);
            TIM_ARRPreloadConfig(getTIM_TypeDef(), ENABLE);
            TIM_Cmd(getTIM_TypeDef(), ENABLE);
        }

        inline void start()
        {
            TIM_Cmd(getTIM_TypeDef(), ENABLE);
        }

        inline void stop()
        {
            TIM_Cmd(getTIM_TypeDef(), DISABLE);
        }

        inline uint32_t getCounter()
        {
            return TIM_GetCounter(getTIM_TypeDef());
        }

        inline void setCounter(uint32_t counter)
        {
            TIM_SetCounter(getTIM_TypeDef(), counter);
        }

        inline void setAutoreload(uint32_t period)
        {
            TIM_SetAutoreload(getTIM_TypeDef(), period);
        }

        inline void setPrescaler(uint16_t prescaler)
        {
            TIM_PrescalerConfig(getTIM_TypeDef(), prescaler, TIM_PSCReloadMode_Update);
        }

        // 基本计时器只有 Update 中断源, 传入其它 TimerITProfile 会被硬件忽略
        void setITRequest(TimerITProfile it = TimerITProfile::Update, FunctionalState newState = ENABLE)
        {
            TIM_ITConfig(getTIM_TypeDef(), static_cast<uint16_t>(it), newState);
        }

        /**
         * @brief 使能 NVIC 中断通道
         * @param priority 优先级配置, 见 NVICPriorityProfile
         */
        void initNVIC(NVICPriorityProfile priority)
        {
            DLX_NVIC_Init(static_cast<NVICITProfile>(getIRQn()), priority);
        }

        /**
         * @brief 设置 Update 中断回调
         *
         * @note 先 setITRequest() + initNVIC() 使能中断, 再注册回调
         * @note 生命周期由本对象保证, 本对象销毁后中断丢失.
         */
        void setITCallback(void (*handler)(BasicTimer *, void *), void *ctx)
        {
            ITContext.context = ctx;
            ITContext.handler = handler;
            DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn()), +[](void *s) {
                auto self = static_cast<BasicTimer *>(s);
                if (self->ITContext.handler != nullptr) {
                    self->ITContext.handler(self, self->ITContext.context);
                }
            }, this);
        }

        inline bool isITPending(TimerITProfile it)
        {
            return TIM_GetITStatus(getTIM_TypeDef(), static_cast<uint16_t>(it)) != RESET;
        }

        inline void clearITPending(TimerITProfile it)
        {
            TIM_ClearITPendingBit(getTIM_TypeDef(), static_cast<uint16_t>(it));
        }
    };

    /**
     * @brief 高级计时器(TIM1 / TIM8), 目前专注于比较输出(PWM)功能, 输入暂不考虑
     *
     * 构造参数 AdvancedTimerProfile 只描述 实例 + 时基(18 种);
     * 预分频/周期/重复计数器/CCR(Pulse) 为数值参数,
     * 输出通道的比较输出配置由 TimerOutputChannelModeProfile 在
     * initOutputChannel() 中按通道单独指定.
     * 提供中断(Update/CC/Break/TriggerCOM 四条 IRQn)与 DMA 接口,
     * DMA 不写死数据流/通道/DMAR 突发目标, 全部交给调用方选择.
     */
    class AdvancedTimer
    {
    private:
        struct IT_CallbackContext {
            void (*handler)(AdvancedTimer *, void *) = nullptr;
            void *context                           = nullptr;
        };

        AdvancedTimerProfile profile;
        // 按 TimerIRQProfile 索引(Update/CC/Break/TriggerCOM), 对应 4 条独立 IRQn
        IT_CallbackContext ITContexts[4];

        // 解码见 dlx_timer_profile.hpp: bit[0] 选择 TIM1 / TIM8
        inline bool isTIM8() const
        {
            return (static_cast<uint16_t>(profile) & 0x0001) != 0;
        }

        inline TIM_TypeDef *getTIM_TypeDef()
        {
            return isTIM8() ? TIM8 : TIM1;
        }

        inline uint32_t getRCC_APB2Periph() const
        {
            return isTIM8() ? RCC_APB2Periph_TIM8 : RCC_APB2Periph_TIM1;
        }

        inline IRQn_Type getIRQn(TimerIRQProfile irq) const
        {
            static const IRQn_Type tim1Irqn[4] = {
                TIM1_UP_TIM10_IRQn, TIM1_CC_IRQn, TIM1_BRK_TIM9_IRQn, TIM1_TRG_COM_TIM11_IRQn,
            };
            static const IRQn_Type tim8Irqn[4] = {
                TIM8_UP_TIM13_IRQn, TIM8_CC_IRQn, TIM8_BRK_TIM12_IRQn, TIM8_TRG_COM_TIM14_IRQn,
            };
            return (isTIM8() ? tim8Irqn : tim1Irqn)[static_cast<uint8_t>(irq)];
        }

        // TIM1 复用 AF1, TIM8 复用 AF3
        inline GPIOModeProfile getGPIOAFProfile() const
        {
            return isTIM8() ? GPIOModeProfile::AF3_PP_NOPULL_50MHz
                            : GPIOModeProfile::AF1_PP_NOPULL_50MHz;
        }

        // 解码见 dlx_timer_profile.hpp, 移位结果即 stm32f4xx_tim.h 宏
        inline uint16_t getCounterMode() const
        {
            return ((static_cast<uint16_t>(profile) >> 8) & 0x3) << 4; // Up/Down/CenterAligned1
        }

        inline uint16_t getClockDivision() const
        {
            return ((static_cast<uint16_t>(profile) >> 10) & 0x3) << 8; // DIV1/DIV2/DIV4
        }

        inline TIM_OCInitTypeDef makeOCInit(TimerOutputChannelModeProfile mode, uint32_t pulse) const
        {
            uint8_t oc = static_cast<uint8_t>(mode);
            TIM_OCInitTypeDef oci;
            oci.TIM_OCMode       = (oc & 0x7) << 4; // 编码即 CCMR OCxM 字段值
            oci.TIM_OutputState  = (oc & 0x08) ? TIM_OutputState_Enable : TIM_OutputState_Disable;
            oci.TIM_OutputNState = (oc & 0x10) ? TIM_OutputNState_Enable : TIM_OutputNState_Disable;
            oci.TIM_Pulse        = pulse;
            // 假定互补输出必须"互补": OCN 极性恒与 OCPolarity 相反
            oci.TIM_OCPolarity   = (oc & 0x20) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
            oci.TIM_OCNPolarity  = (oc & 0x20) ? TIM_OCNPolarity_High : TIM_OCNPolarity_Low;
            // 空闲态同理: OCNIdleState 恒与 OCIdleState 相反
            oci.TIM_OCIdleState  = (oc & 0x40) ? TIM_OCIdleState_Set : TIM_OCIdleState_Reset;
            oci.TIM_OCNIdleState = (oc & 0x40) ? TIM_OCNIdleState_Reset : TIM_OCNIdleState_Set;
            return oci;
        }

        // 按 IRQn 转发回调: 每条 IRQn 用独立模板实例, 把索引写进函数指针
        template <uint8_t N>
        static void forwardIT(void *s)
        {
            auto self = static_cast<AdvancedTimer *>(s);
            if (self->ITContexts[N].handler != nullptr) {
                self->ITContexts[N].handler(self, self->ITContexts[N].context);
            }
        }

    public:
        AdvancedTimer(AdvancedTimerProfile profile)
            : profile(profile)
        {
#warning [Experimental]
        }

        ~AdvancedTimer()
        {
            for (uint8_t i = 0; i < 4; i++) {
                DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn(static_cast<TimerIRQProfile>(i))), nullptr, nullptr);
            }
        }

        inline TIM_TypeDef *getTIM()
        {
            return getTIM_TypeDef();
        }

        /**
         * @brief 初始化时基并启动计时器
         *
         * @param prescaler         TIM_Prescaler, 实际分频 = prescaler + 1
         * @param period            TIM_Period, 实际计数范围 = 0 ~ period
         * @param repetitionCounter TIM_RepetitionCounter(仅 TIM1/TIM8 有效),
         *                          每 (repetitionCounter + 1) 个溢出才产生一次更新事件
         * @note 计数模式/时钟分频来自 AdvancedTimerProfile; 使能 ARR 预装载
         */
        void init(uint16_t prescaler, uint32_t period, uint8_t repetitionCounter = 0)
        {
            RCC_APB2PeriphClockCmd(getRCC_APB2Periph(), ENABLE);

            TIM_TimeBaseInitTypeDef base;
            base.TIM_ClockDivision     = getClockDivision();
            base.TIM_CounterMode       = getCounterMode();
            base.TIM_Period            = period;
            base.TIM_Prescaler         = prescaler;
            base.TIM_RepetitionCounter = repetitionCounter;
            TIM_TimeBaseInit(getTIM_TypeDef(), &base);
            TIM_ARRPreloadConfig(getTIM_TypeDef(), ENABLE);
            TIM_Cmd(getTIM_TypeDef(), ENABLE);
        }

        inline void start()
        {
            TIM_Cmd(getTIM_TypeDef(), ENABLE);
        }

        inline void stop()
        {
            TIM_Cmd(getTIM_TypeDef(), DISABLE);
        }

        // 高级计时器主输出使能(MOE 位), 比较输出要出现在引脚上必须调用
        inline void enableOutputs()
        {
            TIM_CtrlPWMOutputs(getTIM_TypeDef(), ENABLE);
        }

        inline void disableOutputs()
        {
            TIM_CtrlPWMOutputs(getTIM_TypeDef(), DISABLE);
        }

        inline uint32_t getCounter()
        {
            return TIM_GetCounter(getTIM_TypeDef());
        }

        inline void setCounter(uint32_t counter)
        {
            TIM_SetCounter(getTIM_TypeDef(), counter);
        }

        inline void setAutoreload(uint32_t period)
        {
            TIM_SetAutoreload(getTIM_TypeDef(), period);
        }

        inline void setPrescaler(uint16_t prescaler)
        {
            TIM_PrescalerConfig(getTIM_TypeDef(), prescaler, TIM_PSCReloadMode_Update);
        }

        /**
         * @brief 初始化单个通道作为比较输出(OC), 使用指定的输出通道 Mode
         *
         * @param channel 通道号
         * @param mode    比较输出模式/使能/极性/空闲态, 见 TimerOutputChannelModeProfile
         * @param pulse   比较寄存器 CCR 初值(TIM_Pulse)
         * @note 会顺带使能该通道的 OC 预装载; 全部通道配好后调用 enableOutputs()
         */
        void initOutputChannel(TIMChannelProfile channel, TimerOutputChannelModeProfile mode, uint32_t pulse)
        {
            TIM_OCInitTypeDef oci = makeOCInit(mode, pulse);
            switch (channel) {
                case TIMChannelProfile::Channel1:
                    TIM_OC1Init(getTIM_TypeDef(), &oci);
                    TIM_OC1PreloadConfig(getTIM_TypeDef(), TIM_OCPreload_Enable);
                    break;
                case TIMChannelProfile::Channel2:
                    TIM_OC2Init(getTIM_TypeDef(), &oci);
                    TIM_OC2PreloadConfig(getTIM_TypeDef(), TIM_OCPreload_Enable);
                    break;
                case TIMChannelProfile::Channel3:
                    TIM_OC3Init(getTIM_TypeDef(), &oci);
                    TIM_OC3PreloadConfig(getTIM_TypeDef(), TIM_OCPreload_Enable);
                    break;
                case TIMChannelProfile::Channel4:
                    TIM_OC4Init(getTIM_TypeDef(), &oci);
                    TIM_OC4PreloadConfig(getTIM_TypeDef(), TIM_OCPreload_Enable);
                    break;
            }
        }

        /**
         * @brief 一次性以相同的 Mode/Pulse 初始化全部四个输出通道
         *
         * @param mode  比较输出模式/使能/极性/空闲态, 四个通道一致
         * @param pulse 比较寄存器 CCR 初值(TIM_Pulse), 四个通道一致
         */
        void initOutputChannel(TimerOutputChannelModeProfile mode, uint32_t pulse)
        {
            initOutputChannel(TIMChannelProfile::Channel1, mode, pulse);
            initOutputChannel(TIMChannelProfile::Channel2, mode, pulse);
            initOutputChannel(TIMChannelProfile::Channel3, mode, pulse);
            initOutputChannel(TIMChannelProfile::Channel4, mode, pulse);
        }

        /**
         * @brief 运行时更新通道 CCR(配合预装载, 在下一个更新事件生效), 不重新初始化
         * @param channel 通道号
         * @param value   CCR 新值
         */
        void setCompare(TIMChannelProfile channel, uint32_t value)
        {
            switch (channel) {
                case TIMChannelProfile::Channel1:
                    TIM_SetCompare1(getTIM_TypeDef(), value);
                    break;
                case TIMChannelProfile::Channel2:
                    TIM_SetCompare2(getTIM_TypeDef(), value);
                    break;
                case TIMChannelProfile::Channel3:
                    TIM_SetCompare3(getTIM_TypeDef(), value);
                    break;
                case TIMChannelProfile::Channel4:
                    TIM_SetCompare4(getTIM_TypeDef(), value);
                    break;
            }
        }

        // ------------------------------------------------------------------
        // GPIO 工厂: 按本实例的复用号(AF1/AF3)配置通道引脚
        // ------------------------------------------------------------------

        /**
         * @brief 通用工厂: 配置 4 个通道引脚为 AF 模式, 并返回计时器对象
         * @note 只使用其中部分通道时, 其余引脚也会被配置(多余配置无害)
         */
        static inline AdvancedTimer make(AdvancedTimerProfile profile,
                                         GPIOProfile ch1Pin, GPIOProfile ch2Pin,
                                         GPIOProfile ch3Pin, GPIOProfile ch4Pin)
        {
            AdvancedTimer tim(profile);
            GPIOModeProfile af = tim.getGPIOAFProfile();
            GPIO(ch1Pin).init(af);
            GPIO(ch2Pin).init(af);
            GPIO(ch3Pin).init(af);
            GPIO(ch4Pin).init(af);
            return tim;
        }

        // TIM1: CH1=A8, CH2=A9, CH3=A10, CH4=A11
        static inline AdvancedTimer TIM1_PA8_PA9_PA10_PA11(AdvancedTimerProfile profile)
        {
            return make(profile, GPIOProfile::A8, GPIOProfile::A9, GPIOProfile::AA, GPIOProfile::AB);
        }
        // TIM1: CH1=PE9, CH2=PE11, CH3=PE13, CH4=PE14 (飞控电机输出常用组合)
        static inline AdvancedTimer TIM1_PE9_PE11_PE13_PE14(AdvancedTimerProfile profile)
        {
            return make(profile, GPIOProfile::E9, GPIOProfile::EB, GPIOProfile::ED, GPIOProfile::EE);
        }
        // TIM8: CH1=PC6, CH2=PC7, CH3=PC8, CH4=PC9
        static inline AdvancedTimer TIM8_PC6_PC7_PC8_PC9(AdvancedTimerProfile profile)
        {
            return make(profile, GPIOProfile::C6, GPIOProfile::C7, GPIOProfile::C8, GPIOProfile::C9);
        }

        // ------------------------------------------------------------------
        // 中断接口
        // ------------------------------------------------------------------
        void setITRequest(TimerITProfile it, FunctionalState newState = ENABLE)
        {
            TIM_ITConfig(getTIM_TypeDef(), static_cast<uint16_t>(it), newState);
        }

        /**
         * @brief 使能某条 NVIC 中断通道
         * @param irq      通道类别(Update/CC/Break/TriggerCOM), 决定实际 IRQn
         * @param priority 优先级配置, 见 NVICPriorityProfile
         */
        void initNVIC(TimerIRQProfile irq, NVICPriorityProfile priority)
        {
            DLX_NVIC_Init(static_cast<NVICITProfile>(getIRQn(irq)), priority);
        }

        /**
         * @brief 设置指定中断通道的回调
         *
         * @note 先 setITRequest() 使能中断源, 再 initNVIC() 使能 NVIC, 最后注册回调
         * @note 生命周期由本对象保证, 本对象销毁后中断丢失.
         */
        void setITCallback(TimerIRQProfile irq, void (*handler)(AdvancedTimer *, void *), void *ctx)
        {
            uint8_t i = static_cast<uint8_t>(irq);
            ITContexts[i].handler = handler;
            ITContexts[i].context = ctx;
            switch (irq) {
                case TimerIRQProfile::Update:
                    DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn(irq)), &forwardIT<0>, this);
                    break;
                case TimerIRQProfile::CC:
                    DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn(irq)), &forwardIT<1>, this);
                    break;
                case TimerIRQProfile::Break:
                    DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn(irq)), &forwardIT<2>, this);
                    break;
                case TimerIRQProfile::TriggerCOM:
                    DLX_IT_set_callback(static_cast<NVICITProfile>(getIRQn(irq)), &forwardIT<3>, this);
                    break;
            }
        }

        inline bool isITPending(TimerITProfile it)
        {
            return TIM_GetITStatus(getTIM_TypeDef(), static_cast<uint16_t>(it)) != RESET;
        }

        inline void clearITPending(TimerITProfile it)
        {
            TIM_ClearITPendingBit(getTIM_TypeDef(), static_cast<uint16_t>(it));
        }

        // ------------------------------------------------------------------
        // DMA 接口
        // 与串口不同, 不写死 DMA 数据流/通道/突发目标/触发源:
        // 数据流+通道由调用方给 DMAProfile, 突发目标由 DMABase+BurstLength 指定,
        // 触发源由 TimerDMASourceProfile 指定, 外设侧固定为 TIMx->DMAR.
        // ------------------------------------------------------------------
        inline uint32_t getDMAR()
        {
            return reinterpret_cast<uint32_t>(&getTIM_TypeDef()->DMAR);
        }

        /**
         * @brief 配置 DMAR 突发: 起始寄存器 + 突发长度
         * @param dmabBase        起始寄存器, 见 TimerDMABaseProfile(如 CCR1)
         * @param dmabBurstLength 突发长度, 传 stm32f4xx_tim.h 的
         *                        TIM_DMABurstLength_xTransfers 宏
         */
        void setDMABurstConfig(TimerDMABaseProfile dmabBase, uint16_t dmabBurstLength)
        {
            TIM_DMAConfig(getTIM_TypeDef(), static_cast<uint16_t>(dmabBase), dmabBurstLength);
        }

        void setDMARequest(TimerDMASourceProfile source, FunctionalState newState = ENABLE)
        {
            TIM_DMACmd(getTIM_TypeDef(), static_cast<uint16_t>(source), newState);
        }

        /**
         * @brief CCx DMA 请求选择(CCUS 位), DShot 等"更新事件搬运 CCR 突发"场景需要
         */
        void setCCDMA(FunctionalState newState = ENABLE)
        {
            TIM_SelectCCDMA(getTIM_TypeDef(), newState);
        }

        /**
         * @brief 一键配置定时器 DMA 并返回 init 好的 DMA 对象(未启动, 由调用方 start())
         *
         * @param buffer           内存侧缓冲区(内存由调用方管理, DMA 使用期间必须有效)
         * @param dmaProfile       DMA 数据流 + 通道, 见 dlx_dma_profile.hpp
         * @param dmaSource        DMA 触发源(Update / CC1~CC4)
         * @param dmabBase         突发起始寄存器(如 CCR1)
         * @param dmabBurstLength  突发长度(TIM_DMABurstLength_xTransfers 宏)
         * @param mode             DMA 传输模式(方向/递增/数据宽度/循环/优先级)
         * @param fifo             FIFO/突发配置
         * @param ccdma            是否使能 CCx DMA 请求选择(CCUS)
         * @warning buffer.len 按字节计(ByteBuffer 的本质语义); dlx::DMA 会按 mode
         *          的内存侧数据宽度自动换算 NDTR(如 HalfWord 宽度 64 字节 = 32 项),
         *          字节数必须是宽度的整数倍.
         */
        DMA setDMA(ByteBuffer &buffer, DMAProfile dmaProfile,
                   TimerDMASourceProfile dmaSource, TimerDMABaseProfile dmabBase, uint16_t dmabBurstLength,
                   DMAModeProfile mode = DMAModeProfile::M2P_PID_MIE_PHalf_MHalf_Nor_Med,
                   DMAFIFOProfile fifo = DMAFIFOProfile::Direct,
                   bool ccdma = false)
        {
            setDMABurstConfig(dmabBase, dmabBurstLength);
            setDMARequest(dmaSource, ENABLE);
            if (ccdma) {
                setCCDMA(ENABLE);
            }
            DMA dma(buffer, dmaProfile);
            dma.init(mode, getDMAR(), 0, fifo);
            return dma;
        }

        /**
         * @brief DShot 一键工厂: 按速率配置定时器时基/GPIO/输出通道/DMA, 返回配置好的 Dshot
         *
         * @param profile DShot 速率(150/300/600 kHz)
         *
         * 自动完成:
         *   1. 按速率换算时基并 init(每 bit 固定 40 个计数周期, 高/低 CCR = 75%/37.5%)
         *   2. 配置 4 路 PWM1 输出通道并开启主输出:
         *      TIM1 -> PE9/PE11/PE13/PE14(AF1); TIM8 -> PC6/PC7/PC8/PC9(AF3)
         *   3. DMAR 突发 CCR1~CCR4, Update 触发 + CCUS, 循环模式 DMA
         *   4. 创建 Dshot: 内部把 DMA 绑定到双缓冲并注册 TC 中断
         *      (传完一块自动开始下一块, 中断里同步新帧到空闲缓冲)
         *
         * @note 返回的 Dshot 尚未启动, 调用方后续 start();
         *       Dshot 的 DMA 中断与缓冲同步在构造时已全部就绪
         * @warning 实例必须采用 Up 计数模式(如 TIM1_Up_DIV1 / TIM8_Up_DIV1):
         *          Down 模式会翻转占空比映射, Center 模式每个周期触发两次
         *          Update 导致 DMA 突发频率翻倍, 都会破坏 DShot 时序;
         *          ClockDivision(DIV1/2/4) 对 DShot 无影响, 任选即可
         * @warning 会覆盖本定时器已有的时基/通道配置, 适合专用于 DShot 的实例
         */
        Dshot setDshot(DshotProfile profile)
        {
            // 从 RCC 寄存器读实际时钟(RCC_GetClocksFreq 按 SWS/PLLCFGR/PPRE2 计算,
            // 不依赖 SystemCoreClock 变量是否与硬件一致);
            // TIM1/TIM8 挂 APB2, APB2 分频 >1 时定时器时钟 = PCLK2 x 2
            RCC_ClocksTypeDef clocks;
            RCC_GetClocksFreq(&clocks);
            // PPRE2 是 CFGR 的 bit[15:13](掩码 0xE000), 必须 >>13 取出 0~7
            uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
            uint32_t timerClock = clocks.PCLK2_Frequency;
            if (ppre2 != 0) {
                timerClock *= 2;
            }

            // 每 bit 固定 40 个计数周期, 由预分频适配速率
            const uint32_t ticksPerBit = 40;
            uint32_t rate = static_cast<uint32_t>(profile); // kHz
            uint32_t counterClock = timerClock / (rate * 1000 * ticksPerBit); // 目标计数时钟
            // 计数时钟为 0 时 prescaler 会下溢成 0xFFFF(实测表现为单 bit 周期被拉长
            // 到 ~16ms), 超过 65536 则 prescaler 截断失真; 两者都 fail-stop 暴露
            // 时钟/速率配置错误, 而不是静默输出错误周期
            if (counterClock == 0 || counterClock > 65536) {
                while (true);
            }
            uint16_t prescaler = static_cast<uint16_t>(counterClock - 1);
            uint16_t period    = static_cast<uint16_t>(ticksPerBit - 1);

            init(prescaler, period);

            // 4 路 PWM1 输出引脚(初始 CCR = 低电平位, 未收到帧时保持低)
            GPIOModeProfile af = getGPIOAFProfile();
            if (isTIM8()) {
                GPIO(GPIOProfile::C6).init(af);
                GPIO(GPIOProfile::C7).init(af);
                GPIO(GPIOProfile::C8).init(af);
                GPIO(GPIOProfile::C9).init(af);
            } else {
                GPIO(GPIOProfile::E9).init(af);
                GPIO(GPIOProfile::EB).init(af);
                GPIO(GPIOProfile::ED).init(af);
                GPIO(GPIOProfile::EE).init(af);
            }

            uint32_t lowPulse = (period + 1) * 3 / 8;
            initOutputChannel(TIMChannelProfile::Channel1, TimerOutputChannelModeProfile::PWM1_OEN_ONDIS_OH_IRst_NSet, lowPulse);
            initOutputChannel(TIMChannelProfile::Channel2, TimerOutputChannelModeProfile::PWM1_OEN_ONDIS_OH_IRst_NSet, lowPulse);
            initOutputChannel(TIMChannelProfile::Channel3, TimerOutputChannelModeProfile::PWM1_OEN_ONDIS_OH_IRst_NSet, lowPulse);
            initOutputChannel(TIMChannelProfile::Channel4, TimerOutputChannelModeProfile::PWM1_OEN_ONDIS_OH_IRst_NSet, lowPulse);
            enableOutputs();

            // DMAR 突发 CCR1~CCR4, Update 事件触发, CCUS 使能(Update 事件搬运 CCR)
            setDMABurstConfig(TimerDMABaseProfile::CCR1, TIM_DMABurstLength_4Transfers);
            setDMARequest(TimerDMASourceProfile::Update, ENABLE);
            setCCDMA(ENABLE);

            // 创建 DMA(占位缓冲, Dshot 构造时会重新绑定到内部双缓冲并注册 TC 中断)
            uint16_t placeholder[DSHOT_BUFFER_ITEMS] = {0};
            ByteBuffer placeholderBuf((uint8_t *)placeholder, sizeof(placeholder));
            DMAProfile dmaProfile = isTIM8() ? DMAProfile::DMA2_Stream1_Channel7   // TIM8_UP
                                             : DMAProfile::DMA2_Stream5_Channel6;  // TIM1_UP
            DMA dma(placeholderBuf, dmaProfile);
            dma.init(DMAModeProfile::M2P_PID_MIE_PHalf_MHalf_Cir_Med, getDMAR(), 0, DMAFIFOProfile::Direct);
            return Dshot(profile, dma, period);
        }
    };

} // namespace dlx

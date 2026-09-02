#pragma once

#include <stdint.h>

namespace dlx
{
    // ------------------------------------------------------------------
    // BasicTimerProfile: 基本计时器实例选择(TIM6 / TIM7)
    // 枚举值 = IRQn 号(同 stm32f4xx.h), 与 USARTProfile 的取值风格一致
    // ------------------------------------------------------------------
    enum class BasicTimerProfile : uint8_t {
        TIM6_Profile = (0x36), // TIM6_DAC_IRQn = 54
        TIM7_Profile = (0x37), // TIM7_IRQn      = 55
    };

    // ------------------------------------------------------------------
    // TIMChannelProfile: 比较输出通道选择
    // ------------------------------------------------------------------
    enum class TIMChannelProfile : uint8_t {
        Channel1 = (0x00),
        Channel2 = (0x01),
        Channel3 = (0x02),
        Channel4 = (0x03),
    };

    // ------------------------------------------------------------------
    // TimerOutputChannelModeProfile: 单个输出通道的比较输出配置, 共 96 种
    //   bit[2:0]  OCMode       : 0=Timing, 1=Active, 2=Inactive, 3=Toggle,
    //                            6=PWM1, 7=PWM2(即 CCMR 的 OCxM 字段值)
    //   bit[3]    OutputState  : 0=Disable, 1=Enable
    //   bit[4]    OutputNState : 0=Disable, 1=Enable
    //   bit[5]    OCPolarity   : 0=High, 1=Low
    //   bit[6]    IdleState    : 0 = OCIdle Reset / OCNIdle Set,
    //                            1 = OCIdle Set / OCNIdle Reset
    //   bit[7]    reserved
    // 命名: <模式>_<OutputState>_<OutputNState>_<极性>_<Idle>
    // 假定: 互补输出必须"互补", 即 TIM_OCNPolarity 恒为 TIM_OCPolarity 取反(bit[5]);
    //       空闲态同理, OCNIdleState 恒与 OCIdleState 相反, 因此两者合并为 1 位(bit[6]).
    // TIM_Pulse 是数值参数(CCR 值), 不进 Profile.
    // 解码: OCMode = (oc & 0x7) << 4; OCPolarity = (oc & 0x20) ? Low : High;
    //       OCNPolarity 恒与 OCPolarity 相反;
    //       OCIdleState = (oc & 0x40) ? Set : Reset; OCNIdleState 恒与其相反.
    // ------------------------------------------------------------------
    enum class TimerOutputChannelModeProfile : uint8_t {
        Timing_ODIS_ONDIS_OH_IRst_NSet             = (0x00),
        Timing_ODIS_ONDIS_OH_ISet_NRst             = (0x40),
        Timing_ODIS_ONDIS_OL_IRst_NSet             = (0x20),
        Timing_ODIS_ONDIS_OL_ISet_NRst             = (0x60),
        Timing_ODIS_ONEN_OH_IRst_NSet              = (0x10),
        Timing_ODIS_ONEN_OH_ISet_NRst              = (0x50),
        Timing_ODIS_ONEN_OL_IRst_NSet              = (0x30),
        Timing_ODIS_ONEN_OL_ISet_NRst              = (0x70),
        Timing_OEN_ONDIS_OH_IRst_NSet              = (0x08),
        Timing_OEN_ONDIS_OH_ISet_NRst              = (0x48),
        Timing_OEN_ONDIS_OL_IRst_NSet              = (0x28),
        Timing_OEN_ONDIS_OL_ISet_NRst              = (0x68),
        Timing_OEN_ONEN_OH_IRst_NSet               = (0x18),
        Timing_OEN_ONEN_OH_ISet_NRst               = (0x58),
        Timing_OEN_ONEN_OL_IRst_NSet               = (0x38),
        Timing_OEN_ONEN_OL_ISet_NRst               = (0x78),
        Active_ODIS_ONDIS_OH_IRst_NSet             = (0x01),
        Active_ODIS_ONDIS_OH_ISet_NRst             = (0x41),
        Active_ODIS_ONDIS_OL_IRst_NSet             = (0x21),
        Active_ODIS_ONDIS_OL_ISet_NRst             = (0x61),
        Active_ODIS_ONEN_OH_IRst_NSet              = (0x11),
        Active_ODIS_ONEN_OH_ISet_NRst              = (0x51),
        Active_ODIS_ONEN_OL_IRst_NSet              = (0x31),
        Active_ODIS_ONEN_OL_ISet_NRst              = (0x71),
        Active_OEN_ONDIS_OH_IRst_NSet              = (0x09),
        Active_OEN_ONDIS_OH_ISet_NRst              = (0x49),
        Active_OEN_ONDIS_OL_IRst_NSet              = (0x29),
        Active_OEN_ONDIS_OL_ISet_NRst              = (0x69),
        Active_OEN_ONEN_OH_IRst_NSet               = (0x19),
        Active_OEN_ONEN_OH_ISet_NRst               = (0x59),
        Active_OEN_ONEN_OL_IRst_NSet               = (0x39),
        Active_OEN_ONEN_OL_ISet_NRst               = (0x79),
        Inactive_ODIS_ONDIS_OH_IRst_NSet           = (0x02),
        Inactive_ODIS_ONDIS_OH_ISet_NRst           = (0x42),
        Inactive_ODIS_ONDIS_OL_IRst_NSet           = (0x22),
        Inactive_ODIS_ONDIS_OL_ISet_NRst           = (0x62),
        Inactive_ODIS_ONEN_OH_IRst_NSet            = (0x12),
        Inactive_ODIS_ONEN_OH_ISet_NRst            = (0x52),
        Inactive_ODIS_ONEN_OL_IRst_NSet            = (0x32),
        Inactive_ODIS_ONEN_OL_ISet_NRst            = (0x72),
        Inactive_OEN_ONDIS_OH_IRst_NSet            = (0x0A),
        Inactive_OEN_ONDIS_OH_ISet_NRst            = (0x4A),
        Inactive_OEN_ONDIS_OL_IRst_NSet            = (0x2A),
        Inactive_OEN_ONDIS_OL_ISet_NRst            = (0x6A),
        Inactive_OEN_ONEN_OH_IRst_NSet             = (0x1A),
        Inactive_OEN_ONEN_OH_ISet_NRst             = (0x5A),
        Inactive_OEN_ONEN_OL_IRst_NSet             = (0x3A),
        Inactive_OEN_ONEN_OL_ISet_NRst             = (0x7A),
        Toggle_ODIS_ONDIS_OH_IRst_NSet             = (0x03),
        Toggle_ODIS_ONDIS_OH_ISet_NRst             = (0x43),
        Toggle_ODIS_ONDIS_OL_IRst_NSet             = (0x23),
        Toggle_ODIS_ONDIS_OL_ISet_NRst             = (0x63),
        Toggle_ODIS_ONEN_OH_IRst_NSet              = (0x13),
        Toggle_ODIS_ONEN_OH_ISet_NRst              = (0x53),
        Toggle_ODIS_ONEN_OL_IRst_NSet              = (0x33),
        Toggle_ODIS_ONEN_OL_ISet_NRst              = (0x73),
        Toggle_OEN_ONDIS_OH_IRst_NSet              = (0x0B),
        Toggle_OEN_ONDIS_OH_ISet_NRst              = (0x4B),
        Toggle_OEN_ONDIS_OL_IRst_NSet              = (0x2B),
        Toggle_OEN_ONDIS_OL_ISet_NRst              = (0x6B),
        Toggle_OEN_ONEN_OH_IRst_NSet               = (0x1B),
        Toggle_OEN_ONEN_OH_ISet_NRst               = (0x5B),
        Toggle_OEN_ONEN_OL_IRst_NSet               = (0x3B),
        Toggle_OEN_ONEN_OL_ISet_NRst               = (0x7B),
        PWM1_ODIS_ONDIS_OH_IRst_NSet               = (0x06),
        PWM1_ODIS_ONDIS_OH_ISet_NRst               = (0x46),
        PWM1_ODIS_ONDIS_OL_IRst_NSet               = (0x26),
        PWM1_ODIS_ONDIS_OL_ISet_NRst               = (0x66),
        PWM1_ODIS_ONEN_OH_IRst_NSet                = (0x16),
        PWM1_ODIS_ONEN_OH_ISet_NRst                = (0x56),
        PWM1_ODIS_ONEN_OL_IRst_NSet                = (0x36),
        PWM1_ODIS_ONEN_OL_ISet_NRst                = (0x76),
        PWM1_OEN_ONDIS_OH_IRst_NSet                = (0x0E),
        PWM1_OEN_ONDIS_OH_ISet_NRst                = (0x4E),
        PWM1_OEN_ONDIS_OL_IRst_NSet                = (0x2E),
        PWM1_OEN_ONDIS_OL_ISet_NRst                = (0x6E),
        PWM1_OEN_ONEN_OH_IRst_NSet                 = (0x1E),
        PWM1_OEN_ONEN_OH_ISet_NRst                 = (0x5E),
        PWM1_OEN_ONEN_OL_IRst_NSet                 = (0x3E),
        PWM1_OEN_ONEN_OL_ISet_NRst                 = (0x7E),
        PWM2_ODIS_ONDIS_OH_IRst_NSet               = (0x07),
        PWM2_ODIS_ONDIS_OH_ISet_NRst               = (0x47),
        PWM2_ODIS_ONDIS_OL_IRst_NSet               = (0x27),
        PWM2_ODIS_ONDIS_OL_ISet_NRst               = (0x67),
        PWM2_ODIS_ONEN_OH_IRst_NSet                = (0x17),
        PWM2_ODIS_ONEN_OH_ISet_NRst                = (0x57),
        PWM2_ODIS_ONEN_OL_IRst_NSet                = (0x37),
        PWM2_ODIS_ONEN_OL_ISet_NRst                = (0x77),
        PWM2_OEN_ONDIS_OH_IRst_NSet                = (0x0F),
        PWM2_OEN_ONDIS_OH_ISet_NRst                = (0x4F),
        PWM2_OEN_ONDIS_OL_IRst_NSet                = (0x2F),
        PWM2_OEN_ONDIS_OL_ISet_NRst                = (0x6F),
        PWM2_OEN_ONEN_OH_IRst_NSet                 = (0x1F),
        PWM2_OEN_ONEN_OH_ISet_NRst                 = (0x5F),
        PWM2_OEN_ONEN_OL_IRst_NSet                 = (0x3F),
        PWM2_OEN_ONEN_OL_ISet_NRst                 = (0x7F),
    };

    // ------------------------------------------------------------------
    // AdvancedTimerProfile: 高级计时器实例 + 时基配置, 共 2 x 9 = 18 种
    //   bit[0]     实例选择      : 0 = TIM1, 1 = TIM8
    //   bit[9:8]   CounterMode   : 0 = Up, 1 = Down, 2 = CenterAligned1
    //   bit[11:10] ClockDivision : 0 = DIV1, 1 = DIV2, 2 = DIV4
    // 命名: <TIMx>_<计数模式>_<时钟分频>
    // 说明: 中心对齐只保留 CenterAligned1(CA2/CA3 仅影响 CC 中断标志位,
    //       对比较输出波形本身无影响); 预分频/周期/重复计数器为数值参数,
    //       不进 Profile.
    // 注意: 比较输出配置不再并入本 Profile, 由 AdvancedTimer::initOutputChannel()
    //       按通道单独传入 TimerOutputChannelModeProfile.
    // 解码: isTIM8 = (atp & 0x1) != 0;
    //       CounterMode = ((atp >> 8) & 0x3); ClockDivision = ((atp >> 10) & 0x3)
    //       移位结果即 stm32f4xx_tim.h 的 TIM_CounterMode_xxx / TIM_CKD_xxx 宏.
    // ------------------------------------------------------------------
    enum class AdvancedTimerProfile : uint16_t {
        TIM1_Up_DIV1     = (0x0000),
        TIM1_Up_DIV2     = (0x0400),
        TIM1_Up_DIV4     = (0x0800),
        TIM1_Down_DIV1   = (0x0100),
        TIM1_Down_DIV2   = (0x0500),
        TIM1_Down_DIV4   = (0x0900),
        TIM1_Center_DIV1 = (0x0200),
        TIM1_Center_DIV2 = (0x0600),
        TIM1_Center_DIV4 = (0x0A00),
        TIM8_Up_DIV1     = (0x0001),
        TIM8_Up_DIV2     = (0x0401),
        TIM8_Up_DIV4     = (0x0801),
        TIM8_Down_DIV1   = (0x0101),
        TIM8_Down_DIV2   = (0x0501),
        TIM8_Down_DIV4   = (0x0901),
        TIM8_Center_DIV1 = (0x0201),
        TIM8_Center_DIV2 = (0x0601),
        TIM8_Center_DIV4 = (0x0A01),
    };

    // ------------------------------------------------------------------
    // TimerIRQProfile: 高级计时器中断通道(每个实例对应 4 个 IRQn)
    // ------------------------------------------------------------------
    enum class TimerIRQProfile : uint8_t {
        Update     = (0x00), // 更新中断(TIM1_UP_TIM10 / TIM8_UP_TIM13)
        CC         = (0x01), // 捕获比较中断(TIM1_CC / TIM8_CC)
        Break      = (0x02), // 刹车中断(TIM1_BRK_TIM9 / TIM8_BRK_TIM12)
        TriggerCOM = (0x03), // 触发/换相中断(TIM1_TRG_COM_TIM11 / TIM8_TRG_COM_TIM14)
    };

    // ------------------------------------------------------------------
    // TimerITProfile: 定时器中断源, 值 = stm32f4xx_tim.h 的 TIM_IT_xxx 宏
    // ------------------------------------------------------------------
    enum class TimerITProfile : uint16_t {
        Update  = (0x0001), // TIM_IT_Update
        CC1     = (0x0002), // TIM_IT_CC1
        CC2     = (0x0004), // TIM_IT_CC2
        CC3     = (0x0008), // TIM_IT_CC3
        CC4     = (0x0010), // TIM_IT_CC4
        COM     = (0x0020), // TIM_IT_COM
        Trigger = (0x0040), // TIM_IT_Trigger
        Break   = (0x0080), // TIM_IT_Break
    };

    // ------------------------------------------------------------------
    // TimerDMASourceProfile: 定时器 DMA 触发源, 值 = stm32f4xx_tim.h 的 TIM_DMA_xxx 宏
    // ------------------------------------------------------------------
    enum class TimerDMASourceProfile : uint16_t {
        Update = (0x0100), // TIM_DMA_Update
        CC1    = (0x0200), // TIM_DMA_CC1
        CC2    = (0x0400), // TIM_DMA_CC2
        CC3    = (0x0800), // TIM_DMA_CC3
        CC4    = (0x1000), // TIM_DMA_CC4
    };

    // ------------------------------------------------------------------
    // TimerDMABaseProfile: DMAR 突发起始寄存器, 值 = stm32f4xx_tim.h 的 TIM_DMABase_xxx 宏
    // 常用目标: CCR1~CCR4(更新事件搬运比较值) / ARR(更新事件改周期) / PSC 等
    // ------------------------------------------------------------------
    enum class TimerDMABaseProfile : uint8_t {
        CR1   = (0x00), // TIM_DMABase_CR1
        CR2   = (0x01), // TIM_DMABase_CR2
        SMCR  = (0x02), // TIM_DMABase_SMCR
        DIER  = (0x03), // TIM_DMABase_DIER
        SR    = (0x04), // TIM_DMABase_SR
        EGR   = (0x05), // TIM_DMABase_EGR
        CCMR1 = (0x06), // TIM_DMABase_CCMR1
        CCMR2 = (0x07), // TIM_DMABase_CCMR2
        CCER  = (0x08), // TIM_DMABase_CCER
        CNT   = (0x09), // TIM_DMABase_CNT
        PSC   = (0x0A), // TIM_DMABase_PSC
        ARR   = (0x0B), // TIM_DMABase_ARR
        RCR   = (0x0C), // TIM_DMABase_RCR
        CCR1  = (0x0D), // TIM_DMABase_CCR1
        CCR2  = (0x0E), // TIM_DMABase_CCR2
        CCR3  = (0x0F), // TIM_DMABase_CCR3
        CCR4  = (0x10), // TIM_DMABase_CCR4
        BDTR  = (0x11), // TIM_DMABase_BDTR
        DCR   = (0x12), // TIM_DMABase_DCR
        OR    = (0x13), // TIM_DMABase_OR
    };

} // namespace dlx

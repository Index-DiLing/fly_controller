#pragma once

#include <stdint.h>

// ==================================================================
// NVIC 优先级分组配置
//
// 使用 NVICPriorityProfile 之前必须直接定义优先级分组(全工程统一,
// 建议放在 stm32f4xx_conf.h 中):
//
//     #define DLX_NVIC_PRIORITY_GROUP 0x500   // 即 NVIC_PriorityGroup_2
//
// 取值必须与 stm32f4xx.h 中的 NVIC_PriorityGroup_0~4 相同, 写成纯整数:
//   0x700 / 0x600 / 0x500 / 0x400 / 0x300
// 定义其他值会编译报错。DLX_NVIC_AutoConfig() 会直接把它传给
// NVIC_PriorityGroupConfig()。
// ==================================================================

#ifndef DLX_NVIC_PRIORITY_GROUP
#error "必须先定义 NVIC 优先级分组: #define DLX_NVIC_PRIORITY_GROUP 0x500 (0x700/0x600/0x500/0x400/0x300, 同 NVIC_PriorityGroup_0~4)"
#error "Use #define DLX_NVIC_PRIORITY_GROUP 0x500 (0x700/0x600/0x500/0x400/0x300 Before AutoConfig()"
#endif

#if DLX_NVIC_PRIORITY_GROUP == 0x700
#define DLX_NVIC_PREEMPTION_PRIORITY_MAX 0
#define DLX_NVIC_SUB_PRIORITY_MAX 15
#elif DLX_NVIC_PRIORITY_GROUP == 0x600
#define DLX_NVIC_PREEMPTION_PRIORITY_MAX 1
#define DLX_NVIC_SUB_PRIORITY_MAX 7
#elif DLX_NVIC_PRIORITY_GROUP == 0x500
#define DLX_NVIC_PREEMPTION_PRIORITY_MAX 3
#define DLX_NVIC_SUB_PRIORITY_MAX 3
#elif DLX_NVIC_PRIORITY_GROUP == 0x400
#define DLX_NVIC_PREEMPTION_PRIORITY_MAX 7
#define DLX_NVIC_SUB_PRIORITY_MAX 1
#elif DLX_NVIC_PRIORITY_GROUP == 0x300
#define DLX_NVIC_PREEMPTION_PRIORITY_MAX 15
#define DLX_NVIC_SUB_PRIORITY_MAX 0
#else
#error "DLX_NVIC_PRIORITY_GROUP 取值非法: 必须为 0x700/0x600/0x500/0x400/0x300 (同 NVIC_PriorityGroup_0~4)"
#endif

namespace dlx
{
    // 优先级组合枚举, 数值 = (抢占优先级 << 4) | 子优先级
    // 可用项由当前分组的 DLX_NVIC_PREEMPTION_PRIORITY_MAX / DLX_NVIC_SUB_PRIORITY_MAX 决定
    enum class NVICPriorityProfile : uint8_t
    {
#if DLX_NVIC_PRIORITY_GROUP == 0x700
        // group 4 (0x700): 抢占 0~0, 子优先级 0~15
        P0_S0 = (0x00),
        P0_S1 = (0x01),
        P0_S2 = (0x02),
        P0_S3 = (0x03),
        P0_S4 = (0x04),
        P0_S5 = (0x05),
        P0_S6 = (0x06),
        P0_S7 = (0x07),
        P0_S8 = (0x08),
        P0_S9 = (0x09),
        P0_S10 = (0x0A),
        P0_S11 = (0x0B),
        P0_S12 = (0x0C),
        P0_S13 = (0x0D),
        P0_S14 = (0x0E),
        P0_S15 = (0x0F),
#endif

#if DLX_NVIC_PRIORITY_GROUP == 0x600
        // group 3 (0x600): 抢占 0~1, 子优先级 0~7
        P0_S0 = (0x00),
        P0_S1 = (0x01),
        P0_S2 = (0x02),
        P0_S3 = (0x03),
        P0_S4 = (0x04),
        P0_S5 = (0x05),
        P0_S6 = (0x06),
        P0_S7 = (0x07),
        P1_S0 = (0x10),
        P1_S1 = (0x11),
        P1_S2 = (0x12),
        P1_S3 = (0x13),
        P1_S4 = (0x14),
        P1_S5 = (0x15),
        P1_S6 = (0x16),
        P1_S7 = (0x17),
#endif

#if DLX_NVIC_PRIORITY_GROUP == 0x500
        // group 2 (0x500): 抢占 0~3, 子优先级 0~3
        P0_S0 = (0x00),
        P0_S1 = (0x01),
        P0_S2 = (0x02),
        P0_S3 = (0x03),
        P1_S0 = (0x10),
        P1_S1 = (0x11),
        P1_S2 = (0x12),
        P1_S3 = (0x13),
        P2_S0 = (0x20),
        P2_S1 = (0x21),
        P2_S2 = (0x22),
        P2_S3 = (0x23),
        P3_S0 = (0x30),
        P3_S1 = (0x31),
        P3_S2 = (0x32),
        P3_S3 = (0x33),
#endif

#if DLX_NVIC_PRIORITY_GROUP == 0x400
        // group 1 (0x400): 抢占 0~7, 子优先级 0~1
        P0_S0 = (0x00),
        P0_S1 = (0x01),
        P1_S0 = (0x10),
        P1_S1 = (0x11),
        P2_S0 = (0x20),
        P2_S1 = (0x21),
        P3_S0 = (0x30),
        P3_S1 = (0x31),
        P4_S0 = (0x40),
        P4_S1 = (0x41),
        P5_S0 = (0x50),
        P5_S1 = (0x51),
        P6_S0 = (0x60),
        P6_S1 = (0x61),
        P7_S0 = (0x70),
        P7_S1 = (0x71),
#endif

#if DLX_NVIC_PRIORITY_GROUP == 0x300
        // group 0 (0x300): 抢占 0~15, 子优先级 0~0
        P0_S0 = (0x00),
        P1_S0 = (0x10),
        P2_S0 = (0x20),
        P3_S0 = (0x30),
        P4_S0 = (0x40),
        P5_S0 = (0x50),
        P6_S0 = (0x60),
        P7_S0 = (0x70),
        P8_S0 = (0x80),
        P9_S0 = (0x90),
        P10_S0 = (0xA0),
        P11_S0 = (0xB0),
        P12_S0 = (0xC0),
        P13_S0 = (0xD0),
        P14_S0 = (0xE0),
        P15_S0 = (0xF0),
#endif
    };
} // namespace dlx

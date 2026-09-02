#pragma once

#include <stdint.h>

#ifdef __cplusplus
#include "dlx_nvic_it_profile.h"
#include "dlx_nvic_priority_profile.h"
#endif

// 中断回调函数指针类型: void handler(void *context)
typedef void (*DLX_IT_Handler)(void *context);

// 中断通道总数(STM32F40_41xxx 的 IRQn 为 0~81, 共 82 个)
#define DLX_IT_CHANNEL_COUNT 82

#ifdef __cplusplus
extern "C" {
#endif

// 全局上下文指针数组: 与回调一一对应, 各中断通道相互独立
extern void *DLX_IT_Contexts[DLX_IT_CHANNEL_COUNT];

// 全局回调函数指针数组, 长度可容纳所有中断通道
extern DLX_IT_Handler DLX_IT_Handlers[DLX_IT_CHANNEL_COUNT];

// 在中断函数中跳转执行对应通道的回调(兼容 C 语言)
void DLX_IT_invoke_callback(uint8_t channel);

#ifdef __cplusplus
} // extern "C"

// C++ 注册接口: 按枚举通道号设置回调及其上下文(非捕获 lambda 可隐式转换为函数指针)
void DLX_IT_set_callback(dlx::NVICITProfile channel, DLX_IT_Handler handler, void *context = 0);

// C++: 按通道 + 优先级组合两个 profile 使能中断(仅 NVIC 使能)
void DLX_NVIC_Init(dlx::NVICITProfile channel, dlx::NVICPriorityProfile priority);

// C++: 按 DLX_NVIC_PRIORITY_GROUP 调用 NVIC_PriorityGroupConfig
void DLX_NVIC_AutoConfig();

// C++: 合并版 = DLX_NVIC_Init + DLX_IT_set_callback
void DLX_IT_init(dlx::NVICITProfile channel, dlx::NVICPriorityProfile priority,
                 DLX_IT_Handler handler, void *context = 0);

#endif

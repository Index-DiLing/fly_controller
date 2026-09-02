#include "stm32f4xx.h"
#include "dlx_nvic_it.h"

// 全局上下文指针数组, 与回调一一对应, 默认全部为空
void *DLX_IT_Contexts[DLX_IT_CHANNEL_COUNT] = {0};

// 全局回调函数指针数组, 默认全部为空
DLX_IT_Handler DLX_IT_Handlers[DLX_IT_CHANNEL_COUNT] = {0};

// C++ 注册接口: 按枚举通道号设置回调及其上下文
void DLX_IT_set_callback(dlx::NVICITProfile channel, DLX_IT_Handler handler, void *context)
{
    uint8_t ch = static_cast<uint8_t>(channel);
    if (ch < DLX_IT_CHANNEL_COUNT)
    {
        DLX_IT_Handlers[ch]  = handler;
        DLX_IT_Contexts[ch]  = context;
    }
}

// 在中断函数中跳转执行对应通道的回调(兼容 C 语言)
void DLX_IT_invoke_callback(uint8_t channel)
{
    if (channel < DLX_IT_CHANNEL_COUNT && DLX_IT_Handlers[channel] != 0)
    {
        DLX_IT_Handlers[channel](DLX_IT_Contexts[channel]);
    }
}

// C++: 按通道 + 优先级组合两个 profile 使能中断
void DLX_NVIC_Init(dlx::NVICITProfile channel, dlx::NVICPriorityProfile priority)
{
    uint8_t raw = static_cast<uint8_t>(priority);

    NVIC_InitTypeDef nvicInit;
    nvicInit.NVIC_IRQChannel                   = static_cast<uint8_t>(channel);
    nvicInit.NVIC_IRQChannelPreemptionPriority = (raw >> 4) & 0xF;
    nvicInit.NVIC_IRQChannelSubPriority        = raw & 0xF;
    nvicInit.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvicInit);
}

// C++: 按 DLX_NVIC_PRIORITY_GROUP 配置优先级分组
/**
 * @brief 按照预定义宏(建议直接定义在项目预处理器定义中)DLX_NVIC_PRIORITY_GROUP工作.
 * 
 */
void DLX_NVIC_AutoConfig()
{
    NVIC_PriorityGroupConfig(DLX_NVIC_PRIORITY_GROUP);
}

// C++: 合并版 = NVIC 使能 + 注册回调
void DLX_IT_init(dlx::NVICITProfile channel, dlx::NVICPriorityProfile priority,
                 DLX_IT_Handler handler, void *context)
{
    DLX_NVIC_Init(channel, priority);
    DLX_IT_set_callback(channel, handler, context);
}

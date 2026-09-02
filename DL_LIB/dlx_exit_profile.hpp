#pragma once
#include <stdint.h>

namespace dlx
{
    // ------------------------------------------------------------------
    // EXTIModeProfile 位布局(uint16_t, 低位 -> 高位):
    //   bit[1:0]  Trigger : 0=上升沿, 1=下降沿, 2=上升+下降沿
    // (EXTI 固定为中断模式, 不做事件模式; 引脚由 GPIOProfile 表达)
    // ------------------------------------------------------------------
    enum class EXTIModeProfile : uint16_t
    {
        Rising        = (0x0000), ///< 上升沿触发
        Falling       = (0x0001), ///< 下降沿触发
        RisingFalling = (0x0002), ///< 双边沿触发
    };
} // namespace dlx

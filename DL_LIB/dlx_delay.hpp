#pragma once
#include <stdint.h>
#include "stm32f4xx.h"

namespace dlx
{
    /**
     * @brief 基于 SysTick 的阻塞延时(全局复用)
     *
     * 通过检测 SysTick->VAL 的递减计数来计时, 全程不修改 SysTick 配置,
     * 因此可以和 RT-Thread 等把 SysTick 用作系统节拍的 RTOS 共存.
     * 若 SysTick 尚未使能(例如系统初始化早期), 退化为近似的 NOP 循环,
     * 保证调用方不会死等.
     *
     * 精度: 一个 SysTick 计数 = 1/SystemCoreClock 秒(168MHz 下约 5.95ns).
     */
    inline void delay_ticks(uint32_t ticks)
    {
        if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0) {
            // SysTick 未使能: 近似延迟(约 4 周期/次), 保证能退出
            volatile uint32_t n = ticks / 4u;
            while (n--) {
                __NOP();
            }
            return;
        }

        const uint32_t load  = SysTick->LOAD + 1u; // 每个节拍周期包含的计数
        uint32_t start       = SysTick->VAL;       // 递减计数器当前值
        uint32_t elapsed     = 0;

        while (elapsed < ticks) {
            const uint32_t now = SysTick->VAL;
            if (now <= start) {
                elapsed += start - now;
            } else {
                // 计数回绕到 LOAD(COUNTFLAG): 补上本周期剩余 + 下周期已走
                elapsed += start + (load - now);
            }
            start = now;
        }
    }

    inline void delay_ns(uint32_t ns)
    {
        const uint32_t ticks = (uint32_t)(((uint64_t)ns * SystemCoreClock) / 1000000000u);
        delay_ticks(ticks);
    }

    inline void delay_us(uint32_t us)
    {
        const uint32_t ticks = (uint32_t)(((uint64_t)us * SystemCoreClock) / 1000000u);
        delay_ticks(ticks);
    }

    inline void delay_ms(uint32_t ms)
    {
        const uint32_t ticks = (uint32_t)(((uint64_t)ms * SystemCoreClock) / 1000u);
        delay_ticks(ticks);
    }
} // namespace dlx

#pragma once 
#include "stm32f4xx.h"

#include "global.h"


namespace dl
{
    // modified in 2026.1.1
    inline void delay_ms(uint32_t time_ms){
        // SysTick_Config(SystemCoreClock / 1000);
        // while ((time_ms = (time_ms - (((SysTick->CTRL) & (1 << 16)) << 15 >> 31))));
        // SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        const uint32_t end = SystemClockMilliseconds + time_ms;
        while (SystemClockMilliseconds < end);
    }
    inline void delay_ticks(uint32_t ticks){
        while (ticks--);
    }
} // namespace dl

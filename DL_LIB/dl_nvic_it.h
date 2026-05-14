#pragma once
#include "stm32f4xx.h"
#include <functional>


namespace dl
{

    extern std::function<void()> DL_IT_Handlers[81];
    
    void DL_IT_set_callback_plus(std::function<void()> callback,uint8_t IRQChannel);
    void DL_IT_invoke_callback_plus(uint8_t IRQChannel);
    
    void NVIC_IT_init_plus(
        uint8_t IRQChannel,                   // 中断源
        uint8_t IRQChannelPreemptionPriority, // 抢占优先级
        uint8_t IRQChannelSubPriority,        // 子优先级
        std::function<void()> callback
    );

}// namespace dl

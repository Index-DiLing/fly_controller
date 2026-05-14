#include "dl_nvic_it.h"

#include "dl_gpio.hpp"
namespace dl
{
    std::function<void()> DL_IT_Handlers[81] = {nullptr};

    
    void DL_IT_set_callback_plus(std::function<void()> callback,uint8_t IRQChannel){
        DL_IT_Handlers[IRQChannel] = callback;
    }
    void DL_IT_invoke_callback_plus(uint8_t IRQChannel){
        if (DL_IT_Handlers[IRQChannel]!=nullptr) {
            DL_IT_Handlers[IRQChannel]();
        }
    }

    void NVIC_IT_init_plus(
        uint8_t IRQChannel,                   // 中断源
        uint8_t IRQChannelPreemptionPriority, // 抢占优先级
        uint8_t IRQChannelSubPriority,        // 子优先级
        std::function<void()> callback
    )
    {
        // todo:优先级数量的检查
        NVIC_InitTypeDef nvic_init{
            .NVIC_IRQChannel                   = IRQChannel,
            .NVIC_IRQChannelPreemptionPriority = IRQChannelPreemptionPriority,
            .NVIC_IRQChannelSubPriority        = IRQChannelSubPriority,
            .NVIC_IRQChannelCmd                = ENABLE
        };
        NVIC_Init(&nvic_init);
        DL_IT_set_callback_plus(callback, IRQChannel);
    }

} // namespace dl

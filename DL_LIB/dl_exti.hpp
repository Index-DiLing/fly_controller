#include "stm32f4xx.h"
#include "dl_nvic_it.h"
#include "dl_gpio.hpp"
//帮F4重构史山

namespace dl
{


    //自包含时钟开启
    void EXTI_Init(
        GPIO_Pin_Config config,
        EXTITrigger_TypeDef trigger,
        uint8_t IRQ_PreemptionPriority,
        uint8_t IRQ_SubPriority,
        std::function<void()> callback
    ){
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG,ENABLE);

        SYSCFG_EXTILineConfig(config.EXTI_PortSourceGPIO,config.EXTI_Line);

        dl::GPIO_Pin_Init(config,GPIO_Mode_IN,GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL);

        EXTI_InitTypeDef EXTI_InitStructure;
        EXTI_InitStructure.EXTI_Line = config.EXTI_Line;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = trigger;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        dl::NVIC_IT_init_plus(config.EXTI_IRQn,IRQ_PreemptionPriority,IRQ_SubPriority,callback);


    }


} // namespace dl

#pragma once

#include <stdint.h>
#include "stm32f4xx.h"
#include "dlx_gpio_profile.h"

namespace dlx
{

    // 仅单根针
    class GPIO
    {
        GPIOProfile profile;

        inline GPIO_TypeDef *getGPIO_TypeDef()
        {
            return (GPIO_TypeDef *)(AHB1PERIPH_BASE + ((static_cast<uint8_t>(profile) >> 4) & 0xF) * 0x400);
        }
        inline uint8_t getGPIO_PinPos()
        {
            return (static_cast<uint8_t>(profile) & 0XF);
        }
        inline uint16_t getGPIO_Pin()
        {
            return 1<<getGPIO_PinPos(); 
        }

        inline uint8_t getAFPinSource(){
            return getGPIO_PinPos();
        }
        inline uint32_t getRCC_AHB1Periph(){
            return ((uint32_t)1) << ((static_cast<uint8_t>(profile) >> 4)&0xF);
        }

        public:
        GPIO(GPIOProfile profile)
            : profile(profile)
        {
#warning [Experimental]
        }
        /**
         * @note 不保证成功 不处理错误 会开启时钟
         * @note 重写自 stm32f4_gpio
         */
        void init(GPIOModeProfile modeProfile)
        {
            RCC_AHB1PeriphClockCmd(getRCC_AHB1Periph(),ENABLE);

            GPIO_TypeDef *GPIOx = getGPIO_TypeDef();

            uint16_t pinpos = getGPIO_PinPos();

            uint16_t modeProfileVal = static_cast<uint16_t>(modeProfile);
            uint32_t mode  = (modeProfileVal >> 5) & 0x3;
            uint16_t oType = (modeProfileVal >> 4) & 0x1;
            uint32_t pupd  = (modeProfileVal >> 2) & 0x3;
            uint32_t speed = modeProfileVal & 0x3;
            uint8_t  afNum = (modeProfileVal >> 7) & 0xF;

            GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
            GPIOx->MODER |= (mode << (pinpos * 2));

            if ((mode == GPIO_Mode_OUT) || (mode == GPIO_Mode_AF)) {
                /* Speed mode configuration */
                GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
                GPIOx->OSPEEDR |= (speed << (pinpos * 2));

                /* Output mode configuration*/
                GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos));
                GPIOx->OTYPER |= (uint16_t)(oType << ((uint16_t)pinpos));
            }

            /* Pull-up Pull down resistor configuration*/
            GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));
            GPIOx->PUPDR |= (pupd << (pinpos * 2));

            if (mode == GPIO_Mode_AF) {
                GPIO_PinAFConfig(GPIOx, getAFPinSource(), afNum);
            }
        }

        GPIO& operator=(bool val){
            if (val)
            {
                GPIO_SetBits(getGPIO_TypeDef(),getGPIO_Pin());
            }else{
                GPIO_ResetBits(getGPIO_TypeDef(),getGPIO_Pin());
            }
            return *this;
        }

        bool read(){
           return GPIO_ReadInputDataBit(getGPIO_TypeDef(),getGPIO_Pin());
        }

    };

} // namespace dlx

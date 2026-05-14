#include "stm32f4xx.h"
#pragma once

namespace dl
 {
    class Timer{
        public:
        TIM_TypeDef* tim;
        uint32_t timClockFreq = 0;
        uint32_t timPeriodCounts = 0;
        //目前仅有高级寄存器
        Timer()=default;
        Timer(
            TIM_TypeDef* _TIM,//计时器
            uint16_t _TIM_Prescaler,      //预分频器,这是给计时器的时钟
            uint16_t _TIM_CounterMode,    //计数模式
            uint32_t _TIM_Period,         //定时器周期
            uint16_t _TIM_ClockDivision,  //分频,给死区会用到,一般没用
            uint8_t _TIM_RepetitionCounter//重复计时器
        ){
            //目前缺个复用配置和时钟开启
            
            

            tim = _TIM;
            if (tim==TIM1)
            {
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
                timClockFreq = 42000000 / _TIM_Prescaler;//预分频
                timPeriodCounts = _TIM_Period;
            }
            TIM_TimeBaseInitTypeDef base{
                .TIM_ClockDivision = _TIM_ClockDivision,
                .TIM_CounterMode = _TIM_CounterMode,
                .TIM_Period  = _TIM_Period,
                .TIM_Prescaler = _TIM_Prescaler,
                .TIM_RepetitionCounter = _TIM_RepetitionCounter
            };
            TIM_TimeBaseInit(_TIM, &base);
            
            //初始化基本计时器
            TIM_Cmd(_TIM, ENABLE);
           
        }
    };
    //计时器输出通道

    class TimerOutputChannel{
        public:
        uint8_t Channel;
        Timer TIM;
        TimerOutputChannel()=default;
        TimerOutputChannel(
            Timer& timer,
            uint16_t _Channel,
            uint16_t _TIM_OCMode,       //Mode,通常只用PWM1/2
            uint16_t _TIM_OutputState,  //输出使能
            uint16_t _TIM_OutputNState, //互补输出使能
            uint32_t _TIM_Pulse,        //脉冲宽度,0到65535,重映射到计数器周期
            uint16_t _TIM_OCPolarity,   //低/高电平有效
            uint16_t _TIM_OCNPolarity,  //互补输出高/低电平有效
            uint16_t _TIM_OCIdleState,  //主输出被禁止时的电平
            uint16_t _TIM_OCNIdleState  //互补输出被禁止时的电平
        ):TIM(timer){
            Channel = _Channel;
            TIM_OCInitTypeDef oci{
                .TIM_OCMode =       _TIM_OCMode,
                .TIM_OutputState =  _TIM_OutputState,
                .TIM_OutputNState = _TIM_OutputNState,
                .TIM_Pulse =        _TIM_Pulse,
                .TIM_OCPolarity =   _TIM_OCPolarity,
                .TIM_OCNPolarity =  _TIM_OCNPolarity,
                .TIM_OCIdleState =  _TIM_OCIdleState,
                .TIM_OCNIdleState = _TIM_OCNIdleState
            };

            if      (Channel == TIM_Channel_1)
            {
                TIM_OC1Init(timer.tim, &oci);
                TIM_OC1PreloadConfig(timer.tim, TIM_OCPreload_Enable);
            }else if (Channel == TIM_Channel_2)
            {
                TIM_OC2Init(timer.tim,&oci);
                TIM_OC2PreloadConfig(timer.tim,TIM_OCPreload_Enable);
            }else if (Channel == TIM_Channel_3)
            {
                TIM_OC3Init(timer.tim,&oci);
                TIM_OC3PreloadConfig(timer.tim,TIM_OCPreload_Enable);

            }else if (Channel == TIM_Channel_4)
            {
                TIM_OC4Init(timer.tim,&oci);
                TIM_OC4PreloadConfig(timer.tim,TIM_OCPreload_Enable);
            }
            

        }
        void setEnable(){
            TIM_CtrlPWMOutputs(TIM.tim,ENABLE);
        }
        void setPulse(uint16_t Polarity){
            if      (Channel == TIM_Channel_1)
            {
                TIM_OC1PolarityConfig(TIM.tim,Polarity);
            }else if (Channel == TIM_Channel_2)
            {   
                TIM_OC2PolarityConfig(TIM.tim,Polarity);}
            else if (Channel == TIM_Channel_3)
            {
                TIM_OC3PolarityConfig(TIM.tim,Polarity);
            }else if (Channel == TIM_Channel_4)
            {
                TIM_OC4PolarityConfig(TIM.tim,Polarity);
            }
        }

    };
 } // namespace dl
 
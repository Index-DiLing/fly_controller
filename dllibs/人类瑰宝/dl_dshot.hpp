#pragma once
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_timer.hpp"
#include "dl_dma.hpp"
#include "dl_delay.hpp"



namespace dl
{
    enum class DSHOT_RATE : uint16_t {
        DSHOT_1200 = 1200,
        DSHOT_600  = 600,
        DSHOT_300  = 300,
        DSHOT_150  = 150
    };
    enum class DSHOT_TIM {
        DSHOT_TIM1,
        DSHOT_TIM8 
    };

    class DShot
    {
        // 这3*4bits我也不知道干啥用的,写0即可
        public:
        //8约等于7.5,高电平
        //4约等于3.75,低电平
       volatile uint16_t realTimeData[64 + 12] = {15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,15, 15, 15, 15, 6, 6, 6, 6 ,6,6,6,6}; // 当没有数据需要传输时,会维持当前值,自动扩容至传输1K次,48=16*4,单帧

       volatile  uint16_t preLoadData[64]       = {0}; // 预装载数据
       volatile  bool exchangeData              = false;
        dl::Timer tim;
        dl::TimerOutputChannel toc1;
        dl::TimerOutputChannel toc2;
        dl::TimerOutputChannel toc3;
        dl::TimerOutputChannel toc4;
        dl::DMA pwmDMA;
        bool isStart = true;
        uint8_t startDelay = 0;
        uint8_t runningDelay = 0;
    public:
        /**
         * @brief 根据输入的值和telemetry位生成完整的16位Dshot数据
         * 
         * @param value 48-2047
         * @param tel  是否启用telemetry回传
         * @return uint16_t  完整的16位Dshot数据
         */
        inline uint16_t appendCCR4forValue(uint16_t value, bool tel)
        {
            uint16_t packet_telemetry = (value << 1) | (tel ? 1 : 0);
            uint8_t i;
            int csum      = 0;
            int csum_data = packet_telemetry;
            for (i = 0; i < 3; i++) {
                csum ^= csum_data; // xor data by nibbles
                csum_data >>= 4;
            }
            csum &= 0xf;
            packet_telemetry = (packet_telemetry << 4) | csum;
            return packet_telemetry; // append checksum
        }
        /**
         * @brief 预装载Dshot数据到preLoadData数组中
         * @note 预装载数据用于DMA传输 telemerty位固定为0
         *
         */
        inline void encodePreLoadDshotData(
            uint16_t value0,
            uint16_t value1,
            uint16_t value2,
            uint16_t value3
        )
        {

            // 计算CCR
            uint16_t dshotData[4] = {
                appendCCR4forValue(value0, false),
                appendCCR4forValue(value1, false),
                appendCCR4forValue(value2, false),
                appendCCR4forValue(value3, false)
            };
            //接下来要将其写入preLoadData, bit' = bit*7+1
            //注意这个算法
            for (uint8_t motor = 0; motor < 4; motor++) {
                for (uint8_t i = 0; i < 16; i++)
                {
                    preLoadData[(15-i)*4+motor]=(dshotData[motor]&1) *15+15; // 4/8
                    dshotData[motor] >>= 1;
                }
            }
            exchangeData = true;//置位,等待写入数据
        }
        inline void encodePreLoadDshotData(uint16_t value[4]){
            uint16_t dshotData[4] = {
                appendCCR4forValue(value[0], false),
                appendCCR4forValue(value[1], false),
                appendCCR4forValue(value[2], false),
                appendCCR4forValue(value[3], false)
            };
            //接下来要将其写入preLoadData, bit' = bit*7+1
            //注意这个算法
            for (uint8_t motor = 0; motor < 4; motor++) {
                for (uint8_t i = 0; i < 16; i++)
                {
                    preLoadData[(15-i)*4+motor]=(dshotData[motor]&1) *15+15; // 4/8
                    dshotData[motor] >>= 1;
                }
            }
            exchangeData = true;//置位,等待写入数据
        }

        /**
         * @brief 创建一个Dshot对象,只处理了TIM1的情况，使用了DMA,自动配置IO和时钟.占用IO:PE9~12
         *
         * @todo 泛化对象，操他妈的Tel回传我能接到个P
         *      
         * @ref GPIO "怎么他妈才能搞个好用的GPIO对象?"
         *      @ref error "错误处理机制"                                                                                                                                 
         *
         * @param timer 计时器对象,可选值 TIM1 TIM8 通用计时器不清楚
         * @param rate 目前没用,默认为150KHz    
         * @note 启用了 DMA2 Stream5 Channel6 , TIM1 Update 事件触发传输
         */

        
        DShot(DSHOT_TIM timer, DSHOT_RATE rate,uint8_t sDelay=100,uint8_t rDelay=240):
            //TODO:你说得对这里得加一坨,兼容不同rate,不过看起来像是摆设

            /**
             * @note 修改:将
             * 
             */
            tim((timer==DSHOT_TIM::DSHOT_TIM1 ? TIM1:TIM8 ),13,TIM_CounterMode_Up,39,TIM_CKD_DIV1,0), // 1.5MHz/10=150KHz
            //1->0,8->1
            toc1(tim, TIM_Channel_1, TIM_OCMode_PWM1, TIM_OutputState_Enable, TIM_OutputNState_Disable, 15, TIM_OCPolarity_High, TIM_OCNPolarity_High, TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset),
            toc2(tim, TIM_Channel_2, TIM_OCMode_PWM1, TIM_OutputState_Enable, TIM_OutputNState_Disable, 15, TIM_OCPolarity_High, TIM_OCNPolarity_High, TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset),
            toc3(tim, TIM_Channel_3, TIM_OCMode_PWM1, TIM_OutputState_Enable, TIM_OutputNState_Disable, 15, TIM_OCPolarity_High, TIM_OCNPolarity_High, TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset),
            toc4(tim, TIM_Channel_4, TIM_OCMode_PWM1, TIM_OutputState_Enable, TIM_OutputNState_Disable, 15, TIM_OCPolarity_High, TIM_OCNPolarity_High, TIM_OCIdleState_Reset, TIM_OCNIdleState_Reset),
            pwmDMA(
                DMA2_Stream5, // TIM1 Update 的 DMA 建议使用 Stream5
                DMA_FLAG_TCIF5,
                DMA_Channel_6,
                (uint32_t)&TIM1->DMAR, // 写到寄存器地址（取地址）,
                (uint32_t)realTimeData,
                DMA_DIR_MemoryToPeripheral,
                76,
                DMA_PeripheralInc_Disable,
                DMA_MemoryInc_Enable,
                DMA_PeripheralDataSize_HalfWord,
                DMA_MemoryDataSize_HalfWord,
                DMA_Mode_Normal,
                DMA_Priority_Medium,
                DMA_FIFOMode_Disable,
                DMA_FIFOThreshold_Full,
                DMA_MemoryBurst_Single,
                DMA_PeripheralBurst_Single
            ),
            startDelay(sDelay),
            runningDelay(rDelay)
            {

            if (timer == DSHOT_TIM::DSHOT_TIM1) {
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
                // IO复用与时钟开启
                dl::GPIO pwmIO(GPIOE, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14, GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL); // 这个IO被复用
                GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
                GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
                GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
                GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

                TIM_DMAConfig(TIM1, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers); // CCR1~4

                TIM_ARRPreloadConfig(TIM1, ENABLE);
                            
                TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
                              
                TIM_SelectCCDMA(TIM1, ENABLE); // 使能在 Update 事件触发时对 CCR1~CCR4 的 DMA 传输
            
                pwmDMA.setInterrupt([this](DMA_Stream_TypeDef *stream) {
                    if (exchangeData) {
                        for (uint8_t i = 0; i < 64; i++) {
                            realTimeData[i] = preLoadData[i];
                        }
                        exchangeData = false;
                    }
                    // 若无异常,重启DMA
                    pwmDMA.reset(76, (uint32_t *)realTimeData);
                    dl::delay_ticks(isStart? startDelay:runningDelay);
                    pwmDMA.start();
                    DMA_ClearITPendingBit(stream, DMA_IT_TCIF5);
                },
                DMA2_Stream5_IRQn,
                DMA_IT_TC, 
                DMA_IT_TCIF5
            );
            
            } else {
                
            }
        }

        void start()
        {
            
            toc1.setEnable();
            toc2.setEnable();
            toc3.setEnable();
            toc4.setEnable();
            pwmDMA.start();//启动传输
            encodePreLoadDshotData(0,0,0,0);
            dl::delay_ms(3600);
            isStart = false;
        }
    };
} // namespace dl

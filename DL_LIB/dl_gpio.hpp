#pragma once
#include <stm32f4xx.h>
/**
 * @note 不包含时钟开启!请手动开启时钟
 */

namespace dl{





class GPIO{
    GPIO_TypeDef* gpiox;
    uint32_t GPIO_Pin;
    GPIOMode_TypeDef GPIO_Mode;
    public:
    //TODO时钟开启的问题
    GPIO(GPIO_TypeDef* gpiox,uint32_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed,GPIOOType_TypeDef GPIO_OType,GPIOPuPd_TypeDef GPIO_PuPd){
        this->gpiox = gpiox;
        this->GPIO_Pin = GPIO_Pin;
        this->GPIO_Mode = GPIO_Mode;
        GPIO_InitTypeDef GPIO_InitStruct{
            .GPIO_Pin = GPIO_Pin,
            .GPIO_Mode = GPIO_Mode,
            .GPIO_Speed = GPIO_Speed,
            .GPIO_OType = GPIO_OType,
            .GPIO_PuPd = GPIO_PuPd
        };
        GPIO_Init(gpiox, &GPIO_InitStruct);
    }
    void write(BitAction BitVal) const {
        GPIO_WriteBit(gpiox, GPIO_Pin, BitVal);
    }
    void set() const {
        GPIO_SetBits(gpiox, GPIO_Pin);
    }
    void reset() const {
        GPIO_ResetBits(gpiox, GPIO_Pin);
    }
    uint8_t read(){
        
        if (GPIO_Mode != GPIO_Mode_IN)
        {
            //error
        }
       return GPIO_ReadInputDataBit(gpiox, GPIO_Pin);
    }
};
}
#define debugInit dl::GPIO debug(GPIOA,GPIO_Pin_11,GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
#define debugGPIO GPIOA,GPIO_Pin_11

#define debugSet GPIO_SetBits(debugGPIO)

#define debugReset GPIO_ResetBits(debugGPIO) 

namespace dl
{
//不可操作,仅为数据结构体封装,这样传递的GPIO+Pin默认作为参数使用且未初始化+未开启时钟/允许重复开启
struct GPIO_Pin_Type
{
    GPIO_TypeDef* gpiox;
    uint32_t pin;
    dl::GPIO_Pin_Type operator=(const uint8_t val){
        val==0 ? GPIO_ResetBits(gpiox, pin) : GPIO_SetBits(gpiox, pin);
        return *this;
    }
};

//不包含外设时钟!!手动开启外设时钟!
//不要存储此结构体,仅编译期使用.运行期使用GPIO_Pin_Type
struct GPIO_Pin_Config
{
    GPIO_Pin_Type pin;

    uint8_t pinAFSource;//GPIO_PinSource0-15
    
    uint32_t EXTI_Line; //EXTI_Line0-15

    uint8_t EXTI_PortSourceGPIO;//EXTI_PortSourceGPIOA-G

    uint8_t EXTI_IRQn;
};

inline void GPIO_Pin_Init(dl::GPIO_Pin_Config pinConfig,GPIOMode_TypeDef mode,GPIOSpeed_TypeDef speed,GPIOOType_TypeDef otype,GPIOPuPd_TypeDef pupd){
    GPIO_InitTypeDef GPIO_InitStruct{
        .GPIO_Pin = pinConfig.pin.pin,
        .GPIO_Mode = mode,
        .GPIO_Speed = speed,
        .GPIO_OType = otype,
        .GPIO_PuPd = pupd
    };
    GPIO_Init(pinConfig.pin.gpiox, &GPIO_InitStruct);
}




#define dPA0  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_0}
#define dPA1  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_1}
#define dPA2  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_2}
#define dPA3  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_3}
#define dPA4  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_4}
#define dPA5  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_5}
#define dPA6  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_6}
#define dPA7  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_7}
#define dPA8  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_8}
#define dPA9  dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_9}
#define dPA10 dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_10}
#define dPA11 dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_11}
#define dPA12 dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_12}
#define dPA13 dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_13}
#define dPA14 dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_14}
#define dPA15 dl::GPIO_Pin_Type{GPIOA,GPIO_Pin_15}
#define dPB0  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_0}
#define dPB1  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_1}
#define dPB2  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_2}
#define dPB3  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_3}
#define dPB4  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_4}
#define dPB5  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_5}
#define dPB6  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_6}
#define dPB7  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_7}
#define dPB8  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_8}
#define dPB9  dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_9}
#define dPB10 dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_10}
#define dPB11 dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_11}
#define dPB12 dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_12}
#define dPB13 dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_13}
#define dPB14 dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_14}
#define dPB15 dl::GPIO_Pin_Type{GPIOB,GPIO_Pin_15}
#define dPC0  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_0}
#define dPC1  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_1}
#define dPC2  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_2}
#define dPC3  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_3}
#define dPC4  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_4}
#define dPC5  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_5}
#define dPC6  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_6}
#define dPC7  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_7}
#define dPC8  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_8}
#define dPC9  dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_9}
#define dPC10 dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_10}
#define dPC11 dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_11}
#define dPC12 dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_12}
#define dPC13 dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_13}
#define dPC14 dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_14}
#define dPC15 dl::GPIO_Pin_Type{GPIOC,GPIO_Pin_15}
#define dPD0  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_0}
#define dPD1  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_1}
#define dPD2  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_2}
#define dPD3  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_3}
#define dPD4  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_4}
#define dPD5  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_5}
#define dPD6  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_6}
#define dPD7  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_7}
#define dPD8  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_8}
#define dPD9  dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_9}
#define dPD10 dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_10}
#define dPD11 dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_11}
#define dPD12 dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_12}
#define dPD13 dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_13}
#define dPD14 dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_14}
#define dPD15 dl::GPIO_Pin_Type{GPIOD,GPIO_Pin_15}
#define dPE0  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_0}
#define dPE1  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_1}
#define dPE2  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_2}
#define dPE3  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_3}
#define dPE4  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_4}
#define dPE5  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_5}
#define dPE6  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_6}
#define dPE7  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_7}
#define dPE8  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_8}
#define dPE9  dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_9}
#define dPE10 dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_10}
#define dPE11 dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_11}
#define dPE12 dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_12}
#define dPE13 dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_13}
#define dPE14 dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_14}
#define dPE15 dl::GPIO_Pin_Type{GPIOE,GPIO_Pin_15}
#define dPF0  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_0}
#define dPF1  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_1}
#define dPF2  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_2}
#define dPF3  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_3}
#define dPF4  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_4}
#define dPF5  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_5}
#define dPF6  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_6}
#define dPF7  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_7}
#define dPF8  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_8}
#define dPF9  dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_9}
#define dPF10 dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_10}
#define dPF11 dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_11}
#define dPF12 dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_12}
#define dPF13 dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_13}
#define dPF14 dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_14}
#define dPF15 dl::GPIO_Pin_Type{GPIOF,GPIO_Pin_15}
#define dPG0  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_0}
#define dPG1  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_1}
#define dPG2  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_2}
#define dPG3  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_3}
#define dPG4  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_4}
#define dPG5  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_5}
#define dPG6  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_6}
#define dPG7  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_7}
#define dPG8  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_8}
#define dPG9  dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_9}
#define dPG10 dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_10}
#define dPG11 dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_11}
#define dPG12 dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_12}
#define dPG13 dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_13}
#define dPG14 dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_14}
#define dPG15 dl::GPIO_Pin_Type{GPIOG,GPIO_Pin_15}









#define dGPIOAPin0 dl::GPIO_Pin_Config{dPA0,GPIO_PinSource0,EXTI_Line0,EXTI_PortSourceGPIOA,EXTI0_IRQn}
#define dGPIOAPin1 dl::GPIO_Pin_Config{dPA1,GPIO_PinSource1,EXTI_Line1,EXTI_PortSourceGPIOA,EXTI1_IRQn}
#define dGPIOAPin2 dl::GPIO_Pin_Config{dPA2,GPIO_PinSource2,EXTI_Line2,EXTI_PortSourceGPIOA,EXTI2_IRQn}
#define dGPIOAPin3 dl::GPIO_Pin_Config{dPA3,GPIO_PinSource3,EXTI_Line3,EXTI_PortSourceGPIOA,EXTI3_IRQn}
#define dGPIOAPin4 dl::GPIO_Pin_Config{dPA4,GPIO_PinSource4,EXTI_Line4,EXTI_PortSourceGPIOA,EXTI4_IRQn}
#define dGPIOAPin5 dl::GPIO_Pin_Config{dPA5,GPIO_PinSource5,EXTI_Line5,EXTI_PortSourceGPIOA,EXTI9_5_IRQn}
#define dGPIOAPin6 dl::GPIO_Pin_Config{dPA6,GPIO_PinSource6,EXTI_Line6,EXTI_PortSourceGPIOA,EXTI9_5_IRQn}
#define dGPIOAPin7 dl::GPIO_Pin_Config{dPA7,GPIO_PinSource7,EXTI_Line7,EXTI_PortSourceGPIOA,EXTI9_5_IRQn}
#define dGPIOAPin8 dl::GPIO_Pin_Config{dPA8,GPIO_PinSource8,EXTI_Line8,EXTI_PortSourceGPIOA,EXTI9_5_IRQn}
#define dGPIOAPin9 dl::GPIO_Pin_Config{dPA9,GPIO_PinSource9,EXTI_Line9,EXTI_PortSourceGPIOA,EXTI9_5_IRQn}
#define dGPIOAPin10 dl::GPIO_Pin_Config{dPA10,GPIO_PinSource10,EXTI_Line10,EXTI_PortSourceGPIOA,EXTI15_10_IRQn}
#define dGPIOAPin11 dl::GPIO_Pin_Config{dPA11,GPIO_PinSource11,EXTI_Line11,EXTI_PortSourceGPIOA,EXTI15_10_IRQn}
#define dGPIOAPin12 dl::GPIO_Pin_Config{dPA12,GPIO_PinSource12,EXTI_Line12,EXTI_PortSourceGPIOA,EXTI15_10_IRQn}
#define dGPIOAPin13 dl::GPIO_Pin_Config{dPA13,GPIO_PinSource13,EXTI_Line13,EXTI_PortSourceGPIOA,EXTI15_10_IRQn}
#define dGPIOAPin14 dl::GPIO_Pin_Config{dPA14,GPIO_PinSource14,EXTI_Line14,EXTI_PortSourceGPIOA,EXTI15_10_IRQn}
#define dGPIOAPin15 dl::GPIO_Pin_Config{dPA15,GPIO_PinSource15,EXTI_Line15,EXTI_PortSourceGPIOA,EXTI15_10_IRQn}
#define dGPIOBPin0 dl::GPIO_Pin_Config{dPB0,GPIO_PinSource0,EXTI_Line0,EXTI_PortSourceGPIOB,EXTI0_IRQn}
#define dGPIOBPin1 dl::GPIO_Pin_Config{dPB1,GPIO_PinSource1,EXTI_Line1,EXTI_PortSourceGPIOB,EXTI1_IRQn}
#define dGPIOBPin2 dl::GPIO_Pin_Config{dPB2,GPIO_PinSource2,EXTI_Line2,EXTI_PortSourceGPIOB,EXTI2_IRQn}
#define dGPIOBPin3 dl::GPIO_Pin_Config{dPB3,GPIO_PinSource3,EXTI_Line3,EXTI_PortSourceGPIOB,EXTI3_IRQn}
#define dGPIOBPin4 dl::GPIO_Pin_Config{dPB4,GPIO_PinSource4,EXTI_Line4,EXTI_PortSourceGPIOB,EXTI4_IRQn}
#define dGPIOBPin5 dl::GPIO_Pin_Config{dPB5,GPIO_PinSource5,EXTI_Line5,EXTI_PortSourceGPIOB,EXTI9_5_IRQn}
#define dGPIOBPin6 dl::GPIO_Pin_Config{dPB6,GPIO_PinSource6,EXTI_Line6,EXTI_PortSourceGPIOB,EXTI9_5_IRQn}
#define dGPIOBPin7 dl::GPIO_Pin_Config{dPB7,GPIO_PinSource7,EXTI_Line7,EXTI_PortSourceGPIOB,EXTI9_5_IRQn}
#define dGPIOBPin8 dl::GPIO_Pin_Config{dPB8,GPIO_PinSource8,EXTI_Line8,EXTI_PortSourceGPIOB,EXTI9_5_IRQn}
#define dGPIOBPin9 dl::GPIO_Pin_Config{dPB9,GPIO_PinSource9,EXTI_Line9,EXTI_PortSourceGPIOB,EXTI9_5_IRQn}
#define dGPIOBPin10 dl::GPIO_Pin_Config{dPB10,GPIO_PinSource10,EXTI_Line10,EXTI_PortSourceGPIOB,EXTI15_10_IRQn}
#define dGPIOBPin11 dl::GPIO_Pin_Config{dPB11,GPIO_PinSource11,EXTI_Line11,EXTI_PortSourceGPIOB,EXTI15_10_IRQn}
#define dGPIOBPin12 dl::GPIO_Pin_Config{dPB12,GPIO_PinSource12,EXTI_Line12,EXTI_PortSourceGPIOB,EXTI15_10_IRQn}
#define dGPIOBPin13 dl::GPIO_Pin_Config{dPB13,GPIO_PinSource13,EXTI_Line13,EXTI_PortSourceGPIOB,EXTI15_10_IRQn}
#define dGPIOBPin14 dl::GPIO_Pin_Config{dPB14,GPIO_PinSource14,EXTI_Line14,EXTI_PortSourceGPIOB,EXTI15_10_IRQn}
#define dGPIOBPin15 dl::GPIO_Pin_Config{dPB15,GPIO_PinSource15,EXTI_Line15,EXTI_PortSourceGPIOB,EXTI15_10_IRQn}
#define dGPIOCPin0 dl::GPIO_Pin_Config{dPC0,GPIO_PinSource0,EXTI_Line0,EXTI_PortSourceGPIOC,EXTI0_IRQn}
#define dGPIOCPin1 dl::GPIO_Pin_Config{dPC1,GPIO_PinSource1,EXTI_Line1,EXTI_PortSourceGPIOC,EXTI1_IRQn}
#define dGPIOCPin2 dl::GPIO_Pin_Config{dPC2,GPIO_PinSource2,EXTI_Line2,EXTI_PortSourceGPIOC,EXTI2_IRQn}
#define dGPIOCPin3 dl::GPIO_Pin_Config{dPC3,GPIO_PinSource3,EXTI_Line3,EXTI_PortSourceGPIOC,EXTI3_IRQn}
#define dGPIOCPin4 dl::GPIO_Pin_Config{dPC4,GPIO_PinSource4,EXTI_Line4,EXTI_PortSourceGPIOC,EXTI4_IRQn}
#define dGPIOCPin5 dl::GPIO_Pin_Config{dPC5,GPIO_PinSource5,EXTI_Line5,EXTI_PortSourceGPIOC,EXTI9_5_IRQn}
#define dGPIOCPin6 dl::GPIO_Pin_Config{dPC6,GPIO_PinSource6,EXTI_Line6,EXTI_PortSourceGPIOC,EXTI9_5_IRQn}
#define dGPIOCPin7 dl::GPIO_Pin_Config{dPC7,GPIO_PinSource7,EXTI_Line7,EXTI_PortSourceGPIOC,EXTI9_5_IRQn}
#define dGPIOCPin8 dl::GPIO_Pin_Config{dPC8,GPIO_PinSource8,EXTI_Line8,EXTI_PortSourceGPIOC,EXTI9_5_IRQn}
#define dGPIOCPin9 dl::GPIO_Pin_Config{dPC9,GPIO_PinSource9,EXTI_Line9,EXTI_PortSourceGPIOC,EXTI9_5_IRQn}
#define dGPIOCPin10 dl::GPIO_Pin_Config{dPC10,GPIO_PinSource10,EXTI_Line10,EXTI_PortSourceGPIOC,EXTI15_10_IRQn}
#define dGPIOCPin11 dl::GPIO_Pin_Config{dPC11,GPIO_PinSource11,EXTI_Line11,EXTI_PortSourceGPIOC,EXTI15_10_IRQn}
#define dGPIOCPin12 dl::GPIO_Pin_Config{dPC12,GPIO_PinSource12,EXTI_Line12,EXTI_PortSourceGPIOC,EXTI15_10_IRQn}
#define dGPIOCPin13 dl::GPIO_Pin_Config{dPC13,GPIO_PinSource13,EXTI_Line13,EXTI_PortSourceGPIOC,EXTI15_10_IRQn}
#define dGPIOCPin14 dl::GPIO_Pin_Config{dPC14,GPIO_PinSource14,EXTI_Line14,EXTI_PortSourceGPIOC,EXTI15_10_IRQn}
#define dGPIOCPin15 dl::GPIO_Pin_Config{dPC15,GPIO_PinSource15,EXTI_Line15,EXTI_PortSourceGPIOC,EXTI15_10_IRQn}
#define dGPIODPin0 dl::GPIO_Pin_Config{dPD0,GPIO_PinSource0,EXTI_Line0,EXTI_PortSourceGPIOD,EXTI0_IRQn}
#define dGPIODPin1 dl::GPIO_Pin_Config{dPD1,GPIO_PinSource1,EXTI_Line1,EXTI_PortSourceGPIOD,EXTI1_IRQn}
#define dGPIODPin2 dl::GPIO_Pin_Config{dPD2,GPIO_PinSource2,EXTI_Line2,EXTI_PortSourceGPIOD,EXTI2_IRQn}
#define dGPIODPin3 dl::GPIO_Pin_Config{dPD3,GPIO_PinSource3,EXTI_Line3,EXTI_PortSourceGPIOD,EXTI3_IRQn}
#define dGPIODPin4 dl::GPIO_Pin_Config{dPD4,GPIO_PinSource4,EXTI_Line4,EXTI_PortSourceGPIOD,EXTI4_IRQn}
#define dGPIODPin5 dl::GPIO_Pin_Config{dPD5,GPIO_PinSource5,EXTI_Line5,EXTI_PortSourceGPIOD,EXTI9_5_IRQn}
#define dGPIODPin6 dl::GPIO_Pin_Config{dPD6,GPIO_PinSource6,EXTI_Line6,EXTI_PortSourceGPIOD,EXTI9_5_IRQn}
#define dGPIODPin7 dl::GPIO_Pin_Config{dPD7,GPIO_PinSource7,EXTI_Line7,EXTI_PortSourceGPIOD,EXTI9_5_IRQn}
#define dGPIODPin8 dl::GPIO_Pin_Config{dPD8,GPIO_PinSource8,EXTI_Line8,EXTI_PortSourceGPIOD,EXTI9_5_IRQn}
#define dGPIODPin9 dl::GPIO_Pin_Config{dPD9,GPIO_PinSource9,EXTI_Line9,EXTI_PortSourceGPIOD,EXTI9_5_IRQn}
#define dGPIODPin10 dl::GPIO_Pin_Config{dPD10,GPIO_PinSource10,EXTI_Line10,EXTI_PortSourceGPIOD,EXTI15_10_IRQn}
#define dGPIODPin11 dl::GPIO_Pin_Config{dPD11,GPIO_PinSource11,EXTI_Line11,EXTI_PortSourceGPIOD,EXTI15_10_IRQn}
#define dGPIODPin12 dl::GPIO_Pin_Config{dPD12,GPIO_PinSource12,EXTI_Line12,EXTI_PortSourceGPIOD,EXTI15_10_IRQn}
#define dGPIODPin13 dl::GPIO_Pin_Config{dPD13,GPIO_PinSource13,EXTI_Line13,EXTI_PortSourceGPIOD,EXTI15_10_IRQn}
#define dGPIODPin14 dl::GPIO_Pin_Config{dPD14,GPIO_PinSource14,EXTI_Line14,EXTI_PortSourceGPIOD,EXTI15_10_IRQn}
#define dGPIODPin15 dl::GPIO_Pin_Config{dPD15,GPIO_PinSource15,EXTI_Line15,EXTI_PortSourceGPIOD,EXTI15_10_IRQn}
#define dGPIOEPin0 dl::GPIO_Pin_Config{dPE0,GPIO_PinSource0,EXTI_Line0,EXTI_PortSourceGPIOE,EXTI0_IRQn}
#define dGPIOEPin1 dl::GPIO_Pin_Config{dPE1,GPIO_PinSource1,EXTI_Line1,EXTI_PortSourceGPIOE,EXTI1_IRQn}
#define dGPIOEPin2 dl::GPIO_Pin_Config{dPE2,GPIO_PinSource2,EXTI_Line2,EXTI_PortSourceGPIOE,EXTI2_IRQn}
#define dGPIOEPin3 dl::GPIO_Pin_Config{dPE3,GPIO_PinSource3,EXTI_Line3,EXTI_PortSourceGPIOE,EXTI3_IRQn}
#define dGPIOEPin4 dl::GPIO_Pin_Config{dPE4,GPIO_PinSource4,EXTI_Line4,EXTI_PortSourceGPIOE,EXTI4_IRQn}
#define dGPIOEPin5 dl::GPIO_Pin_Config{dPE5,GPIO_PinSource5,EXTI_Line5,EXTI_PortSourceGPIOE,EXTI9_5_IRQn}
#define dGPIOEPin6 dl::GPIO_Pin_Config{dPE6,GPIO_PinSource6,EXTI_Line6,EXTI_PortSourceGPIOE,EXTI9_5_IRQn}
#define dGPIOEPin7 dl::GPIO_Pin_Config{dPE7,GPIO_PinSource7,EXTI_Line7,EXTI_PortSourceGPIOE,EXTI9_5_IRQn}
#define dGPIOEPin8 dl::GPIO_Pin_Config{dPE8,GPIO_PinSource8,EXTI_Line8,EXTI_PortSourceGPIOE,EXTI9_5_IRQn}
#define dGPIOEPin9 dl::GPIO_Pin_Config{dPE9,GPIO_PinSource9,EXTI_Line9,EXTI_PortSourceGPIOE,EXTI9_5_IRQn}
#define dGPIOEPin10 dl::GPIO_Pin_Config{dPE10,GPIO_PinSource10,EXTI_Line10,EXTI_PortSourceGPIOE,EXTI15_10_IRQn}
#define dGPIOEPin11 dl::GPIO_Pin_Config{dPE11,GPIO_PinSource11,EXTI_Line11,EXTI_PortSourceGPIOE,EXTI15_10_IRQn}
#define dGPIOEPin12 dl::GPIO_Pin_Config{dPE12,GPIO_PinSource12,EXTI_Line12,EXTI_PortSourceGPIOE,EXTI15_10_IRQn}
#define dGPIOEPin13 dl::GPIO_Pin_Config{dPE13,GPIO_PinSource13,EXTI_Line13,EXTI_PortSourceGPIOE,EXTI15_10_IRQn}
#define dGPIOEPin14 dl::GPIO_Pin_Config{dPE14,GPIO_PinSource14,EXTI_Line14,EXTI_PortSourceGPIOE,EXTI15_10_IRQn}
#define dGPIOEPin15 dl::GPIO_Pin_Config{dPE15,GPIO_PinSource15,EXTI_Line15,EXTI_PortSourceGPIOE,EXTI15_10_IRQn}
#define dGPIOFPin0 dl::GPIO_Pin_Config{dPF0,GPIO_PinSource0,EXTI_Line0,EXTI_PortSourceGPIOF,EXTI0_IRQn}
#define dGPIOFPin1 dl::GPIO_Pin_Config{dPF1,GPIO_PinSource1,EXTI_Line1,EXTI_PortSourceGPIOF,EXTI1_IRQn}
#define dGPIOFPin2 dl::GPIO_Pin_Config{dPF2,GPIO_PinSource2,EXTI_Line2,EXTI_PortSourceGPIOF,EXTI2_IRQn}

#define dGPIOFPin3 dl::GPIO_Pin_Config{dPF3,GPIO_PinSource3,EXTI_Line3,EXTI_PortSourceGPIOF,EXTI3_IRQn}

#define dGPIOFPin4 dl::GPIO_Pin_Config{dPF4,GPIO_PinSource4,EXTI_Line4,EXTI_PortSourceGPIOF,EXTI4_IRQn}
#define dGPIOFPin5 dl::GPIO_Pin_Config{dPF5,GPIO_PinSource5,EXTI_Line5,EXTI_PortSourceGPIOF,EXTI9_5_IRQn}
#define dGPIOFPin6 dl::GPIO_Pin_Config{dPF6,GPIO_PinSource6,EXTI_Line6,EXTI_PortSourceGPIOF,EXTI9_5_IRQn}
#define dGPIOFPin7 dl::GPIO_Pin_Config{dPF7,GPIO_PinSource7,EXTI_Line7,EXTI_PortSourceGPIOF,EXTI9_5_IRQn}
#define dGPIOFPin8 dl::GPIO_Pin_Config{dPF8,GPIO_PinSource8,EXTI_Line8,EXTI_PortSourceGPIOF,EXTI9_5_IRQn}
#define dGPIOFPin9 dl::GPIO_Pin_Config{dPF9,GPIO_PinSource9,EXTI_Line9,EXTI_PortSourceGPIOF,EXTI9_5_IRQn}
#define dGPIOFPin10 dl::GPIO_Pin_Config{dPF10,GPIO_PinSource10,EXTI_Line10,EXTI_PortSourceGPIOF,EXTI15_10_IRQn}
#define dGPIOFPin11 dl::GPIO_Pin_Config{dPF11,GPIO_PinSource11,EXTI_Line11,EXTI_PortSourceGPIOF,EXTI15_10_IRQn}
#define dGPIOFPin12 dl::GPIO_Pin_Config{dPF12,GPIO_PinSource12,EXTI_Line12,EXTI_PortSourceGPIOF,EXTI15_10_IRQn}
#define dGPIOFPin13 dl::GPIO_Pin_Config{dPF13,GPIO_PinSource13,EXTI_Line13,EXTI_PortSourceGPIOF,EXTI15_10_IRQn}
#define dGPIOFPin14 dl::GPIO_Pin_Config{dPF14,GPIO_PinSource14,EXTI_Line14,EXTI_PortSourceGPIOF,EXTI15_10_IRQn}
#define dGPIOFPin15 dl::GPIO_Pin_Config{dPF15,GPIO_PinSource15,EXTI_Line15,EXTI_PortSourceGPIOF,EXTI15_10_IRQn}
#define dGPIOGPin0 dl::GPIO_Pin_Config{dPG0,GPIO_PinSource0,EXTI_Line0,EXTI_PortSourceGPIOG,EXTI0_IRQn}
#define dGPIOGPin1 dl::GPIO_Pin_Config{dPG1,GPIO_PinSource1,EXTI_Line1,EXTI_PortSourceGPIOG,EXTI1_IRQn}
#define dGPIOGPin2 dl::GPIO_Pin_Config{dPG2,GPIO_PinSource2,EXTI_Line2,EXTI_PortSourceGPIOG,EXTI2_IRQn}
#define dGPIOGPin3 dl::GPIO_Pin_Config{dPG3,GPIO_PinSource3,EXTI_Line3,EXTI_PortSourceGPIOG,EXTI3_IRQn}
#define dGPIOGPin4 dl::GPIO_Pin_Config{dPG4,GPIO_PinSource4,EXTI_Line4,EXTI_PortSourceGPIOG,EXTI4_IRQn}
#define dGPIOGPin5 dl::GPIO_Pin_Config{dPG5,GPIO_PinSource5,EXTI_Line5,EXTI_PortSourceGPIOG,EXTI9_5_IRQn}
#define dGPIOGPin6 dl::GPIO_Pin_Config{dPG6,GPIO_PinSource6,EXTI_Line6,EXTI_PortSourceGPIOG,EXTI9_5_IRQn}
#define dGPIOGPin7 dl::GPIO_Pin_Config{dPG7,GPIO_PinSource7,EXTI_Line7,EXTI_PortSourceGPIOG,EXTI9_5_IRQn}
#define dGPIOGPin8 dl::GPIO_Pin_Config{dPG8,GPIO_PinSource8,EXTI_Line8,EXTI_PortSourceGPIOG,EXTI9_5_IRQn}
#define dGPIOGPin9 dl::GPIO_Pin_Config{dPG9,GPIO_PinSource9,EXTI_Line9,EXTI_PortSourceGPIOG,EXTI9_5_IRQn}
#define dGPIOGPin10 dl::GPIO_Pin_Config{dPG10,GPIO_PinSource10,EXTI_Line10,EXTI_PortSourceGPIOG,EXTI15_10_IRQn}
#define dGPIOGPin11 dl::GPIO_Pin_Config{dPG11,GPIO_PinSource11,EXTI_Line11,EXTI_PortSourceGPIOG,EXTI15_10_IRQn}
#define dGPIOGPin12 dl::GPIO_Pin_Config{dPG12,GPIO_PinSource12,EXTI_Line12,EXTI_PortSourceGPIOG,EXTI15_10_IRQn}
#define dGPIOGPin13 dl::GPIO_Pin_Config{dPG13,GPIO_PinSource13,EXTI_Line13,EXTI_PortSourceGPIOG,EXTI15_10_IRQn}
#define dGPIOGPin14 dl::GPIO_Pin_Config{dPG14,GPIO_PinSource14,EXTI_Line14,EXTI_PortSourceGPIOG,EXTI15_10_IRQn}
#define dGPIOGPin15 dl::GPIO_Pin_Config{dPG15,GPIO_PinSource15,EXTI_Line15,EXTI_PortSourceGPIOG,EXTI15_10_IRQn}
} // namespace dl

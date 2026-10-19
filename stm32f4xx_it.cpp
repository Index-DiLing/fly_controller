/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "rt-thread/include/rtthread.h"

#include "dlx_nvic_it.h"
#include "app/runtime/app_fault.hpp"

#ifdef __cplusplus
extern "C" {
#endif
//#include "dl_nvic_it_plus.hpp"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

//===================================================================================================
// 硬件故障处理器(2026-10-02 补)
//
// 之前: HardFault_Handler 被注释掉(落到启动文件的 weak 默认 = 死循环), MemManage/BusFault/UsageFault
//       各自 while(1) —— **没有任何"把飞机弄安全"的动作**。飞控挂死时电机还在按最后一帧 DShot 转,
//       现场表现只是"灯不闪了、飞机没反应", 看不出是死机还是链路断了。
//
// 现在: 统一进 dlxFaultTrap() —— **先断动力, 再用 LED 闪码报故障类型**。
//       顺序不能反: 闪灯是给人看的, 断动力是保命的。
//
// 刻意不做的: 不加 IWDG(按项目决定)。代价要清楚 ——
//       **没有故障、纯逻辑死循环**(比如某个驱动 while 等标志位)不会被恢复。
//       替代办法是在驱动层把所有忙等都加上超时(B4/B5 会做)。
//
// 自包含原因: 故障可能发生在 GPIO/时钟初始化**之前**, 所以这里只写寄存器, 不调用 GPIO 类、
//       不用 delay_ms()、不用任何可能再次出错的东西。
//===================================================================================================
enum
{
    DLX_FAULT_NONE  = 0,
    DLX_FAULT_HARD  = 1, ///< HardFault
    DLX_FAULT_MEM   = 2, ///< MemManage(MPU 违规)
    DLX_FAULT_BUS   = 3, ///< BusFault(总线错误)
    DLX_FAULT_USAGE = 4, ///< UsageFault(非法指令/未对齐/除零)
    DLX_FAULT_NMI   = 5, ///< NMI(通常 = CSS 时钟失效)
};

static volatile uint32_t g_dlxFaultKind = DLX_FAULT_NONE;

/** 纯空转延时(不用 delay_ms: 它依赖 SysTick/RT-Thread, 故障时不可信)。频率只需"看得出几闪"。 */
static void dlxFaultDelay(volatile uint32_t loops)
{
    while (loops-- != 0u)
    {
    }
}

/**
 * @brief 故障陷阱: 断动力 → LED 闪码 → 死等
 *
 * 闪码约定: 长亮 200ms 作前导, 然后 **短闪 N 次**(N = 故障类型 1..5), 组间停 800ms, 循环。
 * 硬编码 PF3 / TIM1 是刻意的(自包含); 改引脚/定时器时这两处要同步 —— 分别对应
 * cfg::hw::led::kRun 与 cfg::hw::motor::kTimer(config/flight_config.json)。
 */
static void dlxFaultTrap(uint32_t kind)
{
    g_dlxFaultKind = kind;

    // ---- ① 断动力(只写寄存器) ----
    // 与 release.cpp 的停机分支同一顺序: 关 DMA 请求 → 停计数 → 关主输出。
    // 真正让电调停下的是 MOE 那一位; 前两步是防止残余的 DMA 请求继续推动比较输出。
    // 这三位与驱动的对应关系(已核对 dlx_timer.hpp):
    //   DIER.UDE  <- setDMARequest(TimerDMASourceProfile::Update, ENABLE)   (内部 = TIM_DMACmd(TIM_DMA_Update))
    //   CR1.CEN   <- AdvancedTimer::stop()                                 (内部 = TIM_Cmd(DISABLE))
    //   BDTR.MOE  <- AdvancedTimer::disableOutputs()                       (内部 = TIM_CtrlPWMOutputs(DISABLE))
    TIM1->DIER &= ~TIM_DIER_UDE;  // 关 TIM1_UP 的 DMA 请求(DShot 帧不再发出)
    TIM1->CR1 &= ~TIM_CR1_CEN;    // 停计数
    TIM1->BDTR &= ~TIM_BDTR_MOE;  // ★ 关主输出(4 路电机信号回到无效电平, 电调超时停转)

    // ---- ② LED 闪码(直接操作寄存器, 不依赖是否初始化过) ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    GPIOF->MODER &= ~(3u << (3u * 2u)); // PF3 清成输入
    GPIOF->MODER |= (1u << (3u * 2u));  // PF3 = 通用推挽输出
    GPIOF->OTYPER &= ~(1u << 3u);
    GPIOF->OSPEEDR |= (2u << (3u * 2u)); // 中速

    for (;;)
    {
        GPIOF->BSRR = GPIO_BSRR_BS_3; // 前导长亮
        dlxFaultDelay(2000000u);
        GPIOF->BSRR = GPIO_BSRR_BR_3;
        dlxFaultDelay(3000000u);

        for (uint32_t i = 0u; i < kind; ++i)
        {
            GPIOF->BSRR = GPIO_BSRR_BS_3;
            dlxFaultDelay(1200000u);
            GPIOF->BSRR = GPIO_BSRR_BR_3;
            dlxFaultDelay(1200000u);
        }
        dlxFaultDelay(8000000u); // 组间隔
    }
}

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    // NMI 在 F4 上一般是 CSS(时钟安全系统)失效。返回去只会跑在 HSI 上继续飞 —— 进陷阱更安全。
    dlxFaultTrap(DLX_FAULT_NMI);
}
extern "C" rt_err_t dlxExceptionHook(void * /*context*/)
{
    // 不返回：断动力 + LED 闪码由 dlxFaultTrap 完成(它内部是死循环)。
    // 返回 RT_EOK 的话 RT-Thread 会继续往下走它自己的"打印线程信息再挂起"流程 ——
    // 那对我们没有意义：飞行中需要的是**立刻停桨**，不是打印。
    dlxFaultTrap(DLX_FAULT_HARD);
    return -RT_ERROR; // 到不了这里(编译器知道 dlxFaultTrap 不返回)
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    dlxFaultTrap(DLX_FAULT_MEM);
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    dlxFaultTrap(DLX_FAULT_BUS);
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    dlxFaultTrap(DLX_FAULT_USAGE);
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
// void PendSV_Handler(void)
// {
// }

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
// volatile uint32_t SystemClockMilliseconds = 0;
// void SysTick_Handler(void)
// {
//     rt_os_tick_callback();
// }


// 串口中断: 每个 handler 只做"转发到注册表"这一件事(实际处理在注册的回调里)。
// 当前分配(2026-10-02):
//   USART1 = 调试/地面站串口(文本日志 + DLX 报文)
//   USART2 = GNSS NEO-7M(PD5/PD6)  —— 中断只用 IDLE 切帧, 不需要逐字节 RXNE
//   USART3 = E220-400T22S 链路(PD8/PD9) —— 同上
// 注意: **DMA 的中断这里一个都没开**。IDLE 切帧靠"中断里读 NDTR 算写指针",
//       溢出不靠 DMA 中断而靠轮询 TC 标志(见 dlx_usart.hpp 的 USARTDmaReceiver),
//       所以新增两条链路不需要动 DMA handler 表, 也不会与控制环抢中断。
void USART1_IRQHandler(void)
{
    DLX_IT_invoke_callback(USART1_IRQn);
}

void USART2_IRQHandler(void)
{
    DLX_IT_invoke_callback(USART2_IRQn);
}

void USART3_IRQHandler(void)
{
    DLX_IT_invoke_callback(USART3_IRQn);
}

// EXTI: 全部 16 条线都留了 handler(见下面)。VL53L1X 的 GPIO1 在 PA3 ⇒ EXTI3。
// 用哪条线、上升沿还是下降沿, 由 dlx_exti.hpp 的 EXTI 类在 init() 时按配置设。

void DMA2_Stream5_IRQHandler(void)
{
    DLX_IT_invoke_callback(DMA2_Stream5_IRQn);
}

void EXTI0_IRQHandler(void)
{
    DLX_IT_invoke_callback(EXTI0_IRQn);
}

void EXTI1_IRQHandler(void)
{
    DLX_IT_invoke_callback(EXTI1_IRQn);
}

void EXTI2_IRQHandler(void)
{
    DLX_IT_invoke_callback(EXTI2_IRQn);
}

void EXTI3_IRQHandler(void)
{
    DLX_IT_invoke_callback(EXTI3_IRQn);
}

void EXTI4_IRQHandler(void)
{
    DLX_IT_invoke_callback(EXTI4_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    DLX_IT_invoke_callback(EXTI9_5_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
    DLX_IT_invoke_callback(EXTI15_10_IRQn);
}

void TIM7_IRQHandler(void)
{
  DLX_IT_invoke_callback(TIM7_IRQn);
}



#ifdef __cplusplus
}
#endif

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 



/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-24                  the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include "stm32f4xx.h"
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
/*
 * Please modify RT_HEAP_SIZE if you enable RT_USING_HEAP
 * the RT_HEAP_SIZE max value = (sram size - ZI size), 1024 means 1024 bytes
 *
 * 发布版的线程栈全部从这里分配。**实测**(2026-10-03 首次烧录的自检日志):
 *   main 线程栈 RT_MAIN_THREAD_STACK_SIZE = 24KB(控制环 + EKF 的大矩阵临时量)
 *   建线程后 已用=35000 峰值=35000 空闲=5880 / 总=40880 字节
 *   (当时的线程: logger 3KB + link_tel 3KB + 采集 1KB)
 *
 * 40KB 只剩 5.8KB(14%)—— 而 2026-10-03 的台架配置里 **GNSS 与气压计是编译期关掉的**:
 * 把它们打开还要再加两个采集线程(~2.2KB) ⇒ 余量会掉到 3.6KB。余量这么薄的风险不是"跑不动",
 * 而是**某个线程创建失败** —— 表现是"某个传感器永远没数据", 而根因在堆上。
 * 所以提到 48KB: 片内 128KB RAM, 抬高后仍然由链接器把关(装不下会报 L6220E, 不是悄悄变砖)。
 * 注意: 若把 RT_MAIN_THREAD_STACK_SIZE 调大, 这里也要同步调大, 否则线程创建会失败。
 */
#define RT_HEAP_SIZE (48*1024)
static rt_uint8_t rt_heap[RT_HEAP_SIZE];

RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

void rt_os_tick_callback(void)
{
    rt_interrupt_enter();
    
    rt_tick_increase();

    rt_interrupt_leave();
}

void SysTick_Handler(void)
{
    rt_os_tick_callback();
}

/**
 * This function will initial your board.
 */
void rt_hw_board_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
    /* 
     * TODO 1: OS Tick Configuration
     * Enable the hardware timer and call the rt_os_tick_callback function
     * periodically with the frequency RT_TICK_PER_SECOND. 
     */

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

#ifdef RT_USING_CONSOLE

static int uart_init(void)
{
#error "TODO 2: Enable the hardware uart and config baudrate."
    return 0;
}
INIT_BOARD_EXPORT(uart_init);

void rt_hw_console_output(const char *str)
{
#error "TODO 3: Output the string 'str' through the uart."
}

#endif

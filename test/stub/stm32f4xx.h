#pragma once
//===================================================================================================
// test/stub/stm32f4xx.h —— 主机侧测试用的最小替身
//
// 两个信箱的共享状态用 dlx::InterruptLock 保护, 而它只用 CMSIS 的三个原语(见 DL_LIB/dlx_interrupt_lock.hpp)。
// 单线程测试里没有并发可言, 所以这里就是"记下 PRIMASK / 什么都不做", 语义与真实现一致 —— 不给测试开小灶。
//
// 用这个替身而不是给 RT-Thread 做替身, 是因为信箱本身**不再依赖 RT-Thread**(依赖只剩 MCU 厂商头)。
// 编译测试时把本目录放在 -I 的最前面, 它就会遮住真的 stm32f4xx.h。
//===================================================================================================
#include <stdint.h>

inline uint32_t __get_PRIMASK()
{
    return 0u;
}

inline void __disable_irq()
{
}

inline void __set_PRIMASK(uint32_t primask)
{
    (void)primask;
}

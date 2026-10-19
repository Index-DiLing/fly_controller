#pragma once
//===================================================================================================
// dlx_interrupt_lock.hpp —— 中断屏蔽的保存-恢复(RAII, 不依赖 RTOS)
//
// 用途: 保护"ISR 与被它打断的线程都会碰"的那几行共享状态。
//
// 为什么不是 rt_enter_critical(): 它只在修改调度锁计数器的几个周期里关中断, 返回时中断已经打开
//   (见 rt-thread/src/scheduler.c:921-936)—— 它只阻止**线程切换**, 挡不住中断, 互斥范围是"线程 vs 线程"。
//   所以本文件刻意不用 RT-Thread: 直接用 CMSIS 的 PRIMASK 操作, 依赖只剩 MCU 厂商头(core_cm4.h,
//   经 stm32f4xx.h 进来 —— 工程里每个驱动本来就依赖它)。
//
// 语义与开销:
//   - 构造时**保存** PRIMASK 再关中断, 析构时**恢复**保存值 ⇒ 可嵌套, 不会把外层已经关掉的中断打开
//     (比手动 __enable_irq() 安全: 手动恢复会把外层的屏蔽一起打开);
//   - 关中断期间中断是**被推迟**而不是被丢弃: 所以窗口里只能放几条指令。窗口大小由调用方保证 ——
//     每个使用点都在注释里写明窗口里到底有哪几行;
//   - 恢复交给作用域, 防止"某条 return 忘了开中断"这类最难查的问题。
//===================================================================================================
#include <stdint.h>

#include "stm32f4xx.h" // __get_PRIMASK / __disable_irq / __set_PRIMASK(CMSIS, core_cm4.h)

namespace dlx
{

/** @brief 作用域内的中断屏蔽(保存-恢复, 可嵌套) */
class InterruptLock
{
public:
    InterruptLock()
        : savedPrimask(__get_PRIMASK())
    {
        __disable_irq();
    }

    ~InterruptLock()
    {
        __set_PRIMASK(savedPrimask);
    }

    InterruptLock(const InterruptLock &) = delete;
    InterruptLock &operator=(const InterruptLock &) = delete;

private:
    uint32_t savedPrimask;
};

} // namespace dlx

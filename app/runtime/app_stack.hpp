#pragma once
#include "rtthread.h"
#include <stdint.h>
// 线程启动时用 0xA5 标记尚未使用的栈区，再扫描标记估算高水位；不覆盖当前栈帧。

namespace dlx
{
inline uint32_t readStackPointer()
{
    uint32_t sp;
    __asm volatile("mov %0, sp" : "=r"(sp));
    return sp;
}

/** 给当前线程的"未使用栈区"打上 0xA5 标记(线程刚启动时调用一次) */
inline void paintStackWatermark()
{
    rt_thread_t self = rt_thread_self();
    if (self == RT_NULL || self->stack_addr == RT_NULL) {
        return;
    }
    uint8_t *base  = static_cast<uint8_t *>(self->stack_addr);
    uint8_t *limit = reinterpret_cast<uint8_t *>(readStackPointer()) - 64u;
    for (uint8_t *p = base; p < limit; ++p) {
        *p = 0xA5;
    }
}

/** 某个线程用掉多少栈(字节, 近似; 没打过水位线时返回 0) */
inline uint32_t stackUsedBytes(rt_thread_t tid)
{
    if (tid == RT_NULL || tid->stack_addr == RT_NULL) {
        return 0u;
    }
    const uint8_t *base = static_cast<const uint8_t *>(tid->stack_addr);
    uint32_t n          = 0u;
    while (n < tid->stack_size && base[n] == 0xA5u) {
        ++n;
    }
    return (n >= tid->stack_size) ? 0u : (tid->stack_size - n);
}
} // namespace dlx

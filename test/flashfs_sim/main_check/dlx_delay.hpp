#pragma once
#include <stdint.h>

// 主机跑测用: 累计"毫秒", 超过一定次数就抛异常把上板测试的收尾死循环顶出去
struct OnboardRunDone
{
};

inline uint32_t &runClockMs()
{
    static uint32_t t = 0u;
    return t;
}

inline void delay_ms(uint32_t ms)
{
    static uint32_t calls = 0u;
    runClockMs() += ms;
    if (++calls > 350u) {
        throw OnboardRunDone();
    }
}

inline void delay_us(uint32_t) {}

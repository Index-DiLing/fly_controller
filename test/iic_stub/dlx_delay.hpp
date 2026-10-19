#pragma once
//===================================================================================================
// test/iic_stub/dlx_delay.hpp —— 主机空延时(意图声明, 当前被 GCC 的引号 include 规则"绕过")
//
// 现状说明(重要, 免得review时误判):
//   DL_LIB/dlx_iic.hpp 里写的是 `#include "dlx_delay.hpp"`。GCC 对引号形式的搜索顺序是
//   "**被包含文件所在目录** -> -iquote -> -I", 所以 DL_LIB/dlx_delay.hpp 一定排在
//   -I test/iic_stub 前面 —— 同一个 DL_LIB/ 目录下的同名头**无法**用 -I 顺序 shadow
//   (与 test/bmp388_stub 不同: dlx_bmp388.hpp 在 DL_LIB/BMP388/ 下, 那里的引号搜索
//    先看 DL_LIB/BMP388/, 没有 dlx_delay.hpp, 才落到 -I test/bmp388_stub)。
//
//   所以本次真正让"真 dlx_delay.hpp 也能跑"的做法是: test/iic_stub/stm32f4xx.h 补上
//   SysTick / __NOP / SystemCoreClock, 并让 SysTick 的 ENABLE 位保持 0 —— 真 dlx_delay.hpp
//   会走"SysTick 未使能 -> 有界 __NOP 循环"的分支, 主机上既是空延时语义也不会死等。
//
//   本文件保留, 一是把"主机上延时应为空"这个意图写成显式文件, 二是万一将来 include 路径
//   调整(例如把驱动拆到子目录), 它会立刻生效而不用再补。
//===================================================================================================
#include <stdint.h>

namespace dlx
{
    inline void delay_us(uint32_t us)
    {
        (void)us;
    }

    inline void delay_ms(uint32_t ms)
    {
        (void)ms;
    }
} // namespace dlx

// 与本仓库其他替身保持一致: 同时提供全局版本, 免得将来有代码在 dlx 命名空间外调用
inline void delay_us(uint32_t us)
{
    (void)us;
}

inline void delay_ms(uint32_t ms)
{
    (void)ms;
}

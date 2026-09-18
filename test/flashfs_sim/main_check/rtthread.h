#pragma once
// 语法检查用的桩
#include <stdint.h>
#include "dlx_delay.hpp"
inline uint32_t rt_tick_get() { return runClockMs(); }

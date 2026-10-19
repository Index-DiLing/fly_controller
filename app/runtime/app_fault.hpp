#pragma once
#include "rt-thread/include/rtthread.h"
#include "rt-thread/include/rthw.h"

#ifdef __cplusplus
extern "C" {
#endif
rt_err_t dlxExceptionHook(void *context);

#ifdef __cplusplus
}
#endif

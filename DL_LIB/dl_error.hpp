
#pragma once
#include "stdint.h"
uint32_t globalErrorCode = 0;


namespace dl
{
    /**
     * @brief 使用时间戳来生成此代码
     *
     * @param code
     */
    inline void error(uint32_t code, const char *msg)
    {
        globalErrorCode = code;
    }

} // namespace dl

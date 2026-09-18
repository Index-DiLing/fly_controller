#pragma once
#include "dlx_gpio_profile.h"

// 主机端自检用的 GPIO 桩(只保证 flight_config_struct.hpp 里的工具函数能编译)
namespace dlx
{
    class GPIO
    {
    public:
        explicit GPIO(GPIOProfile) {}
        void   init(GPIOModeProfile) {}
        bool   read() const { return true; }
        GPIO  &operator=(bool) { return *this; }
        GPIO  &operator=(int) { return *this; }
    };
} // namespace dlx

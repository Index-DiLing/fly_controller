#pragma once
#include "dlx_gpio_profile.h"
namespace dlx
{
    class GPIO
    {
    public:
        GPIO(GPIOProfile) {}
        void init(GPIOModeProfile) {}
        GPIO &operator=(int) { return *this; }
    };
}

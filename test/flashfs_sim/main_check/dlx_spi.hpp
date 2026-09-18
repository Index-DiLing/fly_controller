#pragma once
#include <stdint.h>
#include "dlx_gpio_profile.h"
namespace dlx
{
    enum class SPIModeProfile
    {
        FD_M_8B_CL_1E_NS_BR8_MSB,
    };
    class SPI
    {
    public:
        static SPI SPI2_SB10_MIC2_MOC3(GPIOProfile) { return SPI(); }
        void init(SPIModeProfile) {}
    };
}

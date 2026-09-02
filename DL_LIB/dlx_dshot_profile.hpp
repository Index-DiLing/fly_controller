#pragma once
#include <stdint.h>

namespace dlx{
    enum class DshotProfile:uint16_t
    {
        Dshot150 = 150,
        Dshot300 = 300,
        Dshot600 = 600,
    };

    // DShot 单块 DMA 缓冲布局(4 通道并行, 每个 DMA 项 = 1 次 Update 突发写入 CCR1~CCR4):
    //   64 个数据项 = 4 电机 x 16 bit(bit15 在前, MSB first)
    //   16 个间隔项 = 4 个 bit 周期 x 4 通道, 全部写 0(恒低), 用于区分两帧
    //   (实测部分电调对 3 bit 间隔不识别, 4 bit 起步才可靠)
    constexpr uint16_t DSHOT_DATA_ITEMS   = 64;
    constexpr uint16_t DSHOT_GAP_ITEMS    = 24;
    constexpr uint16_t DSHOT_BUFFER_ITEMS = DSHOT_DATA_ITEMS + DSHOT_GAP_ITEMS;
    
}

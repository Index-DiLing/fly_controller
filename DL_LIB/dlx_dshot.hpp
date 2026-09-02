#pragma once

#include <string.h>
#include "stm32f4xx.h"
#include "dlx_dma.hpp"
#include "dlx_dshot_profile.hpp"
namespace dlx
{

    /**
     * @brief DShot 输出驱动(4 电机并行)
     *
     * 数据通路:
     *   preloadThrottle() -> preEncode() 展开成 64 个数据项
     *      (布局 preEncodedValue[bit * 4 + motor], bit15 在前) -> transferStatus = 2
     *   -> DMA TC 中断(每块缓冲传完一次): transferStatus > 0 时把 64 数据项 +
     *      DSHOT_GAP_ITEMS 间隔项同步进刚完成的空闲缓冲(load()), transferStatus 递减,
     *      连续两次 TC 后两块缓冲内容一致
     *
     * DMA: 循环 + 双缓冲, Update 事件触发 CCR1~CCR4 突发(4 transfers),
     *      每块缓冲 DSHOT_BUFFER_ITEMS 个 HalfWord =
     *      16 个数据位周期 + DSHOT_GAP_ITEMS/4 个间隔位周期(4 通道并行)
     */
    class Dshot
    {
    private:
        DMA dataTransferDMA;
        // 四个电机当前实际转速油门值
        volatile uint16_t throttle[4];

        // 2 块 DMA 双缓冲; 每块 DSHOT_DATA_ITEMS 数据项 + DSHOT_GAP_ITEMS 间隔项
        uint16_t dshotValue[2][DSHOT_BUFFER_ITEMS];

        // 编码暂存区(仅 DSHOT_DATA_ITEMS 个数据项), preloadThrottle 写入,
        // 中断里由 load() 展开进空闲缓冲
        uint16_t preEncodedValue[DSHOT_DATA_ITEMS];

        uint16_t period;

        // 0 = 双缓冲内容一致, 无待同步; 2 = 刚 preload, 两块都待同步; 1 = 还差一块
        volatile uint8_t transferStatus = 0;

        inline uint16_t getLowBitPeriod()
        {
            // 低电平位: 占空比 37.5%
            return (period + 1) * 3 / 8;
        }
        inline uint16_t getHighBitPeriod()
        {
            // 高电平位: 占空比 75%
            return (period + 1) * 3 / 4;
        }

        /**
         * @brief 根据输入的值和telemetry位生成完整的16位Dshot数据
         * @param value 48-2047
         * @param tel  是否启用telemetry回传
         * @return uint16_t  完整的16位Dshot数据
         */
        inline uint16_t appendCCR4forValue(uint16_t value, bool tel)
        {
            uint16_t packet_telemetry = (value << 1) | (tel ? 1 : 0);
            uint8_t i;
            int csum      = 0;
            int csum_data = packet_telemetry;
            for (i = 0; i < 3; i++) {
                csum ^= csum_data; // xor data by nibbles
                csum_data >>= 4;
            }
            csum &= 0xf;
            packet_telemetry = (packet_telemetry << 4) | csum;
            return packet_telemetry; // append checksum
        }

        void preEncode()
        {
            uint16_t crced[4] = {
                appendCCR4forValue(throttle[0], false),
                appendCCR4forValue(throttle[1], false),
                appendCCR4forValue(throttle[2], false),
                appendCCR4forValue(throttle[3], false)};
            // 并行布局: 每个 DMA 项 = 1 次 Update 突发写 CCR1~CCR4,
            // 因此同一 bit 位置的 4 个电机必须相邻; DShot 帧先发 MSB
            for (uint8_t bit = 0; bit < 16; bit++) {
                uint16_t mask = static_cast<uint16_t>(1u << (15 - bit));
                for (uint8_t motor = 0; motor < 4; motor++) {
                    preEncodedValue[bit * 4 + motor] =
                        (crced[motor] & mask) ? getHighBitPeriod() : getLowBitPeriod();
                }
            }
        }

        void loadInto(uint16_t *buf)
        {
            memcpy(buf, preEncodedValue, DSHOT_DATA_ITEMS * sizeof(uint16_t));
            // 帧尾间隔: DSHOT_GAP_ITEMS 项全部写 0(CCR=0, 恒低, 0% 占空比),
            // 让 ESC 在帧后有真正的空闲低电平来重新同步帧边界
            // (PX4 同款做法: dshot.c 里 gap 槽保持零初始化, 不写 BIT_0)
            for (uint16_t i = DSHOT_DATA_ITEMS; i < DSHOT_BUFFER_ITEMS; i++) {
                buf[i] = 0;
            }
        }

        // 中断中调用此函数: 有新油门时把暂存的 preEncodedValue 同步进当前空闲缓冲
        void load()
        {
            if (transferStatus == 0) {
                return;
            }
            // TC 中断触发时 CT 已切到正在传输的缓冲, 另一块刚完成, 可安全写入
            uint16_t *freeBuf = dataTransferDMA.isTransferingFirstBuffer() ? dshotValue[1] : dshotValue[0];
            loadInto(freeBuf);
            transferStatus--;
        }

        void initPreEncoded()
        {
            throttle[0] = throttle[1] = throttle[2] = throttle[3] = 0;
            preEncode();
            // 两块缓冲放同样的初始零油门帧, 保证双缓冲内容一致
            loadInto(dshotValue[0]);
            loadInto(dshotValue[1]);
            transferStatus = 0;
        }

    public:
        /**
         * @brief 由 AdvancedTimer::setDshot() 自动创建, 一般不直接调用
         *
         * @param profile   DShot 速率(150/300/600 kHz); 速率已由 Timer 换算成时基,
         *                  这里仅保留参数以维持接口一致
         * @param timerDMA  Timer 已初始化好的 DMA(循环 + CCR1~CCR4 突发),
         *                  本构造会把内存侧重新绑定到内部双缓冲并注册 TC 中断
         * @param period    TIM_Period 值(每 bit 计数周期 = period + 1)
         */
        Dshot(DshotProfile profile, DMA timerDMA, uint16_t period)
            : dataTransferDMA(timerDMA), period(period)
        {
            (void)profile;

            ByteBuffer b1((uint8_t *)dshotValue[0], sizeof(dshotValue[0]));
            ByteBuffer b2((uint8_t *)dshotValue[1], sizeof(dshotValue[1]));
            dataTransferDMA.setBuffer(b1, sizeof(dshotValue[0]));
            dataTransferDMA.setDoubleBuffer(b2);

            initPreEncoded();

            // DMA 传完一块缓冲后由硬件自动开始下一块(Circular), 此时把
            // 新油门同步进刚完成的空闲块; 中断在此注册, 生命周期随本对象
            dataTransferDMA.setITRequest(DMAITProfile::TC, ENABLE);
            dataTransferDMA.initNVIC(NVICPriorityProfile::P1_S0);
            dataTransferDMA.setITCallback(+[](DMA *dma, void *ctx) {
                auto self = static_cast<Dshot *>(ctx);
                self->load();
                dma->clearITPending(DMAITProfile::TC);
            }, this);
        }
        ~Dshot()
        {
        }

        /**
         * @brief 启动 DMA 传输(定时器时基已由 setDshot() 启动)
         */
        void start()
        {
            dataTransferDMA.start();
        }

        void preloadThrottle(uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3)
        {
            while (transferStatus != 0); // 等上一次同步完成再覆盖暂存区
            throttle[0]    = value0;
            throttle[1]    = value1;
            throttle[2]    = value2;
            throttle[3]    = value3;
            preEncode();
            transferStatus = 2;
        }
        // 外部需要保证传入的value是4个
        void preloadThrottle(uint16_t value[])
        {
            while (transferStatus != 0); // 等上一次同步完成再覆盖暂存区
            throttle[0]    = value[0];
            throttle[1]    = value[1];
            throttle[2]    = value[2];
            throttle[3]    = value[3];
            preEncode();
            transferStatus = 2;
        }
    };

}

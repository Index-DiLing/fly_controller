#pragma once
#include "stm32f4xx.h"
#include <stdint.h>
#include "dlx_gpio_profile.h"
#include "dlx_nvic_it.h"
//===================================================================================================
// DL_LIB/dlx_exti.hpp —— 外部中断(GPIO → EXTI → NVIC)的最小封装
//
// 为什么需要它
//   VL53L1X 的 GPIO1(数据就绪)接在 PA3 上, 要按"上升沿"触发(那颗芯片默认高有效, 见方案
//   §4.2/§0.1 第 8 条)。这件事散在三个寄存器里: SYSCFG 选端口、EXTI 选边沿、NVIC 开通道,
//   而且 **EXTI5~9 和 EXTI10~15 各自共用一个中断向量** —— 共用意味着"谁能注册回调"必须
//   有地方统一管, 否则第二次注册会把第一个顶掉, 而且是**静默**顶掉。
//
// 本文件的两条设计
//   1. **一条线一个订阅者**(16 个槽)。重复注册返回 false, 不静默覆盖 —— 与事件表同一条规矩。
//   2. **组向量只注册一次**: EXTI9_5 / EXTI15_10 的转发器会扫描本组所有 16 条线的 pending,
//      谁的标志置了就派给谁的订阅者。这样同组挂两个中断源也不会互相顶掉。
//
// 中断里只做"转发给订阅者", 订阅者自己负责清标志之外的判断(它拿得到 line)。
//===================================================================================================

namespace dlx
{
    /** 触发边沿。`Both` 在数据就绪场景里基本没用, 但调试时能"看看到底动没动"。 */
    enum class EXTITrigger : uint8_t
    {
        Rising  = 0,
        Falling = 1,
        Both    = 2,
    };

    class EXTILine
    {
    public:
        // 类名为什么是 EXTILine 而不是 EXTI: **`EXTI` 是 stm32f4xx.h 里的宏**
        // (#define EXTI ((EXTI_TypeDef *) EXTI_BASE))，拿它当类名会变成
        // "declaration of anonymous class must be a definition" 这种看不懂的报错。踩过一次，记在这里。

        /** 订阅者: `line` 是线的编号 0..15(同组共用向量时靠它区分) */
        typedef void (*LineHandler)(uint8_t line, void *ctx);

        /** 由 GPIOProfile 算 EXTI 线号: 高 4 位是端口, 低 4 位是引脚号 */
        static uint8_t lineOf(GPIOProfile pin) { return static_cast<uint8_t>(pin) & 0x0F; }
        static uint8_t portOf(GPIOProfile pin) { return static_cast<uint8_t>(static_cast<uint8_t>(pin) >> 4); }

        static uint32_t lineMask(uint8_t line) { return 1u << line; }

        /** 线号 → 中断通道(0..4 各自一条, 5..9 共用, 10..15 共用) */
        static NVICITProfile irqOf(uint8_t line)
        {
            switch (line) {
                case 0:  return NVICITProfile::EXTI0_IRQn;
                case 1:  return NVICITProfile::EXTI1_IRQn;
                case 2:  return NVICITProfile::EXTI2_IRQn;
                case 3:  return NVICITProfile::EXTI3_IRQn;
                case 4:  return NVICITProfile::EXTI4_IRQn;
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:  return NVICITProfile::EXTI9_5_IRQn;
                default: return NVICITProfile::EXTI15_10_IRQn;
            }
        }

        EXTILine(GPIOProfile pin, EXTITrigger trigger) : _pin(pin), _trigger(trigger)
        {
        }

        uint8_t line() const { return lineOf(_pin); }

        /**
         * @brief 把这条线配成中断并注册回调
         * @return false = 这条线已经有人订阅了(不覆盖); 需要换人就先 unbind()
         *
         * @note 引脚本身的模式(GPIO 输入/上拉)由调用方自己 init —— EXTI 只关心"线"。
         *       忘了配引脚是最常见的错: 中断配好了但引脚还是模拟输入, 永远不触发。
         */
        bool init(NVICPriorityProfile prio, LineHandler handler, void *ctx)
        {
            if (handler == nullptr) {
                return false;
            }
            Slot &s = slot(line());
            if (s.handler != nullptr) {
                return false;
            }
            s.handler = handler;
            s.ctx     = ctx;

            RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
            // 函数名是 SYSCFG_EXTILineConfig, **不是** GPIO_EXTILineConfig —— F1 时代叫后者,
            // F4 挪到了 SYSCFG 里。写错的话报的是"use of undeclared identifier"。
            SYSCFG_EXTILineConfig(static_cast<uint8_t>(portOf(_pin)), line()); // SYSCFG->EXTICR

            EXTI_InitTypeDef it;
            it.EXTI_Line    = lineMask(line());
            it.EXTI_Mode    = EXTI_Mode_Interrupt;
            it.EXTI_Trigger = (_trigger == EXTITrigger::Rising)   ? EXTI_Trigger_Rising
                              : (_trigger == EXTITrigger::Falling) ? EXTI_Trigger_Falling
                                                                    : EXTI_Trigger_Rising_Falling;
            it.EXTI_LineCmd = ENABLE;
            EXTI_ClearITPendingBit(it.EXTI_Line); // 先清一次: 配置过程中可能已经攒了个旧标志
            EXTI_Init(&it);

            DLX_NVIC_Init(irqOf(line()), prio);
            DLX_IT_set_callback(irqOf(line()), &EXTILine::trampoline, reinterpret_cast<void *>(static_cast<uintptr_t>(irqOf(line()))));
            return true;
        }

        void unbind()
        {
            Slot &s = slot(line());
            s.handler = nullptr;
            s.ctx     = nullptr;
            EXTI_InitTypeDef it;
            it.EXTI_Line    = lineMask(line());
            it.EXTI_Mode    = EXTI_Mode_Interrupt;
            it.EXTI_Trigger = EXTI_Trigger_Rising;
            it.EXTI_LineCmd = DISABLE;
            EXTI_Init(&it);
            EXTI_ClearITPendingBit(it.EXTI_Line);
        }

        bool isBound() const { return slot(line()).handler != nullptr; }
        bool isPending() const { return EXTI_GetITStatus(lineMask(line())) != RESET; }
        void clearPending() { EXTI_ClearITPendingBit(lineMask(line())); }

    private:
        struct Slot
        {
            LineHandler handler;
            void       *ctx;
        };

        /**
         * 每条线一个槽。
         *
         * 放在 inline 函数里的 static 局部: C++11 保证**跨编译单元只有一份**(inline 函数的
         * 局部静态是唯一的), 所以不会出现"每个 .cpp 各自一份订阅表、互相看不见"。
         */
        static Slot &slot(uint8_t line)
        {
            static Slot slots[16] = {};
            return slots[line & 0x0F];
        }

        /** 组向量的转发器: ctx 里塞的是通道号, 由它决定扫哪一段线 */
        static void trampoline(void *ctx)
        {
            const uint32_t irqn = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(ctx));
            switch (irqn) {
                case static_cast<uint32_t>(NVICITProfile::EXTI0_IRQn):      dispatch(0, 0); break;
                case static_cast<uint32_t>(NVICITProfile::EXTI1_IRQn):      dispatch(1, 1); break;
                case static_cast<uint32_t>(NVICITProfile::EXTI2_IRQn):      dispatch(2, 2); break;
                case static_cast<uint32_t>(NVICITProfile::EXTI3_IRQn):      dispatch(3, 3); break;
                case static_cast<uint32_t>(NVICITProfile::EXTI4_IRQn):      dispatch(4, 4); break;
                case static_cast<uint32_t>(NVICITProfile::EXTI9_5_IRQn):    dispatch(5, 9); break;
                case static_cast<uint32_t>(NVICITProfile::EXTI15_10_IRQn):  dispatch(10, 15); break;
                default: break;
            }
        }

        /** 扫一段线, 谁 pending 就派给谁。**先清标志再回调**: 回调里若立刻又来了一个边沿, 不会丢。 */
        static void dispatch(uint8_t first, uint8_t last)
        {
            for (uint8_t line = first; line <= last; ++line) {
                const uint32_t mask = lineMask(line);
                if (EXTI_GetITStatus(mask) == RESET) {
                    continue;
                }
                EXTI_ClearITPendingBit(mask);
                Slot &s = slot(line);
                if (s.handler != nullptr) {
                    s.handler(line, s.ctx);
                }
                // 没订阅者的 pending 也必须清掉, 否则这条线会一直把中断拉高、把同组别的线饿死
            }
        }

        GPIOProfile  _pin;
        EXTITrigger  _trigger;
    };

} // namespace dlx

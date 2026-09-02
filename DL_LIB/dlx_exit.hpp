#pragma once
#include <stdint.h>
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_nvic_it.h"
#include "dlx_exit_profile.hpp"

namespace dlx
{
    /**
     * @brief 外部中断 EXTI 封装(项目 dlx 风格)
     *
     * 引脚直接用 GPIOProfile 表达(高 4 位=端口, 低 4 位=引脚号),
     * 因此不再需要单独的 EXTIProfile: 由 GPIOProfile 推导出
     * EXTI_Line / EXTI_PortSourceGPIOx / EXTI0~4 / EXTI9_5 / EXTI15_10 中断号.
     *
     * 中断入口约定与 dlx_nvic_it 一致: stm32f4xx_it.cpp 里的
     * EXTIx_IRQHandler 统一调用 DLX_IT_invoke_callback(channel),
     * 本类 init() 时注册一个内部跳板: 先清本线的 EXTI 挂起位, 再执行用户回调.
     * 因此同一分组(如 EXTI9_5 / EXTI15_10)内同时只建议挂一个回调.
     */
    class EXTI_Line
    {
    private:
        GPIOProfile pin;                 ///< 引脚(端口+位)
        DLX_IT_Handler handler = 0;      ///< 用户回调
        void *context = 0;               ///< 用户回调上下文

        inline uint8_t pinPos() const
        {
            return static_cast<uint8_t>(pin) & 0x0F;
        }

        inline uint8_t portIdx() const
        {
            return (static_cast<uint8_t>(pin) >> 4) & 0x0F;
        }

        inline uint32_t getEXTI_Line() const
        {
            return (uint32_t)1 << pinPos();
        }

        /** GPIOProfile 高 4 位即端口序号 A=0, B=1 ... I=8, 与 EXTI_PortSourceGPIOx 一致 */
        inline uint32_t getEXTI_PortSource() const
        {
            return (uint32_t)EXTI_PortSourceGPIOA + portIdx();
        }

        inline NVICITProfile getIRQn() const
        {
            const uint8_t p = pinPos();
            if (p < 5) {
                return static_cast<NVICITProfile>(EXTI0_IRQn + p);
            }
            if (p < 10) {
                return NVICITProfile::EXTI9_5_IRQn;
            }
            return NVICITProfile::EXTI15_10_IRQn;
        }

        inline uint32_t getEXTI_Trigger(EXTIModeProfile mode) const
        {
            switch (static_cast<uint16_t>(mode) & 0x3) {
                case 0: return EXTI_Trigger_Rising;
                case 1: return EXTI_Trigger_Falling;
                default: return EXTI_Trigger_Rising_Falling;
            }
        }

        /** 内部跳板: 先清本线挂起位(否则边沿中断会立即重入), 再调用户回调 */
        static void trampoline(void *self)
        {
            EXTI_Line *e = static_cast<EXTI_Line *>(self);
            EXTI_ClearITPendingBit(e->getEXTI_Line());
            if (e->handler != 0) {
                e->handler(e->context);
            }
        }

    public:
        EXTI_Line(GPIOProfile pinProfile)
            : pin(pinProfile)
        {
#warning [Experimental]
        }

        ~EXTI_Line()
        {
        }

        /**
         * @brief 配置引脚输入并挂上外部中断
         *
         * 内部完成: GPIO 输入(带上拉) -> SYSCFG 时钟与连线 -> EXTI 中断模式
         * -> 清挂起位 -> NVIC 使能 + 注册回调.
         *
         * @param mode     触发沿(EXTIModeProfile)
         * @param priority NVIC 优先级组合(NVICPriorityProfile)
         * @param handler  中断回调, 在 EXTIx_IRQHandler 中被调用
         * @param context  回调上下文
         */
        void init(EXTIModeProfile mode, NVICPriorityProfile priority,
                  DLX_IT_Handler handler, void *context = 0)
        {
            // 输入引脚: 默认上拉(外部中断输入多为开漏/悬空, 拉高避免悬空误触发)
            GPIO g(pin);
            g.init(GPIOModeProfile::IN_UP);

            RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
            SYSCFG_EXTILineConfig(getEXTI_PortSource(), pinPos());

            EXTI_InitTypeDef s;
            s.EXTI_Line    = getEXTI_Line();
            s.EXTI_Mode    = EXTI_Mode_Interrupt;
            s.EXTI_Trigger = static_cast<EXTITrigger_TypeDef>(getEXTI_Trigger(mode));
            s.EXTI_LineCmd = ENABLE;
            EXTI_Init(&s);
            EXTI_ClearITPendingBit(getEXTI_Line());

            this->handler = handler;
            this->context = context;
            DLX_IT_init(getIRQn(), priority, &EXTI_Line::trampoline, this);
        }

        /**
         * @brief 仅替换回调/上下文, 不重新配置 EXTI(与 NVIC 保持原状)
         */
        void setCallback(DLX_IT_Handler handler, void *context = 0)
        {
            this->handler = handler;
            this->context = context;
        }
    };
} // namespace dlx

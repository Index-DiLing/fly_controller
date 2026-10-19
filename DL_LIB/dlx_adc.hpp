#pragma once
#include "stm32f4xx.h"
#include <stdint.h>
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_exception.h"
//===================================================================================================
// DL_LIB/dlx_adc.hpp —— 电池电压 / 电流采样(ADC1, 规则组软件触发, 轮询)
//
// **本文件不认识 cfg::**（2026-10-02 改）
//   这是 DL_LIB 的规矩: 库里放的是"可复用的驱动", 它只接受**参数**, 不读应用配置。
//   理由不是洁癖 —— 一旦库里 `#include "flight_config.hpp"` 并写 `static_assert(cfg::...)`,
//   换一块板子/复用这个驱动就得改库文件, 而且库和某个工程的 schema 从此绑死。
//
//   所以:
//     - "电压在哪个通道、分压比多少" 这类**接线事实**由调用方通过 `ADC1Config` 传进来,
//       对应的编译期断言在应用层 `hardware_claims.hpp`;
//     - 本文件只保证"给我通道号和换算系数, 我按处理器手册把 ADC 配好并读出物理量"。
//
// 硬件口径(方案 §4.5, 2026-10-02 已定案)
//   PA0 ← 电池电压经 8k/1k 分压 ⇒ **分压比 9.0**, 源阻抗(Thevenin)≈0.89kΩ ⇒ 采样时间要给足;
//   PA1 ← 电流传感器**直连**(无分压), 灵敏度 **12.5mV/A**(80 A/V), 零点 ~1.65V。
//   ADC1_IN0 = PA0, ADC1_IN1 = PA1 —— 通道号与引脚是硬件绑死的, 所以应用层必须断言。
//
// 为什么用"轮询 + 软件触发"而不是 DMA + 定时器:
//   这两个量是**慢变量**(电池电压在秒级变化, 电流环也不需要它), 采快了对控制没有任何帮助,
//   反而要占一条 DMA 流和一个定时器。代价是每次读都是阻塞的, 最坏耗时写在 readSupply 注释里,
//   所以它**只能在地面/IO 线程里调**, 不许进 500Hz 控制环(方案 §5 的硬要求)。
//===================================================================================================

namespace dlx
{
    /** 采样通道与换算系数 —— 全部由调用方(应用层)填, 库不自作主张。 */
    struct ADC1Config
    {
        uint8_t  voltageChannel{0};   ///< 电池电压所在 ADC 通道
        uint8_t  currentChannel{1};   ///< 电流所在 ADC 通道
        uint8_t  sampleCount{8};      ///< 每次平均几次(1~64)
        uint16_t vrefMv{3300};        ///< 参考电压 [mV](标称值)
        float    voltageDividerRatio{1.0f}; ///< 电池电压 → 引脚电压 的分压比
        float    voltageOffsetV{0.0f};      ///< 电压换算偏置 [V](吸收分压电阻误差)
        float    currentOffsetV{1.65f};     ///< 电流零点(0A 时引脚电压)[V]
        float    currentAmpsPerVolt{80.0f}; ///< 电流灵敏度 [A/V] = 1/灵敏度[V/A]
        uint8_t  cellCount{1};              ///< 电池节数(分级保护按"每节"判)
    };

    class ADC1Sampler
    {
    public:
        /** 采样时间档: 分压后的电压源阻抗高, 给最长档; 电流直连, 给中档就够 */
        static constexpr uint16_t kVoltageSampleTime = ADC_SampleTime_480Cycles;
        static constexpr uint16_t kCurrentSampleTime = ADC_SampleTime_144Cycles;

        ADC1Sampler() {}
        explicit ADC1Sampler(const ADC1Config &cfg) : _cfg(cfg) {}

        void setConfig(const ADC1Config &cfg) { _cfg = cfg; }
        const ADC1Config &config() const { return _cfg; }

        /**
         * @brief 初始化 ADC1 + 两个模拟输入脚 + 校准
         * @param vPin / iPin 电压/电流所在**引脚**(模拟输入)。通道号在手册里与引脚绑死,
         *        调用方必须保证传进来的引脚和 `_cfg` 里的通道号是同一对 —— 这件事由应用层断言。
         * @return 失败时 err = POWER_ADC_FAILED
         *
         * 校准不是可选项: 不做的话零点误差能到几十个码, 而 12 位满量程才 4095 ——
         * 相当于整包电压差 1%~2%(本板 6S 满充 25.2V ⇒ 约 0.25~0.5V 的假低压告警)。
         */
        FunctionResult init(GPIOProfile vPin, GPIOProfile iPin, ExceptionCode &err)
        {
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

            // 必须配成 AN(模拟)而不是 IN(数字输入): 数字输入会把引脚接上施密特触发器,
            // 模拟源的驱动能力被拉低, 读数偏低且随温度漂。
            GPIO vp(vPin);
            vp.init(GPIOModeProfile::AN_NOPULL);
            GPIO ip(iPin);
            ip.init(GPIOModeProfile::AN_NOPULL);

            // ADCCLK 上限 36MHz。APB2 = 84MHz ⇒ /4 = 21MHz 合法, 这里用 /8 = 10.5MHz 留余量:
            // 采样保持电容的建立时间与 ADCCLK 成反比, 而我们对速度毫无需求。
            ADC_CommonInitTypeDef common;
            common.ADC_Mode             = ADC_Mode_Independent;
            common.ADC_Prescaler        = ADC_Prescaler_Div8;
            common.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
            common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
            ADC_CommonInit(&common);

            ADC_InitTypeDef init;
            init.ADC_Resolution           = ADC_Resolution_12b;
            init.ADC_ScanConvMode         = DISABLE; // 软件触发逐通道读, 不用扫描表
            init.ADC_ContinuousConvMode   = DISABLE; // 单次: 不读的时候 ADC 不该占着总线
            init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
            init.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T1_CC1; // 未使用(边沿=None)
            init.ADC_DataAlign            = ADC_DataAlign_Right;
            init.ADC_NbrOfConversion      = 1;
            ADC_Init(ADC1, &init);

            ADC_Cmd(ADC1, ENABLE);

            // ---- 校准 ----
            // ⚠️ 这份 StdPeriph(V1.9.0) 的 stm32f4xx_adc.h 里**根本没有** `ADC_StartCalibration` /
            //   `ADC_GetCalibrationStatus` / `ADC_VoltageRegulatorCmd`, stm32f4xx.h 里也没有
            //   `ADC_CR2_ADCAL`(bit31)。所以只能按 RM0090 §13.3.3 直接写寄存器。
            //   不校准, 零点误差折合整包电压 1%~2%, 本板 6S(满充 25.2V)上就是 0.25~0.5V 的假低压告警。
            //   时序: ① ADON=0 → ② ADON=1 唤醒并开内部稳压器 → ③ 等 tSTAB(≥3µs)
            //        → ④ ADCAL=1 → ⑤ 硬件在校准完成时清 ADCAL。
            constexpr uint32_t kCr2Adcal = 0x80000000u; // CR2 bit31 ADCAL
            ADC1->CR2 &= ~ADC_CR2_ADON;                 // ①
            ADC1->CR2 |= ADC_CR2_ADON;                  // ②
            for (volatile uint32_t i = 0; i < 1000u; ++i) {
            }                                           // ③ 空转等稳压器(早期 delay_ms 可能不可用)
            ADC1->CR2 |= kCr2Adcal;                     // ④
            uint32_t guard = 0;
            while ((ADC1->CR2 & kCr2Adcal) != 0u) {     // ⑤
                if (++guard > 1000000u) {
                    err = ExceptionCode::POWER_ADC_FAILED;
                    return FunctionResult::FAILED;
                }
            }
            // 校准后第一次转换丢掉: 参考手册要求它不参与校准结果的建立, 而"上电第一次电压读数"
            // 正好是低压保护的判据来源 —— 脏的那一个不能要。
            (void)convertRaw(_cfg.voltageChannel, kVoltageSampleTime);

            return FunctionResult::SUCCESS;
        }

        /** 原始码 → 满量程比例 0..1(不做任何标定) */
        static float rawToRatio(uint16_t raw) { return static_cast<float>(raw) / 4095.0f; }

        /** 引脚电压 [V] = 比例 × VREF。VREF 用的是**标称值**, 真实值靠台架标定。 */
        float rawToVolts(uint16_t raw) const
        {
            return rawToRatio(raw) * (static_cast<float>(_cfg.vrefMv) * 0.001f);
        }

        /** 电池电压 [V] = 引脚电压 × 分压比 + 偏置 */
        float voltsToPackVolts(float pinVolts) const
        {
            return pinVolts * _cfg.voltageDividerRatio + _cfg.voltageOffsetV;
        }

        /** 电流 [A] = (引脚电压 - 零点) × 灵敏度。**允许出负值**: 那是零点没标好, 上位机要看得见。 */
        float voltsToAmps(float pinVolts) const
        {
            return (pinVolts - _cfg.currentOffsetV) * _cfg.currentAmpsPerVolt;
        }

        /** 每节电池电压 [V] —— 分级保护按"每节"判而不是按整包, 换电池节数不用改阈值 */
        float perCellVolts(float packVolts) const
        {
            const uint8_t n = (_cfg.cellCount == 0) ? 1 : _cfg.cellCount;
            return packVolts / static_cast<float>(n);
        }

        /** 电压合理性: 超过"满量程 × 分压比"说明分压比配错, 或 ADC 读到了悬空脚 */
        bool packVoltsPlausible(float packVolts) const
        {
            const float maxPin = static_cast<float>(_cfg.vrefMv) * 0.001f;
            return packVolts >= 0.0f && packVolts <= maxPin * _cfg.voltageDividerRatio;
        }

        /**
         * @brief 一次完整采样: 电池电压 [V] 与电流 [A]
         *
         * 最坏耗时: 两个通道 × sampleCount 次 × (采样时间 + 12 + 3) 个 ADCCLK。
         *   480 / 144 周期、ADCCLK = 10.5MHz、8 次 ⇒ (480+15 + 144+15) × 8 / 10.5e6 ≈ 0.50ms
         *   —— 这就是为什么它**只能**放在 IO 线程(方案 §5)。
         */
        FunctionResult readSupply(float &packVolts, float &amps, ExceptionCode &err)
        {
            const uint16_t vRaw = convertAveraged(_cfg.voltageChannel, kVoltageSampleTime);
            const uint16_t iRaw = convertAveraged(_cfg.currentChannel, kCurrentSampleTime);

            packVolts = voltsToPackVolts(rawToVolts(vRaw));
            amps      = voltsToAmps(rawToVolts(iRaw));

            return FunctionResult::SUCCESS;
        }

        /** 单通道单次转换(阻塞, 最坏约 47µs @480 周期 / ADCCLK 10.5MHz) */
        uint16_t convertRaw(uint8_t channel, uint16_t sampleTime)
        {
            ADC_RegularChannelConfig(ADC1, channel, 1, sampleTime);
            ADC_SoftwareStartConv(ADC1);
            uint32_t guard = 0;
            while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
                if (++guard > 1000000u) {
                    return 0; // 有界返回: 宁可给个 0 让上层看见异常, 也不在这里死等
                }
            }
            return ADC_GetConversionValue(ADC1);
        }

        /** 单通道 sampleCount 次平均(去掉极值没做: 8 次里去掉两个极值就只剩 6 次, 意义不大) */
        uint16_t convertAveraged(uint8_t channel, uint16_t sampleTime)
        {
            uint32_t sum = 0;
            const uint8_t n = (_cfg.sampleCount == 0) ? 1 : _cfg.sampleCount;
            for (uint8_t i = 0; i < n; ++i) {
                sum += convertRaw(channel, sampleTime);
            }
            return static_cast<uint16_t>(sum / n);
        }

    private:
        ADC1Config _cfg{};
    };

} // namespace dlx

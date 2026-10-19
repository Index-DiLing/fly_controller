#pragma once
//===================================================================================================
// DL_LIB/BMP388/dlx_bmp388.hpp —— BMP388 气压计驱动(I2C 模式, 基于 dlx IICBus/IICDevice)
//
// 职责:
//   1. init(): 软复位 -> 校验 CHIP_ID -> 读 21 字节标定系数 -> 写配置 -> 回读校验;
//   2. read(): 一次突发读 STATUS+数据, 用 Bosch 官方浮点补偿公式算出 气压[Pa] / 温度[°C];
//   3. 把"总线失败"和"长时间没有新样本"两种失效分清楚, 交给上层健康管理(sensor.baro.kFailLimit)。
//
// 用法(与 BME280 完全同构, 老代码只需要把类型名和头文件换掉):
//   IICBus bus = IICBus::IIC1_SB6_DB7();
//   bus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, cfg::hw::baro::kSpeed, 0x51);
//   BMP388 baro(bus);                 // 地址默认取 cfg::hw::baro::kAddr = 0x76
//   ExceptionCode err;
//   if (baro.init(err) != FunctionResult::SUCCESS) { ... }   // 只在初始化/io 线程调用!
//   float pa, degC;
//   if (baro.read(pa, degC, err) == BMP388::ReadResult::NewData) { ... }
//
//---------------------------------------------------------------------------------------------------
// 与 BME280(dlx_bme280.hpp)的差异 —— 不是"同一个传感器的下一代", 是**换了一颗芯片**:
//
//   项目             | BME280(旧)                     | BMP388(本驱动)
//   -----------------|--------------------------------|------------------------------------------
//   CHIP_ID          | 0xD0 == 0x60                   | **0x00 == 0x50**(0xD0 在 388 上是保留寄存器)
//   软复位           | 0xE0 <- 0xB6                   | **0x7E <- 0xB6**, 之后等 >=2ms
//   标定系数         | 0x88(24B)+0xA1+0xE1(7B), 大端  | **0x31, 21B, 小端**, 混 8/16bit 有符号无符号
//   标定 CRC         | 有(calib26 + 0xA0)             | **没有** —— 只能做全 0/全 FF + 量级检查
//   通道使能         | 在 ctrl_meas 的过采样位里      | **PWR_CTRL(0x1B) 的 bit0/bit1**, 与 mode 分开
//   模式             | ctrl_meas[1:0]                 | PWR_CTRL[5:4](00 睡眠 / 01,10 强制 / 11 连续)
//   过采样           | osrs_p[4:2], osrs_t[7:5]       | OSR(0x1C): osr_p[2:0], osr_t[5:3]
//   数据             | 0xF7~0xFE, 20bit 带 4bit 填充  | 0x04~0x09, **24bit 无符号**, 且必须一次读完
//   补偿             | 整数定点 + t_fine              | **浮点 + t_lin**(系数按 2 的幂次缩放, 见下)
//   湿度             | 有                             | **没有**(代码路径整个删掉)
//
// 特别注意"为什么不是这样"的三条(踩过的/容易踩的):
//   1. **CHIP_ID 必须在 0x00 读到 0x50**。照抄 BME280 去读 0xD0 不会报错, 只会读到一个保留寄存器
//      的值, 于是要么永远 ID 不匹配、要么"碰巧"匹配而其实根本没接通。这是本驱动与旧驱动最本质的
//      一条区别, 上板第一件事就是确认它。
//   2. **PWR_CTRL(0x1B) 的 bit4/bit5 才是 press_en/temp_en**(BME280 是相反的顺序: 湿度/温度/气压
//      挤在一个寄存器的高位)。位写反的表现是"ID 能读、配置能回读, 但永远没有新数据"。
//   3. **标定系数的地址与缩放都不能照抄 BME280**: T1 在 0x31(不是 0x33), 且 Bosch 的浮点补偿要求
//      先把 NVM 原始整数按 2 的幂次缩放(par_t1 = NVM_T1 / 2^-8, par_t2 = NVM_T2 / 2^30, ...)。
//      直接把 BME280 的整数公式套过来, 温度会差几十度、气压会差几个数量级。
//
//---------------------------------------------------------------------------------------------------
// 寄存器约定表(手册 BST-BMP388-DS001-07 Rev 1.7 §4.3; 地址与位定义见 dlx_bmp388_config.h)
//
//   0x00 CHIP_ID   读 = 0x50
//   0x02 ERR_REG   [2]conf_err [1]cmd_err [0]fatal_err
//   0x03 STATUS    [6]drdy_temp [5]drdy_press [4]cmd_rdy     <- 本驱动用它判"有没有新样本"
//   0x04~0x06      press_xlsb/lsb/msb   24bit 无符号小端
//   0x07~0x09      temp_xlsb/lsb/msb    24bit 无符号小端
//   0x19 INT_CTRL  = cfg::sensor::baro::kIntCtrl
//   0x1A IF_CONF   保持默认 0x00(I2C 看门狗必须关: 开着会在总线空闲 1.25/40ms 后复位接口)
//   0x1B PWR_CTRL  [5:4]mode | [1]temp_en | [0]press_en = 0x33(连续 + 双通道)
//   0x1C OSR       = cfg::sensor::baro::kOsr       (压力 ×8 / 温度 ×1)
//   0x1D ODR       = cfg::sensor::baro::kOdr       (50Hz)
//   0x1F CONFIG    = cfg::sensor::baro::kConfig    (IIR 系数 3, 位域值 0b010)
//   0x31~0x45      CALIB_DATA, 21 字节(见 readCalib 的逐字节解析)
//   0x7E CMD       写 0xB6 = 软复位(恒回读 0x00, 不能靠轮询它判断完成)
//
// 不用 FIFO: 方案 §4.1 的裁决 —— 50Hz 的 io 线程直接读数据寄存器就够了(ArduPilot 同样做法)。
// FIFO 的价值在于"MCU 睡觉、批量取数、带 sensortime 时间戳", 本机是 20ms 轮询, 用不上,
// 而开 FIFO 会多出 FIFO_CONFIG/水位/溢出/清空一整套状态要维护。读法: 突发 0x03 起 7 字节。
//
//---------------------------------------------------------------------------------------------------
// 时序预算(**驱动内部所有阻塞都在这里说清楚**, 数值是估算不是实测; 依据是 dlx_iic 的
// kEventWaitLimit = 20 万次轮询/事件 + 标称 400kHz 总线的字节时间):
//
//   tryInit()  9 次 I2C 事务 + 2ms 复位等待。
//              正常: 约 1ms 总线 + 2ms 等待 ≈ 3ms。
//              器件不应答: 每次事务最坏受 IICBus::waitEvent 的轮询上限约束(单事务 3~4 个事件
//              等待, 每个上限约 20 万次寄存器轮询) ⇒ 单次 tryInit 最坏几十 ms 量级。
//   init()     重试 kInitRetry(20) 次, 间隔 kInitRetryMs(100) ⇒ **最坏约 2s + 20×tryInit**。
//              这一段是开机一次性成本, 只能由初始化/io 线程调用; 控制环里禁止调用 init()。
//   read()     1 次 7 字节事务。正常 ≈ 0.2ms; 总线挂死时最坏同样是"几十 ms 量级"(见上)。
//              控制环调用 read() 的最坏阻塞时间由 IICBus 的事件等待上限决定, 不由本驱动决定 ——
//              本驱动不新增任何隐藏等待。
//   delay_ms 只用两处: 软复位后的 BMP388_SOFT_RESET_WAIT_MS(2ms, 手册上电首次通信时间),
//                     以及 init() 重试间隔 cfg::sensor::baro::kInitRetryMs。
//              调用方若想自己掌握重试策略(例如放进 io 线程的状态机里, 每次只花 3ms), 用
//              tryInit(): 它只做一次完整尝试, **不睡 kInitRetryMs**。
//===================================================================================================
#include <stdint.h>
#include <math.h>
#include <cmath>
#include "dlx_iic.hpp"
#include "dlx_delay.hpp"
#include "dlx_exception.h"
#include "dlx_bmp388_config.h"

namespace dlx
{
    /**
     * @brief Bosch 官方的系数缩放因子(手册 §9.1)。
     *
     * 为什么用 pow2f(e) 而不是直接写 256.0f / 1.073741824e9f 这种十进制字面量:
     *   缩放全部是 2 的幂次, 而 2 的幂次在 binary32 上是**精确**的(只要不溢出/不下溢);
     *   写成十进制字面量反而要人去核对"这个数到底是不是 2^-30"。pow2f 在编译期递归展开,
     *   生成的就是精确的 2^e, 且能一眼看出与手册一一对应。
     * 为什么全用 float 而不是 double: 系数与中间量都在 float 的动态范围内, 而 STM32F407 配
     *   -mfpu=fpv4-sp-d16 只有**单精度**硬件浮点, double 会退化成软件库调用(慢几十倍)。
     *   Bosch 附录 §9 给出的参考实现本身就是 float。
     */
    constexpr float bmp388_pow2f(int e)
    {
        return (e == 0) ? 1.0f : ((e > 0) ? (2.0f * bmp388_pow2f(e - 1)) : (0.5f * bmp388_pow2f(e + 1)));
    }

    /**
     * @brief BMP388 气压计(只出气压 + 温度, 没有湿度)
     */
    class BMP388
    {
    public:
        enum class ReadResult : uint8_t { NewData, NotReady, Failed };

    private:
        IICDevice dev;

        /**
         * 标定系数(0x31~0x45 读出后**已按手册 §9.1 缩放**)。
         *
         * 为什么在这里就缩放、而不是像 BME280 那样存原始整数、补偿时再乘:
         *   一是补偿公式的每一次调用都要用全部系数, 缩放一次比每次乘 14 次省;
         *   二是缩放是精确的(见 bmp388_pow2f), 提前做不引入额外误差, 也不影响可测性。
         * 类型按手册的符号属性: T1/T2/P5/P6 无符号, T3/P1/P2/P3/P4/P7/P8/P9/P10/P11 有符号。
         */
        struct CalibData
        {
            float par_t1;  ///< NVM_T1 / 2^-8  (== NVM_T1 × 256)
            float par_t2;  ///< NVM_T2 / 2^30
            float par_t3;  ///< NVM_T3 / 2^48
            float par_p1;  ///< (NVM_P1 - 2^14) / 2^20
            float par_p2;  ///< (NVM_P2 - 2^14) / 2^29
            float par_p3;  ///< NVM_P3 / 2^32
            float par_p4;  ///< NVM_P4 / 2^37
            float par_p5;  ///< NVM_P5 / 2^-3  (== NVM_P5 × 8)
            float par_p6;  ///< NVM_P6 / 2^6
            float par_p7;  ///< NVM_P7 / 2^8
            float par_p8;  ///< NVM_P8 / 2^15
            float par_p9;  ///< NVM_P9 / 2^48
            float par_p10; ///< NVM_P10 / 2^48
            float par_p11; ///< NVM_P11 / 2^65
        } calib;

        /**
         * 上一次补偿出来的温度[°C]。
         *
         * 手册 §3.11 明确: 气压补偿要用**同一次测量**的温度。所以这个值是"上一次
         * compensateTemperature() 的结果", 而 read() 保证每次都是"先温度后气压"的顺序计算;
         * 一旦有人把顺序调换, 气压就会用上一次的温度 —— 那是低速飞行时最容易漏掉的误差来源。
         */
        float tLin = 0.0f;

        uint8_t  chipIdValue  = 0u;  ///< 最近一次读到的 CHIP_ID(上板排查用)
        bool     initialized  = false; ///< tryInit() 是否成功过(没成功过不允许 read())
        uint32_t failStreak   = 0u;  ///< 连续 I2C 事务失败次数
        uint32_t staleStreak  = 0u;  ///< 连续"没拿到新样本"次数(含事务失败), 成功一次清零

        /** 写一个寄存器; 返回 false = I2C 传输异常 */
        bool writeReg(BMP388_Reg reg, uint8_t value)
        {
            uint8_t buf[2] = { static_cast<uint8_t>(reg), value };
            ByteBuffer w(buf, 2);
            w.write(buf, 2); // 标记 2 字节已写入, 否则 used()==0, IIC 一个字节都发不出
            return dev.write(w);
        }

        /** 从 reg 开始突发读 n 字节; 返回 false = I2C 传输异常(out 已清零, 不可用) */
        bool readRegs(BMP388_Reg reg, uint8_t *out, uint16_t n)
        {
            // 先把输出清零: I2C 出错(超时)时不会有数据写进来, 不清零的话调用方会读到栈上的旧值,
            // 例如 CHIP_ID 读失败却"碰巧"等于 0x50, 把不可用的气压计当成好的(旧驱动踩过同样的坑)。
            for (uint16_t i = 0; i < n; ++i) {
                out[i] = 0u;
            }
            uint8_t addr[1] = { static_cast<uint8_t>(reg) };
            ByteBuffer w(addr, 1);
            w.write(addr, 1); // 标记寄存器地址已写入, 否则 used()==0, 读操作不带寄存器地址
            ByteBuffer r(out, n);
            return dev.read(r, w, n);
        }

        /** 小端 16bit(标定系数块里所有 16bit 系数都是 LSB 在前) */
        static uint16_t le16(const uint8_t *p)
        {
            return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
        }

        /**
         * @brief 读 21 字节标定系数并解析/缩放
         *
         * 地址与符号属性逐条对照手册 §3.11.1 Table 23(0x31 起, 21 字节, 小端):
         *
         *   偏移 地址  字段            宽度 符号      缩放(§9.1)
         *   0    0x31  NVM_PAR_T1      16   无符号    / 2^-8
         *   2    0x33  NVM_PAR_T2      16   无符号    / 2^30
         *   4    0x35  NVM_PAR_T3      8    有符号    / 2^48
         *   5    0x36  NVM_PAR_P1      16   有符号    (v - 2^14) / 2^20
         *   7    0x38  NVM_PAR_P2      16   有符号    (v - 2^14) / 2^29
         *   9    0x3A  NVM_PAR_P3      8    有符号    / 2^32
         *   10   0x3B  NVM_PAR_P4      8    有符号    / 2^37
         *   11   0x3C  NVM_PAR_P5      16   无符号    / 2^-3
         *   13   0x3E  NVM_PAR_P6      16   无符号    / 2^6
         *   15   0x40  NVM_PAR_P7      8    有符号    / 2^8
         *   16   0x41  NVM_PAR_P8      8    有符号    / 2^15
         *   17   0x42  NVM_PAR_P9      16   有符号    / 2^48
         *   19   0x44  NVM_PAR_P10     8    有符号    / 2^48
         *   20   0x45  NVM_PAR_P11     8    有符号    / 2^65
         *
         * **为什么不是这样**(两条容易照抄错的地方):
         *   - 有任务书写"T1 在 0x33、T2 在 0x31", 那是**反的**。手册 Table 23 与 Bosch 官方 API
         *     (par_t1 = CONCAT(data[1], data[0])) 都是 **T1 在 0x31**。按反的写, 温度会差几十度。
         *   - 只有 P5/P6/T1/T2 是**无符号**; 其余 16bit 系数是有符号(P1/P2 还要先减 2^14)。
         *     统一按有符号解析会把 T1/T2 的高位当符号位, 冷天直接算出负温度。
         *
         * @return false 表示 I2C 失败(err=BARO_READ_FAILED)或系数明显无效(err=BARO_CALIB_INVALID)
         */
        bool readCalib(ExceptionCode &err)
        {
            uint8_t buf[BMP388_CALIB_LEN];
            if (!readRegs(BMP388_Reg::CalibStart, buf, BMP388_CALIB_LEN)) {
                err = ExceptionCode::BARO_READ_FAILED;
                return false;
            }

            // 有效性检查: 全 0 或全 FF。
            // 为什么只查这两个极端值: BMP388 **没有** BME280 那样的标定 CRC(手册没有定义任何
            // 校验寄存器/字段), 所以只能做"明显没读出来"的判定。全 0 = 器件没驱动数据线 /
            // 地址错, 全 FF = NVM 没读出 / 总线悬空。逐系数的量程检查不做: 手册只给了"典型"
            // 范围, 卡严了会把正常器件判死(宁可让上层用一次实飞数据交叉验证)。
            bool allZero = true;
            bool allOnes = true;
            for (uint8_t i = 0; i < BMP388_CALIB_LEN; ++i) {
                allZero = allZero && (buf[i] == 0x00u);
                allOnes = allOnes && (buf[i] == 0xFFu);
            }
            if (allZero || allOnes) {
                err = ExceptionCode::BARO_CALIB_INVALID;
                return false;
            }

            const int32_t  nvm_t1  = static_cast<int32_t>(le16(buf + 0));  // 无符号 16bit
            const int32_t  nvm_t2  = static_cast<int32_t>(le16(buf + 2));  // 无符号 16bit
            const int32_t  nvm_t3  = static_cast<int32_t>(static_cast<int8_t>(buf[4]));
            const int32_t  nvm_p1  = static_cast<int32_t>(static_cast<int16_t>(le16(buf + 5)));
            const int32_t  nvm_p2  = static_cast<int32_t>(static_cast<int16_t>(le16(buf + 7)));
            const int32_t  nvm_p3  = static_cast<int32_t>(static_cast<int8_t>(buf[9]));
            const int32_t  nvm_p4  = static_cast<int32_t>(static_cast<int8_t>(buf[10]));
            const int32_t  nvm_p5  = static_cast<int32_t>(le16(buf + 11)); // 无符号 16bit
            const int32_t  nvm_p6  = static_cast<int32_t>(le16(buf + 13)); // 无符号 16bit
            const int32_t  nvm_p7  = static_cast<int32_t>(static_cast<int8_t>(buf[15]));
            const int32_t  nvm_p8  = static_cast<int32_t>(static_cast<int8_t>(buf[16]));
            const int32_t  nvm_p9  = static_cast<int32_t>(static_cast<int16_t>(le16(buf + 17)));
            const int32_t  nvm_p10 = static_cast<int32_t>(static_cast<int8_t>(buf[19]));
            const int32_t  nvm_p11 = static_cast<int32_t>(static_cast<int8_t>(buf[20]));

            calib.par_t1  = static_cast<float>(nvm_t1) * bmp388_pow2f(8);   // / 2^-8
            calib.par_t2  = static_cast<float>(nvm_t2) * bmp388_pow2f(-30);
            calib.par_t3  = static_cast<float>(nvm_t3) * bmp388_pow2f(-48);
            // P1/P2 先减 2^14 再缩放 —— 减法必须在**整数**里做, 顺序反了(先缩放再减)结果不同
            calib.par_p1  = static_cast<float>(nvm_p1 - 16384) * bmp388_pow2f(-20);
            calib.par_p2  = static_cast<float>(nvm_p2 - 16384) * bmp388_pow2f(-29);
            calib.par_p3  = static_cast<float>(nvm_p3) * bmp388_pow2f(-32);
            calib.par_p4  = static_cast<float>(nvm_p4) * bmp388_pow2f(-37);
            calib.par_p5  = static_cast<float>(nvm_p5) * bmp388_pow2f(3);   // / 2^-3
            calib.par_p6  = static_cast<float>(nvm_p6) * bmp388_pow2f(-6);
            calib.par_p7  = static_cast<float>(nvm_p7) * bmp388_pow2f(-8);
            calib.par_p8  = static_cast<float>(nvm_p8) * bmp388_pow2f(-15);
            calib.par_p9  = static_cast<float>(nvm_p9) * bmp388_pow2f(-48);
            calib.par_p10 = static_cast<float>(nvm_p10) * bmp388_pow2f(-48);
            calib.par_p11 = static_cast<float>(nvm_p11) * bmp388_pow2f(-65);
            return true;
        }

        /* ==================== 补偿算法(手册 §9.2/§9.3 的逐行实现) ==================== */

        /**
         * @brief 温度补偿, 返回 °C, 同时填 tLin 供气压补偿使用
         *
         * 逐行照抄手册 §9.2(不是"等价改写"): 这个式子是定点/浮点混排的历史产物,
         * 改写(例如把 par_t1 移进括号、或者先平方再乘)在浮点下**数值上不等价**,
         * 而它在实机上的正确性只能靠与官方参考值对比来验证 —— 所以保持原样最安全。
         *
         * 精度说明: uncomp_temp(<= 2^24-1)与 par_t1(<= NVM_T1 × 256 < 2^24)都是 binary32
         * 能**精确**表示的整数, 所以两者相减没有舍入; 误差只来自后面与系数相乘的 1e-7 相对量级。
         */
        float compensateTemperature(uint32_t uncompTemp)
        {
            const float partialData1 = static_cast<float>(uncompTemp) - calib.par_t1;
            const float partialData2 = partialData1 * calib.par_t2;
            tLin = partialData2 + (partialData1 * partialData1) * calib.par_t3;
            return tLin;
        }

        /**
         * @brief 气压补偿, 返回 Pa(必须先算过温度, 见 tLin 的说明)
         *
         * 逐行照抄手册 §9.3。调用顺序是**硬约束**: partial_out1 / partial_out2 都含 tLin,
         * 用的是同一次测量的温度。
         */
        float compensatePressure(uint32_t uncompPress)
        {
            const float u = static_cast<float>(uncompPress);

            const float partialData1a = calib.par_p6 * tLin;
            const float partialData2a = calib.par_p7 * (tLin * tLin);
            const float partialData3a = calib.par_p8 * (tLin * tLin * tLin);
            const float partialOut1   = calib.par_p5 + partialData1a + partialData2a + partialData3a;

            const float partialData1b = calib.par_p2 * tLin;
            const float partialData2b = calib.par_p3 * (tLin * tLin);
            const float partialData3b = calib.par_p4 * (tLin * tLin * tLin);
            const float partialOut2   = u * (calib.par_p1 + partialData1b + partialData2b + partialData3b);

            const float partialData1c = u * u;
            const float partialData2c = calib.par_p9 + calib.par_p10 * tLin;
            const float partialData3c = partialData1c * partialData2c;
            const float partialData4  = partialData3c + (u * u * u) * calib.par_p11;

            return partialOut1 + partialOut2 + partialData4;
        }

        ReadResult readBurst(uint32_t &rawPressure, uint32_t &rawTemp, uint8_t &status, ExceptionCode &err)
        {
            uint8_t buffer[BMP388_DATA_BURST_LEN];
            if (!readRegs(BMP388_Reg::Status, buffer, BMP388_DATA_BURST_LEN)) {
                ++failStreak;
                ++staleStreak;
                err = ExceptionCode::BARO_READ_FAILED;
                return ReadResult::Failed;
            }
            failStreak = 0u;
            status = buffer[0];
            if ((status & BMP388_STATUS_DRDY_PRESS) == 0u) {
                ++staleStreak;
                return ReadResult::NotReady;
            }
            rawPressure = (static_cast<uint32_t>(buffer[3]) << 16) |
                          (static_cast<uint32_t>(buffer[2]) << 8) | static_cast<uint32_t>(buffer[1]);
            rawTemp = (static_cast<uint32_t>(buffer[6]) << 16) |
                      (static_cast<uint32_t>(buffer[5]) << 8) | static_cast<uint32_t>(buffer[4]);
            staleStreak = 0u;
            return ReadResult::NewData;
        }

    public:
        /**
         * @param bus 已 init 的 I2C 总线
         * @param address 7 位从机地址, 默认 cfg::hw::baro::kAddr(本板 0x76, SDO 接 GND)
         */
        BMP388(IICBus &bus, uint16_t address = BMP388_IIC_ADDRESS) : dev(bus, address)
        {
        }

        ~BMP388()
        {
        }

        /**
         * @brief 软复位 -> 校验 CHIP_ID -> 读标定系数 -> 写配置 -> 回读校验, **失败自动重试**
         *
         * 最坏阻塞 ≈ kInitRetry × kInitRetryMs + 重试次数 × tryInit() ≈ 2s 量级(见文件头时序预算)。
         * 因此**只能在初始化线程/io 线程调用**, 控制环里禁止调用。
         * 为什么把重试放在驱动里: 开机先初始化 flash, 那一段片内擦写干扰可能让气压计一时不应答
         * (cfg::sensor::baro::kInitRetry 的 evidence 写得很清楚), 让每个调用方各写一遍重试循环
         * 只会写出 20 个不一样的版本。想自己掌握节奏的调用方用 tryInit()。
         *
         * @param err 出参: 最后一次失败的原因; 成功时**不写**(调用方看返回值)。
         *            **FAILED ⇒ err 一定有值**(每一次尝试都会在失败时写)。
         *            注意只保留"最后一次"的原因(不做失败历史的统计): 20 次里原因变来变去时, 驱动
         *            不替上层做诊断 —— 那种情况要看总线/供电, 而不是看错误码。
         * @return FunctionResult::SUCCESS 表示器件已配置好并开始连续转换
         */
        FunctionResult init(ExceptionCode &err)
        {
            for (uint32_t attempt = 0u; attempt < cfg::sensor::baro::kInitRetry; ++attempt) {
                if (attempt != 0u) {
                    // 为什么间隔要睡在这里而不是让调用方退避: 重试间隔就是"给器件/总线恢复的时间",
                    // 与重试次数是一对参数(kInitRetry/kInitRetryMs), 拆开放在两边只会让两边都不完整。
                    delay_ms(cfg::sensor::baro::kInitRetryMs);
                }
                if (tryInit(err) == FunctionResult::SUCCESS) {
                    return FunctionResult::SUCCESS;
                }
            }
            /**
             * 走到这里 = 每次都失败了, 原因由**最后一次** tryInit 写进 err。
             *
             * 唯一没被写到的情形是 kInitRetry 被配成 0(一次都没试): 那时 err 还是调用方的旧值。
             * 所以在这里补一个原因, 而不是像以前那样在入口先写一个兜底 —— 入口写会在**成功**路径上
             * 留下一句谎话(成功却带着"没找到"), 而成功路径根本不写 err(见 dlx_exception.h)。
             */
            if (cfg::sensor::baro::kInitRetry == 0u) {
                err = ExceptionCode::BARO_NOT_FOUND;
            }
            return FunctionResult::FAILED;
        }

        /**
         * @brief 只做**一次**完整初始化尝试(不含 kInitRetryMs 退避), 正常约 3ms
         *
         * 留给"想自己在 io 线程里做状态机重试"的调用方: 每次调用最多阻塞到一次 tryInit 的上界,
         * 不睡 100ms。语义与 init() 的单次尝试完全一致。
         */
        FunctionResult tryInit(ExceptionCode &err)
        {
            // 一进来就把"已初始化"作废: 软复位会把配置全部清空, 中途失败时绝不能继续当"已初始化"
            // 用 —— 那会让 read() 拿着默认配置(过采样 ×4、ODR 最慢)的数据当正常数据喂给 EKF。
            initialized = false;

            // ---- 1. 软复位 ----
            // 手册 §4.3.29: CMD(0x7E) <- 0xB6。复位会把**所有用户配置**恢复默认(手册 §3.2),
            // 所以下面的配置必须写在复位之后 —— 先配好再复位等于白配。
            if (!writeReg(BMP388_Reg::Cmd, BMP388_CMD_SOFT_RESET)) {
                err = ExceptionCode::BARO_NOT_FOUND; // 写都没写进去: 总线上没有这个东西
                return FunctionResult::FAILED;
            }
            // 为什么不能靠"读 CMD 看它变回 0x00"来判断复位完成: CMD 恒回读 0x00(手册 §4.3.29 明说)。
            delay_ms(BMP388_SOFT_RESET_WAIT_MS);

            // ---- 2. CHIP_ID(0x00, 期望 0x50) ----
            uint8_t id = 0u;
            if (!readRegs(BMP388_Reg::ChipId, &id, 1u)) {
                err = ExceptionCode::BARO_NOT_FOUND;
                return FunctionResult::FAILED;
            }
            chipIdValue = id;
            if (id != BMP388_CHIP_ID_VALUE) {
                err = ExceptionCode::BARO_CHIP_ID_MISMATCH;
                return FunctionResult::FAILED;
            }

            // ---- 3. 标定系数 ----
            if (!readCalib(err)) {
                return FunctionResult::FAILED;
            }

            // ---- 4. 写配置(工作参数全部来自 cfg::sensor::baro) ----
            // 顺序: 先静态配置(过采样/数据率/滤波/中断), 最后 PWR_CTRL 开测量。
            // 为什么 PWR_CTRL 放最后: 它一写下去器件就开始转换, 先开的活会让前几个样本用旧配置,
            // 而且手册 §3.4.3 说"写 CONFIG 会复位 IIR" —— 先写滤波再开测量, 第一个样本就是干净的。
            const bool wroteOk = writeReg(BMP388_Reg::Osr, cfg::sensor::baro::kOsr) &&
                                 writeReg(BMP388_Reg::Odr, cfg::sensor::baro::kOdr) &&
                                 writeReg(BMP388_Reg::Config, cfg::sensor::baro::kConfig) &&
                                 writeReg(BMP388_Reg::IntCtrl, cfg::sensor::baro::kIntCtrl) &&
                                 writeReg(BMP388_Reg::PwrCtrl, BMP388_PWR_CTRL_VALUE);
            if (!wroteOk) {
                err = ExceptionCode::BARO_NOT_FOUND;
                return FunctionResult::FAILED;
            }

            // ---- 5. 回读校验 ----
            // 为什么要回读: 气压计的价值全在"气压的绝对值可信", 而配置没生效的表现是
            // "数据看着正常、噪声/延迟却不对"(例如 ODR 没写进去, 采样率还停在默认)。
            // 一次性读 0x19~0x1D(5 字节, 全是定义过的寄存器) + 单独读 0x1F, 共 2 次事务。
            // 为什么跳过 0x1E: 它是保留寄存器, 读它手册没保证 —— 为省一次事务去踩没依据的地址不值得。
            //
            // 为什么这里敢**立刻**回读: 手册 §3.9.1 说模式下发起的模式切换要等当前测量周期结束,
            // 而本驱动是"软复位(进睡眠) -> 写配置 -> 开连续转换": 写 PWR_CTRL 时器件在睡眠态、
            // 没有正在进行的测量, 所以立刻生效、立刻回读是对的。**将来若改成运行中切模式**
            // (例如先用强制模式跑自检), 这一处回读就可能读到旧值, 那时必须改成"等一个转换周期
            // 再回读"或轮询 STATUS.cmd_rdy —— 别把它当成"回读不可靠"而干脆删掉。
            uint8_t rb[BMP388_CFG_READBACK_LEN];
            if (!readRegs(BMP388_Reg::IntCtrl, rb, BMP388_CFG_READBACK_LEN)) {
                err = ExceptionCode::BARO_READ_FAILED;
                return FunctionResult::FAILED;
            }
            uint8_t cfgReg = 0u;
            if (!readRegs(BMP388_Reg::Config, &cfgReg, 1u)) {
                err = ExceptionCode::BARO_READ_FAILED;
                return FunctionResult::FAILED;
            }
            const bool readbackOk = (rb[0] == cfg::sensor::baro::kIntCtrl) &&
                                    (rb[1] == BMP388_IF_CONF_VALUE) &&
                                    (rb[2] == BMP388_PWR_CTRL_VALUE) &&
                                    (rb[3] == cfg::sensor::baro::kOsr) &&
                                    (rb[4] == cfg::sensor::baro::kOdr) &&
                                    (cfgReg == cfg::sensor::baro::kConfig);
            if (!readbackOk) {
                // 为什么用 NOT_FOUND 而不是新错误码: 0x0400~0x0405 已经分配满(本驱动不新增错误码),
                // 而"写进去的值读回来不一样"最贴切的语义就是"总线上的东西不是我们以为的那颗芯片"
                // (地址撞车 / 器件在 SPI 模式 / 供电不足导致寄存器写不进)。
                err = ExceptionCode::BARO_NOT_FOUND;
                return FunctionResult::FAILED;
            }

            failStreak  = 0u;
            staleStreak = 0u;
            initialized = true;
            return FunctionResult::SUCCESS;
        }

        ReadResult readRaw(uint32_t &rawPressure, uint32_t &rawTemp, ExceptionCode &err)
        {
            if (!initialized) {
                err = ExceptionCode::BARO_CALIB_INVALID;
                return ReadResult::Failed;
            }
            uint8_t status = 0u;
            return readBurst(rawPressure, rawTemp, status, err);
        }

        ReadResult read(float &pressurePa, float &temperatureC, ExceptionCode &err)
        {
            uint32_t rawPressure = 0u;
            uint32_t rawTemp = 0u;
            const ReadResult result = readRaw(rawPressure, rawTemp, err);
            if (result != ReadResult::NewData) return result;
            const float temperature = compensateTemperature(rawTemp);
            const float pressure = compensatePressure(rawPressure);
            if (!std::isfinite(temperature) || !std::isfinite(pressure) || pressure <= 0.0f) {
                err = ExceptionCode::BARO_READ_FAILED;
                return ReadResult::Failed;
            }
            temperatureC = temperature;
            pressurePa = pressure;
            return ReadResult::NewData;
        }

        /**
         * @brief 气压 -> 高度 [m](国际标准大气, 与旧 BME280 驱动同一个公式)
         *
         * 为什么公式不变: 它只跟气压有关(BMP388 没有湿度, 反而少了一个误差源), 而且是上层
         * 相对高度/定高环一直在用的定义, 换传感器不该改变"高度"的语义。
         */
        float getAltitude(float pressurePa, float seaLevelPressure = 101325.0f) const
        {
            return 44330.0f * (1.0f - powf(pressurePa / seaLevelPressure, 0.190295f));
        }

        /** 连续 I2C 事务失败次数，事务成功清零。 */
        uint32_t failCount() const { return failStreak; }

        /** 连续"没有新样本"次数(含事务失败); 健康管理可用它做更细的判定 */
        uint32_t staleCount() const { return staleStreak; }

        /** tryInit()/init() 是否成功过 */
        bool isReady() const { return initialized; }

        /** 最近一次读到的 CHIP_ID(上板排查用: 0x50 才是 BMP388) */
        uint8_t lastChipId() const { return chipIdValue; }
    };
} // namespace dlx

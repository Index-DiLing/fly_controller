#pragma once
//===================================================================================================
// DL_LIB/VL53L1X/dlx_vl53l1x.hpp —— VL53L1X (ST FlightSense ToF) 激光测距驱动, I2C 模式
//
// 【职责】
//   把"上电到能读距离"的全过程封在这一个类里: 等启动 → 切 2V8 I/O 模式 → 校验身份 →
//   下载 91 字节 ULD 配置 → 设距离模式/时间预算 → 设中断极性 → 启动连续测距 → 按需取一帧。
//   不做的事: 不控 XSHUT 引脚(调用方持有)、不做 GPIO/EXTI 注册、不做滤波/健康位/日志、
//   不做标定流程(只把标定值写进寄存器)。这些都属于 B6 的 range 线程。
//
//   ！！时间约束(方案 §5: 一次 I/O 不许出现在控制环里)！！
//   控制环是 500Hz / 2ms 一拍, 下面这些全是**阻塞**操作(400kHz 下 1 字节 ≈ 22.5us, 估算见末表):
//     init()             典型 ≈ 5ms, 最坏 ≈ 28ms(启动轮询窗口占大头) —— 只能出现在开机路径
//     startMeasurement() ≈ 0.4ms
//     readIfReady()      未就绪 ≈ 0.2ms; 拿到数据 ≈ 0.85ms(就绪位 + 17 字节块 + 清中断)
//   0.85ms 已经是控制环预算(2ms)的 40%, 而且它还不含"取不到数据要重试"的抖动 ⇒ **必须在
//   time.range.kFallbackMs(100ms) 兜底轮询 / EXTI3 唤醒的 range 线程里调用**, 不许在控制环里调。
//   init() 的最坏 28ms 同理: 只能放在开机初始化里, 而且要接受"整机开机多等 30ms"。
//   **一次测距本身**由器件自己跑完, 就是时间预算 kTimingBudgetMs = 33ms(配置里 kInterMeasMs=50ms
//   ⇒ 20Hz); 驱动**不阻塞**等它 —— readIfReady() 只回答"这一帧好了没", 所以调用方要按轮询/中断
//   反复调它, 而不是期望一次调用就把 33ms 等掉。
//
// 【寄存器约定】(DS12385 §4.2 + 图 13~16)
//   * 寄存器地址是 **16 位**, 总线顺序 **高字节在前**(regHi, regLo)。
//   * 多字节访问自动递增, 数据 **MSB 在前**; 所以 16 位量(0x005E/0x0061)先发高字节,
//     32 位量(0x006C)先发最高字节。突发读一次可以跨多个寄存器(本驱动靠它一次读 17 字节结果块)。
//   * 一条"寄存器写"事务 = START + 从机地址+W + regHi + regLo + data... + STOP。
//   * 一条"寄存器读"事务 = START + 从机地址+W + regHi + regLo + 重复START + 从机地址+R + data... + STOP。
//   * 中断必须清(DS §3.4: "A clear interrupt is mandatory to allow the next ranging data to be
//     updated") ⇒ 每读走一帧就往 0x0086 写 0x01。
//
// 【状态码表】(0x0089 低 5 位; 侦察文档 §7 的 ST ULD 映射; 只有 9 被本驱动当作有效)
//     9  = valid          量程有效                                  ← 唯一接受
//     6  = sigma_fail     σ 超过 0x0064 门限
//     4  = signal_fail    回波率低于 0x0066 门限 / MSRC 无目标
//     8  = min_range_clip 量程有效但被最小距离裁剪(再被 kMinValidMm 拦一次)
//     5  = out_of_bounds  相位/量程越界
//     3  = hardware_fail  硬件故障(API 把 1/2/17 也归到这类)
//     7  = wrap_target_fail 相位一致性(缠绕)失败
//     12 = xtalk_signal_fail 串扰/量程忽略门限导致的信号失败
//     13 = min_range_fail USERROICLIP 最小距离失败
//     18 = sync_int       同步中断(GPH stream count 0 ready)
//     其它 = unknown
//   为什么"只认 9"而不是"mm 在窗口内就算有效": 6/4/12 这些状态下器件照样会吐一个 mm 数,
//   那是噪声不是距离。把它当高度喂进 EKF, 表现是"飞机在悬停但高度在抖", 比直接标无效难查得多
//   (侦察 §7 陷阱 b: 反射率一变, Long 模式的量程能从 360cm 掉到 170cm)。
//
// 【与手册/任务书的差异 —— 逐条都有依据】
//   1. **任务书写"读 0x0031(range status)"是错的**: 手册/寄存器映射里 0x0031 是
//      GPIO__TIO_HV_STATUS(中断状态位, ST ULD 用它做 CheckForDataReady), 器件的测距状态码在
//      **0x0089 = RESULT__RANGE_STATUS** 的低 5 位。本驱动按手册实现: 0x0031 用来判"新数据到没到",
//      0x0089 用来取状态码并原样交给调用方, 距离取 0x0096(结果块偏移 +13)。
//      两者都落在同一个 17 字节块里, 所以本驱动一次突发读 0x0089..0x0099 —— 拆成"读状态"+
//      "读距离"两条事务的话, 两次之间可能已经换了一帧, 状态与距离就不是同一帧的了。
//   2. 数据手册**没有任何寄存器级信息**(DS §4.2 明确把读者推给 UM2356, 而 UM2356 本次拿不到)
//      ⇒ 寄存器地址与 91 字节数组都来自 ST 官方驱动(见 dlx_vl53l1x_config.h 文件头的来源说明)。
//      **这是本驱动最大的风险点: 数组未经上板验证, 也没有第二份独立来源。**
//   3. 2V8 I/O 模式: 模块**没有 DVDD/IOVDD 引脚**, I/O 电平是一个"模式"(复位默认 1V8)。3.3V 供电
//      必须写 0x002E bit0=1(默认 `kIOMode2V8=true`) 与 0x002F bit0=1, 否则 I2C/GPIO 的高电平
//      门限不对, 可能直接完全不通。本驱动把它放在**读模型 ID 之前**, 写完还回读确认。
//   4. ULD 表里 0x0081 = 0x9B, 而 ST API 的 preset_mode_standard_ranging 写 0x8B(bit4 差 1 位):
//      本驱动跟随 ULD 表(0x9B) —— ST 自己的 VL53L1X_SensorInit() 也是照抄这张表, 不要"顺手修正"。
//   5. ULD 表里 0x0061/0x0062 = 0x01F1, 而固定表的 Long+100ms 是 0x01EA: 说明这张表是在某一片
//      具体器件上抓出来的, 不是精确的理论值。本驱动随后会显式改写模式与预算寄存器, 不受它影响。
//   6. ULD 收尾的 0x0008=0x09 / 0x000B=0x00("two bounds VHV"/"从上次温度开始") 与那次 dummy 测距
//      **本驱动不做**: 那两步只有在"已经成功测过一次"之后写才有意义(否则"上次温度"根本不存在),
//      要么整段做(含一次完整测距, 33ms) 要么整段不做。开机路径上省掉一个 TB, 而且第一帧真数据
//      本来就在 33ms 后到。若以后发现首帧精度不够, 再按侦察 §6.1 的 3~7 步补上。
//   7. 固定表路径不写 MM_CONFIG__TIMEOUT_MACROP_A/_B(0x005A/0x005C): 它们只在 Pololu 的"动态
//      预算"路径里被算出来, 而低功耗自动模式下 MM 步进本来就被禁用(侦察 §4b)。
//   8. 串扰/零偏标定值来自配置(kCalibCrosstalk/kCalibOffsetMm), 默认 0 = 未标定 ⇒ 一个寄存器都不写。
//      串扰的 7.9 定标公式只有侦察文档一处依据("xtalk << 9 / 1000"), 上板标定时要再核一次。
//
// 【XSHUT 引脚: 由调用方控制, 驱动只等】
//   * 硬件待机 = 有电 + XSHUT 低; 手册要求 "XSHUT has to be raised only when the power supply is
//     tied on"(上电期间必须先把它按住), 这只有调用方(开机初始化)知道电源时序 ⇒ 驱动的构造函数
//     不接收 GPIOProfile, 也就不可能在主机单测里误初始化一个真实引脚。
//   * **拉高后要等多久**: 手册只给了 "boot duration 1.2ms maximum"(DS §3.6), 没给最小值; 这段时间
//     内器件不响应 I2C(会 NACK)。所以驱动不去"睡固定 1.2ms", 而是轮询
//     FIRMWARE__SYSTEM_STATUS(0x00E5) bit0 —— 见 waitBoot(): ACK 了但 bit0=0 → 还在启动;
//     20ms 内一直不 ACK → 判定总线上没有这个器件(RANGE_NOT_FOUND)。
//   * 调用方如果愿意自己等, 请等 "≥1.2ms + 余量(建议 2ms)" 再调 init(); 即使不等也不会错,
//     驱动自己的轮询窗口(20ms)足够覆盖。**但绝不能在 AVDD 还没上电时把 XSHUT 拉高**。
//
// 【阻塞时间预算】(估算前提: 400kHz, 9bit/字节 ⇒ 22.5us/字节; 侦察 §9)
//   1 字节寄存器写 ≈ 5 字节 ≈ 0.11ms      1 字节寄存器读 ≈ 8 字节 ≈ 0.18ms
//   17 字节结果块读 ≈ 24 字节 ≈ 0.54ms    91 字节 ULD 块写(2+91 字节) ≈ 2.1ms
//   init 典型 ≈ 5ms(启动已就绪时首轮就 ACK) / 最坏 ≈ 28ms(启动窗口 20ms + 20 次失败读 ≈ 3.6ms)
//   readIfReady 未就绪 ≈ 0.18ms / 拿到一帧 ≈ 0.85ms
//   91 字节块**一次事务**下发, 而不是 91 次单字节写: 后者 ≈ 91×0.11 ≈ 10ms, 慢 5 倍。
//   栈占用: 块写缓冲 2+91 字节(仅 init 期间), 读缓冲最多 17 字节 ⇒ 最坏 ≈ 115 字节(不含库调用)。
//===================================================================================================
#include <stdint.h>
#include "dlx_iic.hpp"
#include "dlx_delay.hpp"
#include "dlx_exception.h"
#include "dlx_vl53l1x_config.h"

namespace dlx
{
    /**
     * @brief VL53L1X 激光测距(ToF)驱动, I2C 模式, 阻塞式同步接口
     *
     * 用法(时序责任在调用方):
     *   // 上电期间 XSHUT 必须为低(硬件待机)
     *   GPIO xshut(cfg::hw::range_::kXshut); xshut.init(OUT_PP_...); xshut = 0; delay_ms(2);
     *   IICBus bus = IICBus::IIC2_SF1_DF0();          // SCL=PF1 / SDA=PF0
     *   bus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, cfg::hw::range_::kSpeed, cfg::hw::range_::kAddr);
     *   xshut = 1;                                     // 释放 XSHUT(等多久见文件头)
     *   VL53L1X range(bus);                            // 默认地址 cfg::hw::range_::kAddr = 0x29
     *   ExceptionCode err;                             // 只在失败时有意义(看返回值, 不看码是不是 0)
     *   if (!range.init(err)) { ... }                  // 典型 5ms, 最坏 28ms
     *   // range 线程(20~50ms 一拍, 或 EXTI3 唤醒):
     *   float m; uint8_t st;
     *   switch (range.readIfReady(m, st, err)) {        // 状态由返回值给, 不靠"err 是不是 OK"分流
     *   case VL53L1X::RangeRead::NewData:  ...本次有新数据...; break;
     *   case VL53L1X::RangeRead::NotReady: break;      // 还没测完: **不是故障**
     *   case VL53L1X::RangeRead::Invalid:  break;      // 提示性: 量程无效
     *   case VL53L1X::RangeRead::Failed:   DLX_HANDLE_EXCEPTION(err); break;
     *   }
     */
    class VL53L1X
    {
    private:
        IICDevice dev; ///< 总线引用 + 从机地址(与 BME280 同样的薄封装)

        // ---------------- 运行状态 ----------------
        bool     initialized  = false; ///< init() 是否成功过(startMeasurement 的准入条件)
        bool     intActiveLow = false; ///< 缓存 0x0030 bit4: false = 高有效(GPIO1 出厂默认)
        uint32_t oscCalibrate = 0u;    ///< 0x00DE 振荡器标定值(算连续测距间隔用)
        uint8_t  modelId      = 0u;    ///< 0x010F
        uint8_t  moduleType   = 0u;    ///< 0x0110
        uint8_t  revisionId   = 0u;    ///< 0x0111(掩模版本, 只记录不强制 —— 见 readIdentity)
        uint8_t  failCount    = 0u;    ///< 连续失败计数(成功即清零)
        uint16_t lastRawMm    = 0u;    ///< 最近一帧器件原始 mm(未做增益修正)
        uint16_t lastPeakSignal = 0u;  ///< 0x0098 峰值回波率(串扰修正后)
        uint16_t lastAmbient    = 0u;  ///< 0x0090 环境光计数率

        /** 块写缓冲上限: 只有 ULD 下载用得着(2 字节地址 + 91 字节数据) */
        static constexpr uint16_t kMaxWriteData = VL53L1X_ULD_SIZE;

        //===========================================================================================
        // 私有小工具: 全部返回 false = I2C 传输异常(不是"值不对")
        //===========================================================================================

        /** 写块: START + 地址+W + regHi + regLo + data[n] + STOP(n ≤ kMaxWriteData) */
        bool writeRegs(VL53L1X_Reg reg, const uint8_t *data, uint16_t n)
        {
            if (n > kMaxWriteData) {
                return false; // 防御: 只有 ULD 91 字节会走这条路, 超了说明代码写错了
            }
            const uint16_t addr = static_cast<uint16_t>(reg);
            uint8_t buf[2 + kMaxWriteData];
            buf[0] = static_cast<uint8_t>(addr >> 8);        // 寄存器地址高字节在前(DS §4.2)
            buf[1] = static_cast<uint8_t>(addr & 0xFFu);
            for (uint16_t i = 0; i < n; ++i) {
                buf[2 + i] = data[i];
            }
            ByteBuffer w(buf, static_cast<uint16_t>(2 + n));
            w.write(buf, static_cast<uint16_t>(2 + n)); // 标记已写入, 否则 used()==0, 一个字节都发不出
            return dev.write(w);
        }

        bool writeReg(VL53L1X_Reg reg, uint8_t value)
        {
            return writeRegs(reg, &value, 1);
        }

        /** 16 位量: **MSB 在前**(DS §4.2) */
        bool writeReg16(VL53L1X_Reg reg, uint16_t value)
        {
            const uint8_t d[2] = { static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFFu) };
            return writeRegs(reg, d, 2);
        }

        /** 32 位量(0x006C 连续测距间隔): **最高字节在前** */
        bool writeReg32(VL53L1X_Reg reg, uint32_t value)
        {
            const uint8_t d[4] = { static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
                                   static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value) };
            return writeRegs(reg, d, 4);
        }

        /** 从 reg 起突发读 n 字节; false = I2C 异常(out 已清零, 内容不可用) */
        bool readRegs(VL53L1X_Reg reg, uint8_t *out, uint16_t n)
        {
            // 先清零: I2C 出错(超时/NACK)时一个字节都不会写进来, 不清零调用方就会读到栈上旧值,
            // 例如模型 ID 读失败却"碰巧"等于 0xEA, 把不可用的测距当成好的(与 BME280::readRegs 同理)。
            for (uint16_t i = 0; i < n; ++i) {
                out[i] = 0u;
            }
            const uint16_t addr = static_cast<uint16_t>(reg);
            uint8_t a[2] = { static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFFu) };
            ByteBuffer w(a, 2);
            w.write(a, 2); // 标记寄存器地址已写入, 否则读事务不带寄存器地址
            ByteBuffer r(out, n);
            return dev.read(r, w, n);
        }

        bool readReg(VL53L1X_Reg reg, uint8_t &value)
        {
            uint8_t v = 0u;
            if (!readRegs(reg, &v, 1)) {
                value = 0u;
                return false;
            }
            value = v;
            return true;
        }

        bool readReg16(VL53L1X_Reg reg, uint16_t &value)
        {
            uint8_t b[2] = { 0u, 0u };
            if (!readRegs(reg, b, 2)) {
                value = 0u;
                return false;
            }
            value = static_cast<uint16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
            return true;
        }

        /** 0x0031 bit0 等于它就表示"新样本就绪"(ST ULD 的 CheckForDataReady: IntPol = !bit4) */
        uint8_t readyLevel() const
        {
            return intActiveLow ? 0u : 1u;
        }

        /** 连续失败计数; 到达 cfg::sensor::range::kFailLimit 就把 err 升级成 RANGE_DATA_STALE */
        void noteFailure(ExceptionCode &err)
        {
            if (failCount < 0xFFu) {
                ++failCount;
            }
            if (failCount >= cfg::sensor::range::kFailLimit) {
                err = ExceptionCode::RANGE_DATA_STALE;
            }
        }

        //===========================================================================================
        // init 的各步骤(顺序不能换, 理由见每一步的注释)
        //===========================================================================================

        /**
         * ① 等启动完成(XSHUT 已由调用方释放)。
         * 为什么轮询而不是睡固定 1.2ms: 手册只给了 boot 的**上限**, 而且启动期间器件 NACK ——
         * 睡固定值要么浪费(运气好时 0.2ms 就起来了), 要么不够(电源爬升慢); 轮询两者兼顾。
         * 为什么区分两个错误码: "一直不 ACK" = 总线上没有这个器件(RANGE_NOT_FOUND);
         * "ACK 了但 bit0 一直是 0" = 器件在, 但没起来(RANGE_BOOT_TIMEOUT)。这两种故障的现场
         * 处理完全不同(前者查接线/供电/地址, 后者查 XSHUT 与复位), 混成一个码会白跑一趟。
         */
        bool waitBoot(ExceptionCode &err)
        {
            bool anyAck = false;
            for (uint16_t i = 0; i < VL53L1X_BOOT_POLL_ATTEMPTS; ++i) {
                uint8_t status = 0u;
                if (readReg(VL53L1X_Reg::FirmwareSystemStatus, status)) {
                    anyAck = true;
                    if ((status & 0x01u) != 0u) {
                        return true;
                    }
                }
                delay_ms(VL53L1X_BOOT_POLL_INTERVAL_MS);
            }
            err = anyAck ? ExceptionCode::RANGE_BOOT_TIMEOUT : ExceptionCode::RANGE_NOT_FOUND;
            return false;
        }

        /**
         * ② 切 2V8 I/O 模式(0x002E bit0 = 1, 0x002F bit0 = 1)。
         *
         * **这是头号坑**: 模块没有 DVDD/IOVDD 引脚, I/O 电平是个"模式"; 复位默认 1V8, 而板上
         * I2C 上拉到 3.3V。不切 2V8, pad 的参考电平就不对, 表现可能是"完全不通"或"时通时不通"。
         * 为什么必须在读模型 ID 之前: 一旦后面任何一步跑起来(尤其是 91 字节块下载), 都是在这两个
         * 寄存器的状态上继续; 而且先把电平修好, 后面所有事务的时序余量才成立。
         * 为什么用"读-改-写"而不是直接写 0x01: 这两个寄存器只有 bit0 有定义, 其余位属于
         * "不可修改"区, 直接整字节写会把 ST 可能预置的位抹掉。
         * 为什么写完要回读: "写进去"≠"生效" —— 这一步失败必须立刻报 RANGE_IO_VOLTAGE_FAILED,
         * 而不是带着错误的电平继续跑(项目里 LINK_CONFIG_VERIFY_FAILED 就是这个教训)。
         */
        bool enableIo2V8(ExceptionCode &err)
        {
            const VL53L1X_Reg regs[2] = { VL53L1X_Reg::PadI2cHvExtsupConfig, VL53L1X_Reg::GpioHvPadCtrl };
            for (uint8_t i = 0u; i < 2u; ++i) {
                uint8_t v = 0u;
                if (!readReg(regs[i], v)) { // 能走到这里说明 waitBoot 已经 ACK 过, 读失败就是总线坏了
                    err = ExceptionCode::RANGE_IO_VOLTAGE_FAILED;
                    return false;
                }
                if ((v & 0x01u) == 0u) { // 已经是 2V8(重试 init 的情况)就不重复写
                    if (!writeReg(regs[i], static_cast<uint8_t>(v | 0x01u))) {
                        err = ExceptionCode::RANGE_IO_VOLTAGE_FAILED;
                        return false;
                    }
                    uint8_t back = 0u;
                    if (!readReg(regs[i], back) || ((back & 0x01u) == 0u)) {
                        err = ExceptionCode::RANGE_IO_VOLTAGE_FAILED;
                        return false;
                    }
                }
            }
            return true;
        }

        /**
         * ③ 身份校验: 0x010F 模型 ID 必须 0xEA, 0x0110 模块类型必须 0xCC。
         * 为什么两个都验: 它们只差一个字节地址, 一起读几乎不花时间; 只验模型 ID 时, 若地址/字节序
         * 错位(常见错法)会读到别的寄存器, 恰好等于 0xEA 的概率虽然低但不是零。
         * 为什么 0x0111(掩模版本)只记录不强制: 不同批次/后续掩模版本可能不是 0x10, 用一个
         * "典型值"去卡死会让好板子起不来; 记下来给 diag.kVerifySensorIds 用就够了。
         */
        bool readIdentity(ExceptionCode &err)
        {
            uint8_t id[3] = { 0u, 0u, 0u };
            if (!readRegs(VL53L1X_Reg::IdentificationModelId, id, 3)) {
                err = ExceptionCode::RANGE_NOT_FOUND;
                return false;
            }
            modelId    = id[0];
            moduleType = id[1];
            revisionId = id[2];
            if (modelId != VL53L1X_MODEL_ID || moduleType != VL53L1X_MODULE_TYPE) {
                err = ExceptionCode::RANGE_NOT_FOUND;
                return false;
            }
            return true;
        }

        /**
         * ④ 下载 91 字节 ULD 配置块(0x2D..0x87), **逐字节 91 次单字节写**。
         *
         * 为什么必须按 kIOMode2V8 给 0x2E/0x2F 打补丁: ST 的原表这两字节是 0x00(1V8), 而这一段
         * **正好包含第 ② 步刚设好的这两个寄存器**; 不补丁, 下载就把 2V8 位清回 0,
         * 现场表现是"写完配置以后 I2C 就不通了", 极难查。
         * 失败 → RANGE_ULD_LOAD_FAILED(配置阶段所有写失败都归这个码)。
         *
         * 为什么不再用"一次 93 字节块写 + 回读裁定"(2026-10-04 清理): 那条路在真板上**每次都报失败**
         * (最后一字节的事件等待超时), 只能靠回读 0x002D.. 之后"其实写进去了"来放行 —— 实测确实落地了
         * (11/16 逐字节一致, 不可能是巧合)。既然逐字节写**每步都有返回码**、不再需要裁定, 就把那条
         * 路连同它的容差/裁定状态一起删掉了(代价只有 ~8ms, 见下)。
         *
         * 逐字节而不是块写的依据(PX4 vl53l1x.cpp 的 VL53L1X_SensorInit 原文):
         *     for (Addr = 0x2D; Addr <= 0x87; Addr++)
         *         status |= VL53L1_WrByte(Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
         * 块写省 2.1ms→10ms 的差值, 在开机路径上完全可以忽略; 换来的是**与一个在真机上跑了多年的
         * 实现一致**, 不再依赖那条 93 字节事务。
         */
        bool loadUld(ExceptionCode &err)
        {
            uint8_t blob[VL53L1X_ULD_SIZE];
            for (uint16_t i = 0; i < VL53L1X_ULD_SIZE; ++i) {
                blob[i] = VL51L1X_DEFAULT_CONFIGURATION[i];
            }
            if (cfg::sensor::range::kIOMode2V8) {
                blob[VL53L1X_ULD_INDEX_2E] = static_cast<uint8_t>(blob[VL53L1X_ULD_INDEX_2E] | 0x01u);
                blob[VL53L1X_ULD_INDEX_2F] = static_cast<uint8_t>(blob[VL53L1X_ULD_INDEX_2F] | 0x01u);
            }
            for (uint16_t i = 0; i < VL53L1X_ULD_SIZE; ++i) {
                if (!writeReg(static_cast<VL53L1X_Reg>(VL53L1X_ULD_START + i), blob[i])) {
                    err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                    return false;
                }
            }
            return true;
        }

        /**
         * ⑤ 距离模式 + 时间预算(必须做: ST 的 ULD 表默认是 Long + 100ms)。
         * 模式寄存器块按 cfg::sensor::range::kDistanceMode 查表, 预算按 kTimingBudgetMs 查固定表
         * (ST ULD 只接受离散值, 合法性已经在 dlx_vl53l1x_config.h 里 static_assert 过)。
         * 为什么不写 0x0068(有效相位下限): 三种模式都是 0x08, 表里已经是对的值。
         * 为什么最后补一句 0x0082 = 0: 分组参数保持位只要是非 0, 一次测量都跑不完(侦察 §7 陷阱 h),
         * 而 0x0071/0x007C 这些"分组保持"相关寄存器在别处的写入顺序里可能被带起来 —— 显式清零最便宜。
         */
        bool applyModeAndBudget(ExceptionCode &err)
        {
            const cfg::RangeDistanceMode mode = cfg::sensor::range::kDistanceMode;
            const VL53L1X_ModeRegs m = vl53l1xModeRegs(mode);

            uint16_t tbIndex = VL53L1X_TB_TABLE_SIZE; // == 表长 ⇒ 没找到
            for (uint16_t i = 0; i < VL53L1X_TB_TABLE_SIZE; ++i) {
                if (vl53l1xTbEntryAt(mode, i).ms == cfg::sensor::range::kTimingBudgetMs) {
                    tbIndex = i;
                    break;
                }
            }
            if (tbIndex >= VL53L1X_TB_TABLE_SIZE) { // 编译期已经 static_assert 过, 这里只是防御
                err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                return false;
            }
            const VL53L1X_TbEntry tb = vl53l1xTbEntryAt(mode, tbIndex); // 复制一份, 别留指针

            bool ok = true;
            ok = ok && writeReg(VL53L1X_Reg::RangeConfigVcselPeriodA, m.vcselPeriodA);
            ok = ok && writeReg(VL53L1X_Reg::RangeConfigVcselPeriodB, m.vcselPeriodB);
            ok = ok && writeReg(VL53L1X_Reg::RangeConfigValidPhaseHigh, m.validPhaseHigh);
            ok = ok && writeReg(VL53L1X_Reg::SdConfigWoiSd0, m.woiSd0);
            ok = ok && writeReg(VL53L1X_Reg::SdConfigWoiSd1, m.woiSd1);
            ok = ok && writeReg(VL53L1X_Reg::SdConfigInitialPhaseSd0, m.initialPhase);
            ok = ok && writeReg(VL53L1X_Reg::SdConfigInitialPhaseSd1, m.initialPhase);
            ok = ok && writeReg(VL53L1X_Reg::PhasecalConfigTimeoutMacrop, m.phasecalTimeout);
            ok = ok && writeReg16(VL53L1X_Reg::RangeConfigTimeoutMacropA, tb.macroA);
            ok = ok && writeReg16(VL53L1X_Reg::RangeConfigTimeoutMacropB, tb.macroB);
            ok = ok && writeReg(VL53L1X_Reg::SystemGroupedParameterHold, 0x00u); // 必须为 0
            ok = ok && writeReg(VL53L1X_Reg::SystemInterruptConfigGpio, 0x20u);  // 0x20 = 新样本就绪
            if (!ok) {
                err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                return false;
            }
            return true;
        }

        /**
         * 读振荡器标定值(0x00DE, 低 10 位有效) —— 算连续测距间隔用。
         * 为什么为 0 就当失败: 它是 NVM 标定常量, 读到 0 说明没读出来(或者读到保留区)。拿 0 去写
         * 0x006C 会让器件按最快速度连测, 与 kInterMeasMs 的心智模型完全不符(而且间隔 < 时间预算
         * 是手册不允许的组合)。宁可开机就报错。
         */
        bool readOscCalibrate(ExceptionCode &err)
        {
            uint16_t v = 0u;
            if (!readReg16(VL53L1X_Reg::ResultOscCalibrateVal, v) || ((v & 0x03FFu) == 0u)) {
                err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                return false;
            }
            oscCalibrate = static_cast<uint32_t>(v & 0x03FFu);
            return true;
        }

        /** 连续测距间隔 = OSC_CALIBRATE_VAL × kInterMeasMs × 1.075(ST ULD 的算法, 见配置头) */
        uint32_t interMeasTicks() const
        {
            const uint64_t ticks = static_cast<uint64_t>(oscCalibrate) * cfg::sensor::range::kInterMeasMs *
                                   VL53L1X_INTERMEAS_NUM / VL53L1X_INTERMEAS_DEN;
            return static_cast<uint32_t>(ticks);
        }

        /** 标定值下发(默认都是 0 = 未标定, 一个寄存器都不写) */
        bool applyCalibration(ExceptionCode &err)
        {
            if (cfg::sensor::range::kCalibOffsetMm != 0) {
                // 0x001E = 零偏(mm) × 4(侦察 §5: "value = offset_mm × 4", 高字节符号)
                const int32_t raw = static_cast<int32_t>(cfg::sensor::range::kCalibOffsetMm) * 4;
                if (!writeReg16(VL53L1X_Reg::AlgoPartToPartRangeOffsetMm, static_cast<uint16_t>(raw))) {
                    err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                    return false;
                }
            }
            if (cfg::sensor::range::kCalibCrosstalk != 0) {
                // 7.9 定标: 值 = 串扰(cps) << 9 / 1000 —— 依据只有侦察 §5 一行, 上板标定时必须复核
                const uint32_t xtalk = (static_cast<uint32_t>(cfg::sensor::range::kCalibCrosstalk) << 9) / 1000u;
                if (!writeReg16(VL53L1X_Reg::AlgoCrosstalkCompensationPlaneOffsetKcps,
                                static_cast<uint16_t>(xtalk))) {
                    err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                    return false;
                }
            }
            return true;
        }

    public:
        /**
         * @param bus 已 init 的 I2C 总线(引脚/速率由调用方按 cfg::hw::range_ 配好)
         * @param address 7 位从机地址, 默认 cfg::hw::range_::kAddr(0x29)
         */
        VL53L1X(IICBus &bus, uint16_t address = VL53L1X_IIC_ADDRESS) : dev(bus, address)
        {
        }

        ~VL53L1X()
        {
        }

        /**
         * @brief 完整初始化(典型 5ms, 最坏 28ms —— 阻塞, 只能在开机路径调用)
         *
         * 顺序(每一步的"为什么是这个顺序"见对应私有函数):
         *   ① 等启动完成(0x00E5 bit0)           失败 → RANGE_NOT_FOUND / RANGE_BOOT_TIMEOUT
         *   ② 2V8 I/O 模式(0x002E/0x002F)       失败 → RANGE_IO_VOLTAGE_FAILED
         *   ③ 模型 ID/模块类型(0x010F/0x0110)   失败 → RANGE_NOT_FOUND
         *   ④ 91 字节 ULD 块(0x2D..0x87)        失败 → RANGE_ULD_LOAD_FAILED
         *   ⑤ 距离模式 + 时间预算 + 0x0082=0     失败 → RANGE_ULD_LOAD_FAILED
         *   ⑥ 中断极性(0x0030, 按 kIntPolarity) 失败 → RANGE_ULD_LOAD_FAILED
         *   ⑦ 标定值(零偏/串扰, 默认 0 不写)    失败 → RANGE_ULD_LOAD_FAILED
         *   ⑧ 启动连续测距(0x006C/0x0086/0x0087) 失败 → RANGE_NOT_FOUND(见 startMeasurement)
         *
         * @param err 输出: **只在返回 false 时有意义** —— 具体的 ExceptionCode(0x0500~0x0506)
         * @return true = 全部成功, 可以 readIfReady()
         */
        bool init(ExceptionCode &err)
        {
            initialized = false;
            failCount   = 0u;

            if (!waitBoot(err)) {
                return false;
            }
            if (cfg::sensor::range::kIOMode2V8 && !enableIo2V8(err)) {
                return false;
            }
            if (!readIdentity(err)) {
                return false;
            }
            if (!loadUld(err)) {
                return false;
            }
            if (!applyModeAndBudget(err)) {
                return false;
            }
            if (!setInterruptPolarity(cfg::sensor::range::kIntPolarity, &err)) {
                return false;
            }
            if (!applyCalibration(err)) {
                return false;
            }
            if (!readOscCalibrate(err)) {
                return false;
            }
            // 身份与配置都已确认, 先置位再启动 —— startMeasurement() 的准入条件就是 initialized
            // (它的语义是"总线上确认过这颗器件"), 否则这里会自己把自己拦下来。
            initialized = true;
            if (!startMeasurement(&err)) {
                initialized = false;
                return false;
            }
            return true;
        }

        /**
         * @brief 设 GPIO1 中断极性(0x0030 bit4: 0 = 高有效, 1 = 低有效)
         *
         * 默认是**高有效**(ST 出厂配置 0x0030 = 0x01, bit4 = 0), 与网上很多"新数据拉低"的描述相反 ⇒
         * PA3/EXTI3 要按上升沿配。改这个函数等于改 EXTI 触发边沿, 两边必须一起改。
         * bits[3:0] 必须保持 0x1, 所以是"保留高 4 位 + 强制低 4 位 = 0x1", 不是整字节写。
         * 0x0046(bits[2:0] 中断模式)不动: 我们用的是 0x20"新样本就绪", 它不看窗口模式位
         * (ST 的表与 Pololu 也都是 0x20) —— 以后若启用阈值窗口中断, 那里才需要跟着极性改。
         *
         * @param err 可为 nullptr
         */
        bool setInterruptPolarity(cfg::RangeIntPolarity pol, ExceptionCode *err = nullptr)
        {
            uint8_t v = 0u;
            if (!readReg(VL53L1X_Reg::GpioHvMuxCtrl, v)) {
                if (err != nullptr) {
                    *err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                }
                return false;
            }
            uint8_t nv = static_cast<uint8_t>(v & 0xEFu); // 清 bit4
            if (pol == cfg::RangeIntPolarity::ActiveLow) {
                nv = static_cast<uint8_t>(nv | 0x10u);
            }
            nv = static_cast<uint8_t>((nv & 0xF0u) | 0x01u); // bits[3:0] 必须 0x1
            if (!writeReg(VL53L1X_Reg::GpioHvMuxCtrl, nv)) {
                if (err != nullptr) {
                    *err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                }
                return false;
            }
            uint8_t back = 0u;
            if (!readReg(VL53L1X_Reg::GpioHvMuxCtrl, back) || back != nv) { // 写进去≠生效
                if (err != nullptr) {
                    *err = ExceptionCode::RANGE_ULD_LOAD_FAILED;
                }
                return false;
            }
            intActiveLow = (pol == cfg::RangeIntPolarity::ActiveLow);
            return true;
        }

        /**
         * @brief 启动连续(自主)测距: 清中断 → 写间隔 → 0x0087 = 0x40
         *
         * 为什么用连续模式(0x40)而不是单次(0x10): 配置里有 kInterMeasMs(50ms) —— 它正是"连续测距
         * 间隔"寄存器(0x006C)的值; 手册也说自主连续测距是器件的默认工作方式。单次模式每帧都要重新
         * 触发, 间隔寄存器就没意义了。
         * 为什么先清中断: 上一次的残留标志会让第一次 readIfReady() 立刻"就绪"并读到旧数据。
         *
         * @param err 可为 nullptr; 未 init 或 I2C 写失败 → RANGE_NOT_FOUND(测距段没有单独的 I/O 错误码)
         */
        bool startMeasurement(ExceptionCode *err = nullptr)
        {
            bool ok = initialized;
            if (ok) {
                ok = writeReg(VL53L1X_Reg::SystemInterruptClear, 0x01u);
            }
            if (ok) {
                ok = writeReg32(VL53L1X_Reg::SystemIntermeasurementPeriod, interMeasTicks());
            }
            if (ok) {
                ok = writeReg(VL53L1X_Reg::SystemModeStart, 0x40u); // 0x40 = 自主/连续启动
            }
            if (!ok && err != nullptr) {
                *err = ExceptionCode::RANGE_NOT_FOUND; // 没 init(器件未确认) 或写不通(器件不响应)
            }
            return ok;
        }

        /**
         * @brief 停止测距(0x0087 = 0x00)。写 0x00(停止)而不是 0x80(中止): 我们不赶时间,
         *        让它把当前这一帧走完再停, 器件状态更干净。
         * @param err 可为 nullptr
         */
        bool stopMeasurement(ExceptionCode *err = nullptr)
        {
            if (!writeReg(VL53L1X_Reg::SystemModeStart, 0x00u)) {
                if (err != nullptr) {
                    *err = ExceptionCode::RANGE_NOT_FOUND;
                }
                return false;
            }
            return true;
        }

        /**
         * readIfReady() 的结果。
         *
         * 为什么是枚举, 而不是 "bool + 一个'没有异常'的码": 那个 bool 的 false 有两个完全不同的
         * 意思 —— "这一帧还没测完(**不是故障**, 原样重试)"和"真出错"; 而"没有异常"本身不是一个异常码,
         * 拿一个 OK 码去表达它, 就逼着调用方去分辨"码是 0 还是压根没被赋值"。所以拆开:
         * 状态归返回值, 原因归 err。
         *   NewData / NotReady → err **没被写过**, 调用方不要读它;
         *   Invalid / Failed   → err 里是具体原因(见下面 readIfReady 的语义表)。
         */
        enum class RangeRead : uint8_t
        {
            NewData  = 0u, ///< 有新数据: rangeM / rangeStatus 有效
            NotReady = 1u, ///< 这一帧还没测完 —— 不是故障, 原样重试即可
            Invalid  = 2u, ///< 提示性: 量程无效(不是 9 或距离出窗口), 不动健康位
            Failed   = 3u, ///< 真失败: err = RANGE_NOT_FOUND / RANGE_DATA_STALE
        };

        /**
         * @brief 有新数据就取一帧(非阻塞; 未就绪立刻返回)
         *
         * 语义(调用方按**返回值**分流; err 只在 Invalid/Failed 时有意义):
         *   NewData  → 本次是新数据, rangeM/rangeStatus 有效
         *   NotReady → **这一帧还没测完, 不是故障**(kInterMeasMs=50ms > 轮询周期时最常见), 原样重试
         *   Invalid  → 读到了, 但状态码不是 9 或距离出了 kMinValidMm/kMaxValidMm 窗口; rangeStatus
         *              原样带出器件状态码(见文件头状态码表)。连续失败累计到
         *              cfg::sensor::range::kFailLimit 次会升级成 Failed(原因码 RANGE_DATA_STALE)
         *   Failed   → err = RANGE_NOT_FOUND(I2C 读写失败/清中断失败/器件不响应)
         *               或 RANGE_DATA_STALE(连续失败已经到 kFailLimit 次)
         *
         * @param rangeM      输出: 距离 [m], 非 NewData 时置 0(**0 永远是"无效", 绝不能当"贴地"**)
         * @param rangeStatus 输出: 0x0089 低 5 位(原样); 没读到结果块时为 VL53L1X_STATUS_NONE
         * @param err         输出: **只在 Invalid/Failed 时被写入**; NewData/NotReady 时不碰它
         */
        RangeRead readIfReady(float &rangeM, uint8_t &rangeStatus, ExceptionCode &err)
        {
            // 先清输出: 调用方拿到非 NewData 却沿用上一次的 rangeM 会把"没数据"当"有数据"(同 BME280 的理由)
            rangeM      = 0.0f;
            rangeStatus = VL53L1X_STATUS_NONE;

            if (!initialized) {
                err = ExceptionCode::RANGE_NOT_FOUND;
                return RangeRead::Failed;
            }

            // ① 就绪? 读 0x0031 bit0 与"缓存的极性"比对(ST ULD 的 CheckForDataReady)。
            //    为什么缓存极性而不是每次读 0x0030: 少一条事务(0.18ms/次); 本类是唯一写 0x0030 的人。
            //    为什么"未就绪"不算失败: 见上面的语义表 —— 它只说明这一帧还没测完。
            uint8_t tio = 0u;
            if (!readReg(VL53L1X_Reg::GpioTioHvStatus, tio)) {
                err = ExceptionCode::RANGE_NOT_FOUND;
                noteFailure(err);
                return RangeRead::Failed;
            }
            if ((tio & 0x01u) != readyLevel()) {
                return RangeRead::NotReady; // 还没测完: err 不写, 调用方按返回值分流
            }

            // ② 一次突发读 17 字节结果块(0x0089..0x0099): 状态码(0x0089)与距离(0x0096)必须同帧
            uint8_t b[VL53L1X_RESULT_BLOCK_BYTES];
            if (!readRegs(VL53L1X_Reg::ResultRangeStatus, b, VL53L1X_RESULT_BLOCK_BYTES)) {
                err = ExceptionCode::RANGE_NOT_FOUND;
                noteFailure(err);
                return RangeRead::Failed;
            }

            // ③ 清中断(0x0086 = 0x01): DS §3.4 明确"不清就不会更新下一帧数据"。
            //    清失败怎么办: 这一帧数据是好的, 但下一帧永远不会来 —— 宁可丢这一帧也不能让测距停摆。
            if (!writeReg(VL53L1X_Reg::SystemInterruptClear, 0x01u)) {
                err = ExceptionCode::RANGE_NOT_FOUND;
                noteFailure(err);
                return RangeRead::Failed;
            }

            const uint8_t st = static_cast<uint8_t>(b[0] & 0x1Fu); // 低 5 位才是状态码
            uint16_t mm = static_cast<uint16_t>((static_cast<uint16_t>(
                              b[VL53L1X_RESULT_RANGE_OFFSET]) << 8) |
                          b[VL53L1X_RESULT_RANGE_OFFSET + 1]);
            lastPeakSignal = static_cast<uint16_t>(
                (static_cast<uint16_t>(b[VL53L1X_RESULT_PEAK_OFFSET]) << 8) |
                b[VL53L1X_RESULT_PEAK_OFFSET + 1]);
            lastAmbient = static_cast<uint16_t>(
                (static_cast<uint16_t>(b[VL53L1X_RESULT_AMBIENT_OFFSET]) << 8) |
                b[VL53L1X_RESULT_AMBIENT_OFFSET + 1]);
            lastRawMm   = mm;
            rangeStatus = st;

            // ④ 增益修正(ST tuning 默认的 ~98.2%): 先算再判窗口, 否则 2000mm 边界上会误判
            if (cfg::sensor::range::kGainCorrect) {
                mm = static_cast<uint16_t>((static_cast<uint32_t>(mm) * VL53L1X_GAIN_NUM +
                                            VL53L1X_GAIN_ROUND) /
                                           VL53L1X_GAIN_DEN);
            }

            // ⑤ 有效性: 状态码必须是 9(唯一"量程有效"), 且距离落在配置窗口内
            if (st != static_cast<uint8_t>(VL53L1X_RangeStatus::Valid)) {
                err = ExceptionCode::RANGE_INVALID;
                noteFailure(err); // 连续失败到 kFailLimit 会把它升级成 RANGE_DATA_STALE
                // 升级了就是真失败(要动健康位), 没升级只是"这一帧量程无效"这个提示性结论
                return (err == ExceptionCode::RANGE_INVALID) ? RangeRead::Invalid : RangeRead::Failed;
            }
            if (mm < cfg::sensor::range::kMinValidMm || mm > cfg::sensor::range::kMaxValidMm) {
                // 0mm 也落在这里(器件"无目标"时返回 0): 0 绝不能当成"贴地"
                err = ExceptionCode::RANGE_INVALID;
                noteFailure(err); // 连续失败到 kFailLimit 会把它升级成 RANGE_DATA_STALE
                // 升级了就是真失败(要动健康位), 没升级只是"这一帧量程无效"这个提示性结论
                return (err == ExceptionCode::RANGE_INVALID) ? RangeRead::Invalid : RangeRead::Failed;
            }

            failCount = 0u; // 成功一帧就清零, 让"偶发无效"不会累积成 DATA_STALE
            rangeM    = static_cast<float>(mm) / 1000.0f;
            return RangeRead::NewData;
        }

        // ---------------- 只读状态(给健康判定/日志用) ----------------

        uint8_t getModelId() const { return modelId; }
        uint8_t getModuleType() const { return moduleType; }
        uint8_t getRevisionId() const { return revisionId; }

        /** 掩模版本是否等于典型值 0x10(不等不代表坏, 只给 diag 参考) */
        bool revisionMatches() const { return revisionId == VL53L1X_REVISION_ID; }

        uint8_t  getFailCount() const { return failCount; }        ///< 连续失败次数(成功即清零)
        uint16_t getLastRawMm() const { return lastRawMm; }        ///< 最近一帧器件原始 mm(未修正)
        uint16_t getLastPeakSignal() const { return lastPeakSignal; } ///< 0x0098 峰值回波率
        uint16_t getLastAmbient() const { return lastAmbient; }    ///< 0x0090 环境光计数率
        bool     isActiveLow() const { return intActiveLow; }      ///< 当前中断极性是否为低有效

        /**
         * @brief 连续测距间隔寄存器的当前实际值(osc × ms × 1.075), 只给日志/自检用
         */
        uint32_t getInterMeasTicks() const { return interMeasTicks(); }
    };
} // namespace dlx

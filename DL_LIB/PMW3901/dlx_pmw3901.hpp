#pragma once
#include <stdint.h>
#include "dlx_spi.hpp"
#include "dlx_gpio.hpp"
#include "dlx_delay.hpp"
#include "dlx_exception.h"
#include "dlx_pmw3901_config.h"

/**
 * @file dlx_pmw3901.hpp
 * @brief PMW3901MB-TXQT 光流传感器驱动(SPI3, 软件片选 PA4)
 *
 * ── 职责(只做这些) ────────────────────────────────────────────────────────
 *  1. 把芯片从"上电未知状态"带到"能出数据": 片选复位脉冲 → 0x3A←0x5A 上电复位 →
 *     等 tMOT-RST → ID 校验 → 无条件读一次 0x02~0x06 → 顺序写厂家初始化表(76 条,
 *     中间一次 10ms 停顿) → 回读校验;
 *  2. 每次调用 readMotion() 做**一次原子锁存**的 12 字节突发读, 输出**原始**
 *     delta_x / delta_y(counts)与 squal;
 *  3. 只报"通信/质量"层面的问题(FLOW_* 错误码), 不做任何姿态解耦、尺度换算或
 *     速度积分 —— 那是 OpticalFlowEstimator 的事(别的批次), 这里混进来会让
 *     "驱动对不对"和"标定对不对"两件事再也分不开。
 *
 * ── 寄存器约定(整个文件最容易翻车的一条) ─────────────────────────────────
 *  地址字节 **bit7 = 1 表示写, bit7 = 0 表示读** —— 与 PMW3360 / ADNS-9800
 *  **完全相反**(那两家的习惯是 bit7=1 读)。写反了的表现极其阴险:
 *  器件把"读"当成对某个只读寄存器的"写"直接忽略、MISO 始终不被驱动, 于是
 *  **读回全 0, 而且不报任何错**。这就是 FLOW_INVERSE_PRODUCT_ID(0x0602)存在的
 *  理由 —— init() 读 Product_ID(0x00, 期望 0x49)失败时必须报它, 而不是含糊的
 *  "没找到", 否则现场只能靠猜。
 *  对应的两个静态函数 writeAddrByte()/readAddrByte() 是这条约定的**唯一出处**,
 *  改约定只改它们; 单测直接断言它们的输出(见 test/pmw3901_check)。
 *
 * ── 与 BMI088 驱动的三处不同(照抄 BMI088 会踩) ───────────────────────────
 *  1. 片选方向: BMI088 是 bit7=1 读 / bit7=0 写, PMW3901 反过来;
 *  2. 读事务: BMI088 是"地址 + dummy + 数据"背靠背连续时钟, PMW3901 **必须在
 *     地址字节与数据字节之间插入 ≥35µs(tSRAD)的真实时间间隙**, 因此一次寄存器读
 *     是**两次独立的 SPI 传输**, 中间保持 CS 拉低 —— STM32 的 SPI 外设无法在字节
 *     之间插空隙, 所以这里只能 "CS 低 → swap(地址) → 等待 → swap(空字节, 收到的是
 *     数据) → CS 高"。**不能**用"发地址紧接着发数据"的一串: 那样时钟会在器件还没
 *     开始驱动 MISO 时就把 8 个位移进来, 读到的是 0x00/0xFF;
 *  3. 片选归属: 本类自己翻转 CS(构造时传入的 GPIO), **不用** spi.select()/
 *     deselect() —— 这条 SPI 是用"无片选"工厂 SPI3_SCA_MICB_MOCC() 建的,
 *     select()/deselect() 在那条总线上是空操作。典型接法:
 *     @code
 *       auto spi = SPI::SPI3_SCA_MICB_MOCC();      // 无片选: PA4 由驱动自己管
 *       spi.init(cfg::hw::flow::kSpi);             // 模式 3, APB1/32 = 1.3125MHz
 *       dlx::PMW3901 flow(spi, cfg::hw::flow::kCs);
 *       ExceptionCode err;
 *       flow.init(err);
 *     @endcode
 *
 * ── 需要的时序(手册 AC 表, 全部由本驱动落实) ─────────────────────────────
 *  | 符号 | 最小值 | 出现在哪 | 本驱动做法 |
 *  |---|---|---|---|
 *  | tNCS-SCLK | 120ns | CS 拉低 → 第一个时钟 | CS 拉低后等 kAddrToDataUs(50µs) |
 *  | tSRAD | **35µs** | 地址字节最后一个时钟 → 数据字节第一个时钟(读) | 两次 swap() 之间等 kAddrToDataUs |
 *  | tSWW/tSWR | 45µs | 一次写的末字节 → 下一次访问 | 写完后 CS 保持低 kWriteTailUs, 拉高后再等 kWriteTailUs |
 *  | tSCLK-NCS(写) | **35µs** | 写数据末字节 → CS 拉高 | CS 拉高之前等 kWriteTailUs |
 *  | tBEXIT | 500ns | burst 后 CS 拉高 → 下次访问 | 同上的 50µs 顺带覆盖 |
 *
 *  为什么用"空转延时"而不是 rt_thread_mdelay(): 这里要的是几十微秒级的**真间隙**,
 *  RTOS 的 tick 是毫秒级(10µs 的间隙根本表达不出来), 而且 init() 可能在调度器
 *  起来之前、或在中断上下文里被调用。默认钩子走 dlx_delay.hpp 的 SysTick 空转
 *  (不修改 SysTick 配置, 可与 RT-Thread 共存), 单测则换成假时钟钩子。
 *
 * ── Motion_Burst(0x16) 12 字节字节序 ─────────────────────────────────────
 *  | 下标 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
 *  |---|---|---|---|---|---|---|---|---|---|---|---|---|
 *  | 字段 | motion | observation | dX_L | dX_H | dY_L | dY_H | squal | raw_sum | max_raw | min_raw | shutter_H | shutter_L |
 *
 *  delta 是 16 位二进制补码、**低字节在前**; shutter 却是**高字节在前** —— 同一个
 *  突发里两种字节序, 见 dlx_pmw3901_config.h 的 PMW3901_BurstIndex。
 *  连读这 12 字节期间**不能**插任何间隙(手册要求连续时钟), 地址字节之前/之后才需要。
 */
namespace dlx
{
    /**
     * @brief 微秒/毫秒延时钩子(可注入)
     *
     * 上板默认用 dlx_delay 的 SysTick 空转; 主机单测注入一个"假时钟 + 记录表"的实现,
     * 从而能断言"地址字节与数据字节之间真的隔了 ≥35µs" —— 这是本驱动唯一无法靠肉眼
     * 或代码走查保证的事情, 必须能被测到。
     */
    struct PMW3901DelayHook
    {
        void (*us)(uint32_t); ///< 微秒级延时(几十µs 的间隙用它)
        void (*ms)(uint32_t); ///< 毫秒级延时(复位等待/初始化停顿)

        /** 上板默认钩子: SysTick 空转, 不碰 RTOS 节拍 */
        static PMW3901DelayHook busyWait()
        {
            PMW3901DelayHook h;
            h.us = &delay_us;
            h.ms = &delay_ms;
            return h;
        }
    };

    /**
     * @brief PMW3901 光流传感器驱动
     *
     * @note 非模板、无动态分配、无异常/RTTI: 延时钩子用函数指针表达, 这样
     *       `armclang -fsyntax-only` 能检查到**全部**成员函数体(模板没实例化时不行)。
     */
    class PMW3901
    {
    private:
        SPI &spi;              ///< 只借 SCK/MISO/MOSI, NSS 不参与
        GPIO cs;               ///< 片选(PA4), 由本类翻转
        PMW3901DelayHook hook; ///< 延时钩子

        /* ---- 诊断留档(readMotion() 的"最近一帧") ---- */
        uint8_t productId;      ///< init() 里实际读到的 0x00
        uint8_t inverseId;      ///< init() 里实际读到的 0x5F
        uint8_t lastMotionFlags;///< 突发字节 0(motion/observation 状态位)
        uint8_t lastSqual;      ///< 最近一帧的 squal
        uint16_t lastShutter;   ///< 最近一帧的 shutter(已按高字节在前还原)
        bool verifyOk;          ///< init() 末尾回读校验是否全部符合
        bool ready;             ///< init() 成功后才允许 readMotion()
        uint8_t _verifyRaw[PMW3901_VERIFY_TABLE_SIZE] = {0u}; ///< 回读校验逐条原始值(自检报告用)
        uint8_t _verifyBadCount = 0u;                          ///< 回读不一致的条数(0 = 全对)
        uint8_t _verifyFirstBad = 0xFFu;                       ///< 第一条不一致的下标(0xFF = 无)

        /* ==================== 时序原语(全部经延时钩子, 便于单测) ==================== */

        inline void waitUs(uint32_t us) { hook.us(us); }
        inline void waitMs(uint32_t ms) { hook.ms(ms); }

        /* ==================== SPI 寄存器访问 ==================== */

        /** @brief 开始一次访问: 重申 SPI 模式 + 拉低片选 + 等 tNCS-SCLK(120ns, 这里给足余量) */
        inline void beginTransfer()
        {
            // 为什么这里要重申模式: PMW3901 的片选是自己翻转的普通 GPIO, 不经过 SPI::select(),
            // 所以 SPI 类没法替我们做这件事。而 SPI3 上还挂着别的器件(NRF24L01, 模式 0),
            // 它们各自 init() 时都会改同一个模式寄存器 —— 不重申就会用别人的 CPOL/CPHA 收发,
            // 症状是"读回来全是垃圾但器件不报错"。见 dlx_spi.hpp 的 applyProfile()。
            spi.applyProfile();
            cs = 0;
            waitUs(cfg::hw::flow::kAddrToDataUs);
        }

        /** @brief 结束一次访问: 拉高片选 + 总线静默(tSWW/tSWR/tSRR + tBEXIT≥500ns) */
        inline void endTransfer()
        {
            cs = 1;
            waitUs(cfg::hw::flow::kWriteTailUs);
        }

        /**
         * @brief 写寄存器: 地址字节(bit7=1) → ≥45µs 真间隙 → 数据字节 → 等 tSCLK-NCS → CS 高
         *
         * 为什么末尾要等: 手册 tSCLK-NCS(写) ≥35µs —— 数据字节最后一个时钟之后必须
         * 再保持 CS 低 ≥35µs, 否则器件可能收不到这一字节(初始化序列会**静默**失败,
         * 不报任何错, 表现同样是"初始化后 motion 恒 0")。
         */
        void writeReg(uint8_t reg, uint8_t value)
        {
            beginTransfer();
            spi.swap(writeAddrByte(reg));
            waitUs(cfg::hw::flow::kAddrToDataUs);
            spi.swap(value);
            waitUs(cfg::hw::flow::kWriteTailUs); // tSCLK-NCS(写) ≥35µs: CS 拉高**之前**必须等够
            endTransfer();                       // 拉高之后再静默一段时间(tSWW/tSWR/tSRR)
        }

        /**
         * @brief 读寄存器: 地址字节(bit7=0) → ≥35µs 真间隙 → 一个空写字节(它的**返回值**就是数据)
         *
         * 这里的两次 swap() 是**两次独立的 SPI 传输**, 中间 CS 一直保持低 —— 中间那次
         * 等待就是手册的 tSRAD ≥35µs。用 dummy 字节"顶"过去是不行的: 那 8 个位在器件
         * 开始驱动 MISO 之前就被采样了, 读回来是 0x00/0xFF。
         */
        uint8_t readReg(uint8_t reg)
        {
            beginTransfer();
            spi.swap(readAddrByte(reg));
            waitUs(cfg::hw::flow::kAddrToDataUs);
            const uint8_t value = spi.swap(static_cast<uint8_t>(0x00));
            // 读的 tSCLK-NCS 只要 120ns, 但这一点余量在 GPIO 翻转只有几十 ns 的余量下不可靠;
            // 复用写的 50µs 值(手册/各驱动对读也建议 ≥50µs), 对 100Hz 轮询的开销可忽略。
            waitUs(cfg::hw::flow::kWriteTailUs);
            endTransfer();
            return value;
        }

        /**
         * @brief 从 Motion_Burst(0x16) 连读 n 字节: 一次原子锁存, 字节之间**无**间隙
         * @param out 输出缓冲(至少 n 字节)
         * @param n   字节数(正常为 12)
         */
        void burstRead(uint8_t *out, uint16_t n)
        {
            beginTransfer();
            spi.swap(readAddrByte(static_cast<uint8_t>(PMW3901_Reg::MotionBurst)));
            waitUs(cfg::hw::flow::kAddrToDataUs); // 地址→突发数据之间的那次间隙
            for (uint16_t i = 0; i < n; ++i) {
                // 连读期间不加任何延时: 手册要求连续时钟, 中间插空隙会破坏突发
                out[i] = spi.swap(static_cast<uint8_t>(0x00));
            }
            waitUs(cfg::hw::flow::kWriteTailUs);
            endTransfer();
        }

        /** @brief 回读校验(选 bank → 读 → 比对); raw 可为空, 非空时回填原始读值 */
        bool verifyInitRegisters(uint8_t *raw)
        {
            bool ok = true;
            int currentBank = -1;
            _verifyBadCount  = 0u;
            _verifyFirstBad  = 0xFFu;
            for (int i = 0; i < PMW3901_VERIFY_TABLE_SIZE; ++i) {
                const PMW3901_VerifyStep &step = PMW3901_VERIFY_TABLE[i];
                if (static_cast<int>(step.bank) != currentBank) {
                    writeReg(static_cast<uint8_t>(PMW3901_Reg::BankSelect), step.bank);
                    currentBank = static_cast<int>(step.bank);
                }
                const uint8_t v = readReg(step.reg);
                if (raw != 0) {
                    raw[i] = v;
                }
                _verifyRaw[i] = v;
                if (v != step.val) {
                    ok = false;
                    /**
                     * 记住**第一条不一致**与总条数。
                     *
                     * 为什么值得留这几个字节(2026-10-03): 上板第一次跑通 ID 之后卡在
                     * `FLOW_SPI_GARBAGE` —— 而那个码只说"回读全 0/全 FF", 不说是**哪几条**不对。
                     * 三种情况的修法完全不同:
                     *   全部 0x00/0xFF      ⇒ SPI 根本没读到东西(供电/片选/时序);
                     *   少数几条不一致      ⇒ 表里那几个期望值抄错了(或者是别的模块批次);
                     *   带 bank 的都不对    ⇒ bank 切换没生效(写 0x7F 的时序/值不对)。
                     * 没有明细就只能猜, 而猜错的代价是重烧一次。
                     */
                    if (_verifyBadCount == 0u) {
                        _verifyFirstBad = static_cast<uint8_t>(i);
                    }
                    ++_verifyBadCount;
                }
            }
            return ok;
        }

    public:
        /**
         * @param interface 已 init 好的 SPI(模式 3, ≤2MHz)。**不要**用带软件片选的工厂:
         *                  PMW3901 的 CS 是本类自己翻转的普通 GPIO, 见文件级说明。
         * @param csPin     片选脚, 一般传 cfg::hw::flow::kCs(PA4)
         * @param delayHook 延时钩子, 默认 SysTick 空转; 主机单测注入假时钟
         */
        PMW3901(SPI &interface, GPIOProfile csPin, PMW3901DelayHook delayHook = PMW3901DelayHook::busyWait())
            : spi(interface),
              cs(csPin),
              hook(delayHook),
              productId(0x00),
              inverseId(0x00),
              lastMotionFlags(0x00),
              lastSqual(0x00),
              lastShutter(0x0000),
              verifyOk(false),
              ready(false)
        {
        }

        ~PMW3901()
        {
        }

        /* ==================== 初始化 ==================== */

        /**
         * @brief 上电复位 + ID 校验 + 厂家初始化表 + 回读校验
         *
         * 顺序(依据 docs/_research/pmw3901.md §5 Step 0/Step 1 与
         * docs/2026-10-02_upgrade_plan_v3.md §4.3):
         *   ① CS 高→低→高 复位 SPI 口;
         *   ② 写 0x3A←0x5A, 等 kResetWaitMs(≥50ms, 手册 tMOT-RST);
         *   ③ 读 0x00(须 0x49)与 0x5F(须 0xB6), 失败整体重试 PMW3901_ID_RETRY 次;
         *   ④ 无条件读一次 0x02~0x06(把运动流水线点起来, 与 MOTION 脚无关);
         *   ⑤ **顺序**写 76 条厂家表(A 段 59 条 → 停顿 kInitPauseMs → B 段 14 条 → C 段 3 条);
         *   ⑥ 回读关键寄存器。
         *
         * 表**不能逆序**: 表里 {0x7F, bank} 是银行选择, 它决定后面那些写落在哪个 bank,
         * 逆序会让每条写都进错 bank 而器件一声不吭。详见 dlx_pmw3901_config.h。
         *
         * @param err 出参: 只在返回 false 时有意义 —— FLOW_INVERSE_PRODUCT_ID(ID 不对, 头号原因是 bit7
         *            读写方向搞反) / FLOW_SPI_GARBAGE(读回全 0/全 FF, 或初始化写没落地)。
         *            成功时**不写**(异常码里没有"OK", 见 dlx_exception.h)
         * @return true 表示器件已在"能出数据"的状态; false 时 err 给出原因
         */
        bool init(ExceptionCode &err)
        {
            cs.init(GPIOModeProfile::OUT_PP_UP_50MHz);
            cs = 1;
            ready = false;
            verifyOk = false;

            /* ---------- ① 片选复位脉冲 ---------- */
            // 目的: 让器件看到一次完整的 NCS 跳变(把它的 SPI 状态机复位到 Idle)。
            // 每段 2ms 是 Bitcraze 的做法; 手册只要求 tNCS-SCLK ≥120ns, 这里纯余量,
            // 但上电阶段 VCSEL 与 1.8V 轨还在爬(手册 tRT 最长 20ms), 不值得省。
            cs = 0;
            waitMs(2);
            cs = 1;
            waitMs(2);

            /* ---------- ② + ③ 上电复位与 ID 校验(可重试) ---------- */
            uint8_t pid = 0x00;
            uint8_t inv = 0x00;
            bool idsOk = false;
            for (int attempt = 0; (attempt < PMW3901_ID_RETRY) && !idsOk; ++attempt) {
                writeReg(static_cast<uint8_t>(PMW3901_Reg::PowerUpReset), PMW3901_POWER_UP_RESET_VAL);
                // 手册 tMOT-RST = 50ms: 复位后到此为止器件的运动数据无效。
                // 不等够的话后面所有寄存器写都落在复位窗口里 —— 又是"motion 恒 0"。
                waitMs(cfg::hw::flow::kResetWaitMs);
                pid = readReg(static_cast<uint8_t>(PMW3901_Reg::ProductId));
                inv = readReg(static_cast<uint8_t>(PMW3901_Reg::InverseProductId));
                idsOk = (pid == PMW3901_PRODUCT_ID) && (inv == PMW3901_INVERSE_PRODUCT_ID);
            }
            productId = pid;
            inverseId = inv;
            if (!idsOk) {
                // 必须报这个特异错误码, 不要报 FLOW_NOT_FOUND: 0x00 读不到 0x49 的头号
                // 原因是 bit7 读写方向搞反(与 PMW3360 相反), 现场表现是"读回全 0 且完全
                // 不报错"。把判断留在错误码里, 上位机/日志一眼能定位到那一行。
                err = ExceptionCode::FLOW_INVERSE_PRODUCT_ID;
                return false;
            }

            /* ---------- ④ 无条件读一次 0x02~0x06 ---------- */
            // 三份驱动与官方文档都写明这一步是**必须**的: 它锁存/启动运动流水线, 与
            // MOTION 引脚状态无关(本设计没接 MOTION 脚, 只轮询)。省掉它, 之后读到的
            // delta 可能整帧不动。
            for (uint8_t r = static_cast<uint8_t>(PMW3901_Reg::Motion);
                 r <= static_cast<uint8_t>(PMW3901_Reg::DeltaYH); ++r) {
                (void)readReg(r);
            }
            waitMs(1);

            /* ---------- ⑤ 厂家初始化表(顺序执行) ---------- */
            for (int i = 0; i < PMW3901_INIT_TABLE_SIZE; ++i) {
                const PMW3901_InitStep &step = PMW3901_INIT_TABLE[i];
                writeReg(step.reg, step.val);
                if (step.pause != 0) {
                    // 全表唯一一处停顿(第 59 条之后)。时长取配置, 位置由表决定。
                    // 为什么只在"表说的那一处"停、而不是每条之间都停: 手册/各驱动只在
                    // 这里要求等待, 其余条目靠 tSWW(≥45µs, 已由 writeReg 保证)即可;
                    // 每条都停 10ms 会让上电初始化多花 0.7s 且没有任何依据。
                    waitMs(cfg::hw::flow::kInitPauseMs);
                }
            }

            /* ---------- ⑥ 回读校验: 这批写到底落进去了没有 ---------- */
            uint8_t raw[PMW3901_VERIFY_TABLE_SIZE];
            verifyOk = verifyInitRegisters(raw);
            bool allZero = true;
            bool allFF = true;
            for (int i = 0; i < PMW3901_VERIFY_TABLE_SIZE; ++i) {
                if (raw[i] != 0x00) {
                    allZero = false;
                }
                if (raw[i] != 0xFF) {
                    allFF = false;
                }
            }
            if (allZero || allFF) {
                // 全 0/全 FF = MISO 根本没人驱动: 供电、片选或时序余量不足。
                // 单独报 FLOW_SPI_GARBAGE, 与"ID 方向搞反"区分开。
                err = ExceptionCode::FLOW_SPI_GARBAGE;
                return false;
            }
            if (!verifyOk && PMW3901_VERIFY_STRICT) {
                // 非退化的不符 = 部分寄存器没写进去(时序余量不足的典型表现)。
                // 这里**故意**判失败: 半配置的器件会安静地喂错速度, 比"没有光流"危险。
                // 若上板实测发现这几个厂家保留寄存器不可回读, 把 PMW3901_VERIFY_STRICT
                // 置 false(见 config.h 的说明), 而不是放宽整条时序。
                err = ExceptionCode::FLOW_SPI_GARBAGE;
                return false;
            }

            ready = true;
            return true;
        }

        /* ==================== 取数 ==================== */

        /**
         * @brief 读一帧原始运动数据(一次原子锁存的 12 字节突发读)
         *
         * @param dx    出参: 水平位移, 单位 counts, **已按 cfg::sensor::flow::kSignX 施加符号**
         * @param dy    出参: 垂直位移, 单位 counts, **已按 cfg::sensor::flow::kSignY 施加符号**
         * @param squal 出参: 表面质量(0 = 完全无特征; 越大越可信)
         * @param err   出参: 只在返回 false, 或"读到了但质量低"(返回 true)时被写 ——
         *              FLOW_LOW_QUALITY(提示性, 数据仍然有效) /
         *              FLOW_SPI_GARBAGE(读回全 0/全 FF, 数据无效) / FLOW_NOT_FOUND(未 init)
         *
         * @return true = 本次通信正常(即使 err 是 FLOW_LOW_QUALITY, dx/dy/squal 依然可用);
         *         false = 本帧作废(dx/dy 已清零), 调用方应丢弃并计数
         *
         * 质量约定(刻意与"数据是否给出"解耦):
         *  - squal < kMinSqual → err = FLOW_LOW_QUALITY 但**照给数据**: 悬停在地毯/瓷砖上
         *    会持续低质量, 该不该用是上层(带高度的估计器)的事, 驱动无权替它丢;
         *  - |delta| ≥ kSatDelta(2047) → 已饱和, 线性度没了。驱动**只提供判据**
         *    (isSaturated()), 不在这里丢样本;
         *  - |delta| > kMaxDelta(240) → 典型 SPI 垃圾(PX4 判据)。注意增量在器件里是
         *    累积的, 轮询慢到 5Hz 时正常运动也可能超过 240 —— 这个门限只有和"轮询间隔"
     *    一起看才有意义, 所以判据同样交给上层(isImplausible())。
         */
        bool readMotion(int16_t &dx, int16_t &dy, uint8_t &squal, ExceptionCode &err)
        {
            dx = 0;
            dy = 0;
            squal = 0;
            if (!ready) {
                // 没成功 init() 就来取数: 明确报错, 而不是返回一帧看起来像"静止"的 0
                err = ExceptionCode::FLOW_NOT_FOUND;
                return false;
            }

            uint8_t b[PMW3901_BURST_BYTES];
            burstRead(b, static_cast<uint16_t>(PMW3901_BURST_BYTES));

            bool allZero = true;
            bool allFF = true;
            for (int i = 0; i < PMW3901_BURST_BYTES; ++i) {
                if (b[i] != 0x00) {
                    allZero = false;
                }
                if (b[i] != 0xFF) {
                    allFF = false;
                }
            }
            if (allZero || allFF) {
                // 全 0/全 FF: 供电余量不足、SPI 时钟太快、或某次 CS 时序被打断时的典型
                // 面孔。单独报 FLOW_SPI_GARBAGE, 让上层能把它和 FLOW_LOW_QUALITY(真实
                // 的弱纹理)分开统计 —— 两者的处置完全不同。
                lastMotionFlags = 0x00;
                lastSqual = 0x00;
                lastShutter = 0x0000;
                err = ExceptionCode::FLOW_SPI_GARBAGE;
                return false;
            }

            const int16_t rawDx = decodeDelta(b[PMW3901_BURST_DELTA_X_L], b[PMW3901_BURST_DELTA_X_H]);
            const int16_t rawDy = decodeDelta(b[PMW3901_BURST_DELTA_Y_L], b[PMW3901_BURST_DELTA_Y_H]);
            dx = applySign(rawDx, cfg::sensor::flow::kSignX);
            dy = applySign(rawDy, cfg::sensor::flow::kSignY);

            squal = b[PMW3901_BURST_SQUAL];
            lastMotionFlags = b[PMW3901_BURST_MOTION];
            lastSqual = squal;
            lastShutter = static_cast<uint16_t>(
                (static_cast<uint16_t>(b[PMW3901_BURST_SHUTTER_UPPER]) << 8) | b[PMW3901_BURST_SHUTTER_LOWER]);

            if (squal < cfg::sensor::flow::kMinSqual) {
                err = ExceptionCode::FLOW_LOW_QUALITY; // 提示性: 数据照给, 该不该用是上层的事
            }
            // 质量合格时**不写 err**: 成功路径不写码(异常码里没有"OK", 见 dlx_exception.h)。
            // 调用方只在返回 false 时才该读它 —— 成功那一支里它会留着上一次的值。
            return true;
        }

        /* ==================== 解码 / 判据(纯函数, 便于单测) ==================== */

        /**
         * @brief 线上地址字节: 读 = bit7 清 0
         * @note 与 PMW3360/ADNS-9800 **相反**。这是全驱动唯一拼地址字节的地方,
         *       单测直接断言它(改约定只需改这两个函数)。
         */
        static inline uint8_t readAddrByte(uint8_t reg) { return static_cast<uint8_t>(reg & 0x7F); }

        /** @brief 线上地址字节: 写 = bit7 置 1 */
        static inline uint8_t writeAddrByte(uint8_t reg) { return static_cast<uint8_t>(reg | 0x80); }

        /**
         * @brief 突发里的 delta 解码: **16 位二进制补码, 低字节在前**
         * @note 不是 12 位! PMW3360/ADNS-9800 才是 12 位补码塞在同一个位置;
         *       若发现 bit15..12 总是复制 bit11, 说明手上的器件是老格式(本驱动按
         *       手册的"16-bit motion data"实现, 见 docs §4)。
         */
        static inline int16_t decodeDelta(uint8_t low, uint8_t high)
        {
            return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low));
        }

        /**
         * @brief 施加坐标符号(kSignX/kSignY 就是给"装反了/镜像"用的, 别去改标定)
         * @note value = -32768 取反后超出 int16 范围, 按位回绕仍是 -32768 —— 这个角落
         *       不可能出现在真实数据里(饱和值 2047), 这里只保证不做未定义行为。
         */
        static inline int16_t applySign(int16_t value, int8_t sign)
        {
            return (sign < 0) ? static_cast<int16_t>(0 - static_cast<uint16_t>(value)) : value;
        }

        /** @brief int16 的绝对值(提升到 int32, -32768 也安全) */
        static inline int32_t abs16(int16_t v) { return (v < 0) ? -static_cast<int32_t>(v) : static_cast<int32_t>(v); }

        /**
         * @brief |delta| 是否已达饱和门限(cfg::sensor::flow::kSatDelta)
         * @note 饱和样本没有线性度, 谁用谁负责丢; 驱动只给判据, 不替上层做决定。
         */
        static inline bool isSaturated(int16_t dx, int16_t dy)
        {
            const int32_t lim = static_cast<int32_t>(cfg::sensor::flow::kSatDelta);
            return (abs16(dx) >= lim) || (abs16(dy) >= lim);
        }

        /**
         * @brief |delta| 是否超过"单帧物理不可能"门限(cfg::sensor::flow::kMaxDelta)
         * @note 该门限只在**已知轮询间隔**时才有意义(增量在器件内累积), 所以判据放这里,
         *       由 flow 线程按自己的周期决定用不用。
         */
        static inline bool isImplausible(int16_t dx, int16_t dy)
        {
            const int32_t lim = static_cast<int32_t>(cfg::sensor::flow::kMaxDelta);
            return (abs16(dx) > lim) || (abs16(dy) > lim);
        }

        /* ==================== 诊断(只读, 不改配置) ==================== */

        /**
         * @brief 现场重读两个 ID(不复位、不改配置), 给"地面串口调试模式"用
         * @param pid   0x00 的当前值(期望 0x49)
         * @param invId 0x5F 的当前值(期望 0xB6)
         * @note init() 只回一个 bool/错误码, 分不清是 ID 方向错、器件不应答还是初始化
         *       没落地 —— 把原始值读出来就有方向了(全 0x00 = bit7 方向/MISO 没驱动,
         *       0xFF = 器件没上电或片选没接上)。
         */
        void readIds(uint8_t &pid, uint8_t &invId)
        {
            pid = readReg(static_cast<uint8_t>(PMW3901_Reg::ProductId));
            invId = readReg(static_cast<uint8_t>(PMW3901_Reg::InverseProductId));
        }

        /** @brief init() 里读到的 0x00(诊断留档) */
        uint8_t productIdReg() const { return productId; }
        /** @brief init() 里读到的 0x5F(诊断留档) */
        uint8_t inverseProductIdReg() const { return inverseId; }
        /** @brief init() 末尾回读校验是否全部符合(即使 init() 返回 true, 也建议记一次) */
        bool initVerifyOk() const { return verifyOk; }
        /** 回读校验: 不一致的条数(0 = 全对)与第一条的下标(0xFF = 无) */
        uint8_t verifyBadCount() const { return _verifyBadCount; }
        uint8_t verifyFirstBadIndex() const { return _verifyFirstBad; }
        /** 第 i 条回读到的原始值(自检报告用; i 越界返回 0) */
        uint8_t verifyRawAt(uint8_t i) const
        {
            return (i < (uint8_t)PMW3901_VERIFY_TABLE_SIZE) ? _verifyRaw[i] : 0u;
        }
        /** @brief 是否已成功 init() */
        bool isReady() const { return ready; }
        /** @brief 最近一帧的 shutter(高字节在前已还原; 低光下会显著变大 → 运动模糊) */
        uint16_t shutter() const { return lastShutter; }
        /** @brief 最近一帧的突发字节 0 原值(bit7 = motion 发生) */
        uint8_t motionFlags() const { return lastMotionFlags; }
        /** @brief 最近一帧器件是否报告"有运动"(突发字节 0 的 bit7) */
        bool motionDetected() const { return (lastMotionFlags & 0x80) != 0x00; }
        /** @brief 最近一帧的 squal */
        uint8_t squal() const { return lastSqual; }
    };

} // namespace dlx

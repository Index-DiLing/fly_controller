//===================================================================================================
// test/bench/baro_bench_main.cpp —— 气压计偶发 0x404(BARO_READ_FAILED) 定位台架固件
//
// 这不是主线固件, 也不参与主线构建(EIDE 的 srcDirs = app/DL_LIB/rt-thread, 不含 test/)。
// 由 tools/bench/build_baro_bench.ps1 单独编译: 复用 build/FLY/compile_commands.json 里的 armclang
// 参数, 用本文件的 main 替换 release.cpp 的 main 参加链接, 产物 build/bench_baro/dl_fly_baro_bench.*。
//
// 目的(docs/气压计0x404定位_台架.md §5): `0x404` 不等于"I2C 读失败" —— 它至少有四个来源, 其中两个
// 与 I2C 事务无关。本台架把每次失败按**驱动状态判**出来, 而不是靠猜:
//   kind=tx    : baro.read() 的突发读事务本身失败(evTimeouts 有增量 / failCount>0) —— 来源 A/C
//   kind=comp  : 突发读其实成功了, 但补偿结果非有限或 pressure<=0(dlx_bmp388.hpp:513-515) —— 来源 B
//   kind=other : err 不是 0x404(典型: 器件未初始化 ⇒ 0x0403)。它**不参与** tx/comp 统计 ——
//                否则"忘了 init"会被误判成 comp, 得出与真结论相反的签名(见 failureKindName 的注释)。
// 两者都报同一个 0x404, 所以必须分开统计; 判据见 runAttempt() 里的 errCode + txKind。
//
// 上电会自己做**一次** tryInit 并打 `[boot] baro init=<OK|FAIL> err=0x....`(纵深防御: 脚本忘了发 `init`
// 时也不会静默跑在未初始化状态); `soak`/`inject` 在器件未初始化时直接回 `[err] not-initialized`。
//
// 硬件(引脚全部取自 cfg::hw::*, 不在此写死; 依据见 DL_LIB/generated/flight_config_gen.hpp):
//   USART1 PA9/PA10  控制台文本命令, 115200 8N1(台架自定, 主线是 2Mbaud; 只影响打印, 不影响测量)
//   I2C1   PB6/PB7   BMP388(0x76, 400kHz) —— 被测对象; SR1/SR2 与 PB6/PB7 电平随每次失败快照
//   I2C2   PF1/PF0   VL53L1X(0x29, XSHUT=PD2) —— 只在 stress 里当并发负载
//   SPI1   PA5/6/7   BMI088(CS=PB12/PB13)      —— 只在 stress 里当并发负载
//   ADC1   PA0/PA1   电池电压/电流             —— 只在 stress 里当并发负载
//   TIM7             只在 irqblock 里当"高优先级 ISR": 在更新中断里忙等 us 微秒, 打断进行中的事务
//
// 三条必须知道的口径(否则数字会被误解):
//   1. **失败瞬间的快照必须在任何 recover/下一次事务之前拍**。IICDevice::read()/write() 在下一次
//      调用时会先 `if (bus.hasError()) bus.recover();`(dlx_iic.hpp:352 / :331), 而那 9 个 SCL 脉冲
//      会把 AF 与总线状态破坏掉。所以顺序是: read() 返回 -> 立刻读 SR1/SR2 + PB6/PB7 电平 +
//      lastFailedEvent/evTimeouts 增量 -> 之后才允许做任何别的事。
//      r2 起这两个电平会**打进 [fail] 行尾的 `scl=`/`sda=`**(A21): H4("总线被从机拉住")的唯一直接
//      证据就是"失败那一刻 scl/sda 是不是 0"; 此前只能在别的时刻敲 `bus`, 所以 H4 没有自动化证据。
//   2. **[fail] 行的 status/raw 只有 kind=comp 时是真数据**。comp 意味着突发读成功过, 但驱动不导出
//      那个缓冲(dlx_bmp388.hpp:327 是局部数组), 所以台架在快照之后补一次**诊断性**突发读, 取
//      "失败之后那一刻器件寄存器里的现场"; kind=tx 时驱动已经把缓冲清零(dlx_bmp388.hpp:173-177),
//      固定打 0x00 / 000000000000 —— tx 的诊断靠 sr1/sr2/lastEv/durUs, 不靠 raw。
//      (kind=tx 为什么**不**补诊断读: 失败后 busError 是置位的, 再发事务必须先 clearError, 那会绕过
//       下一次尝试里的自动 recover(), 把 H1/H4 要观测的恢复行为改掉。宁可少一个字段, 不改被测对象。)
//   3. **本台架不启动 RT-Thread, delay_ms 在 SysTick 未使能时会走 NOP 近似分支**(dlx_delay.hpp:19-26)。
//      该分支约 4 周期/次循环, 所以延时量级仍然对(BMP388 软复位后等 2ms 足够, init 重试间隔 100ms
//      也能对上); 但所有**微秒级测量**一律走 DWT, 不依赖 SysTick。另外: 台架是裸机 + 阻塞式命令,
//      没有复现"sns_baro(22) 空转 → link_tel(24) 饿死"这条 RT-Thread 抢占链(见该文档 §10)。
//   4. **DWT 时基有硬回绕点: 2^32 / 168MHz = 25.566s**(审计文档 :70 点过这一类历史前科)。
//      所有"单次差值"用法都在调用点写明了上界(必须 ≪ 25.566s); 唯一跨回绕点的**绝对经过时间**
//      —— soak 的 t —— 改成逐轮累加(uint64), 见 cmdSoak 里的 elapsedCyc 推导。
//      r2 及以前 soak 用单次差值: `soak <ms>` 只要 ms > 25566 就**永远不自己结束**(D4, 上板实测)。
//
// 命令(USART1 输入, 大小写不敏感, 行式, 提示符 `ok>`), 契约冻结在 docs/气压计0x404定位_台架.md §5:
//   help | ver | cfg | sr | bus
//   init [hz]                           驱动的分阶段 init(按 cfg::sensor::baro::kInitRetry 重试)
//   soak <ms> [period_ms]               持续读; 每秒一行 [soak], 结束一行 [soak.end]; 敲任意键中止
//                                       ms ∈ [1, 3600000(1h)], period ∈ [1, 1000]ms(r3 起长 soak 才真的可用)
//   inject <n>                          对不存在的地址(0x33)写 => 必然 NACK => 记录恢复要几次尝试
//   speed <hz>                          重启 I2C1 + 重新 init 器件(H2: 100k/400k 对比)
//   stress <on|off>                     SPI1 BMI088 读 + I2C2 VL53L1X 读 + ADC 采样(H6: 并发负载)
//   irqblock <us> <period_ms> <on|off>  TIM7 更新中断里忙等 us µs(H5/H7: 事务中被 ISR 打断)
//                                       约束: us < period_ms×1000, period_ms ≤ 65(16 位 ARR 的硬上限);
//                                       且 TIM7 计数时钟必须整除 1MHz, 否则不启动(时间不精确就不假装)
//   recover | stat | reset
//===================================================================================================
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "BMP388/dlx_bmp388.hpp"
#include "dlx_adc.hpp"
#include "dlx_bytebuffer.hpp"
#include "dlx_delay.hpp"
#include "dlx_gpio.hpp"
#include "dlx_iic.hpp"
#include "dlx_nvic_it.h"
#include "dlx_spi.hpp"
#include "dlx_timer.hpp"
#include "dlx_usart.hpp"
#include "flight_config.hpp"   // cfg::hw::* / cfg::sensor::baro::* / cfg::power::*
#include "hardware_claims.hpp" // claims::kAdcVoltagePin / kAdcCurrentPin

using namespace dlx;

namespace
{
    //-------------------------------------------------------------------------------------------
    // 一、常量
    //-------------------------------------------------------------------------------------------
    /**
     * bench 固件版本 —— banner 第一行, `ver` 也打它。自动化脚本靠它判断"板上是不是最新 bench"。
     * 每次改 bench 行为(口径/命令/输出字段)都要 **+1** 并把日期改掉, 否则脚本会拿旧口径的数据
     * 当新数据解析。
     *
     * r2 (2026-10-18, 相对 r1): `[fail]` 行尾追加 `scl=%u sda=%u`(A21, H4 需要失败瞬间的总线电平);
     * `irqblock ... on` 在 TIM7 计数时钟不是 1MHz 整数倍时拒绝启动(保证"打印值=实际值")。
     * r3 (2026-10-18, 相对 r2): **修 soak 的绝对计时** —— 由"起点对终点一次减(32 位差值)"改成逐轮累加
     * (D4: 旧写法在 DWT 的 25.566s 回绕点归零, `soak <ms>` 只要 ms > 25566 就永远不自己结束)。
     * 输出字段**一个都没变**, 但 r2 的 soak 语义是坏的(长 soak 停不下来) ⇒ 版本门禁只认 r3。
     * ⇒ 解析器**不认识 r1/r2**: 拿旧脚本跑 r3 会缺 `scl/sda` 字段, 拿新脚本跑 r1/r2 会缺值 —— 门禁必须同步。
     */
    constexpr const char *kBenchVersion = "baro bench r3 2026-10-18";

    /** 台架控制台波特率。主线 cfg::hw::console::kBaud = 2Mbaud(日志流), 台架面向普通 USB-TTL, 固定 115200。 */
    constexpr uint32_t kConsoleBaud    = 115200u;
    constexpr uint16_t kConsoleRxBytes = 256u;

    /** I2C 本机地址(主机模式下无实际意义, 与 dlx_bmp388.hpp:12 的用法示例保持一致)。 */
    constexpr uint16_t kI2CSelfAddress = 0x51u;

    /**
     * inject 用的"总线上不存在的地址"。
     * 为什么是 0x33: BMP388 占 0x76/0x77(SDO 决定), VL53L1X 在**另一条**总线(I2C2)且是 0x29 ——
     * 0x33 不会撞到任何一颗, 所以"写它"必然 NACK, 正好用来把 SR1.AF 置起来看恢复行为。
     */
    constexpr uint16_t kGhostAddress = 0x33u;

    /** inject: 注入后最多再试几次 read() 判定"恢复了"; 超过就报 attempts_to_ok=-1。 */
    constexpr uint32_t kInjectMaxAttempts = 10u;

    // 参数合法范围(挡住手打错误; 超范围一律 [err] bad-arg)
    /**
     * soak 时长上限 [ms] = 1h。**r3 起这个上限才真的可用**: r2 及以前用"起点对终点一次减"的 32 位差值,
     * 超过 25.566s 就归零 ⇒ 只有 ≤25s 的 soak 能自己结束(D4)。现在改成逐轮累加(uint64), 1h 也在量程内
     * (推导见 cmdSoak)。再长建议用脚本多次跑, 别让一条命令占住串口。
     */
    constexpr uint32_t kSoakMaxMs         = 3600000u;
    constexpr uint32_t kSoakMaxPeriodMs   = 1000u;
    constexpr uint32_t kSoakDefaultPeriod = 8u;       ///< 契约默认 8ms(比主线 io 线程的 20ms 更密)
    constexpr uint32_t kInjectMaxN        = 100u;
    constexpr uint32_t kSpeedMinHz        = 10000u;
    constexpr uint32_t kSpeedMaxHz        = 1000000u;
    constexpr uint32_t kIrqBlockMaxUs     = 1000000u; ///< us 的**解析**上限; 真正约束它的是下面的关系校验
    /**
     * `irqblock` 的周期上限 [ms] —— **这个上限是硬件的, 不是随手取的**。
     *
     * TIM7 是 16 位定时器: ARR 只有 16 位(RM0090 的 TIM6/TIM7 时基单元; 本仓库没有手册摘录, 但
     * "ARR ≤ 65535" 是寄存器位宽的硬事实)。本台架把 TIM7 的计数时钟配成 1MHz(kIrqTimerTickHz),
     * 于是 `ARR = periodMs×1000 − 1 ≤ 65535` ⇒ **periodMs ≤ 65**。
     *
     * 为什么必须在这里拦住(D3, verifier 2026-10-18 抓到): 上限原来写 10000, 而 ARR 只有 16 位 ⇒
     * 硬件静默截断, 但 `[irqblock]` 行打的还是**请求值**: periodMs=100 实际是 34.5ms、periodMs=66
     * 实际是 0.46ms。实验参数与打印不符, 结论会被误读 —— 与其打一个自己不知道的周期, 不如 `[err] bad-arg`。
     * H4/H5 的"短停顿"实验用 1~10ms 就够, 65ms 的上限不影响任何实验。
     */
    constexpr uint32_t kIrqBlockMaxPeriodMs = 65u;

    // --- stress 用的器件事实(不在这里写寄存器地址的数字以外的东西: 配置全部取 cfg::) ---
    constexpr uint8_t  kImuAccelChipIdReg = 0x00u;   ///< BMI088 加速度计 CHIP_ID 寄存器
    constexpr uint8_t  kImuAccelChipId    = 0x1Eu;   ///< BMI088 加速度计 CHIP_ID 期望值
    constexpr uint8_t  kImuDataStartReg   = 0x12u;   ///< 加速度计数据起址(0x12~0x17, 6 字节)
    constexpr uint8_t  kImuDataBytes      = 6u;
    constexpr uint8_t  kImuStressReads    = 8u;      ///< 每个 stress 步进读几组 IMU 数据(造 SPI 负载)
    constexpr uint16_t kRangeModelIdReg   = 0x010Fu; ///< VL53L1X 模型 ID(16bit 寄存器地址, 大端先行)
    constexpr uint8_t  kRangeModelId      = 0xEAu;
    constexpr uint8_t  kRangeBootDelayMs  = 2u;      ///< XSHUT 释放后给它一个干净起点(驱动内部还会轮询)
    /** stress 步进的最小间隔: 5ms ≈ 200Hz, 与主线"IMU 1kHz / 电源 50Hz"同量级, 足以造成总线与中断竞争。 */
    constexpr uint32_t kStressStepMs = 5u;

    /** irqblock 的 TIM7 计数时钟目标(1MHz ⇒ 1000 计数 = 1ms, 好算)。 */
    constexpr uint32_t kIrqTimerTickHz = 1000000u;

    /** 气压计的 I2C 寄存器组(只这一处出现 I2C1 这个外设名, 便于阅读时确认快照来自哪条总线)。 */
    I2C_TypeDef *const kBaroI2c = I2C1;

    //-------------------------------------------------------------------------------------------
    // 二、时基(DWT) —— 与 link_bench_main.cpp:90-104 同一套写法
    //-------------------------------------------------------------------------------------------
    inline uint32_t cycNow() { return DWT->CYCCNT; }
    inline uint32_t cycPerUs() { return static_cast<uint32_t>(SystemCoreClock / 1000000u); }
    /**
     * 周期差 → 微秒。**只对"单次差值"正确**, 而且有硬上界:
     *   DWT->CYCCNT 是 32 位自由运行计数器, 168MHz 下每 2^32 / 168e6 = **25.566s** 绕回一次。
     *   无符号减法只在 `laterCyc - earlierCyc < 2^32` 时等于真实差值 —— 也就是**起点与终点必须落在
     *   同一个 25.566s 窗口内**。这不是"一般都很小"的宽松说法: 审计文档 :70 点过 25.57s CYCCNT 回绕
     *   这一类历史前科, 而台架自己就踩过一次(\"soak 60000 永远不结束\", 见 cmdSoak 的 D4 说明)。
     * 两类用法, 别混:
     *   ① **单次差值**(一次事务的 durUs、相邻两次尝试的 gapUs、5ms 节拍判断、微秒级忙等):
     *      调用方必须能证明上界 ≪ 25.566s —— 每一处都在调用点写了这个上界;
     *   ② **跨多个回绕点的绝对经过时间**(soak 的 t): 绝不能用"起点对终点一次减出来",
     *      必须逐轮累加(见 cmdSoak 的 elapsedCyc 及其推导)。
     */
    inline uint32_t usBetween(uint32_t laterCyc, uint32_t earlierCyc)
    {
        return static_cast<uint32_t>(laterCyc - earlierCyc) / cycPerUs();
    }
    inline uint32_t usToCyc(uint32_t us) { return us * cycPerUs(); }
    /**
     * 等到某个周期戳(忙等; 台架精度优先, 不让出 CPU)。
     * 判据用**有符号**差值: 等待量 < 2^31 周期(= 12.78s)时正确, 且目标已过去时立刻返回(不会误等 25.57s)。
     * 调用点上界: 目标 = 本轮尝试起点 + periodCyc, 而 soak 的 periodMs ≤ kSoakMaxPeriodMs(1000ms)
     * ⇒ 等待量 ≤ 1s ≪ 12.78s。
     */
    inline void waitUntilCyc(uint32_t targetCyc)
    {
        while (static_cast<int32_t>(cycNow() - targetCyc) < 0)
        {
        }
    }
    /**
     * 忙等 us 微秒。为什么不用 delay_us(): delay_us 走 SysTick, 而 irqblock 要**在 ISR 里**等 ——
     * 那里不能依赖 SysTick 计数是否使能, 也不能被别的中断打断计时(DWT 是自由运行的周期计数)。
     * 单次差值, 上界由调用方保证: `irqblock` 已被 `us < periodMs×1000 且 periodMs ≤ 65` 限制
     * ⇒ us ≤ 64999µs ≈ 65ms ≪ 25.566s。
     */
    inline void busyWaitUs(uint32_t us)
    {
        const uint32_t t0 = cycNow();
        while (usBetween(cycNow(), t0) < us)
        {
        }
    }

    //-------------------------------------------------------------------------------------------
    // 三、全局状态
    //-------------------------------------------------------------------------------------------
    USART          *g_con     = nullptr; ///< USART1 控制台
    RingByteBuffer *g_conRing = nullptr;

    IICBus   *g_bus    = nullptr; ///< I2C1(被测总线)
    BMP388   *g_baro   = nullptr; ///< I2C1 上的 BMP388
    GPIO     *g_sclPin = nullptr; ///< PB6: 只读 IDR 判总线电平, 不重新 init(见 cmdBus)
    GPIO     *g_sdaPin = nullptr; ///< PB7

    /** 当前 I2C1 速率(Hz)。`cfg` 打它; `speed`/`init <hz>` 改它。 */
    uint32_t g_speedHz = cfg::hw::baro::kSpeed;

    // --- stress 用的对象与健康位 ---
    bool           g_stressOn      = false;
    bool           g_stressSpiOk   = false; ///< "自上次 stress on 以来 SPI 是否一直成功"
    bool           g_stressI2c2Ok  = false;
    bool           g_stressAdcOk   = false;
    bool           g_spiReady      = false;
    bool           g_rangeReady    = false;
    bool           g_adcReady      = false;
    uint32_t       g_stressLastCyc = 0u;
    SPI           *g_spiImu    = nullptr;
    GPIO          *g_accelCs   = nullptr;
    GPIO          *g_xshut     = nullptr;
    IICBus        *g_rangeBus  = nullptr;
    IICDevice     *g_rangeDev  = nullptr;
    ADC1Sampler   *g_adc       = nullptr;

    // --- irqblock ---
    bool     g_irqBlockOn       = false;
    uint32_t g_irqBlockUs       = 0u;
    uint32_t g_irqBlockPeriodMs = 1u;
    uint32_t g_irqBlockIsrCount = 0u; ///< TIM7 实际打断次数(契约里没有可打的字段, 仅供台架自查)

    //-------------------------------------------------------------------------------------------
    // 四、统计
    //-------------------------------------------------------------------------------------------
    /** 一组尝试计数。g_all = 自 reset 起累计(stat 用); g_run = 本次 soak(g_run 用, soak.end 用)。 */
    struct Counters
    {
        uint32_t att       = 0u;
        uint32_t newData   = 0u;
        uint32_t notReady  = 0u;
        uint32_t fail      = 0u;
        uint32_t tx        = 0u;
        uint32_t comp      = 0u;
        uint32_t other     = 0u; ///< 非 0x404 的失败(典型: 未初始化 0x0403)。恒有 fail == tx+comp+other
        uint32_t recovered = 0u;
        uint32_t maxGapUs  = 0u;
        uint32_t maxDurUs  = 0u;

        void reset() { *this = Counters(); }
        void noteGap(uint32_t v)
        {
            if (v > maxGapUs) maxGapUs = v;
        }
        void noteDur(uint32_t v)
        {
            if (v > maxDurUs) maxDurUs = v;
        }
    };
    Counters g_all;
    Counters g_run;

    //===========================================================================================
    // 五、控制台
    //===========================================================================================
    /**
     * 打印一行。缓冲区要放得下最长的行: `[fail]` 在字段全满(i/t/evD/durUs/gapUs/streak 都是 10 位)
     * 时约 200 字符, 留到 288 才安全 —— 截断了的行对 CSV 解析是"字段少一个", 比崩溃更难查。
     */
    static void conPrintf(const char *fmt, ...)
    {
        char    buf[288];
        va_list ap;
        va_start(ap, fmt);
        const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        if (n <= 0 || g_con == nullptr)
        {
            return;
        }
        const uint16_t len =
            static_cast<uint16_t>((n < static_cast<int>(sizeof(buf))) ? n : static_cast<int>(sizeof(buf) - 1));
        g_con->send(reinterpret_cast<const uint8_t *>(buf), len);
    }
    static void printPrompt() { g_con->send(reinterpret_cast<const uint8_t *>("ok> "), 4); }
    static bool consoleAvailable() { return g_conRing->available() > 0u; }

    //===========================================================================================
    // 六、I2C 底层原语(台架自用)
    //===========================================================================================
    /**
     * 从 I2C1 上的 BMP388(qlen 字节)突发读, **不走 IICDevice**。
     *
     * 为什么不用 IICDevice::read(): 它在入口有 `if (bus.hasError()) bus.recover();`(dlx_iic.hpp:352)。
     * 台架做"失败现场"诊断时正是要看"总线还没被 recover 过时是什么样", 所以这里只用 IICBus 的
     * 原语, 并且自己决定是否 clearError() —— 也就是:
     *   - kind=comp 之后的诊断读: busError 本来就是 0, 这次 clearError() 是空操作, 行为不变;
     *   - 其余任何地方都不用这个函数, 免得把自动 recover 的路径绕掉。
     *
     * @return true = 本次事务全程正常
     */
    static bool benchBurstRead(uint8_t reg, uint8_t *out, uint16_t n)
    {
        for (uint16_t i = 0; i < n; ++i)
        {
            out[i] = 0u; // 与驱动的 readRegs 一致: 失败时不留栈上旧值(否则"碰巧"看起来正常)
        }
        g_bus->clearError();
        g_bus->start();
        g_bus->sendAddress(cfg::hw::baro::kAddr, I2C_Direction_Transmitter);
        g_bus->sendByte(reg);
        g_bus->start();
        g_bus->sendAddress(cfg::hw::baro::kAddr, I2C_Direction_Receiver);
        for (uint16_t i = 0; i < n; ++i)
        {
            out[i] = g_bus->receiveByte(static_cast<uint16_t>(i) + 1u < n);
        }
        g_bus->stop();
        g_bus->setAck(true);
        return !g_bus->hasError();
    }

    /** 读一个 16bit 寄存器地址的器件(VL53L1X 的寄存器地址是 2 字节, 高字节在前)。 */
    static bool iicRead16(IICDevice &dev, uint16_t reg, uint8_t *out, uint16_t n)
    {
        uint8_t addr[2] = { static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFFu) };
        ByteBuffer w(addr, sizeof(addr));
        w.write(addr, sizeof(addr)); // 必须标记已写入, 否则 used()==0, 读事务不带寄存器地址
        ByteBuffer r(out, n);
        return dev.read(r, w, n);
    }

    //===========================================================================================
    // 七、一次尝试: 现场快照 + tx/comp 判别
    //===========================================================================================
    /** 一次尝试的全部现场(全在本函数内取完, 顺序即契约要求的"先快照, 后处理")。 */
    struct Attempt
    {
        BMP388::ReadResult result;
        ExceptionCode      err;
        uint16_t           errCode;  ///< err 的数值形式, 用于分类([fail] 行尾的 err=)
        uint32_t startCyc;
        uint32_t durUs;
        uint32_t gapUs;   ///< 距上一次尝试起点(DWT start-to-start); 首次为 0
        uint32_t evD;     ///< 本次尝试的 IICBus::eventTimeouts() 增量
        uint32_t lastEv;  ///< 快照时刻的 lastFailedEvent()(必须在任何后续事务前取)
        uint32_t streak;  ///< 快照时刻的 baro.failCount()
        uint16_t sr1;
        uint16_t sr2;
        uint8_t  scl;     ///< PB6 电平(1 = 高 = 空闲; 0 = 被从机拉住)
        uint8_t  sda;     ///< PB7 电平
        /** true = tx 类(事务失败)。**仅在 errCode==0x0404 时有意义**(见 failureKindName)。 */
        bool     txKind;
        uint8_t  status;  ///< 仅 comp 类有效(见文件头口径 2)
        uint8_t  raw[6];  ///< 仅 comp 类有效: 失败之后那一刻的 0x04~0x09
    };

    /**
     * 失败类别(进 [fail] 行的 kind=)。**先按错误码分流, 再在 0x404 内部二分**:
     *   0x0404 BARO_READ_FAILED  → tx(事务失败) / comp(补偿结果不可信)
     *   其它(0x0403 BARO_CALIB_INVALID / 0x0401 NOT_FOUND ...) → other, **不混进 tx/comp**
     * 为什么必须有这道门禁: `!initialized` 时 read() 直接在 dlx_bmp388.hpp:497-500 返回 0x0403,
     * 那次调用**一个寄存器都没读**(evD==0、failStreak 不变) ⇒ 用 tx/comp 的判据会把它判成 comp,
     * 于是健康板子在"忘了 init"时反而报 fail≈att/comp≈att —— 签名与"comp 占大头 ⇒ 与 I2C 无关"
     * 这条真结论**恰好相反**。其它错误码单列 other, 让它一眼可见, 而不是被 silently 归到某一类。
     */
    static const char *failureKindName(const Attempt &a)
    {
        if (a.result != BMP388::ReadResult::Failed)
        {
            return "none";
        }
        if (a.errCode != static_cast<uint16_t>(ExceptionCode::BARO_READ_FAILED))
        {
            return "other";
        }
        return a.txKind ? "tx" : "comp";
    }

    /** 本次尝试的 I2C 事务是否通了: comp 类失败算通(突发读成功过); 未初始化(0x0403)没有事务, 不算通。 */
    static bool transactionOk(const Attempt &a)
    {
        if (a.result != BMP388::ReadResult::Failed)
        {
            return true;
        }
        return (a.errCode == static_cast<uint16_t>(ExceptionCode::BARO_READ_FAILED)) && !a.txKind;
    }

    static Attempt runAttempt(uint32_t prevStartCyc)
    {
        Attempt a;
        a.err     = ExceptionCode::EXCEPTION_NONE;
        a.errCode = static_cast<uint16_t>(ExceptionCode::EXCEPTION_NONE);
        a.status  = 0u;
        a.sr1     = 0u;
        a.sr2     = 0u;
        a.scl     = 0u;
        a.sda     = 0u;
        a.lastEv  = 0u;
        a.evD     = 0u;
        a.streak  = 0u;
        a.durUs   = 0u;
        a.gapUs   = 0u;
        a.txKind  = false;
        for (uint8_t i = 0; i < 6u; ++i)
        {
            a.raw[i] = 0u;
        }

        const uint32_t evBefore = g_bus->eventTimeouts();
        a.startCyc              = cycNow();
        // 单次差值, 上界 = soak 周期(≤ kSoakMaxPeriodMs) + 上一次尝试耗时(一次事务, 最坏几十 ms)
        // ≤ 1.1s ≪ 25.566s ⇒ 不需要累加(见 usBetween 的说明)。
        a.gapUs                 = (prevStartCyc == 0u) ? 0u : usBetween(a.startCyc, prevStartCyc);

        float pressurePa    = 0.0f;
        float temperatureC  = 0.0f;
        a.result            = g_baro->read(pressurePa, temperatureC, a.err);
        const uint32_t endCyc = cycNow();

        // ---- 立刻快照(见文件头口径 1: 此刻还没人调用过 recover) ----
        a.sr1    = kBaroI2c->SR1;
        a.sr2    = kBaroI2c->SR2;
        a.scl    = static_cast<uint8_t>(g_sclPin->read() ? 1u : 0u);
        a.sda    = static_cast<uint8_t>(g_sdaPin->read() ? 1u : 0u);
        a.lastEv = g_bus->lastFailedEvent();
        a.streak = g_baro->failCount();
        a.evD    = g_bus->eventTimeouts() - evBefore;
        // 单次差值, 上界 = 一次 `baro.read()` 的最坏阻塞: IICBus 每个事件最多 kEventWaitLimit(20 万)
        // 次轮询(dlx_iic.hpp:25, 每次约几十周期 ⇒ 单事件几 ms), 一次读最多 ~7 个事件 ⇒ 几十 ms
        // ≪ 25.566s ⇒ 不需要累加。
        a.durUs  = usBetween(endCyc, a.startCyc);

        /**
         * ---- tx / comp 判别(本台架的核心) ----
         * 判据只用**驱动的观测值**, 不含任何推测, 而且**先要求 err 是 0x404**:
         *   err != 0x404            → kind=other(不参与 tx/comp 统计; 典型是未初始化的 0x0403)
         *   tx   : 本次事务超时过(evTimeouts 有增量) 或 连续事务失败计数>0
         *          ⇒ readBurst() 走了 dlx_bmp388.hpp:328-333 那条路(来源 A; init 阶段同类 = 来源 C)
         *   comp : 两者都没动 ⇒ 事务成功(failStreak 在 :334 被清零)、结果是在 :513-515 被否掉的(来源 B)
         * 反过来也成立: failStreak 只在 readBurst 失败时 ++、成功时清 0, 所以 comp 必然 streak==0。
         */
        a.errCode = static_cast<uint16_t>(a.err);
        a.txKind  = false;
        if (a.result == BMP388::ReadResult::Failed &&
            a.errCode == static_cast<uint16_t>(ExceptionCode::BARO_READ_FAILED))
        {
            a.txKind = (a.evD != 0u) || (a.streak != 0u);
        }

        // comp 类补一次诊断读: 突发读成功过, 但驱动不导出缓冲, 只有再读一次才知道"器件当时在说什么"。
        // 放在快照之后 ⇒ 它改动的 SR1/SR2 不会污染 a 的字段; 且此路径 busError 为 0, 不绕过 recover()。
        if (a.result == BMP388::ReadResult::Failed &&
            a.errCode == static_cast<uint16_t>(ExceptionCode::BARO_READ_FAILED) && !a.txKind)
        {
            uint8_t burst[BMP388_DATA_BURST_LEN];
            if (benchBurstRead(static_cast<uint8_t>(BMP388_Reg::Status), burst, BMP388_DATA_BURST_LEN))
            {
                a.status = burst[0];
                for (uint8_t i = 0; i < 6u; ++i)
                {
                    a.raw[i] = burst[1u + i];
                }
            }
        }
        return a;
    }

    static void recordAttempt(Counters &c, const Attempt &a)
    {
        ++c.att;
        c.noteGap(a.gapUs);
        c.noteDur(a.durUs);
        switch (a.result)
        {
        case BMP388::ReadResult::NewData:
            ++c.newData;
            break;
        case BMP388::ReadResult::NotReady:
            ++c.notReady;
            break;
        case BMP388::ReadResult::Failed:
            ++c.fail;
            // 先按错误码分流(见 failureKindName): 只有 0x404 才进 tx/comp, 其它单列 other。
            // 不变量: fail == tx + comp + other([soak]/[soak.end] 只印 fail/tx/comp —— 契约字段不变,
            // 所以 other 需由 "fail - tx - comp" 读出; [fail] 行的 kind=other 会把它点名。)
            if (a.errCode != static_cast<uint16_t>(ExceptionCode::BARO_READ_FAILED))
            {
                ++c.other;
            }
            else if (a.txKind)
            {
                ++c.tx;
            }
            else
            {
                ++c.comp;
            }
            break;
        }
    }

    /**
     * 契约字段顺序(冻结): i t kind sr1 sr2 lastEv evD status raw durUs gapUs streak。
     * 行尾**追加**两个诊断字段(lead 批准, 都在最后, 不动既有字段名/顺序):
     *   `err=0x....`  —— kind 有第三个取值 other, 它必须能自证原因;
     *   `scl=%u sda=%u`(r2 起, A21) —— 失败当刻 PB6/PB7 的电平, 与 sr1/sr2 同属 runAttempt() 里那一个
     *     快照块(:405-408), 都在 `read()` 返回之后、任何 recover/下一次事务之前。
     *     为什么需要它: H4("总线被从机拉住")的唯一直接证据就是"失败那一刻 scl/sda 是不是 0"; 此前只有
     *     手工敲 `bus` 才看得到(那是查询时刻, 不是失败时刻), 所以 H4 一直没有自动化证据。
     * 追加在最后不影响 run_baro.py 的表驱动解析(它只锚行首的 i/t/kind, 其余字段逐个 search)。
     */
    static void printFail(uint32_t i, uint32_t tMs, const Attempt &a)
    {
        conPrintf("[fail] i=%u t=%u kind=%s sr1=0x%04X sr2=0x%04X lastEv=0x%08X evD=%u "
                  "status=0x%02X raw=%02X%02X%02X%02X%02X%02X durUs=%u gapUs=%u streak=%u err=0x%04X "
                  "scl=%u sda=%u\r\n",
                  static_cast<unsigned>(i), static_cast<unsigned>(tMs), failureKindName(a),
                  static_cast<unsigned>(a.sr1), static_cast<unsigned>(a.sr2), static_cast<unsigned>(a.lastEv),
                  static_cast<unsigned>(a.evD), static_cast<unsigned>(a.status),
                  static_cast<unsigned>(a.raw[0]), static_cast<unsigned>(a.raw[1]), static_cast<unsigned>(a.raw[2]),
                  static_cast<unsigned>(a.raw[3]), static_cast<unsigned>(a.raw[4]), static_cast<unsigned>(a.raw[5]),
                  static_cast<unsigned>(a.durUs), static_cast<unsigned>(a.gapUs), static_cast<unsigned>(a.streak),
                  static_cast<unsigned>(a.errCode), static_cast<unsigned>(a.scl), static_cast<unsigned>(a.sda));
    }

    //===========================================================================================
    // 八、器件 init(带 attempts 计数)与分阶段诊断
    //===========================================================================================
    /** 与 BMP388::init() 同一策略的重试循环(dlx_bmp388.hpp:376-399), 区别只是把尝试次数带出来。 */
    static bool driverInitWithAttempts(uint32_t &attempts, ExceptionCode &err)
    {
        for (uint32_t i = 0u; i < cfg::sensor::baro::kInitRetry; ++i)
        {
            if (i != 0u)
            {
                delay_ms(cfg::sensor::baro::kInitRetryMs); // 重试间隔与次数是一对参数, 不在这里另立一套
            }
            ++attempts;
            if (g_baro->tryInit(err) == FunctionResult::SUCCESS)
            {
                return true;
            }
        }
        return false;
    }

    /**
     * 换 I2C1 速率。为什么用 I2C_DeInit 而不是只再调一次 IICBus::init():
     *   CCR/TRISE 只在 PE=0 时可写(StdPeriph 的 I2C_Init 内部会关 PE), 但**不会清 SR1/SR2 的粘滞
     *   错误标志**。"100k vs 400k 失败率对比"要公平, 就得让两条速率从同一个干净状态出发 ——
     *   DeInit 顺带把标志和状态机一起复位, 也顺手回答"H1 里 AF 粘住是否只有 DeInit 能解"这个问题。
     */
    static void setI2cSpeed(uint32_t hz)
    {
        I2C_DeInit(kBaroI2c);
        g_bus->init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, hz, kI2CSelfAddress);
        g_bus->clearError();
        g_speedHz = hz;
    }

    /**
     * 分阶段诊断(**只读**) —— 驱动 tryInit() 只给一个 ExceptionCode, 但契约要求 stage=。
     * 为什么能用只读判断: tryInit 的 5 个阶段各自都有一个"读特征":
     *   reset    : 连 CHIP_ID(0x00) 都读不回来 ⇒ 写 CMD/读 CHIP_ID 这一步就过不去(总线无人应答)
     *   chipid   : 读得回来但不是 0x50(读到保留寄存器 = 地址/接线错)
     *   calib    : 21 字节标定系数读不回来, 或全 0/全 FF
     *   write    : 回读能读回来但值不对 ⇒ 写进去的配置没落地(驱动在 :481-487 判 NOT_FOUND 的那条)
     *   readback : 回读事务本身失败(驱动在 :466-474 判 READ_FAILED 的那条)
     * 只读意味着它不会改器件状态, 因此可以放心在失败的 init 之后跑。
     */
    static const char *probeStage(uint16_t &errCode, uint8_t &chipId)
    {
        chipId = 0u;
        uint8_t id = 0u;
        if (!benchBurstRead(static_cast<uint8_t>(BMP388_Reg::ChipId), &id, 1u))
        {
            errCode = static_cast<uint16_t>(ExceptionCode::BARO_NOT_FOUND);
            return "reset";
        }
        chipId = id;
        if (id != BMP388_CHIP_ID_VALUE)
        {
            errCode = static_cast<uint16_t>(ExceptionCode::BARO_CHIP_ID_MISMATCH);
            return "chipid";
        }
        uint8_t calib[BMP388_CALIB_LEN];
        if (!benchBurstRead(static_cast<uint8_t>(BMP388_Reg::CalibStart), calib, BMP388_CALIB_LEN))
        {
            errCode = static_cast<uint16_t>(ExceptionCode::BARO_READ_FAILED);
            return "calib";
        }
        bool allZero = true;
        bool allOnes = true;
        for (uint8_t i = 0; i < BMP388_CALIB_LEN; ++i)
        {
            allZero = allZero && (calib[i] == 0x00u);
            allOnes = allOnes && (calib[i] == 0xFFu);
        }
        if (allZero || allOnes)
        {
            errCode = static_cast<uint16_t>(ExceptionCode::BARO_CALIB_INVALID);
            return "calib";
        }
        uint8_t rb[BMP388_CFG_READBACK_LEN];
        if (!benchBurstRead(BMP388_CFG_READBACK_ADDR, rb, BMP388_CFG_READBACK_LEN))
        {
            errCode = static_cast<uint16_t>(ExceptionCode::BARO_READ_FAILED);
            return "readback";
        }
        uint8_t cfgReg = 0u;
        if (!benchBurstRead(static_cast<uint8_t>(BMP388_Reg::Config), &cfgReg, 1u))
        {
            errCode = static_cast<uint16_t>(ExceptionCode::BARO_READ_FAILED);
            return "readback";
        }
        const bool readbackOk = (rb[0] == cfg::sensor::baro::kIntCtrl) && (rb[1] == BMP388_IF_CONF_VALUE) &&
                                (rb[2] == BMP388_PWR_CTRL_VALUE) && (rb[3] == cfg::sensor::baro::kOsr) &&
                                (rb[4] == cfg::sensor::baro::kOdr) && (cfgReg == cfg::sensor::baro::kConfig);
        if (!readbackOk)
        {
            errCode = static_cast<uint16_t>(ExceptionCode::BARO_NOT_FOUND);
            return "write";
        }
        // 全部读通且与 cfg 一致 ⇒ 这次失败是瞬态的(驱动自己那 20 次重试里的一次性抖动)
        errCode = static_cast<uint16_t>(ExceptionCode::EXCEPTION_NONE);
        return "readback";
    }

    //===========================================================================================
    // 九、stress: 并发负载(SPI1 BMI088 + I2C2 VL53L1X + ADC1)
    //===========================================================================================
    /** 读 BMI088 加速度计 CHIP_ID(时序照 dlx_bmi088.hpp:117-123: 地址字节 bit7=1 表示读, 后跟 dummy)。 */
    static uint8_t bmiAccelChipId()
    {
        GPIO &cs = *g_accelCs;
        cs = 0;
        g_spiImu->swap(static_cast<uint8_t>(0x80u | kImuAccelChipIdReg));
        g_spiImu->swap(static_cast<uint8_t>(0x00));
        const uint8_t id = g_spiImu->swap(static_cast<uint8_t>(0x00));
        cs = 1;
        return id;
    }

    /** 一次 SPI1 负载步进: 1 次 CHIP_ID 读(定健康) + kImuStressReads 组数据读(造总线负载)。 */
    static bool stressSpiStep()
    {
        const uint8_t id = bmiAccelChipId();
        GPIO       &cs   = *g_accelCs;
        for (uint8_t i = 0u; i < kImuStressReads; ++i)
        {
            cs = 0;
            g_spiImu->swap(static_cast<uint8_t>(0x80u | kImuDataStartReg));
            g_spiImu->swap(static_cast<uint8_t>(0x00));
            for (uint8_t b = 0u; b < kImuDataBytes; ++b)
            {
                (void)g_spiImu->swap(static_cast<uint8_t>(0x00));
            }
            cs = 1;
        }
        return id == kImuAccelChipId;
    }

    /** 一次 I2C2 负载步进: 读 VL53L1X 模型 ID(0x010F, 期望 0xEA)。 */
    static bool stressI2c2Step()
    {
        uint8_t id = 0u;
        if (!iicRead16(*g_rangeDev, kRangeModelIdReg, &id, 1u))
        {
            return false;
        }
        return id == kRangeModelId;
    }

    /** 一次 ADC 负载步进: 一次完整电压/电流采样(与主线 bringUpPower 同一套配置)。 */
    static bool stressAdcStep()
    {
        float         packVolts = 0.0f;
        float         amps      = 0.0f;
        ExceptionCode err;
        return g_adc->readSupply(packVolts, amps, err) == FunctionResult::SUCCESS;
    }

    /**
     * stress 首次开启时把三条负载各自 bring up 一次并探测, 返回该路是否可用。
     * 三路的构造都照 app_sensors.hpp 的接线事实, 但**不搬 Sensors 结构体**, 也不建任何线程 ——
     * 台架只需要"同一条总线/同一套引脚上真的有流量"。
     */
    static bool probeSpi()
    {
        if (!g_spiReady)
        {
            g_spiImu->init(cfg::hw::imu::kSpi);
            g_accelCs->init(GPIOModeProfile::OUT_PP_UP_50MHz);
            *g_accelCs = 1;
            g_spiReady = true;
        }
        return bmiAccelChipId() == kImuAccelChipId;
    }

    static bool probeI2c2()
    {
        if (!g_rangeReady)
        {
            // 顺序照 app_sensors.hpp:193-204 的 bringUpRange: XSHUT 先低(保持硬件待机) -> 总线 init
            // -> 释放 XSHUT -> 等启动。XSHUT 低有效, 所以"释放"= 驱动为高。
            g_xshut->init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
            *g_xshut = 0;
            delay_ms(kRangeBootDelayMs);
            g_rangeBus->init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, cfg::hw::range_::kSpeed, kI2CSelfAddress);
            *g_xshut = 1;
            delay_ms(kRangeBootDelayMs);
            g_rangeReady = true;
        }
        return stressI2c2Step();
    }

    static bool probeAdc()
    {
        if (!g_adcReady)
        {
            ADC1Config c;
            c.voltageChannel      = cfg::hw::adc::kVoltageCh;
            c.currentChannel      = cfg::hw::adc::kCurrentCh;
            c.sampleCount         = cfg::hw::adc::kSampleCount;
            c.vrefMv              = cfg::power::kVrefMv;
            c.voltageDividerRatio = cfg::power::kVoltageDividerRatio;
            c.voltageOffsetV      = static_cast<float>(cfg::power::kVoltageOffsetMv) * 0.001f;
            c.currentOffsetV      = cfg::power::kCurrentOffsetV;
            c.currentAmpsPerVolt  = cfg::power::kCurrentAmpsPerVolt;
            c.cellCount           = cfg::power::kCellCount;
            g_adc->setConfig(c);
            ExceptionCode err;
            if (g_adc->init(claims::kAdcVoltagePin, claims::kAdcCurrentPin, err) != FunctionResult::SUCCESS)
            {
                g_adcReady = false; // 下次 stress on 还会再试一次(ADC 失败往往是电源/接线, 手工复位后可恢复)
                return false;
            }
            g_adcReady = true;
        }
        return stressAdcStep();
    }

    /**
     * 主循环(以及 soak 循环)里的负载步进, 限速到 kStressStepMs。
     * 某一路一旦失败就**停掉该路后续步进**: 器件不在时每次事务都会在 waitEvent 的 20 万次轮询上限上
     * 空转(单次几十毫秒, 见 dlx_iic.hpp:149-172), 那种空转会把主循环和测量本身拖垮, 变成台架自己的
     * 伪影。停掉之后 `[stress]` 行仍会如实报 fail。
     */
    static void stressService()
    {
        if (!g_stressOn)
        {
            return;
        }
        const uint32_t now = cycNow();
        // 单次差值(只有 5ms 量级的节拍判断), 与上一次步进相隔最多 kStressStepMs +
        // 上一次步进自己的耗时(≤ ~1ms) ⇒ 上界约 6ms ≪ 25.566s。注意这里**故意**只用单次差值:
        // 节拍是"限速"而不是"计时", 就算跨了回绕点也只会让某一步提前/延后一次, 不影响任何被测量的语义。
        if (g_stressLastCyc != 0u && usBetween(now, g_stressLastCyc) < kStressStepMs * 1000u)
        {
            return;
        }
        g_stressLastCyc = now;
        if (g_stressSpiOk && !stressSpiStep())
        {
            g_stressSpiOk = false;
        }
        if (g_stressI2c2Ok && !stressI2c2Step())
        {
            g_stressI2c2Ok = false;
        }
        if (g_stressAdcOk && !stressAdcStep())
        {
            g_stressAdcOk = false;
        }
    }

    //===========================================================================================
    // 十、irqblock: TIM7 更新中断里忙等(H5/H7)
    //===========================================================================================
    /**
     * 为什么必须是"定时器 ISR 里忙等", 而不是"主循环里 __disable_irq() + 忙等":
     *   H7 说的是**事务进行途中**被打断 —— 400kHz 下一个字节只有 22.5µs, 软件晚一步读 DR 就 OVR。
     *   主循环里关中断只会推迟中断, 推后的是它自己的下一轮轮询, 绝不会插进一次正在进行的 I2C 事务中间,
     *   所以它复现不了 H7。TIM7 的更新中断优先级高于任何线程(裸机下就是抢占主循环), 在它里面忙等 us µs,
     *   就会实打实地插在 start()/sendByte()/receiveByte() 之间。代价是"关中断"这一项没做 —— 明确记在
     *   README 的局限里。
     */
    static void irqBlockIsr(BasicTimer *tim, void *ctx)
    {
        (void)ctx;
        ++g_irqBlockIsrCount;
        tim->clearITPending(TimerITProfile::Update); // 先清标志再忙等: 否则忙等期间的新事件会被吃掉
        busyWaitUs(g_irqBlockUs);
    }

    /** 全台架共用一个 TIM7 实例: 启动/停止操作的是同一组寄存器, 分成两个对象容易让人看错。 */
    static BasicTimer &irqTimer()
    {
        // TIM7 是 stm32f4xx_it.cpp:288-291 唯一留了转发的定时器(转 DLX_IT_invoke_callback(TIM7_IRQn)),
        // 所以台架不用自己覆盖 IRQHandler, 直接用 dlx_nvic_it 的回调机制。
        static BasicTimer tim(BasicTimerProfile::TIM7_Profile);
        return tim;
    }

    /**
     * TIM7 的计数时钟 [Hz]。TIM7 挂 APB1: APB1 分频 ≠ 1 时定时器时钟 = PCLK1 × 2
     * (同 dlx_timer.hpp:612-622 的算法; 从 RCC 寄存器实测, 不依赖 SystemCoreClock 变量)。
     * 本板: PCLK1 = 42MHz, PPRE1 ≠ 1 ⇒ 84MHz。
     */
    static uint32_t irqTimerClockHz()
    {
        RCC_ClocksTypeDef clocks;
        RCC_GetClocksFreq(&clocks);
        uint32_t timClock = clocks.PCLK1_Frequency;
        if (((RCC->CFGR & RCC_CFGR_PPRE1) >> 10) != 0u)
        {
            timClock *= 2u;
        }
        return timClock;
    }

    /**
     * irqblock 的计数时钟能否**精确**做到 1MHz(即 period = periodMs 毫秒, 且 [irqblock] 打的 periodMs 就是实际值)。
     *
     * 为什么要有这条: 预分频是整数 ⇒ 只有 `timClock % kIrqTimerTickHz == 0` 时 1MHz 才精确。本板 84MHz/84
     * = 1.000000MHz 恰好整除, 行为与以前一致; 但将来若换非整数 MHz 的 HCLK, 截断后的计数时钟会让实际周期
     * 与打印值差 ~1% —— 那正是 D3 想要根除的"打印值 ≠ 实际值"。所以不静默降级, 直接拒绝启动。
     */
    static bool irqTimerClockExact(uint32_t &timClock)
    {
        timClock = irqTimerClockHz();
        return (timClock % kIrqTimerTickHz) == 0u;
    }

    static void startIrqBlockTimer()
    {
        BasicTimer &tim = irqTimer();

        const uint32_t timClock  = irqTimerClockHz(); // 调用方已用 irqTimerClockExact() 放行过
        const uint16_t prescaler = static_cast<uint16_t>((timClock / kIrqTimerTickHz) - 1u);
        const uint32_t period    = g_irqBlockPeriodMs * (kIrqTimerTickHz / 1000u) - 1u;

        tim.stop();
        tim.init(prescaler, period); // init 内部会开计数, 立刻停掉, 等配置齐了再 start
        tim.stop();
        tim.clearITPending(TimerITProfile::Update);
        tim.setITRequest(TimerITProfile::Update, ENABLE);
        tim.initNVIC(NVICPriorityProfile::P0_S2); // 最高抢占优先级: 保证能插进事务中间
        tim.setITCallback(&irqBlockIsr, nullptr);
        tim.start();
    }

    static void stopIrqBlockTimer()
    {
        static BasicTimer tim(BasicTimerProfile::TIM7_Profile);
        tim.stop();
        tim.clearITPending(TimerITProfile::Update);
        tim.setITRequest(TimerITProfile::Update, DISABLE);
    }

    //===========================================================================================
    // 十一、命令实现
    //===========================================================================================
    static void printCfg()
    {
        conPrintf("[cfg] i2c=I2C1 scl=PB6 sda=PB7 addr=0x%02X speed=%u osr=0x%02X odr=0x%02X cfg=0x%02X "
                  "int=0x%02X pwr=0x%02X\r\n",
                  static_cast<unsigned>(cfg::hw::baro::kAddr), static_cast<unsigned>(g_speedHz),
                  static_cast<unsigned>(cfg::sensor::baro::kOsr), static_cast<unsigned>(cfg::sensor::baro::kOdr),
                  static_cast<unsigned>(cfg::sensor::baro::kConfig),
                  static_cast<unsigned>(cfg::sensor::baro::kIntCtrl),
                  static_cast<unsigned>(BMP388_PWR_CTRL_VALUE));
    }

    /** `sr`: I2C1 的原始状态。注意读 SR1 之后再读 SR2 会清 ADDR 标志 —— 这是查询命令, 可接受。 */
    static void printSr()
    {
        const uint16_t sr1 = kBaroI2c->SR1;
        const uint16_t sr2 = kBaroI2c->SR2;
        conPrintf("[sr] sr1=0x%04X sr2=0x%04X af=%u berr=%u arlo=%u busy=%u msL=%u tra=%u txE=%u rxne=%u btf=%u "
                  "ccr=%u trise=%u ackCfg=%u\r\n",
                  static_cast<unsigned>(sr1), static_cast<unsigned>(sr2),
                  static_cast<unsigned>((sr1 & I2C_SR1_AF) ? 1u : 0u),
                  static_cast<unsigned>((sr1 & I2C_SR1_BERR) ? 1u : 0u),
                  static_cast<unsigned>((sr1 & I2C_SR1_ARLO) ? 1u : 0u),
                  static_cast<unsigned>((sr2 & I2C_SR2_BUSY) ? 1u : 0u),
                  static_cast<unsigned>((sr2 & I2C_SR2_MSL) ? 1u : 0u),
                  static_cast<unsigned>((sr2 & I2C_SR2_TRA) ? 1u : 0u),
                  static_cast<unsigned>((sr1 & I2C_SR1_TXE) ? 1u : 0u),
                  static_cast<unsigned>((sr1 & I2C_SR1_RXNE) ? 1u : 0u),
                  static_cast<unsigned>((sr1 & I2C_SR1_BTF) ? 1u : 0u),
                  static_cast<unsigned>(kBaroI2c->CCR), static_cast<unsigned>(kBaroI2c->TRISE),
                  static_cast<unsigned>((kBaroI2c->CR1 & I2C_CR1_ACK) ? 1u : 0u));
    }

    /**
     * `bus`: PB6/PB7 的**当前输入电平**(判"H4: 总线是不是被从机拉死")。
     * 为什么可以直接读: GPIO::read() 读的是 IDR(dlx_gpio.hpp:92-94), 而开漏 AF 输出时 IDR 仍然反映
     * 引脚真实电平, 所以**不需要**把引脚切成输入 —— 切了反而会破坏 AF4 配置, 变成"测一次坏一次"。
     */
    static void printBus()
    {
        conPrintf("[bus] scl=%u sda=%u cpu=%u cycPerUs=%u\r\n",
                  static_cast<unsigned>(g_sclPin->read() ? 1u : 0u),
                  static_cast<unsigned>(g_sdaPin->read() ? 1u : 0u),
                  static_cast<unsigned>(SystemCoreClock), static_cast<unsigned>(cycPerUs()));
    }

    static void cmdInit(bool hasHz, uint32_t hz)
    {
        if (hasHz)
        {
            setI2cSpeed(hz);
        }
        uint32_t      attempts = 0u;
        ExceptionCode err      = ExceptionCode::EXCEPTION_NONE;
        const bool    ok       = driverInitWithAttempts(attempts, err);

        uint8_t  chipId  = g_baro->lastChipId();
        uint16_t errCode = static_cast<uint16_t>(err);
        const char *stage = "readback"; // 驱动成功 = 五个阶段全过
        if (!ok)
        {
            // 失败时才跑只读分阶段诊断(会多发几次事务, 但只在失败路径上)
            uint16_t probeErr  = static_cast<uint16_t>(ExceptionCode::EXCEPTION_NONE);
            uint8_t  probeChip = 0u;
            stage              = probeStage(probeErr, probeChip);
            if (chipId == 0u)
            {
                chipId = probeChip;
            }
            if (errCode == 0u)
            {
                errCode = probeErr;
            }
        }
        conPrintf("[init] speed=%u stage=%s result=%s err=0x%04X chipId=0x%02X attempts=%u\r\n",
                  static_cast<unsigned>(g_speedHz), stage, ok ? "OK" : "FAIL", static_cast<unsigned>(errCode),
                  static_cast<unsigned>(chipId), static_cast<unsigned>(attempts));
    }

    static void cmdSpeed(uint32_t hz)
    {
        setI2cSpeed(hz);
        uint32_t      attempts = 0u;
        ExceptionCode err      = ExceptionCode::EXCEPTION_NONE;
        const bool    ok       = driverInitWithAttempts(attempts, err);
        conPrintf("[speed] hz=%u result=%s\r\n", static_cast<unsigned>(hz), ok ? "OK" : "FAIL");
    }

    /**
     * `soak <ms> [period_ms]`: 参考周期的持续读。契约冻结:
     *   运行期每秒一行 [soak] ...(计数与最大值都是**本次 soak** 的, 从 0 起算)
     *   结束一行   [soak.end] ... failrate=... evTimeouts=<bus 累计值> recovered=...
     * t 一律是 soak 时间(ms), **单调不减**且等于真实经过时间 —— 长 soak 与回绕点的处理见下面的 D4 推导。
     * 任意键中止(与 link bench 的长循环一致): 立刻收尾并打 soak.end, 不会把"中止"伪装成"跑满"。
     * ms ∈ [1, kSoakMaxMs=3600000(1h)], period_ms ∈ [1, 1000]; 二者都真的可用(r2 及以前 >25.566s 停不下来)。
     */
    static void cmdSoak(uint32_t totalMs, uint32_t periodMs)
    {
        // 器件没初始化就拒绝跑: 那时 read() 会在 dlx_bmp388.hpp:497-500 直接返回 0x0403, 一次寄存器
        // 都没读 —— 继续跑只会产出一堆 kind=other 的失败行, 让"基线失败率"变成仪器自己的噪声。
        // 打契约内的 [err] 形式让它**响**: 脚本等不到 [soak.end] 就会自己报 FAIL。
        if (!g_baro->isReady())
        {
            conPrintf("[err] not-initialized\r\n");
            return;
        }
        // 刻意**不**打任何契约外的回显行: 解析器按 "[soak] t=..." / "[soak.end] t=..." 两条正则工作,
        // 多一行"[soak] begin ms=..."虽然会被它跳过, 但契约写的是"每条命令只输出规定的行"。
        g_run.reset();
        const uint32_t periodCyc = usToCyc(periodMs * 1000u);
        uint32_t       prevStart = 0u;
        uint32_t       lastSec   = 0u;
        uint32_t       tMs       = 0u;
        bool           prevFail  = false;

        /**
         * ==== soak 的绝对计时必须**逐轮累加**, 不能"起点对终点一次减出来"(D4, 用户上板实测) ====
         *
         * 旧写法是 `tMs = usBetween(cycNow(), t0) / 1000`(t0 = soak 起点)。`usBetween()` 是 32 位无符号
         * 差值, 只在差值 < 2^32 周期时等于真实值 —— 而 DWT->CYCCNT 每 2^32/168MHz = **25.566s** 绕回一次。
         * 于是一旦 soak 超过 25.566s, 差值绕回、`tMs` 归零, `tMs >= totalMs` 对 `soak 60000` **永远不成立**:
         * 用户实跑 16:02 那次, `[soak] t` 从 24999 跳回 3、又从 25006 跳回 1, 最后靠脚本发的 `inject`
         * 才把 soak 打断(审计文档 :70 点过这一类 25.57s CYCCNT 回绕的前科)。
         *
         * 现在每轮只取**单轮差值**并累加进 uint64_t。为什么这样对任意 totalMs 都成立(这段就是要被验的推导):
         *   ① **单轮差值有硬上界**: 一轮 = stressService 一步(≤ kStressStepMs = 5ms)
         *      + 一次 runAttempt(一次 I2C 事务, 最坏几十 ms, 见 durUs 的说明)
         *      + waitUntilCyc 等到节拍(≤ periodMs ≤ kSoakMaxPeriodMs = 1000ms)
         *      ⇒ 上界 ≈ 1.1s, 远小于 25.566s。所以 `(uint32_t)(nowCyc - lastCyc)` **不会**发生绕回,
         *        每次加进去的都是真实增量(无符号减法在"真实差 < 2^32"时恒等于真实差)。
         *   ② **单调且有下界增长**: elapsedCyc 只累加非负值 ⇒ 单调不减; 每轮至少推进
         *      waitUntilCyc 的等待 ⇒ 对任意 totalMs 都存在某一轮使 `tMs >= totalMs` ⇒ 一定 break。
         *      `soak 60000` = 60000ms、`soak 1800000` = 30min 都落在同一结论里, 与绕回次数无关。
         *   ③ **量程**: kSoakMaxMs = 3600000(1h) ⇒ 3600s × 168e6 = 6.05e11 周期 < 2^64(且远超 2^32,
         *      这就是累加器必须是 uint64 而不是 uint32 的原因 —— uint32 在 25.566s 处自己就溢出了)。
         * ⇒ `[soak] t=` 与 `[soak.end] t=` 也随之单调、跨过 25.57s 后仍等于真实经过时间。
         */
        uint64_t elapsedCyc = 0u;
        uint32_t lastCyc    = cycNow();

        while (true)
        {
            // 先结算"上一轮到此刻"的真实增量, 再判是否到点(顺序: 判点用到的 tMs 一定是已结算的值)
            const uint32_t nowCyc = cycNow();
            elapsedCyc += static_cast<uint32_t>(nowCyc - lastCyc);
            lastCyc = nowCyc;
            const uint64_t elapsedUs = elapsedCyc / cycPerUs();
            tMs = static_cast<uint32_t>(elapsedUs / 1000u);
            if (tMs >= totalMs)
            {
                break;
            }

            stressService(); // stress/irqblock 可以与 soak 叠加: 那才是"带负载的失败率"实验

            const Attempt a = runAttempt(prevStart);
            prevStart       = a.startCyc;
            recordAttempt(g_run, a);
            recordAttempt(g_all, a);
            if (a.result == BMP388::ReadResult::Failed)
            {
                printFail(g_run.att, tMs, a);
            }
            else if (prevFail)
            {
                // 契约定义: 观察到"失败之后**下一次**尝试就成功"的次数
                ++g_run.recovered;
                ++g_all.recovered;
            }
            prevFail = (a.result == BMP388::ReadResult::Failed);

            const uint32_t sec = static_cast<uint32_t>(elapsedUs / 1000000u);
            if (sec != lastSec)
            {
                lastSec = sec;
                conPrintf("[soak] t=%u att=%u new=%u notready=%u fail=%u tx=%u comp=%u maxgapUs=%u maxdurUs=%u\r\n",
                          static_cast<unsigned>(tMs), static_cast<unsigned>(g_run.att),
                          static_cast<unsigned>(g_run.newData), static_cast<unsigned>(g_run.notReady),
                          static_cast<unsigned>(g_run.fail), static_cast<unsigned>(g_run.tx),
                          static_cast<unsigned>(g_run.comp), static_cast<unsigned>(g_run.maxGapUs),
                          static_cast<unsigned>(g_run.maxDurUs));
            }

            waitUntilCyc(a.startCyc + periodCyc); // 目标是周期节拍; 事务超期时这条立刻返回, 由 gapUs 如实记录

            if (consoleAvailable())
            {
                break; // 任意键中止(字节留给 serviceConsole 解析, 与 link bench 同款语义)
            }
        }

        // 收尾: 把最后一段(最后一次尝试 + 等待 + 按键检查)也结算进去, [soak.end] 的 t 才是真实时长
        {
            const uint32_t endCyc = cycNow();
            elapsedCyc += static_cast<uint32_t>(endCyc - lastCyc);
            tMs = static_cast<uint32_t>((elapsedCyc / cycPerUs()) / 1000u);
        }
        const double failRate = (g_run.att == 0u) ? 0.0 : (static_cast<double>(g_run.fail) / static_cast<double>(g_run.att));
        conPrintf("[soak.end] t=%u att=%u new=%u notready=%u fail=%u tx=%u comp=%u failrate=%.1e maxgapUs=%u "
                  "maxdurUs=%u evTimeouts=%u recovered=%u\r\n",
                  static_cast<unsigned>(tMs), static_cast<unsigned>(g_run.att), static_cast<unsigned>(g_run.newData),
                  static_cast<unsigned>(g_run.notReady), static_cast<unsigned>(g_run.fail),
                  static_cast<unsigned>(g_run.tx), static_cast<unsigned>(g_run.comp), failRate,
                  static_cast<unsigned>(g_run.maxGapUs), static_cast<unsigned>(g_run.maxDurUs),
                  static_cast<unsigned>(g_bus->eventTimeouts()), static_cast<unsigned>(g_run.recovered));
    }

    /**
     * `inject <n>`: 对不存在的地址发一次写 ⇒ 必然 NACK ⇒ 看接下来要几次尝试才恢复。
     * 这是 H1("AF 粘住导致连续失败")的决定性实验: attempts_to_ok==1 说明注入的这一次 NACK 没有留下
     * 后效; >1 说明确实要额外的事务(靠 recover() 或别的东西)才能把总线救回来。
     * 注入只写 0x33: **不碰 0x76 的任何寄存器**, 所以不会改变器件配置(不需要备份/恢复)。
     */
    static void cmdInject(uint32_t n)
    {
        // 与 soak 同样的门禁: 未初始化时 runAttempt() 返回的是 0x0403(无事务), attempts_to_ok 会恒为 1,
        // 判别力被清零 —— 宁可不跑, 也不产出"看起来正常"的假结果。
        if (!g_baro->isReady())
        {
            conPrintf("[err] not-initialized\r\n");
            return;
        }
        for (uint32_t i = 1u; i <= n; ++i)
        {
            const uint16_t preSr1 = kBaroI2c->SR1;
            const uint32_t preAf  = (preSr1 & I2C_SR1_AF) ? 1u : 0u;

            uint8_t    dummy = 0x00u;
            ByteBuffer w(&dummy, 1u);
            w.write(&dummy, 1u);
            IICDevice ghost(*g_bus, kGhostAddress);
            const bool writeOk = ghost.write(w);     // 期望 false(器件不存在 ⇒ 地址阶段就 NACK)
            const uint32_t nackOk = writeOk ? 0u : 1u;

            const uint16_t postSr1 = kBaroI2c->SR1;
            const uint32_t postAf  = (postSr1 & I2C_SR1_AF) ? 1u : 0u;

            // 恢复判定: 只认"事务是否通了"(transactionOk)。comp 类失败说明事务其实是通的, 所以它
            // 也算恢复; tx 类/其它错误码(如未初始化 0x0403, 根本没发事务)才算"还没恢复"。
            int32_t matches = -1;
            for (uint32_t k = 1u; k <= kInjectMaxAttempts; ++k)
            {
                const Attempt a = runAttempt(0u);
                if (transactionOk(a))
                {
                    matches = static_cast<int32_t>(k);
                    break;
                }
            }

            conPrintf("[inject] i=%u pre_sr1=0x%04X af=%u nack_ok=%u post_sr1=0x%04X post_af=%u next_ok=%u "
                      "attempts_to_ok=%d\r\n",
                      static_cast<unsigned>(i), static_cast<unsigned>(preSr1), static_cast<unsigned>(preAf),
                      static_cast<unsigned>(nackOk), static_cast<unsigned>(postSr1), static_cast<unsigned>(postAf),
                      static_cast<unsigned>(matches > 0 ? 1u : 0u), static_cast<int>(matches));
        }
    }

    static void cmdStress(bool on)
    {
        g_stressOn = on;
        if (on)
        {
            g_stressSpiOk   = probeSpi();
            g_stressI2c2Ok  = probeI2c2();
            g_stressAdcOk   = probeAdc();
            g_stressLastCyc = 0u;
        }
        // spi/i2c2/adc 的口径: "自这次 stress on 以来, 该子系统是否一直成功"。一旦某步失败就永久置 fail
        // (同时停掉该路后续步进, 见 stressService 的说明), 所以它是**累计**健康位而不是瞬时值。
        conPrintf("[stress] state=%s spi=%s i2c2=%s adc=%s\r\n", on ? "on" : "off",
                  g_stressSpiOk ? "ok" : "fail", g_stressI2c2Ok ? "ok" : "fail", g_stressAdcOk ? "ok" : "fail");
    }

    static void cmdIrqBlock(uint32_t us, uint32_t periodMs, bool on)
    {
        // 时钟不精确就**不启动、也不打 [irqblock] 行**(判定在打印之前): 保证"打印的 periodMs = 实际周期"
        // 这条不变量在任何 HCLK 配置下都成立。错误行沿用 [err] bad-arg 的形状, 附上实测的 timClock 便于定位。
        // 只有 `on` 才需要它 —— `off` 的语义是"停掉", 不依赖任何时钟, 必须照常可执行(否则一次误开就关不掉)。
        if (on)
        {
            uint32_t timClock = 0u;
            if (!irqTimerClockExact(timClock))
            {
                conPrintf("[err] bad-arg irqblock-clock timClock=%u\r\n", static_cast<unsigned>(timClock));
                return;
            }
        }
        g_irqBlockUs       = us;
        g_irqBlockPeriodMs = periodMs;
        g_irqBlockOn       = on;
        if (on)
        {
            startIrqBlockTimer();
        }
        else
        {
            stopIrqBlockTimer();
        }
        conPrintf("[irqblock] us=%u periodMs=%u state=%s\r\n", static_cast<unsigned>(us),
                  static_cast<unsigned>(periodMs), on ? "on" : "off");
    }

    static void cmdRecover()
    {
        const uint16_t sr1Before = kBaroI2c->SR1;
        const uint16_t sr2Before = kBaroI2c->SR2;
        g_bus->recover();
        const uint16_t sr1After = kBaroI2c->SR1;
        const uint16_t sr2After = kBaroI2c->SR2;
        const Attempt  a        = runAttempt(0u);
        // next_ok 同 inject: "事务通了"才算 ok(comp 类失败的事务本身是通的; 未初始化的 0x0403 没有事务)
        const uint32_t nextOk = transactionOk(a) ? 1u : 0u;
        conPrintf("[recover] sr1_before=0x%04X sr2_before=0x%04X sr1_after=0x%04X sr2_after=0x%04X next_ok=%u\r\n",
                  static_cast<unsigned>(sr1Before), static_cast<unsigned>(sr2Before),
                  static_cast<unsigned>(sr1After), static_cast<unsigned>(sr2After),
                  static_cast<unsigned>(nextOk));
    }

    static void cmdStat()
    {
        conPrintf("[stat] att=%u new=%u notready=%u fail=%u tx=%u comp=%u evTimeouts=%u recovered=%u "
                  "maxgapUs=%u maxdurUs=%u\r\n",
                  static_cast<unsigned>(g_all.att), static_cast<unsigned>(g_all.newData),
                  static_cast<unsigned>(g_all.notReady), static_cast<unsigned>(g_all.fail),
                  static_cast<unsigned>(g_all.tx), static_cast<unsigned>(g_all.comp),
                  static_cast<unsigned>(g_bus->eventTimeouts()), static_cast<unsigned>(g_all.recovered),
                  static_cast<unsigned>(g_all.maxGapUs), static_cast<unsigned>(g_all.maxDurUs));
    }

    static void cmdReset()
    {
        g_all.reset();
        g_run.reset();
        conPrintf("[reset] ok\r\n");
    }

    //===========================================================================================
    // 十二、命令解析
    //===========================================================================================
    char    g_line[96];
    uint8_t g_lineLen = 0;
    /** dispatch 会把行**就地转小写**(link bench 风格), 所以原文单独留一份给 bad-arg 用。 */
    char g_rawLine[96];

    static void badArg(const char *raw)
    {
        conPrintf("[err] bad-arg %s\r\n", (raw != nullptr) ? raw : "");
    }

    /** 解析十进制无符号参数。返回 false = 参数缺失/非数字/超上限(调用方打 bad-arg)。 */
    static bool parseU32(const char *s, uint32_t maxValue, uint32_t &out)
    {
        if (s == nullptr || *s == '\0')
        {
            return false;
        }
        uint32_t v = 0u;
        for (const char *p = s; *p != '\0'; ++p)
        {
            if (*p < '0' || *p > '9')
            {
                return false;
            }
            v = v * 10u + static_cast<uint32_t>(*p - '0');
            if (v > maxValue)
            {
                return false;
            }
        }
        out = v;
        return true;
    }

    /** 取下一个空格分隔的参数(把串内空格改成 '\0')。 */
    static char *nextArg(char **cursor)
    {
        char *p = *cursor;
        if (p == nullptr)
        {
            return nullptr;
        }
        while (*p == ' ') ++p;
        if (*p == '\0')
        {
            *cursor = p;
            return nullptr;
        }
        char *start = p;
        while (*p != '\0' && *p != ' ') ++p;
        if (*p == ' ')
        {
            *p++ = '\0';
        }
        *cursor = p;
        return start;
    }

    static bool isOnOff(const char *s, bool &on)
    {
        if (s == nullptr)
        {
            return false;
        }
        if (strcmp(s, "on") == 0)
        {
            on = true;
            return true;
        }
        if (strcmp(s, "off") == 0)
        {
            on = false;
            return true;
        }
        return false;
    }

    static void printHelp()
    {
        conPrintf("ver | help | cfg | sr | bus\r\n");
        conPrintf("init [hz] | soak <ms> [period_ms] | inject <n> | speed <hz>\r\n");
        conPrintf("stress <on|off> | irqblock <us> <period_ms> <on|off> (us<period_ms*1000, period_ms<=65) | recover | stat | reset\r\n");
    }

    static void dispatch(char *line)
    {
        while (*line == ' ') ++line;
        char *end = line + strlen(line);
        while (end > line && end[-1] == ' ') --end;
        *end = '\0';
        if (*line == '\0')
        {
            return; // 空行: 只回一个提示符
        }
        // 大小写不敏感 + 下划线当空格(照抄 link_bench_main.cpp:1015-1020)
        for (char *p = line; *p != '\0'; ++p)
        {
            if (*p == '_') *p = ' ';
            else if (*p >= 'A' && *p <= 'Z') *p = static_cast<char>(*p - 'A' + 'a');
        }

        char *cur = line;
        char *cmd = nextArg(&cur);
        if (cmd == nullptr)
        {
            return;
        }

        uint32_t v1 = 0u;
        uint32_t v2 = 0u;
        bool     on = false;

        if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
        {
            printHelp();
        }
        else if (strcmp(cmd, "ver") == 0 || strcmp(cmd, "version") == 0)
        {
            conPrintf("[ver] %s\r\n", kBenchVersion);
        }
        else if (strcmp(cmd, "cfg") == 0)
        {
            printCfg();
        }
        else if (strcmp(cmd, "sr") == 0)
        {
            printSr();
        }
        else if (strcmp(cmd, "bus") == 0)
        {
            printBus();
        }
        else if (strcmp(cmd, "init") == 0)
        {
            char *hzArg = nextArg(&cur);
            if (hzArg == nullptr)
            {
                cmdInit(false, 0u); // 不带参数: 用当前速率初始化
            }
            else if (parseU32(hzArg, kSpeedMaxHz, v1) && v1 >= kSpeedMinHz)
            {
                cmdInit(true, v1);
            }
            else
            {
                badArg(g_rawLine);
            }
        }
        else if (strcmp(cmd, "soak") == 0)
        {
            char *msArg     = nextArg(&cur);
            char *periodArg = nextArg(&cur);
            if (!parseU32(msArg, kSoakMaxMs, v1) || v1 == 0u)
            {
                badArg(g_rawLine);
            }
            else if (periodArg == nullptr)
            {
                cmdSoak(v1, kSoakDefaultPeriod);
            }
            else if (parseU32(periodArg, kSoakMaxPeriodMs, v2) && v2 != 0u)
            {
                cmdSoak(v1, v2);
            }
            else
            {
                badArg(g_rawLine);
            }
        }
        else if (strcmp(cmd, "inject") == 0)
        {
            if (parseU32(nextArg(&cur), kInjectMaxN, v1) && v1 != 0u)
            {
                cmdInject(v1);
            }
            else
            {
                badArg(g_rawLine);
            }
        }
        else if (strcmp(cmd, "speed") == 0)
        {
            if (parseU32(nextArg(&cur), kSpeedMaxHz, v1) && v1 >= kSpeedMinHz)
            {
                cmdSpeed(v1);
            }
            else
            {
                badArg(g_rawLine);
            }
        }
        else if (strcmp(cmd, "stress") == 0)
        {
            if (isOnOff(nextArg(&cur), on))
            {
                cmdStress(on);
            }
            else
            {
                badArg(g_rawLine);
            }
        }
        else if (strcmp(cmd, "irqblock") == 0)
        {
            char *usArg     = nextArg(&cur);
            char *periodArg = nextArg(&cur);
            char *stateArg  = nextArg(&cur);
            /**
             * 两个校验都要有:
             *   范围:  us ≤ kIrqBlockMaxUs, periodMs ∈ [1, kIrqBlockMaxPeriodMs=65] —— 上限是 TIM7 的
             *          16 位 ARR 决定的(见该常量的注释), 超了会被硬件静默截断。
             *   关系:  us < periodMs×1000 —— ISR 里忙等 us 微秒、每 periodMs 毫秒一次, 若 us 占满整个
             *          周期, 中断会吃掉几乎全部 CPU: soak 期间连按键都中止不了, 量到的全成了台架自己的伪影。
             *          (periodMs ≤ 65 ⇒ periodMs×1000 ≤ 65000, 不会溢出。)
             */
            if (parseU32(usArg, kIrqBlockMaxUs, v1) && parseU32(periodArg, kIrqBlockMaxPeriodMs, v2) && v2 != 0u &&
                v1 < v2 * 1000u && isOnOff(stateArg, on))
            {
                cmdIrqBlock(v1, v2, on);
            }
            else
            {
                badArg(g_rawLine);
            }
        }
        else if (strcmp(cmd, "recover") == 0)
        {
            cmdRecover();
        }
        else if (strcmp(cmd, "stat") == 0)
        {
            cmdStat();
        }
        else if (strcmp(cmd, "reset") == 0)
        {
            cmdReset();
        }
        else
        {
            // 契约只有一种错误输出行, 未知命令也按它走(不让解析器看到契约外的 tag)
            badArg(g_rawLine);
        }
    }

    /** 从控制台环形缓冲取一行并执行(一次一行, 与 link_bench_main.cpp:1097-1123 同款)。 */
    static void serviceConsole()
    {
        while (true)
        {
            uint8_t b = 0u;
            if (!g_conRing->read(b))
            {
                return;
            }
            if (b == '\r')
            {
                continue;
            }
            if (b == '\n')
            {
                g_line[g_lineLen] = '\0';
                // 原文先留一份: dispatch 会把 g_line 就地转小写, bad-arg 要打"原文"
                const uint16_t rawLen = (g_lineLen < sizeof(g_rawLine) - 1u) ? g_lineLen : (sizeof(g_rawLine) - 1u);
                memcpy(g_rawLine, g_line, rawLen);
                g_rawLine[rawLen] = '\0';
                dispatch(g_line);
                g_lineLen = 0;
                printPrompt();
                continue;
            }
            if (g_lineLen < sizeof(g_line) - 1u)
            {
                g_line[g_lineLen++] = static_cast<char>(b);
            }
        }
    }
} // namespace

//===================================================================================================
// 十三、main
//===================================================================================================
int main()
{
    // ① 中断分组 + DWT 微秒时基(168MHz ⇒ 168 周期 = 1us; SystemCoreClock 由 SystemInit 配好)
    DLX_NVIC_AutoConfig();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // ② USART1 控制台(PA9/PA10, 115200 8N1)
    static uint8_t       conRxStorage[kConsoleRxBytes];
    static ByteBuffer    conRxView(conRxStorage, sizeof(conRxStorage));
    static RingByteBuffer conRing(conRxView);
    g_conRing = &conRing;
    static USART con = USART::USART1_TA9_RAA();
    con.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, kConsoleBaud, conRing);
    g_con = &con;

    // ③ I2C1 + BMP388(被测对象)。顺序: 先配引脚/外设, 再构造驱动(驱动的构造函数只存引用, 不碰总线)
    static IICBus bus = IICBus::IIC1_SB6_DB7();
    bus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, cfg::hw::baro::kSpeed, kI2CSelfAddress);
    g_bus     = &bus;
    g_speedHz = cfg::hw::baro::kSpeed;
    static BMP388 baro(bus, cfg::hw::baro::kAddr);
    g_baro = &baro;

    // ④ 总线电平探针(PB6/PB7)。GPIO 构造函数只存 profile, 这里**不调 init()** —— 引脚已经是 AF4,
    //    重新 init 成输入会破坏 I2C 复用(见 printBus 的说明)。
    static GPIO sclPin(cfg::hw::baro::kScl);
    static GPIO sdaPin(cfg::hw::baro::kSda);
    g_sclPin = &sclPin;
    g_sdaPin = &sdaPin;

    // ⑤ stress 用的对象: 这里只构造, 不 init —— 三条负载都要等 `stress on` 才碰总线/引脚
    static SPI          spiImu{SPI::SPI1_SA5_MIA6_MOA7()};
    static GPIO         accelCs(cfg::hw::imu::kAccelCs);
    static GPIO         xshut(cfg::hw::range_::kXshut);
    static IICBus       rangeBus = IICBus::IIC2_SF1_DF0();
    static IICDevice    rangeDev(rangeBus, cfg::hw::range_::kAddr);
    static ADC1Sampler  adc;
    g_spiImu  = &spiImu;
    g_accelCs = &accelCs;
    g_xshut   = &xshut;
    g_rangeBus = &rangeBus;
    g_rangeDev = &rangeDev;
    g_adc      = &adc;

    // ⑥ banner: **第一行必须是版本行**(自动化脚本靠它判断板上是不是最新 bench, 见文件头)
    con.send(reinterpret_cast<const uint8_t *>("\r\n\r\n"), 4);
    conPrintf("[ver] %s\r\n", kBenchVersion);
    conPrintf("=== DLX baro bench (BMP388 / I2C1 PB6/PB7) ===\r\n");
    printCfg();
    printSr();
    printBus();
    conPrintf("type 'help'\r\n");

    /**
     * ⑦ 上电自动初始化一次(纵深防御, 2026-10-18 按 lead 的 D1 要求加)。
     *
     * 为什么必须有: 初始版本只把"初始化"留给 `init`/`speed` 命令, 而 run_baro.py 的默认流程不发 `init`
     * ⇒ 器件一直 `!initialized`, `read()` 在 dlx_bmp388.hpp:497-500 直接返回 0x0403(一个寄存器都没读),
     * 于是健康板子的基线 soak 也会报 fail≈att, 而且会被旧判据误记成 kind=comp —— 签名与真结论相反。
     *
     * 为什么只做**一次** tryInit(不用 init() 的 kInitRetry×kInitRetryMs 退避): run_baro.py 的串口探测
     * 只给 2.0s 等 `help` 的回答(run_baro.py:839), 20 次重试最长 2s+ 会把探测拖超时。单次 tryInit
     * 健康器件 ~3ms, 器件不应答时也有界(一次的 waitEvent 上限 ≈ 几十 ms, 且 busError 会短路后续步骤)。
     * 需要完整重试时用 `init` 命令。
     *
     * 行首用 `[boot]` 而**不是** `[init]`: 后者会和 `init` 命令的解析撞车(run_baro.py 的 _RE_INIT)。
     */
    {
        ExceptionCode bootErr = ExceptionCode::EXCEPTION_NONE;
        const bool    bootOk  = (g_baro->tryInit(bootErr) == FunctionResult::SUCCESS);
        conPrintf("[boot] baro init=%s err=0x%04X\r\n", bootOk ? "OK" : "FAIL",
                  static_cast<unsigned>(static_cast<uint16_t>(bootErr)));
    }
    printPrompt();

    // ⑧ 主循环: 控制台是唯一输入; 命令都是阻塞式测量(长循环内部检查 consoleAvailable 以便中止)
    while (true)
    {
        serviceConsole();
        stressService(); // stress 开启时, 停在提示符也要继续造负载
    }
}

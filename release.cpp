//===================================================================================================
// release.cpp —— 发布版主程序(含 main)
//
// 与 main_final_test.cpp(首飞测试版)相比, 流程结构保持一致, 但这几块换掉了:
//   1. flash 日志换成 FlashManager 文件系统: 一次上电 = 一个会话, 默认预留 32 槽(≈2.8 万条,
//      50Hz 记一条约 9.4 分钟); 上电只擦需要回收的槽, 支持 列会话/删最老会话/整会话回传/整片格式化;
//   2. 姿态估计换成 16 维 EKF(FlightControlFilter), 并引入 BME280 气压高度作为高度/垂速观测
//      (与 main_att_ekf.cpp 同一套做法: 快路径每个 IMU 样本积分, 慢路径每 N 个样本传播协方差+倾角更新);
//   3. 上电模式判定解耦成 decideBootMode(): 跳线帽(PB8 & PB9)没接 -> 直接飞行且**忽略串口**;
//      接上 -> 等上位机"解锁"命令, 命令参数决定进 串口调试 还是 日志管理;
//   4. 日志内容 = LogEntry(见 dlx_flash_manager_config.h), 串口调试模式额外回 DWT 负载统计。
//
// 开关:
//   DLX_BARO_ENABLED(定义在 flight_config_struct.hpp, 默认 1): 置 0 = 编译期裁掉整个气压计 ——
//   不初始化 I2C1/BME280、不驱动 SDO 地址脚、EKF 只吃 IMU 积分(高度/垂速会漂、定高不可用)、
//   遥测 height 字段退回"高度"语义, 且不报气压计故障(主动关掉 ≠ 坏掉)。
//
// 文件分工:
//   flight_config_struct.hpp : 全部编译期配置 + 运行时状态 + 地面模式报文约定 + 共用上下文/小工具
//   release_log_manage.hpp   : 日志管理模式(与飞行完全分离, 永不输出电机)
//   release.cpp(本文件)      : main + 上电模式判定 + 飞行/串口调试主流程
//
// 三种模式:
//   Flight      : 正常飞行。日志写 flash, 不处理串口;
//   SerialDebug : 台架调试。日志(与 flash 同格式的原始帧)直接走串口, 另发 PerfMonitor 负载统计;
//                 可以解锁电机(遥控指令照常);
//   LogManage   : 日志管理。见 release_log_manage.hpp。
//
// 线程结构(与首飞版一致):
//   主线程       : 控制环 500Hz(2ms) —— 读 IMU FIFO -> EKF -> 控制器 -> 混控 -> DShot, 日志入队;
//   nrf_tel 线程 : 50Hz —— 收遥控指令(写共享状态) + 回遥测;
//   logger 线程  : 5ms —— 排空日志队列 -> 写 flash(飞行模式) / 打包发串口(调试模式)。
//
// 接线(引脚/SPI 模式/设备地址都在 kConfig.hw 里, 见 flight_config_struct.hpp):
//   USART1 PA9/PA10 2Mbaud | BMI088 SPI1 PA5/6/7 CS=PC4/PC5 | NRF24L01 SPI3 CE=PC7 CSN=PC8
//   BME280 I2C1 PB6/PB7 0x76 | W25Q128 SPI2 CS=PD1 | 电机 TIM1 PE9/11/13/14 DShot300
//   LED PF3(心跳) / PF2(擦除) | 跳线帽 PB8 & PB9(接上 = 地面模式)
//===================================================================================================
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "dlx_spi.hpp"
#include "dlx_iic.hpp"
#include "dlx_dma.hpp"
#include "dlx_delay.hpp"
#include "dlx_dshot.hpp"
#include "dlx_timer.hpp"
#include "rtthread.h"
#include "BMI088/dlx_bmi088.hpp"
#include "BME280/dlx_bme280.hpp"
#include "NRF24L01/dlx_nrf24l01.hpp"
#include "flight_control/flight_controller.hpp"
#include "flight_control/flight_control_mixer.hpp"
#include "flight_control/flight_control_fliter.hpp"
#include "W25Q128/dlx_flash_manager.hpp" // FlashManager / W25Q128(头文件里只用前置声明)
#include "flight_config_struct.hpp"
#include "release_log_manage.hpp"

using namespace dlx;

/** 一次排空日志队列最多取多少条(别让日志线程长时间占着) */
constexpr uint16_t kMaxEntriesPerRun = 24;

//===================================================================================================
// 一、控制线程 -> 日志线程 的日志队列(单生产者单消费者)
//===================================================================================================
struct LogQueue {
    static constexpr uint16_t kCapacity = (uint16_t)kConfig.logQueueDepth; ///< 16 条 × 10ms = 160ms 缓冲
    LogEntry items[kCapacity];
    uint16_t head;    ///< 读位置(日志线程)
    uint16_t tail;    ///< 写位置(控制线程)
    uint16_t count;   ///< 当前条数
    uint16_t dropped; ///< 因为满而丢掉的条数(累计)
};

/** DWT 负载统计累加器(控制线程写; 日志线程只写 logSum/logCnt, 用临界区) */
struct PerfAccum {
    uint32_t loopSum, loopMax;
    uint32_t ekfSum, ekfCnt;
    uint32_t ctrlSum, ctrlCnt;
    uint32_t ioSum, ioCnt;
    uint32_t logSum, logCnt;
    uint16_t samples;
};

/** NRF 遥控线程上下文(50Hz) */
struct NrfThreadCtx {
    NRF24L01 *nrf;
    DLX_ProtocolBuffer *txProto;
    ByteBuffer *txBuf;
    DLX_ProtocolBuffer *rxProto;
    RingByteBuffer *rxRing;
    rt_sem_t sem;
};

/** 日志线程上下文(5ms 唤醒) */
struct LogThreadCtx {
    LogQueue *queue;
    FlashManager *fs; ///< 非空 = 写 flash(飞行模式)
    DMA *serialDma;   ///< 非空 = 发串口(串口调试模式)
    ByteBuffer *txBuf;
    DLX_ProtocolBuffer *protocol; ///< 负载统计报文走生成代码
    rt_sem_t sem;
    bool debugStream;     ///< true = 走串口
    uint32_t seq;         ///< 调试流的条目序号(flash 模式下由文件系统填)
    uint32_t lastPerfMs;  ///< 上次发负载统计的时间
    uint32_t lastStackMs; ///< 上次打印栈水位线的时间
};

static FlightRuntimeState g_rt;       ///< 跨线程飞行状态
static FlightControlFilter g_ekf;     ///< 16 维 EKF
static FlightController g_controller; ///< 姿态/角速度控制器(PID 见 flight_control_params.hpp)
static LogQueue g_logQueue;           ///< 日志队列
static PerfAccum g_perf;              ///< 负载统计累加器

static rt_thread_t g_tidMain = RT_NULL; ///< 主线程(控制环)
static rt_thread_t g_tidNrf  = RT_NULL; ///< nrf_tel 线程
static rt_thread_t g_tidLog  = RT_NULL; ///< logger 线程

//===================================================================================================
// 二、小工具
//===================================================================================================
static void logQueuePush(const LogEntry &e)
{
    rt_enter_critical();
    if (g_logQueue.count >= LogQueue::kCapacity) {
        g_logQueue.head  = static_cast<uint16_t>((g_logQueue.head + 1u) % LogQueue::kCapacity);
        g_logQueue.count = static_cast<uint16_t>(g_logQueue.count - 1u);
        ++g_logQueue.dropped;
    }
    g_logQueue.items[g_logQueue.tail] = e;
    g_logQueue.tail                   = static_cast<uint16_t>((g_logQueue.tail + 1u) % LogQueue::kCapacity);
    ++g_logQueue.count;
    rt_exit_critical();
}

static bool logQueuePop(LogEntry &out)
{
    bool ok = false;
    rt_enter_critical();
    if (g_logQueue.count > 0u) {
        out             = g_logQueue.items[g_logQueue.head];
        g_logQueue.head = static_cast<uint16_t>((g_logQueue.head + 1u) % LogQueue::kCapacity);
        --g_logQueue.count;
        ok = true;
    }
    rt_exit_critical();
    return ok;
}

static void perfAdd(volatile uint32_t &sum, volatile uint32_t &cnt, uint32_t cycles)
{
    sum += cycles;
    ++cnt;
}

/** 日志线程用: 加完再退临界区(控制线程会周期性清零这些累加器) */
static void perfAddLocked(volatile uint32_t &sum, volatile uint32_t &cnt, uint32_t cycles)
{
    rt_enter_critical();
    sum += cycles;
    ++cnt;
    rt_exit_critical();
}

//===================================================================================================
// 栈水位线测量(只在串口调试模式打印, 用来核对"静态估算 vs 实际用量")
//
//  做法: 线程刚起来时把自己"未使用"的那段栈填 0xA5(用 SP 以下的部分, 不会碰到正在用的栈帧),
//        之后扫描这层 0xA5: 从栈底往上第一个被写过的位置就是最深用过的位置。
//  为什么需要它: 静态分析(GCC -fstack-usage/-fcallgraph-info)给的是"最坏调用链"的保守值,
//        armclang 实际分配的栈帧可能不同, 上板量一次才知道真实水位。
//===================================================================================================
/** 读当前 SP(内联汇编, 用 SP 以下 64B 作为安全边界) */
static inline uint32_t readStackPointer()
{
    uint32_t sp;
    __asm volatile("mov %0, sp" : "=r"(sp));
    return sp;
}

/** 给当前线程的"未使用栈区"打上 0xA5 标记(线程刚启动时调用一次) */
static void paintStackWatermark()
{
    rt_thread_t self = rt_thread_self();
    if (self == RT_NULL || self->stack_addr == RT_NULL) {
        return;
    }
    uint8_t *base  = static_cast<uint8_t *>(self->stack_addr);
    uint8_t *limit = reinterpret_cast<uint8_t *>(readStackPointer()) - 64u;
    for (uint8_t *p = base; p < limit; ++p) {
        *p = 0xA5;
    }
}

/** 某个线程用掉多少栈(字节, 近似; 没打过水位线时返回 0) */
static uint32_t stackUsedBytes(rt_thread_t tid)
{
    if (tid == RT_NULL || tid->stack_addr == RT_NULL) {
        return 0u;
    }
    const uint8_t *base = static_cast<const uint8_t *>(tid->stack_addr);
    uint32_t n          = 0u;
    while (n < tid->stack_size && base[n] == 0xA5u) {
        ++n;
    }
    return (n >= tid->stack_size) ? 0u : (tid->stack_size - n);
}

//===================================================================================================
// 三、上电模式判定(与飞行逻辑解耦的独立函数)
//===================================================================================================
/**
 * @brief 跳线帽(PB8 & PB9)是否接上
 *
 * 两相检测, 兼容两种接法:
 *   - 一个跳线帽把 PB8 与 PB9 短接: 驱动一端为低, 另一端读回低 → 接上;
 *   - 两脚各自跳线到 GND: 任一脚被拉低 → 接上。
 * 检测期间两个脚一会儿输出一会儿输入(带上拉), 不会对外部电路灌大电流(只经过跳线)。
 */
static bool groundJumpersPresent()
{
    GPIO jumpA(kConfig.hw.jumpA);
    GPIO jumpB(kConfig.hw.jumpB);
    bool aLow = false;
    bool bLow = false;

    // 相位 1: A 输出低 + B 输入上拉 -> 读 B
    jumpA.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    jumpA = 0;
    jumpB.init(GPIOModeProfile::IN_UP);
    delay_us(50);
    bLow = !jumpB.read();

    // 相位 2: B 输出低 + A 输入上拉 -> 读 A
    jumpB.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    jumpB = 0;
    jumpA.init(GPIOModeProfile::IN_UP);
    delay_us(50);
    aLow = !jumpA.read();

    // 两脚恢复输入上拉(不驱动)
    jumpA.init(GPIOModeProfile::IN_UP);
    jumpB.init(GPIOModeProfile::IN_UP);
    return aLow || bLow;
}

/** 上电阶段收到的"解锁"命令(解析回调 -> 判定循环) */
struct BootCommand {
    volatile bool got;
    uint8_t mode;
};

static void onBootUnBlock(UnBlock *u, void *raw)
{
    BootCommand *b = static_cast<BootCommand *>(raw);
    b->mode        = u->param();
    b->got         = true;
}

static void onBootGroundCmd(GroundCmd *cmd, void *raw)
{
    BootCommand *b    = static_cast<BootCommand *>(raw);
    const uint16_t op = cmd->op();
    // op == 1 是现在的"解锁"; op == 0 是早期上位机用过的写法, 一并兼容
    if (op == static_cast<uint16_t>(GroundOp::Unlock) || op == 0u) {
        b->mode = static_cast<uint8_t>(cmd->mode());
        b->got  = true;
    } else {
        b->mode = 0xFF; // 非解锁命令: 当作无效
        b->got  = true;
    }
}

/**
 * @brief 决定上电后进哪个模式
 *
 *   跳线帽没接   -> 直接飞行模式(不读串口, 上电即能飞);
 *   跳线帽接上了 -> 停在"地面模式", 等上位机的"解锁"命令:
 *                    - 新报文 GroundCmd(op=Unlock, mode=0/1/2)
 *                    - 或者旧报文 UnBlock(param=1 串口调试 / 2 日志管理), 兼容首飞版操作习惯
 *                  命令里的模式就是最终模式; 超时(默认 0 = 一直等)则停在**日志管理**模式(安全, 不动电机)。
 */
static BootDecision decideBootMode(ReleaseContext &ctx)
{
    BootDecision decision = {BootMode::Flight, false, false, 0};
    decision.groundMode   = groundJumpersPresent();
    if (!decision.groundMode) {
        return decision; // 没接跳线: 忽略串口, 直接飞
    }

    logLine("\r\n[Boot] 检测到地面模式跳线(PB8 & PB9), 等待上位机解锁命令...\r\n");
    BootCommand cmd = {false, 0};
    ctx.protocol->setUnBlockCallbackFunction(onBootUnBlock, &cmd);
    ctx.protocol->setGroundCmdCallbackFunction(onBootGroundCmd, &cmd);

    // 丢掉上电/开串口瞬间可能进来的杂散字节(某些 USB 转串口在打开端口时会抖出一个字节),
    // 否则后面的帧会整体错位一个字节, 解析出一堆"看着合法"的垃圾。
    ctx.uartRx->reset();

    const uint32_t startMs = rt_tick_get_millisecond();
    uint32_t lastBlinkMs   = startMs;
    bool ledOn             = false;

    while (true) {
        const uint32_t now = rt_tick_get_millisecond();
        if ((now - lastBlinkMs) >= kConfig.groundBlinkMs) {
            lastBlinkMs = now;
            ledOn       = !ledOn;
            *ctx.ledRun = ledOn ? 1 : 0;
        }

        drainDlx(*ctx.protocol, *ctx.uartRx);
        if (cmd.got) {
            cmd.got = false;
            if (cmd.mode <= 2u) {
                decision.mode      = static_cast<BootMode>(cmd.mode);
                decision.byCommand = true;
                decision.rawParam  = cmd.mode;
                logLine("[Boot] 收到解锁命令: mode=%u\r\n", static_cast<unsigned>(cmd.mode));
                return decision;
            }
            logLine("[Boot] 命令无效(param=%u), 继续等待\r\n", static_cast<unsigned>(cmd.mode));
            replyGround(ctx, static_cast<uint16_t>(GroundOp::Unlock), GroundStatus::BadArgument, 0, 0);
            continue;
        }

        if (kConfig.groundWaitTimeoutMs != 0u && (now - startMs) > kConfig.groundWaitTimeoutMs) {
            logLine("[Boot] 等待超时, 进入日志管理模式(不动电机)\r\n");
            decision.mode = BootMode::LogManage;
            return decision;
        }
    }
}

//===================================================================================================
// 四、多线程: NRF 遥控/遥测 + 日志
//===================================================================================================
static void nrfTelThreadEntry(void *arg)
{
    NrfThreadCtx *ctx = static_cast<NrfThreadCtx *>(arg);
    uint8_t nrfPkt[NRF24L01_RX_PACKET_SIZE];
    paintStackWatermark(); // 记录本线程栈水位线(调试模式会打印)
    while (true) {
        rt_sem_take(ctx->sem, RT_WAITING_FOREVER);

        // ---- 回遥测(姿态/电机/高度) ----
        Quaternion quat;
        uint16_t motor[4];
        bool enabled;
        float height;
        float vvel;
        uint8_t status;
        uint8_t error;
        rt_enter_critical();
        quat      = g_rt.telem.quat;
        enabled   = g_rt.telem.enabled;
        height    = g_rt.telem.height_m;
        vvel      = g_rt.telem.vertical_vel_mps;
        status    = g_rt.telem.status;
        error     = g_rt.telem.error;
        for (int i = 0; i < 4; ++i) {
            motor[i] = g_rt.telem.motor[i];
        }
        rt_exit_critical();

        // FlightCoreStatus 的结构体没变(最后一个 float 仍叫 height), 但语义按约定改为:
        //   气压计有效 -> 发 EKF 估计的垂向速度; 气压计不可用 -> 退回发高度
        //   字段名不改; 判断装的是哪一个必须看 status 的 kTelStatusBaroOk, 不要用 error
        //   的 kTelErrBaro(它俩虽然同源, 但 status 才是有"气压计此刻可用"语义的那个字段)。
        const float heightOrVz = (status & kTelStatusBaroOk) ? vvel : height;

        ctx->txBuf->reset();
        if (ctx->txProto->FlightCoreStatusW(
                quat.data[0], quat.data[1], quat.data[2], quat.data[3], enabled ? motor[0] : 0,
                enabled ? motor[1] : 0, enabled ? motor[2] : 0, enabled ? motor[3] : 0, heightOrVz,
                status, error)) {
            ctx->nrf->write(ctx->txBuf->src, ctx->txBuf->used(), NRF24L01_WAIT_TIMEOUT_MS);
        }

        // ---- 收遥控指令 ----
        if (ctx->nrf->read(nrfPkt, NRF24L01_RX_PACKET_SIZE, NRF24L01_WAIT_TIMEOUT_MS)) {
            ctx->rxRing->write(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            while (ctx->nrf->available() >= NRF24L01_RX_PACKET_SIZE) {
                if (!ctx->nrf->read(nrfPkt, NRF24L01_RX_PACKET_SIZE, 0)) {
                    break;
                }
                ctx->rxRing->write(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            }
        }
        while (ctx->rxRing->available() >= NRF24L01_RX_PACKET_SIZE) {
            const uint16_t       before = ctx->rxRing->available();
            const DLXCheckResult result = ctx->rxProto->check();
            if (result == DLXCheckResult::SUCCESS) {
                rt_enter_critical();
                g_rt.lastValidCtrl = rt_tick_get_millisecond();
                rt_exit_critical();
                continue;
            }
            if (ctx->rxRing->available() == before) {
                // 一个字节都没吞(半帧 / 纯噪声): 丢一包, 免得卡住(也不能只丢 1 字节,
                // 否则和 NRF 的 32B 包边界错位)
                ctx->rxRing->read(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            }
        }
    }
}

static void logThreadEntry(void *arg)
{
    LogThreadCtx *ctx = static_cast<LogThreadCtx *>(arg);
    LogEntry entry;
    uint32_t lastBlinkMs = rt_tick_get_millisecond();
    bool ledOn           = false;
    GPIO ledRun(kConfig.hw.ledRun);
    ledRun.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    paintStackWatermark(); // 记录本线程栈水位线(下面每 10s 打印一次)
    ctx->lastStackMs = rt_tick_get_millisecond();

    while (true) {
        rt_sem_take(ctx->sem, RT_WAITING_FOREVER);

        // ---- 心跳灯: 500ms 翻转一次, 说明系统在跑 ----
        const uint32_t nowMs = rt_tick_get_millisecond();
        if ((nowMs - lastBlinkMs) >= 500u) {
            lastBlinkMs = nowMs;
            ledOn       = !ledOn;
            ledRun      = ledOn ? 1 : 0;
        }

        if (ctx->fs != nullptr) {
            // ---------------- 飞行模式: 写 flash ----------------
            uint16_t n = 0;
            while (n < kMaxEntriesPerRun && logQueuePop(entry)) {
                ExceptionCode err      = ExceptionCode::FLASH_OK;
                const uint32_t t0      = DWT->CYCCNT;
                const FunctionResult r = ctx->fs->appendLog(entry, err);
                perfAddLocked(g_perf.logSum, g_perf.logCnt, DWT->CYCCNT - t0);
                if (r != FunctionResult::SUCCESS) {
                    g_rt.logError = true;
                    if (err == ExceptionCode::FLASH_LOG_FULL) {
                        g_rt.logFull = true;
                    }
                } else {
                    g_rt.logEntries = ctx->fs->currentEntryCount();
                }
                ++n;
            }
        } else if (ctx->debugStream) {
            // ---------------- 串口调试模式: 日志(DLX FlightLog) + 负载统计 ----------------
            // 上一次发完(只在我们启动过传输时才等; 见 flight_config_struct.hpp 的 serialDmaBusy), 缓冲可以改写
            if (serialDmaBusy()) {
                ctx->serialDma->wait();
                serialDmaBusy() = false;
            }
            ctx->txBuf->reset();
            uint16_t n = 0;
            while (n < kMaxEntriesPerRun && ctx->txBuf->remaining() >= kFlightLogFrameBytes && logQueuePop(entry)) {
                entry.sessionId   = 0u;         // 不写 flash, 没有会话号
                entry.seq         = ++ctx->seq; // 用自增序号代替
                const uint32_t t0 = DWT->CYCCNT;
                writeLogEntry(*ctx->protocol, entry);
                perfAddLocked(g_perf.logSum, g_perf.logCnt, DWT->CYCCNT - t0);
                ++n;
            }
            if ((nowMs - ctx->lastPerfMs) >= kConfig.perfReportMs) {
                ctx->lastPerfMs = nowMs;
                PerfStats perf;
                rt_enter_critical();
                perf = g_rt.perf;
                rt_exit_critical();
                // 走 KSP 生成的报文(追加在日志流后面)
                ctx->protocol->PerfMonitorW(perf.loopAvgCycle, perf.loopMaxCycle, perf.ekfCycle,
                                            perf.ctrlCycle, perf.logCycle, perf.ioCycle,
                                            perf.samples, perf.drops);
            }
            // 每 10s 打一行栈水位线: 默认关, 量栈预算时把 kConfig.logStackWatermark 打开
            // (静态估算见 notebook/2026-09-11_release_item.md §8)
            if (kConfig.logStackWatermark && (nowMs - ctx->lastStackMs) >= 10000u) {
                ctx->lastStackMs = nowMs;
                char text[128];
                const int n = snprintf(text, sizeof(text),
                                       "stack used: main=%u/%u nrf=%u/%u logger=%u/%u\n",
                                       static_cast<unsigned>(stackUsedBytes(g_tidMain)),
                                       static_cast<unsigned>(g_tidMain ? g_tidMain->stack_size : 0u),
                                       static_cast<unsigned>(stackUsedBytes(g_tidNrf)),
                                       static_cast<unsigned>(g_tidNrf ? g_tidNrf->stack_size : 0u),
                                       static_cast<unsigned>(stackUsedBytes(g_tidLog)),
                                       static_cast<unsigned>(g_tidLog ? g_tidLog->stack_size : 0u));
                if (n > 0 && ctx->txBuf->remaining() > (uint16_t)(n + 4)) {
                    ctx->protocol->SimpleLogW(reinterpret_cast<int8_t *>(text),
                                              static_cast<uint16_t>(n));
                }
            }
            if (ctx->txBuf->used() > 0u) {
                ctx->serialDma->reset(ctx->txBuf->used());
                ctx->serialDma->start();
                serialDmaBusy() = true;
            }
        } else {
            // 没有日志出口(flash 坏了): 只丢队列, 别卡住控制线程
            while (logQueuePop(entry)) {
            }
        }
    }
}

//===================================================================================================
// 五、飞行 / 串口调试主流程(两种模式共用, 只是日志出口不同)
//===================================================================================================
static void runFlightStack(ReleaseContext &app)
{
    const bool flightMode = (app.mode == BootMode::Flight);

    // ---- 1. 传感器与执行器 ----
    auto spiImu = SPI::SPI1_SA5_MIA6_MOA7(kConfig.hw.imuSpiNss);
    spiImu.init(kConfig.hw.imuSpi);
    auto bmi         = BMI088(spiImu, kConfig.hw.imuAccelCs, kConfig.hw.imuGyroCs);
    const bool bmiOk = (bmi.init() != 0);

    // ---- BME280 气压计(总开关 DLX_BARO_ENABLED, 见 flight_config_struct.hpp) ----
#if DLX_BARO_ENABLED
    IICBus bus = IICBus::IIC1_SB6_DB7();
    bus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, kConfig.hw.bmeSpeed, 13);
    BME280 bme(bus, kConfig.hw.bmeAddr);
    // 气压计初始化: 失败就重试(开机那段 flash 擦写的干扰可能让它一时不应答, 见 FlightConfig 注释)
    bool baroOk = false;
    for (uint32_t attempt = 1u; attempt <= kConfig.baroInitRetry && !baroOk; ++attempt) {
        baroOk = bme.init();
        if (!baroOk) {
            rt_thread_mdelay(kConfig.baroInitRetryMs);
        }
    }
    bool     baroHealthy    = baroOk; ///< 运行时是否还有效(连续读失败会翻成 false, 成功后恢复)
    uint32_t baroFailStreak = 0;      ///< 连续读失败计数(到 baroFailLimit 就认定掉线)
#else
    // 主动禁用: 不碰 I2C1/BME280, 直接按"气压计不可用"走纯 IMU 那条路(后面气压相关的代码都裁掉)
    const bool baroOk      = false;
    bool       baroHealthy = false;
#endif

    auto spiNrf = SPI::SPI3_SCA_MICB_MOCC(kConfig.hw.nrfSpiNss);
    spiNrf.init(kConfig.hw.nrfSpi);
    uint8_t nrfRxRaw[128];
    ByteBuffer nrfRxByteBuf(nrfRxRaw, sizeof(nrfRxRaw));
    NRF24L01 nrf(spiNrf, kConfig.hw.nrfCe, kConfig.hw.nrfCsn, nrfRxByteBuf);
    nrf.init();

    auto timer1 = AdvancedTimer(kConfig.hw.motorTimer);
    auto dshot  = timer1.setDshot(kConfig.hw.dshotMode);

    logLine("[Boot] BMI088=%s BME280=%s flash=%s\r\n", bmiOk ? "OK" : "FAIL",
            !kBaroEnabled ? "OFF" : (baroOk ? "OK" : "FAIL"), (app.fs != nullptr) ? "OK" : "-");

    // ---- 2. EKF / 气压计基准 ----
    // 整定与 main_att_ekf.cpp 实测一致: 纯 IMU 不估加速度零偏(不可观), 倾角门限放宽, 气压噪声放宽
    FlightControlFilterParams ekfParams = kConfig.ekf;
    ekfParams.estimate_accel_bias       = kConfig.ekfEstimateAccelBias;
    ekfParams.accel_tilt_gate_mss       = kConfig.ekfAccelTiltGateMss;
    ekfParams.baro_sigma_m              = kConfig.ekfBaroSigmaM;
    g_ekf.set(ekfParams);
    g_ekf.reset();

    float baroAbs   = 0.0f; ///< 最近一次绝对气压高度 [m]
    float baroRel   = 0.0f; ///< 相对起飞点的高度 [m]
    float baroRef   = 0.0f; ///< 起飞点气压高度基准
    bool baroRefSet = false;
    // 解锁后重取气压基准: 用启动序列(默认 3.2s)里那 20ms 一次的采样慢慢平均,
    // 不在控制环里阻塞等待(见 5.5)
    bool     baroZeroPending = false; ///< 正在为本次起飞累积气压基准
    float    baroZeroSum     = 0.0f;  ///< 累积和 [m]
    uint16_t baroZeroCnt     = 0;     ///< 已累积样本数
#if DLX_BARO_ENABLED
    if (baroOk) {
        // 多采样几次求平均
        for (uint8_t i = 0; i < 16; i++) {
            bool     ok  = false;
            const EnviromentRaw raw = bme.getRaw(&ok);
            if (ok) { // I2C 读失败就不算进平均(否则会拿 0 当气压值)
                baroAbs += bme.getAltitude(raw);
            }
            rt_thread_mdelay(50);
        }
        baroAbs /= 16;
        baroRef    = baroAbs;
        baroRefSet = true;
    }
#endif

    // ---- 3. 遥控回调: 只写共享状态, 硬件操作留给控制线程 ----
    uint8_t nrfDummyRaw[64];
    ByteBuffer nrfDummyByteBuf(nrfDummyRaw, sizeof(nrfDummyRaw));
    uint8_t nrfFrRaw[36];
    ByteBuffer nrfFrame(nrfFrRaw, sizeof(nrfFrRaw));
    uint8_t nrfRingRaw[256];
    ByteBuffer nrfRingBuf(nrfRingRaw, sizeof(nrfRingRaw));
    RingByteBuffer nrfRing(nrfRingBuf);
    DLX_ProtocolBuffer nrfRxProtocol(nrfDummyByteBuf, &nrfRing, &nrfFrame);
    nrfRxProtocol.setControlToFlightCallbackFunction(
        +[](ControlToFlight *ctl, void *) {
            rt_enter_critical();
            g_rt.command = ctl->command();
            if (g_rt.command == 1) { // 直接给电调值(台架)
                g_rt.motorValue[0] = ctl->motor0();
                g_rt.motorValue[1] = ctl->motor1();
                g_rt.motorValue[2] = ctl->motor2();
                g_rt.motorValue[3] = ctl->motor3();
            }
            if (g_rt.command == 0) { // 自稳: 手动油门 + 期望姿态(只取俯仰/横滚, 偏航不控)
                g_rt.manualThrottle    = ctl->throttle();
                g_rt.setpoint.attitude = quatFromEulerZYX(0.0f, ctl->targetPitch() * kDegToRad,
                                                          ctl->targetRoll() * kDegToRad);
                g_rt.setpoint.height_m = ctl->targetHeight();
            }
            if (g_rt.command == 2) {
                g_rt.startMotor = true;
            }
            if (g_rt.command == 3) {
                g_rt.stopMotor = true;
            } else if (g_rt.command > 3) { // 硬停机
                g_rt.stopMotor = true;
                g_rt.hardStop  = true;
            }
            rt_exit_critical();
        },
        nullptr);
    uint8_t nrfTxRaw[36];
    ByteBuffer nrfTxByteBuf(nrfTxRaw, sizeof(nrfTxRaw));
    DLX_ProtocolBuffer nrfTxProtocol(nrfTxByteBuf);

    // ---- 4. 线程与定时器 ----
    auto ctrlSem   = rt_sem_create("ctrl", 0, RT_IPC_FLAG_PRIO);
    auto ctrlTimer = rt_timer_create("ctrlt", +[](void *s) { rt_sem_release(static_cast<rt_sem_t>(s)); }, ctrlSem, kConfig.ctrlTickMs, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);

    auto telSem         = rt_sem_create("tel", 0, RT_IPC_FLAG_PRIO);
    auto telTimer       = rt_timer_create("telt", +[](void *s) { rt_sem_release(static_cast<rt_sem_t>(s)); }, telSem, kConfig.telTickMs, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);
    NrfThreadCtx nrfCtx = {&nrf, &nrfTxProtocol, &nrfTxByteBuf, &nrfRxProtocol, &nrfRing, telSem};
    auto nrfTid         = rt_thread_create("nrf_tel", nrfTelThreadEntry, &nrfCtx, 3072, 24, 20);
    if (nrfTid != RT_NULL) {
        rt_thread_startup(nrfTid);
    }

    auto logSem         = rt_sem_create("log", 0, RT_IPC_FLAG_PRIO);
    auto logTimer       = rt_timer_create("logt", +[](void *s) { rt_sem_release(static_cast<rt_sem_t>(s)); }, logSem, kConfig.logTickMs, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);
    LogThreadCtx logCtx = {};
    logCtx.queue        = &g_logQueue;
    logCtx.fs           = flightMode ? app.fs : nullptr;
    logCtx.serialDma    = app.serialDma;
    logCtx.txBuf        = app.txBuf;
    logCtx.protocol     = app.protocol;
    logCtx.sem          = logSem;
    logCtx.debugStream  = !flightMode;
    auto logTid         = rt_thread_create("logger", logThreadEntry, &logCtx, 3072, 26, 20);
    if (logTid != RT_NULL) {
        rt_thread_startup(logTid);
    }
    g_tidNrf = nrfTid;
    g_tidLog = logTid;
    // 线程创建失败 = 堆不够(RT_HEAP_SIZE)或被别处吃光: 这种状态下只允许地面观察, 不允许解锁飞行
    const bool threadsOk = (nrfTid != RT_NULL) && (logTid != RT_NULL);
    if (!threadsOk) {
        logLine("[Boot] 线程创建失败(nrf=%s logger=%s): 检查 board.c 的 RT_HEAP_SIZE!\r\n",
                (nrfTid == RT_NULL) ? "FAIL" : "OK", (logTid == RT_NULL) ? "FAIL" : "OK");
    }

    // ---- 5. 控制环状态 ----
    uint8_t acbb[256];
    uint8_t gybb[256];
    ByteBuffer acb(acbb, sizeof(acbb));
    ByteBuffer gyb(gybb, sizeof(gybb));
    auto ac = Queue<AccelerometerRaw>(acb);
    auto gy = Queue<GyroscopeRaw>(gyb);

    AccelerometerG lastAcc     = {{0.0f, 0.0f, 1.0f}};
    GyroscopeRads lastGyro     = {{0.0f, 0.0f, 0.0f}};
    FlightControlState fcs     = {};
    uint16_t eventFlags        = 0;
    uint16_t lastCmd           = 9;
    bool failsafeLatch         = false;
    uint16_t imuCnt            = 0;
    float imuDtAccum           = 0.0f;
#if DLX_BARO_ENABLED
    uint32_t lastBaroMs        = 0;
#endif
    uint32_t lastLogMs         = 0;
    uint32_t lastPerfMs        = 0;
    uint32_t lastLoopCyc       = DWT->CYCCNT;
    uint32_t logCounter        = 0;
    uint32_t perfWarmup        = kConfig.perfWarmupLoops; // 起跑热身圈(见 5.10)
    const uint32_t logPeriodMs = flightMode ? kConfig.flightLogPeriodMs : kConfig.debugLogPeriodMs;

    // 起跑线: 先应答上位机"进入模式了"(线程起来之前, 免得和 DMA 抢串口)
    if (!flightMode) {
        replyGround(app, static_cast<uint16_t>(GroundOp::Unlock), GroundStatus::Ok, 0,
                    static_cast<uint32_t>(app.mode));
        logLine("[Boot] 进入串口调试模式(可台架试车)\r\n");
    }
    rt_timer_start(ctrlTimer);
    rt_timer_start(telTimer);
    rt_timer_start(logTimer);

    while (true) {
        rt_sem_take(ctrlSem, RT_WAITING_FOREVER);
        const uint32_t loopStartCyc = DWT->CYCCNT;

        // ---- 5.1 取共享状态(遥控线程写的部分) ----
        uint16_t cmd;
        float manThr;
        FlightControlSetpoint sp;
        uint16_t motor[4];
        bool en, starting, start, stop, hardStop;
        uint32_t lastValid, motorStart;
        rt_enter_critical();
        cmd        = g_rt.command;
        manThr     = g_rt.manualThrottle;
        sp         = g_rt.setpoint;
        en         = g_rt.enabledMotor;
        starting   = g_rt.startingMotor;
        start      = g_rt.startMotor;
        stop       = g_rt.stopMotor;
        hardStop   = g_rt.hardStop;
        lastValid  = g_rt.lastValidCtrl;
        motorStart = g_rt.motorStartTime;
        for (int i = 0; i < 4; ++i) {
            motor[i] = g_rt.motorValue[i];
        }
        g_rt.startMotor = false;
        g_rt.stopMotor  = false;
        g_rt.hardStop   = false;
        rt_exit_critical();
        if (hardStop) {
            stop = true; // 硬停机一定走停机分支
        }

        const uint32_t now_ms     = rt_tick_get_millisecond();
        const uint32_t ioStartCyc = DWT->CYCCNT;

        // ---- 5.2 IMU: FIFO -> 快路径名义积分 + 慢路径协方差/倾角 ----
        bmi.fifoRead(ac, gy);
        const auto gySize = gy.size();
        const auto acSize = ac.size();
        AccelerometerRaw acr;
        AccelerometerG curAcc = lastAcc;
        uint16_t nearestAC    = 0;
        if (acSize > 0) {
            ac.pop(acr);
            curAcc = BMI088::getAccelerationG(acr);
            remapSensorFrame(kConfig.imuYawDeg, curAcc.data[0], curAcc.data[1], curAcc.data[2]);
            lastAcc   = curAcc;
            nearestAC = 1;
        }
        uint16_t gyCnt = 0;
        GyroscopeRaw gyr;
        while (gy.pop(gyr)) {
            // 加速度计比陀螺慢(1600Hz vs 2000Hz): 按比例取"最近"的加速度样本喂给 EKF
            if (nearestAC < acSize && ((gyCnt * acSize + gySize / 2) / gySize > nearestAC)) {
                if (ac.pop(acr)) {
                    curAcc = BMI088::getAccelerationG(acr);
                    remapSensorFrame(kConfig.imuYawDeg, curAcc.data[0], curAcc.data[1], curAcc.data[2]);
                    lastAcc = curAcc;
                    nearestAC++;
                }
            }
            GyroscopeRads gyroR = BMI088::getAngularVelocityRads(gyr);
            remapSensorFrame(kConfig.imuYawDeg, gyroR.data[0], gyroR.data[1], gyroR.data[2]);
            lastGyro = gyroR;

            g_ekf.integrateNominal(gyroR, curAcc, kImuDt); // 每个样本都积分(2000Hz)
            imuDtAccum += kImuDt;
            if (++imuCnt >= kConfig.ekfDecim) {
                const uint32_t t0 = DWT->CYCCNT;
                g_ekf.propagateCovariance(gyroR, curAcc, imuDtAccum);
                g_ekf.updateAccelerometer(curAcc);
                const uint32_t dSlow = DWT->CYCCNT - t0;
                perfAdd(g_perf.ekfSum, g_perf.ekfCnt, dSlow);
                imuCnt     = 0;
                imuDtAccum = 0.0f;
            }
            ++gyCnt;
        }
        while (ac.pop(acr)) { // 剩下的加速度样本只更新"最新值"
            lastAcc = BMI088::getAccelerationG(acr);
            remapSensorFrame(kConfig.imuYawDeg, lastAcc.data[0], lastAcc.data[1], lastAcc.data[2]);
        }
        ac.clear();
        gy.clear();
        // ---- 5.3 气压计: 高度观测(每 baroPeriodMs 一次) ----
        bool baroUpdated = false; // 禁用(DLX_BARO_ENABLED=0)时下面整段不存在, 这个标志恒 false
#if DLX_BARO_ENABLED
        if (baroOk && (now_ms - lastBaroMs) >= kConfig.baroPeriodMs) {
            lastBaroMs = now_ms;
            bool                baroReadOk = false;
            const EnviromentRaw baroRaw    = bme.getRaw(&baroReadOk);
            if (baroReadOk) { // I2C 出错(超时)时这份读数不可信: 跳过本次观测, 不要喂给 EKF
                baroFailStreak = 0;
                baroHealthy    = true;
                baroAbs    = bme.getAltitude(baroRaw);
                if (!baroRefSet) {
                    baroRef    = baroAbs;
                    baroRefSet = true;
                }
                baroRel = baroAbs - baroRef;
                g_ekf.updateBarometer(baroRel);
                baroUpdated = true;
                if (baroZeroPending) { // 起飞基准累积: 复用这一份读数, 不额外读传感器
                    baroZeroSum += baroAbs;
                    ++baroZeroCnt;
                }
            } else {
                // 连续读失败到阈值 ⇒ 认定气压计中途掉了: 不再喂 EKF, 遥测/日志状态位也跟着翻
                if (++baroFailStreak >= kConfig.baroFailLimit) {
                    baroHealthy = false;
                }
            }
        }
#endif
        const uint32_t ioCycles = DWT->CYCCNT - ioStartCyc;
        perfAdd(g_perf.ioSum, g_perf.ioCnt, ioCycles);

        // ---- 5.4 组装控制器输入 ----
        fcs.attitude                = g_ekf.getQuaternion();
        fcs.body_rate               = lastGyro;
        fcs.height_m                = g_ekf.getHeight();
        fcs.vertical_velocity_mps   = g_ekf.getVerticalVelocity();
        fcs.vertical_velocity_valid = baroHealthy; // 有气压观测时垂速才可信(定高用)

        // ---- 5.5 失控保护 / 解锁 / 停机 ----
        bool failsafe = false;
        if ((now_ms - lastValid) > kConfig.linkTimeoutMs && (en || starting)) {
            stop     = true;
            failsafe = true;
        }
        if (start && threadsOk) {
            dshot.preloadThrottle(0, 0, 0, 0);
            timer1.start();         // 先跑时基
            timer1.enableOutputs(); // 再开主输出(MOE), 让 DShot 信号上引脚
            dshot.start();          // 最后启动 DMA
            motorStart = now_ms;
            starting   = true;
            eventFlags |= kLogEventArm;
            // 气压基准: 这里只"开始累积", 真正提交放到控制接管那一刻(见下面的 5.5 末段)。
            // 千万别在这里 rt_thread_mdelay 求平均 —— 那是几百 ms 的阻塞, IMU FIFO 会溢出、
            // EKF 会丢数据, loopMax 也会直接爆表。
            if (baroOk) {
                baroZeroPending = true;
                baroZeroSum     = 0.0f;
                baroZeroCnt     = 0;
            }
        }
        if (stop) {
            timer1.stop();
            timer1.disableOutputs();
            dshot.stop();
            en              = false;
            starting        = false;
            baroZeroPending = false;
            eventFlags |= kLogEventDisarm;
        }
        if (!en && starting && (now_ms - motorStart) > kConfig.motorArmDelayMs) {
            en = true; // 启动序列走完, 控制接管
            // 控制接管这一刻: 把整个启动序列里平均出来的气压高度当起飞基准, 并把 EKF
            // 高度/垂速归零(姿态保持), 免得地面搬运的高度漂移带进飞行
            if (baroZeroPending) {
                if (baroZeroCnt > 0u) {
                    baroRef    = baroZeroSum / static_cast<float>(baroZeroCnt);
                    baroRefSet = true;
                    baroRel    = 0.0f;
                }
                baroZeroPending = false;
            }
            g_ekf.reset(fcs.attitude, Vector3f{}, Vector3f{});
            eventFlags |= kLogEventEkfRezero;
        }
        if (failsafe) {
            failsafeLatch = true;
            eventFlags |= kLogEventFailsafe;
        }

        // ---- 5.6 角度环 + 混控 + DShot ----
        FlightControlOutput out     = {};
        bool pidSat                 = false;
        const uint32_t ctrlStartCyc = DWT->CYCCNT;
        if (cmd == 0 && en && !stop) {
            g_controller.updateAngle(sp, fcs, kLoopDt, out);
            out.throttle         = (manThr > kConfig.minThrottle) ? manThr : kConfig.minThrottle;
            const auto ctrlMotor = mixMotors(out, g_controller.params());
            // 逻辑电机 -> 物理 DShot 通道(kConfig.motorMap)
            for (int i = 0; i < 4; ++i) {
                const int phys = kConfig.motorMap[i];
                motor[phys]    = (uint16_t)(ctrlMotor.motor[i] * kConfig.dshotUnit + kConfig.dshotOffset);
                if (motor[phys] <= 55u || motor[phys] >= 1945u) {
                    pidSat = true;
                }
            }
        }
        perfAdd(g_perf.ctrlSum, g_perf.ctrlCnt, DWT->CYCCNT - ctrlStartCyc);
        if (en && !stop) {
            dshot.preloadThrottle(motor);
        }
        if (pidSat) {
            eventFlags |= kLogEventMotorSat;
        }
        // ---- 5.7 回写共享状态 ----
        const Quaternion q = g_ekf.getQuaternion();
        // 回传位域: status / error 都是**实时**值(这一刻怎么样就报怎么样, 不锁存)
        uint8_t telStatus = 0;
        if (en) telStatus |= kTelStatusMotorEnabled;
        if (starting) telStatus |= kTelStatusMotorStarting;
        if (cmd == 0 && en && !stop) telStatus |= kTelStatusAngleLoop;
        if ((now_ms - lastValid) <= kConfig.linkTimeoutMs) telStatus |= kTelStatusLinkOk;
        if (hardStop) telStatus |= kTelStatusHardStop;
        if (failsafeLatch) telStatus |= kTelStatusFailsafe;
        if (g_rt.logFull) telStatus |= kTelStatusLogFull;
        if (baroHealthy) telStatus |= kTelStatusBaroOk; // 决定 height 字段装的是垂速还是高度
        uint8_t telError = 0;
        if (g_rt.logError) telError |= kTelErrLogWrite;
        if (!bmiOk) telError |= kTelErrImu;
        if (!baroHealthy && kBaroEnabled) telError |= kTelErrBaro; // 主动禁用不算故障
        rt_enter_critical();
        for (int i = 0; i < 4; ++i) {
            g_rt.motorValue[i]  = motor[i];
            g_rt.telem.motor[i] = motor[i];
        }
        g_rt.enabledMotor           = en;
        g_rt.startingMotor          = starting;
        g_rt.motorStartTime         = motorStart;
        g_rt.telem.quat             = q;
        g_rt.telem.height_m         = fcs.height_m;
        g_rt.telem.vertical_vel_mps = fcs.vertical_velocity_mps;
        g_rt.telem.status           = telStatus;
        g_rt.telem.error            = telError; // 实时: 问题解决就回 0(粘滞交给显示端做)
        g_rt.telem.enabled          = en;
        rt_exit_critical();

        // ---- 5.8 事件位 ----
        if (cmd != lastCmd) {
            eventFlags |= kLogEventCommandChanged;
            lastCmd = cmd;
        }
        if (g_rt.logError || g_rt.logFull) {
            eventFlags |= kLogEventLogFault;
        }
        if (baroUpdated) {
            eventFlags |= kLogEventBaroUpdate;
        }

        // ---- 5.9 日志: 按周期打包入队(事件位随这一条日志归档后清零) ----
        const uint32_t loopCycles = DWT->CYCCNT - loopStartCyc;
        if ((now_ms - lastLogMs) >= logPeriodMs) {
            lastLogMs = now_ms;
            ++logCounter;
            LogEntry e  = {};
            e.tickMs    = now_ms;
            e.linkAgeMs = static_cast<uint16_t>(
                ((now_ms - lastValid) > 0xFFFFu) ? 0xFFFFu : (now_ms - lastValid));
            e.loopPeriodUs = static_cast<uint16_t>(
                ((loopStartCyc - lastLoopCyc) / (SystemCoreClock / 1000000u)) & 0xFFFFu);
            e.events = eventFlags;

            uint16_t flags = 0;
            if ((now_ms - lastValid) <= kConfig.linkTimeoutMs) flags |= kLogFlagLinkOk;
            if (bmiOk) flags |= kLogFlagImuOk;
            if (baroHealthy) flags |= kLogFlagBaroOk;
            if (cmd == 0 && en && !stop) flags |= kLogFlagAngleLoop;
            if (en) flags |= kLogFlagMotorEnabled;
            if (starting) flags |= kLogFlagMotorStarting;
            if (hardStop) flags |= kLogFlagHardStop;
            if (flightMode && app.fs != nullptr) flags |= kLogFlagFlashOk;
            if (g_rt.logFull) flags |= kLogFlagLogFull;
            if (g_rt.logError) flags |= kLogFlagLogError;
            if (pidSat) flags |= kLogFlagMotorSat;
            if (failsafeLatch) flags |= kLogFlagFailsafe;
            if (!flightMode) flags |= kLogFlagGroundMode;
            e.flags = flags;

            e.quat[0] = q.data[0];
            e.quat[1] = q.data[1];
            e.quat[2] = q.data[2];
            e.quat[3] = q.data[3];
            for (int i = 0; i < 3; ++i) {
                e.bodyRate[i]     = fcs.body_rate.data[i];
                e.accelG[i]       = lastAcc.data[i];
                e.rateSetpoint[i] = out.rate_setpoint.data[i];
                e.torque[i]       = out.torque.data[i];
            }
            const Vector3f spEuler = quatToEuler(sp.attitude);
            e.targetPitchDeg       = spEuler.y() * kRadToDeg;
            e.targetRollDeg        = spEuler.z() * kRadToDeg;
            e.targetHeightM        = sp.height_m;
            e.throttle             = out.throttle;
            e.manualThrottle       = manThr;
            e.heightM              = fcs.height_m;
            e.vertVelMps           = fcs.vertical_velocity_mps;
            e.baroRelM             = baroRel;
            e.baroAbsM             = baroAbs;
            if ((logCounter % kConfig.ekfLogDivider) == 0u) { // 零偏变化慢, 隔几条记一次
                const FlightFilterState16 st = g_ekf.getState16();
                e.gyroBias[0]                = st.data[FF_BGX];
                e.gyroBias[1]                = st.data[FF_BGY];
                e.gyroBias[2]                = st.data[FF_BGZ];
            }
            for (int i = 0; i < 4; ++i) {
                e.motor[i] = motor[i];
            }
            logQueuePush(e);
            eventFlags = 0;
        }
        lastLoopCyc = loopStartCyc;

        // ---- 5.10 负载统计窗口(串口调试模式用它回传 DWT 计数) ----
        // 起跑热身圈不计入统计: 控制线程刚起来的前几圈含 BMI088 FIFO 积压 + EKF 首次整定
        // (实测单圈能到 5ms), 计进 loopMax 会让上位机的"超预算"提醒变成假警报。
        if (perfWarmup > 0u) {
            --perfWarmup;
            lastPerfMs  = now_ms;
            lastLoopCyc = loopStartCyc;
            g_perf      = {};
            continue; // 本块在循环体末尾, 直接进入下一圈
        }
        ++g_perf.samples;
        if (loopCycles > g_perf.loopMax) {
            g_perf.loopMax = loopCycles;
        }
        g_perf.loopSum += loopCycles;
        if ((now_ms - lastPerfMs) >= kConfig.perfReportMs) {
            lastPerfMs     = now_ms;
            PerfStats perf = {};
            rt_enter_critical();
            perf.loopAvgCycle  = g_perf.samples ? (g_perf.loopSum / g_perf.samples) : 0u;
            perf.loopMaxCycle  = g_perf.loopMax;
            perf.ekfCycle      = g_perf.ekfCnt ? (g_perf.ekfSum / g_perf.ekfCnt) : 0u;
            perf.ctrlCycle     = g_perf.ctrlCnt ? (g_perf.ctrlSum / g_perf.ctrlCnt) : 0u;
            perf.ioCycle       = g_perf.ioCnt ? (g_perf.ioSum / g_perf.ioCnt) : 0u;
            perf.logCycle      = g_perf.logCnt ? (g_perf.logSum / g_perf.logCnt) : 0u;
            perf.samples       = g_perf.samples;
            perf.drops         = g_logQueue.dropped;
            g_rt.perf          = perf;
            g_perf.loopSum     = 0;
            g_perf.loopMax     = 0;
            g_perf.ekfSum      = 0;
            g_perf.ekfCnt      = 0;
            g_perf.ctrlSum     = 0;
            g_perf.ctrlCnt     = 0;
            g_perf.ioSum       = 0;
            g_perf.ioCnt       = 0;
            g_perf.logSum      = 0;
            g_perf.logCnt      = 0;
            g_perf.samples     = 0;
            g_logQueue.dropped = 0;
            rt_exit_critical();
        }
    }
}

//===================================================================================================
// 六、main
//===================================================================================================
int main()
{
    DLX_NVIC_AutoConfig();

    // ---- 高分辨率计时(DWT): 负载统计用; 168MHz 下 168 周期 = 1us ----
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_tidMain = rt_thread_self(); // 主线程(控制环)句柄: 栈水位线用
    paintStackWatermark();

    // ---- 指示灯 ----
    GPIO ledRun(kConfig.hw.ledRun);
    ledRun.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    ledRun = 0;
    GPIO ledFlash(kConfig.hw.ledFlash);
    ledFlash.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    ledFlash = 1; // 熄灭(低电平点亮)

    // ---- BME280 地址脚(SDO/PD2): 钉死在低电平 ⇒ 从机地址固定 0x76 ----
    // SDO 悬空时电平会漂, 地址会在 0x76/0x77 之间翻(实测被 flash 擦除的干扰一激就翻),
    // 现象是气压计"原地址不 ACK(AF=1)"、备用地址 0x77 却能答。这里在任何 I2C 访问之前
    // 就把它拉低; 用开漏是为了不和模块上的上拉电阻打架。
    // DLX_BARO_ENABLED=0 时不用气压计, PD2 也就完全不碰(引脚留给别的用途/别的固件)。
#if DLX_BARO_ENABLED
    GPIO bmeSdo(kConfig.hw.bmeSdo);
    bmeSdo.init(GPIOModeProfile::OUT_OD_NOPULL_50MHz);
    bmeSdo = 0;
#endif

    // ---- 调试串口 + 发送 DMA + DLX 协议对象 ----
    uint8_t rxRaw[kConfig.hw.rxBufBytes];
    ByteBuffer rxByteBuf(rxRaw, kConfig.hw.rxBufBytes);
    RingByteBuffer uartRx(rxByteBuf);
    auto usart1 = USART::USART1_TA9_RAA();
    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, kConfig.serialBaud, uartRx);
    uint8_t txRaw[kConfig.hw.txBufBytes];
    ByteBuffer txBuf(txRaw, kConfig.hw.txBufBytes);
    uint8_t frameRaw[kConfig.hw.frameBufBytes];
    ByteBuffer frameBuf(frameRaw, kConfig.hw.frameBufBytes);
    DLX_ProtocolBuffer protocol(txBuf, &uartRx, &frameBuf); // 地面模式报文(生成代码)走这里
    auto serialDma = usart1.setDMASend(txBuf);
    debugPort()    = &usart1;

    ReleaseContext app = {&usart1, &serialDma, &txBuf, &uartRx, &protocol, nullptr, &ledRun,
                          BootMode::Flight};
    logContext()       = &app; // 文本日志要按 DLX SimpleLog 帧发出去(见 flight_config_struct.hpp)

    logLine("\r\n\r\n=== DLX 飞控发布版 v%u ===\r\n", static_cast<unsigned>(FLIGHT_CONFIG_VERSION));

    // ---- 上电模式判定 ----
    const BootDecision decision = decideBootMode(app);
    app.mode                    = decision.mode;

    // 飞行模式: 从这一刻起串口**完全静音** —— 上面那行版本号是飞行模式下唯一会发出的东西。
    // (地面模式要等上位机命令、管理模式要交互, 所以不能提前静音; 下面那些 [Flash] 日志
    //  只在调试/管理模式才看得到。真飞行时日志走 flash, 状态位记在日志里。)
    if (decision.mode == BootMode::Flight) {
        debugPort() = nullptr;
    }

    // ---- flash: 飞行模式(写日志)与日志管理模式(查/删/回传/格式化)都要 ----
    SPI spiFlash = SPI::SPI2_SB10_MIC2_MOC3(kConfig.hw.flashCs);
    W25Q128 flash(spiFlash);
    FlashManager fs(flash);
    bool flashChipOk = false; ///< W25Q128 认出来了(JEDEC 校验通过)
    bool flashReady  = false; ///< 文件系统 init 成功(能读写日志)
    if (decision.mode != BootMode::SerialDebug) {
        spiFlash.init(kConfig.hw.flashSpi);
        if (!flash.init()) {
            logLine("[Flash] 识别失败(JEDEC=%06X)\r\n", static_cast<unsigned>(flash.readJEDEC()));
        } else {
            flashChipOk = true;
            fs.setSessionSlotLimit(decision.mode == BootMode::Flight ? kConfig.flightLogSlots
                                                                     : kConfig.manageLogSlots);
            fs.setFullPolicy(FlashFullPolicy::Halt); // 飞行中绝不现场擦除
            ExceptionCode err      = ExceptionCode::FLASH_OK;
            const uint32_t t0      = rt_tick_get_millisecond();
            const FunctionResult r = fs.init(FlashFullPolicy::Halt, onFlashBusy, err);
            onFlashIdle(); // 擦除结束: 熄灭进度灯(否则可能刚好停在"亮", 排查时会被误当成故障灯)
            logLine("[Flash] init=%d err=0x%04X 会话=%u 预留=%u 槽 上限=%u 条 用时=%ums\r\n",
                    static_cast<int>(r), static_cast<unsigned>(err),
                    static_cast<unsigned>(fs.currentSessionId()),
                    static_cast<unsigned>(fs.sessionReservedSlots()),
                    static_cast<unsigned>(fs.sessionEntryLimit()),
                    static_cast<unsigned>(rt_tick_get_millisecond() - t0));
            flashReady = (r == FunctionResult::SUCCESS);
            if (err == ExceptionCode::FLASH_GEOMETRY_MISMATCH) {
                logLine("[Flash] 布局与旧数据不符: 进日志管理模式发 5(整片格式化) 即可恢复\r\n");
                flashReady = false;
                if (kConfig.autoFormatOnMismatch) { // 想省事就打开这个开关(会丢旧日志)
                    logLine("[Flash] autoFormatOnMismatch = true, 直接整片重建...\r\n");
                    ExceptionCode fmtErr = ExceptionCode::FLASH_OK;
                    if (fs.format(onFlashBusy, fmtErr) == FunctionResult::SUCCESS) {
                        flashReady = (fs.init(FlashFullPolicy::Halt, onFlashBusy, err) == FunctionResult::SUCCESS);
                    }
                    onFlashIdle();
                }
            }
        }
        if (flashReady) {
            app.fs = &fs;
        }
    }

    // ---- 分发 ----
    if (decision.mode == BootMode::LogManage) {
        if (!flashChipOk) { // 芯片都认不到: 只能停在这提示(接线/供电问题)
            logLine("[Manage] flash 芯片识别失败, 无法进入日志管理模式; 复位重试\r\n");
            bool on = false;
            while (true) {
                delay_ms(500);
                on     = !on;
                ledRun = on ? 1 : 0;
            }
        }
        if (!flashReady) {
            // 文件系统没就绪(最常见: 布局指纹不符) —— 仍要进管理模式, 否则整片格式化这条唯一的恢复路就断了
            app.fs = &fs;
            logLine("[Manage] 文件系统未就绪(布局与旧数据不符?): 发命令 5 = 整片格式化即可恢复\r\n");
        }
        while (true) {
            const BootMode next = runLogManage(app); // 内部死循环, 只有收到解锁才返回
            if (next == BootMode::LogManage) {
                continue;
            }
            logLine("[Manage] 切换到模式 %u ...\r\n", static_cast<unsigned>(next));
            if (next == BootMode::SerialDebug) {
                app.mode = BootMode::SerialDebug;
                app.fs   = nullptr; // 调试模式不碰 flash
            } else {
                // 转飞行模式: 用飞行档的预留槽数重开一个会话(管理模式用的是最小预留)
                app.mode = BootMode::Flight;
                if (!fs.isReady()) { // 注意用 fs 的实时状态: 管理模式里可能刚做过整片格式化
                    logLine("[Manage] flash 不可用, 不能转飞行模式(请复位)\r\n");
                    continue;
                }
                fs.setFullPolicy(FlashFullPolicy::Halt);
                fs.setSessionSlotLimit(kConfig.flightLogSlots);
                ExceptionCode err = ExceptionCode::FLASH_OK;
                if (fs.init(FlashFullPolicy::Halt, onFlashBusy, err) != FunctionResult::SUCCESS) {
                    logLine("[Manage] 重开会话失败 err=0x%04X\r\n", static_cast<unsigned>(err));
                    continue;
                }
                app.fs = &fs;
            }
            runFlightStack(app); // 不返回
        }
    }

    // 飞行模式: flash 挂了也照样飞(只是没有日志), 状态位会记下来; 串口一声不响
    if (decision.mode == BootMode::Flight && !flashReady) {
        g_rt.logError = true;
    }
    runFlightStack(app);
    return 0;
}

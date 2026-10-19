//===================================================================================================
// release.cpp —— 发布版主程序(**只剩"组合根"**: main + 两段上电编排)
//
// 结构(为什么 main 在最前面):
//   本文件读起来应该是"上电之后**按什么顺序**发生什么", 而不是"控制环怎么写"。所以顺序是:
//     §一 main()            —— 装安全网 -> 时钟/DWT -> 指示灯 -> 调试串口 -> 上电模式判定 -> flash -> 分发
//     §二 bringUpFlash()    —— flash 文件系统挂载(飞行与日志管理都要)
//     §三 runFlightStack()  —— 建 FlightStack 并进控制环(app_flight_stack.hpp)
//   细节都在下面这些文件里, 本文件不再重复它们:
//     app_flight_stack.hpp  : FlightStack(硬件 + 上电生命周期 + 控制环五步)
//     app_link_thread.hpp   : 链路(遥控/遥测)线程
//     app_log_thread.hpp    : 日志线程(flash / 串口 / 丢弃三条出口) + 延迟异常的发送点
//     app_flight_shared.hpp : 三个线程共用的那块内存
//     app_boot_mode.hpp     : 上电模式判定(跳线帽 / 上位机命令 / 超时)
//     app_exception_handler.hpp : **异常的统一出口**(LED / 文本 / DLX 报文 / 无线 hook + 延迟队列)
//     app_health.hpp        : 全局健康位(一个字节代替 v2.0 的那一堆 bool)
//     app_bringup.hpp       : 逐个器件的上电(失败分类 + 异常码) + 解锁门槛
//     app_boot_report.hpp   : 启动自检报告(寄存器回读, 只在串口调试模式输出)
//     app_fault.hpp         : 硬件故障钩子(接 RT-Thread 的 rt_hw_exception_install)
//     release_log_manage.hpp: 日志管理模式(与飞行完全分离, 永不输出电机)
//
// 三种模式:
//   Flight      : 正常飞行。启动诊断走串口，运行期日志写 flash;
//   SerialDebug : 台架调试。日志(与 flash 同格式的原始帧)直接走串口, 另发 PerfMonitor 负载统计;
//                 可以解锁电机(遥控指令照常);
//   LogManage   : 日志管理。见 release_log_manage.hpp。
//
// 线程结构:
//   主线程       : 控制环 500Hz(2ms) —— 读 IMU FIFO -> EKF -> 控制器 -> 混控 -> DShot, 日志入队;
//   link_tel 线程: 50Hz —— 收遥控指令(写共享状态) + 回遥测;
//   logger 线程  : 5ms —— 排空日志队列 -> 写 flash(飞行模式) / 打包发串口(调试模式)。
//===================================================================================================
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "dlx_spi.hpp"
#include "dlx_delay.hpp"
#include "rtthread.h"
#include "W25Q128/dlx_flash_manager.hpp"
#include "app/runtime/release_header.h"
#include "app/runtime/app_fault.hpp"
#include "app/boot/app_boot_mode.hpp"
#include "app/runtime/app_exception_handler.h" // 异常出口: LED / 文本 / DLX 报文(按生命周期接替)
#include "app/control/app_flight_stack.hpp"
#include "app/log/release_log_manage.hpp"
#include "app/control/app_state.h"
using namespace dlx;

// ---- 前向声明: 定义在下面(这样 main() 能待在文件最前面, 读的人先看到"上电顺序") ----
struct FlashBringUp {
    bool chipOk{false}; ///< W25Q128 认出来了(JEDEC 校验通过)
    bool ready{false};  ///< 文件系统 init 成功(能读写日志)
};
static FunctionResult bringUpFlash(ReleaseContext &app, SPI &spiFlash, W25Q128 &flash, FlashManager &fs,
                                   BootMode mode);

constexpr bool autoFormatFlashAtMismatch = false;

//===================================================================================================
// 一、main —— 上电顺序就是本函数的顺序
//===================================================================================================
int main()
{

    // ---- 第一件事: 把 flash 的 /CS 拉高(见 dlx_w25q128_config.h 的完整说明) ----
    // 本板 W25Q128 的 /CS(PD1) **没有外部上拉**, MCU 复位期间它是浮空输入(实测默认低),
    // 芯片会在"被选中"状态下经历上电/复位; 只要 SCK(PB10) 上有一点噪声被当成时钟,
    // 就可能灌进半条/一整条杂散指令(写状态/擦除…), 于是接下来一段时间它只忙自己的事、
    // 读不到 JEDEC ID(现场日志: JEDEC=0x171717 SR1=0x17, 一秒后又恢复).
    // 这里在**任何其它初始化之前**把它驱动到高电平, 把危险窗口从"整段启动期(几十~几百 ms)"
    // 压缩到"复位后的几条指令". 注意: 这只是缓解, 硬件上仍建议给 /CS 加 10k 上拉.
    {
        GPIO flashCs(cfg::hw::flash::kCs);
        flashCs.init(GPIOModeProfile::OUT_PP_UP_50MHz);
        flashCs = 1; // /CS 高 = 不选中
    }

    DLX_NVIC_AutoConfig();
    rt_hw_exception_install(dlxExceptionHook);

    // ---- 高分辨率计时(DWT): 负载统计用; 168MHz 下 168 周期 = 1us ----
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#if CFG_FEATURE_LINK
    E220::auxPinSafeNow(cfg::hw::link::kAux);
#endif

    paintStackWatermark();

    // ---- 指示灯 ----
    GPIO ledRun(cfg::hw::led::kRun);
    ledRun.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    ledRun = 0;
    GPIO ledFlash(cfg::hw::led::kFlash);
    ledFlash.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    ledFlash = 1; // 熄灭(低电平点亮)
    // ---- 调试串口 + 发送 DMA + DLX 协议对象 ----

    uint8_t rxRaw[cfg::hw::buf::kRx];
    ByteBuffer rxByteBuf(rxRaw, cfg::hw::buf::kRx);
    RingByteBuffer uartRx(rxByteBuf);

    auto usart1 = USART::USART1_TA9_RAA();
    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, cfg::hw::console::kBaud, uartRx);

    uint8_t txRaw[cfg::hw::buf::kTx];
    ByteBuffer txBuf(txRaw, cfg::hw::buf::kTx);
    uint8_t frameRaw[cfg::hw::buf::kFrame];
    ByteBuffer frameBuf(frameRaw, cfg::hw::buf::kFrame);

    DLX_ProtocolBuffer protocol(txBuf, &uartRx, &frameBuf); // 地面模式报文(生成代码)走这里
    auto serialDma = usart1.setDMASend(txBuf);

    // 日志和全局ReleaseContext;
    uint8_t textPoolRawA[textmbox::POOL_BYTES];
    uint8_t textPoolRawB[textmbox::POOL_BYTES];
    ByteBuffer textPoolA(textPoolRawA, sizeof(textPoolRawA));
    ByteBuffer textPoolB(textPoolRawB, sizeof(textPoolRawB));
    TextMailbox textBox(textPoolA, textPoolB);
    ExceptionMailbox exceptBox;
    LogChannel channel{&textBox, &exceptBox};

    ReleaseContext app = {&usart1, &serialDma, &txBuf, &uartRx, &protocol, nullptr, &ledRun,
                          BootMode::Flight, &channel};

    logContext = &app; // 绑定到全局变量.
    bootLogLine("\r\n\r\n=== DLX 飞控发布版 v%u ===\r\n", static_cast<unsigned>(cfg::kFirmwareVersion));

    // ---- 上电模式判定 ----
    const BootDecision decision = decideBootMode(app);

    

    app.mode                    = decision.mode;

    //非飞行模式下打开文本模式的全局错误处理,根据需要可以在这里切换
    if (decision.mode != BootMode::Flight) {
        ENABLE_EXCEPTION_TEXT();
        ENABLE_EXCEPTION_DLX();
    }

    // ---- flash: 飞行模式(写日志)与日志管理模式(查/删/回传/格式化)都要 ----
    SPI spiFlash = SPI::SPI2_SB10_MIC2_MOC3(cfg::hw::flash::kCs);
    W25Q128 flash(spiFlash);

    FlashManager fs(flash);

    FunctionResult flashInitState = bringUpFlash(app, spiFlash, flash, fs, decision.mode);

    flushBootLog(app);
    if (decision.mode == BootMode::LogManage) {
        // 即使 flash 没挂上**也必须**进管理模式: 那里是唯一的恢复入口(列会话/回传/格式化重建).
        // 以前这里在 init 失败时卡成"状态灯无限闪", 等于把恢复路径一起关掉了.
        // 管理循环自己会报 "flash 未就绪", 各命令用 GroundReply 回失败原因.
        while (true) {
            (void)runLogManage(app); // 内部死循环, 只有收到解锁才返回
        }
    }


    if (flashInitState == FunctionResult::FAILED) {
        bootLogLine("[Flash] 本次运行没有可用的 flash 文件系统: %s\r\n",
                (decision.mode == BootMode::Flight)
                    ? "飞行日志没有落盘出口(控制/异常出口仍在, 但这次飞行不留黑匣子!)"
                    : "日志改走串口; 飞行/管理相关接口不受影响");
    }

    logDshotTiming();

    FlightStack stack(app);

    stack.sh.tidMain = rt_thread_self(); // 主线程(控制环)句柄: 栈水位线用

    stack.run();                         // 不返回
    return 0;
}

//===================================================================================================
// 二、flash 挂载
//===================================================================================================
static FunctionResult bringUpFlash(ReleaseContext &app, SPI &spiFlash, W25Q128 &flash, FlashManager &fs,
                                   BootMode mode)
{
    spiFlash.init(cfg::hw::flash::kSpi);

    // ---- 1. 识别芯片 + 清块保护(驱动内部已带 66h/99h 软复位重试, 见 dlx_w25q128.hpp) ----
    const uint32_t t0 = rt_tick_get_millisecond();
    if (!flash.init()) {
        // 报**实际读到的 ID** 和试了几轮: 只写"识别失败"没法区分
        //   "芯片没应答(0x171717/0x000000/0xFFFFFF)" 和 "型号不对(别的 EF xx xx)".
        bootLogLine("[Flash] 识别失败: JEDEC=0x%06X(期望 0xEF4018) 试了 %u 轮 用时=%ums\r\n",
                static_cast<unsigned>(flash.lastJedecId()),
                static_cast<unsigned>(flash.lastInitAttempts()),
                static_cast<unsigned>(rt_tick_get_millisecond() - t0));
        DLX_HANDLE_EXCEPTION(ExceptionCode::FLASH_HARDWARE_ERROR);
        // 管理模式要保留 fs 句柄: 那是用户唯一的"格式化重建"入口;
        // 飞行/串口调试模式则**必须**置空 —— 否则运行期会以为有 flash 出口(见下面 3.)
        app.fs = (mode == BootMode::LogManage) ? &fs : nullptr;
        return FunctionResult::FAILED;
    }

    // ---- 2. 挂文件系统 ----
    fs.setSessionSlotLimit(mode == BootMode::Flight ? cfg::log::kFlightSlots : cfg::log::kManageSlots);
    fs.setFullPolicy(FlashFullPolicy::Halt);
    // 防御: 任何返回 FAILED 的路径都必须写 err; 初值给一个"肯定不是成功"的码,
    // 万一哪条路漏写, 上报的是 FLASH_HARDWARE_ERROR 而不是栈上的垃圾值.
    ExceptionCode err = ExceptionCode::FLASH_HARDWARE_ERROR;
    const uint32_t t1 = rt_tick_get_millisecond();
    FunctionResult r  = fs.init(FlashFullPolicy::Halt, onFlashBusy, err);
    onFlashIdle();
    if (r == FunctionResult::SUCCESS) {
        bootLogLine("[Flash] init=OK 会话=%u 预留=%u 槽 上限=%u 条 用时=%ums(识别轮次=%u)\r\n",
                static_cast<unsigned>(fs.currentSessionId()),
                static_cast<unsigned>(fs.sessionReservedSlots()),
                static_cast<unsigned>(fs.sessionEntryLimit()),
                static_cast<unsigned>(rt_tick_get_millisecond() - t1),
                static_cast<unsigned>(flash.lastInitAttempts()));
        app.fs = &fs;
        return FunctionResult::SUCCESS;
    }

    bootLogLine("[Flash] init=FAILED err=0x%04X 会话=%u 预留=%u 槽 上限=%u 条 用时=%ums\r\n",
            static_cast<unsigned>(err),
            static_cast<unsigned>(fs.currentSessionId()),
            static_cast<unsigned>(fs.sessionReservedSlots()),
            static_cast<unsigned>(fs.sessionEntryLimit()),
            static_cast<unsigned>(rt_tick_get_millisecond() - t1));
    DLX_HANDLE_EXCEPTION(err);

    // ---- 3. 失败就是失败: 绝不把"未就绪"的文件系统当成功交给运行期 ----
    // 以前这里会落到函数尾部 app.fs = &fs; return SUCCESS, 于是:
    //   * 自检打印 "flash=OK" / "文件系统就绪 会话=0 预留=0 条"(全是 0 就是根本没 init);
    //   * 飞行模式照样可以解锁, 每条日志 appendLog() 都返回 FLASH_NOT_READY 并被静默丢弃,
    //     等于**黑匣子坏了还照飞**, 事后没有任何数据可查.
    // 现在: 飞行/调试模式置空(日志出口明确降级), 管理模式保留(恢复入口).
    if (err == ExceptionCode::FLASH_GEOMETRY_MISMATCH) {
        bootLogLine("[Flash] 布局与旧数据不符: 进日志管理模式发 5(整片格式化) 即可恢复\r\n");
        if (autoFormatFlashAtMismatch) {
            bootLogLine("[Flash] autoFormatFlashAtMismatch = true, 直接整片重建...\r\n");
            ExceptionCode fmtErr = ExceptionCode::FLASH_ERASE_FAILED;
            if (fs.format(onFlashBusy, fmtErr) == FunctionResult::SUCCESS) {
                r = fs.init(FlashFullPolicy::Halt, onFlashBusy, err);
            } else {
                DLX_HANDLE_EXCEPTION(fmtErr);
            }
            onFlashIdle();
            if (r == FunctionResult::SUCCESS) {
                bootLogLine("[Flash] 重建完成 会话=%u 预留=%u 槽 上限=%u 条\r\n",
                        static_cast<unsigned>(fs.currentSessionId()),
                        static_cast<unsigned>(fs.sessionReservedSlots()),
                        static_cast<unsigned>(fs.sessionEntryLimit()));
                app.fs = &fs;
                return FunctionResult::SUCCESS;
            }
        }
    }
    app.fs = (mode == BootMode::LogManage) ? &fs : nullptr;
    return FunctionResult::FAILED;
}


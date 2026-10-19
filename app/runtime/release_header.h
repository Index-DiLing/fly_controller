#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "dlx_bytebuffer.hpp"
#include "dlx_gpio.hpp"
#include "dlx_usart.hpp"
#include "dlx_dma.hpp"
#include "dlx.hpp"
#include "flight_config.hpp"
#include "hardware_claims.hpp"
#include "app/channels/app_log_channel.hpp"
#include "app/log/app_log_mailbox.hpp"
#include "app/link/app_link_cmd.hpp"
#include "W25Q128/dlx_flash_manager_config.h"
#include "dlx_sensor_data.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_spi_profile.hpp"
#include "dlx_timer_profile.hpp"
#include "dlx_dshot_profile.hpp"
#include "flight_control/flight_controller.hpp"
#include "flight_control/flight_control_fliter.hpp"

namespace dlx
{

    class FlashManager; // 只用指针, 前置声明即可(避免头文件依赖整个文件系统实现)

    enum class BootMode : uint8_t
    {
        Flight      = 0, ///< 正常飞行(日志写 flash)
        SerialDebug = 1, ///< 串口直连调试(日志走串口, 可台架试车)
        LogManage   = 2, ///< 日志管理(不动电机)
    };

    /** 启动模式判定结果。 */
    struct BootDecision
    {
        BootMode mode;       ///< 最终进入的模式
    };

    // 地面模式命令字段；报文布局由 DLX 生成代码定义。
    /** 地面命令 op: 与 Kotlin 侧 GroundOp 一一对应(两边同时改) */
    enum class GroundOp : uint16_t
    {
        Unlock       = 1, ///< 解锁并进入 mode 指定的模式(0 飞行 / 1 串口调试 / 2 日志管理)
        ListSessions = 2, ///< 列出全部日志会话(逐条 SessionInfo + 一条汇总应答)
        DeleteOldest = 3, ///< 丢掉最老的 count 个会话(环形队列语义, 腾出的槽并入当前会话)
        DumpSession  = 4, ///< 按原始日志帧流回传整个会话
        FormatFlash  = 5, ///< 整片擦除重建(丢日志+参数, 几十秒)
        Ping         = 6, ///< 空指令, 回一条应答用于探链路
    };

    // 四、日志状态位 / 事件位(LogEntry::flags / events)
    /** flags —— 电平型状态(这一条日志"当时处于什么状态") */
    enum : uint16_t
    {
        kLogFlagLinkOk        = 0x0001, ///< 遥控链路在线(链路延迟 < linkTimeoutMs)
        kLogFlagImuOk         = 0x0002, ///< BMI088 自检通过
        kLogFlagBaroOk        = 0x0004, ///< 气压计可用
        kLogFlagAngleLoop     = 0x0008, ///< 角度环正在工作(command==0 且已使能)
        kLogFlagMotorEnabled  = 0x0010, ///< 电机已使能(控制已接管)
        kLogFlagMotorStarting = 0x0020, ///< 电机启动序列进行中
        kLogFlagHardStop      = 0x0040, ///< 收到硬停机命令
        kLogFlagFlashOk       = 0x0080, ///< flash 日志可用(飞行模式)
        kLogFlagLogFull       = 0x0100, ///< flash 日志写满, 已停止记录
        kLogFlagLogError      = 0x0200, ///< flash 写入出错
        kLogFlagMotorSat      = 0x0400, ///< 有一路电机输出饱和
        kLogFlagFailsafe      = 0x0800, ///< 失控保护触发过(本会话)
        kLogFlagGroundMode    = 0x1000, ///< 处于地面模式(串口调试)
    };

    /** events —— 边沿型事件(这一条日志"期间发生了什么"), 随日志归档后清零 */
    enum : uint16_t
    {
        kLogEventCommandChanged = 0x0001, ///< 遥控指令字变化
        kLogEventArm            = 0x0002, ///< 解锁(启动电机)
        kLogEventDisarm         = 0x0004, ///< 停机
        kLogEventFailsafe       = 0x0008, ///< 触发失控保护
        kLogEventMotorSat       = 0x0010, ///< 出现电机饱和
        kLogEventLogFault       = 0x0020, ///< 日志写满/写错
        kLogEventBaroUpdate     = 0x0040, ///< 本周期更新了气压高度
        kLogEventEkfRezero      = 0x0080, ///< EKF 高度归零(解锁瞬间)
        kLogEventArmRejected    = 0x0100, ///< 收到"启动电机"但被安全门槛拒绝
    };

    /** 飞行日志的光流、测距及水平观测有效性标志。 */
    enum : uint8_t
    {
        kFlowFlagRangeOk   = 0x02, ///< 测距本帧有新数据(VL53L1X)
        kFlowFlagGnssVel   = 0x04, ///< GNSS 水平速度有效(已进 EKF)
        kFlowFlagVelHold   = 0x08, ///< 速度环正在接管水平速度(VEL_HOLD)
    };

    // 四之二、遥控回传位域(FlightCoreStatus 的 status / error, 各 8 位)
    /** 实时遥测状态位；height 字段含义由 kTelStatusBaroOk 判定，不由 error 位判定。 */
    enum : uint8_t
    {
        kTelStatusMotorEnabled  = 0x01, ///< 电机已使能(解锁完成, 控制已接管)
        kTelStatusMotorStarting = 0x02, ///< 启动序列进行中
        kTelStatusAngleLoop     = 0x04, ///< 角度环正在工作
        kTelStatusLinkOk        = 0x08, ///< 遥控链路在线
        kTelStatusHardStop      = 0x10, ///< 收到硬停机命令
        kTelStatusFailsafe      = 0x20, ///< 失控保护触发过(本会话锁存)
        kTelStatusLogFull       = 0x40, ///< flash 日志写满, 已停止记录
        kTelStatusBaroOk        = 0x80, ///< 气压计可用 ⇒ height 字段装的是垂速 [m/s], 否则是高度 [m]
    };

    /** 实时错误位，问题恢复后清零；需要历史锁存时由显示端实现。 */
    enum : uint8_t
    {
        kTelErrLogWrite = 0x01, ///< flash 日志写入失败
        kTelErrImu      = 0x02, ///< BMI088 自检失败
        kTelErrBaro     = 0x04, ///< 气压计不可用
    };

    // 跨线程发布的数据类型
    /** 遥测(控制线程 -> 链路线程) */
    struct FlightTelemetry
    {
        Quaternion quat{{1.0f, 0.0f, 0.0f, 0.0f}}; ///< EKF 姿态(机体 -> 世界)
        uint16_t   motor[4]{0, 0, 0, 0};           ///< 四路电机输出(DShot 计数)
        float      height_m{0.0f};                 ///< EKF 高度 [m]
        float      vertical_vel_mps{0.0f};         ///< EKF 垂向速度 [m/s]
        /** 世界系速度 [m/s]，由控制线程从 EKF 复制后发布；链路线程不直接访问 EKF。 */
        Vector3f   velWorldMps{};
        /** 世界系位置 [m]；水平位置没有有效位置观测时保持未使能语义。 */
        Vector3f   posWorldM{};
        float      batteryV{0.0f}; ///< 电池组电压 [V](来自 PowerSample; 没采到就是 0)
        float      batteryA{0.0f}; ///< 电流 [A]
        uint8_t    status{0};      ///< 回传状态位(实时, 见 kTelStatus*)
        uint8_t    error{0};       ///< 回传错误位(粘滞, 见 kTelErr*)
        bool       enabled{false}; ///< 电机是否已使能
    };

    // 调试采样: 原始观测(气压/光流/测距) + 同一时刻的 EKF 融合结果
    /**
     * @brief 一条"融合状况"调试采样
     *
     * 控制线程每个日志周期(串口调试模式 10ms / 100Hz)填一条, 经 DebugQueue(SPSC)交给日志线程,
     * 由日志线程编码成 DebugSensor 报文发串口; **Flight 模式不产生**(调试串口在飞行模式下是静音的)。
     *
     * 为什么把原始观测和融合结果放进同一条: 上位机绘图用的是"同一条目第 N 个点"作横轴,
     *   两边的更新频率必须一致才能直接对比 —— 拼在一条记录里就不用在上位机做时间对齐。
     * 为什么保留 pressurePa / 原始增量 / rangeRawMm: 事后要能分清"传感器没数据 / 被门限拒了 /
     *   数据进了 EKF 但尺度或零偏算错"这三种情况, 只记融合后的高度分不出来。
     */
    struct DebugSample
    {
        uint32_t tickMs{0u};      ///< 控制线程记录时刻 [ms](上电起算)
        uint16_t flags{0u};       ///< 本周期内各观测通道的处置结果, 见 kDebug* 位
        uint8_t  flowSqual{0u};   ///< 光流表面质量(SQUAL, 0~255; 0 = 本周期没有光流样本)
        uint8_t  rangeStatus{0u}; ///< 测距状态码(VL53L1X; 0 = 本周期没有测距样本)
        float    pressurePa{0.0f};         ///< 气压计: 绝对气压 [Pa]
        float    altitudeM{0.0f};          ///< 气压计: 绝对高度 [m](标准海平面 101325 Pa)
        float    relativeAltitudeM{0.0f};  ///< 气压计: 相对参考高度 [m](控制环喂给 EKF 的就是它)
        int16_t  flowDx{0};                ///< 光流: 原始增量 x [count](未乘尺度)
        int16_t  flowDy{0};                ///< 光流: 原始增量 y [count](未乘尺度)
        uint16_t flowDtMs{0u};             ///< 光流: 增量积分间隔 [ms]
        uint16_t rangeRawMm{0u};           ///< 测距: 原始距离 [mm]
        float    rangeM{0.0f};             ///< 测距: 距离 [m](喂给 EKF 的值)
        float    ekfHeightM{0.0f};         ///< 融合后 EKF 高度 [m](本周期结束时)
        float    ekfVertVelMps{0.0f};      ///< 融合后 EKF 垂速 [m/s]
        float    ekfVelXMps{0.0f};         ///< 融合后 EKF 世界系 x 速度 [m/s]
        float    ekfVelYMps{0.0f};         ///< 融合后 EKF 世界系 y 速度 [m/s]
    };

    /** DebugSample::flags: 本周期内各通道"有没有新样本 / 有没有进 EKF" */
    enum : uint16_t
    {
        kDebugBaroNew      = 0x0001, ///< 本周期收到了新的气压样本
        kDebugBaroOk       = 0x0002, ///< 该气压样本被 EKF 接受
        kDebugBaroRefReset = 0x0004, ///< 本周期重建了气压参考系(EKF 高度归零)
        kDebugRangeNew     = 0x0008, ///< 本周期收到了新的测距样本
        kDebugRangeOk      = 0x0010, ///< 该测距样本被 EKF 接受
        kDebugFlowNew      = 0x0020, ///< 本周期收到了新的光流样本
        kDebugFlowOk       = 0x0040, ///< 该光流样本换算出的速度被 EKF 接受
    };

    /** 计算负载统计(控制线程填, 日志线程发) */
    struct PerfStats
    {
        uint32_t loopAvgCycle{0}; ///< 主循环平均耗时 [CPU 周期]
        uint32_t loopMaxCycle{0}; ///< 主循环最大耗时
        uint32_t ekfCycle{0};     ///< EKF 慢路径平均耗时
        uint32_t ctrlCycle{0};    ///< 控制器 + 混控平均耗时
        uint32_t logCycle{0};     ///< 日志打包/落盘平均耗时
        uint32_t ioCycle{0};      ///< IMU FIFO + 气压计读取平均耗时
        uint16_t samples{0};      ///< 本统计窗口内的主循环次数
        uint16_t drops{0};        ///< 本窗口内因为队列满被丢掉的日志条数
    };

    struct LinkStatistics
    {
        uint16_t uplinkStreak{0u};
        /** 链路线程本地维护计数，再通过 LockedCell 整体发布给读者。 */
        uint32_t linkBytesIn{0};    ///< 从模块搬进来的字节数(一直 0 = 模块没吐数据/接线)
        uint32_t linkRxFrames{0};   ///< 解出的**合法**上行帧数(字节在涨而它为 0 ⇒ 全是噪声/错帧)
        uint32_t linkRxRejects{0};  ///< 被拒的字节处数(CRC 错/类型非法; 坏帧只前进 1 字节)
        uint32_t linkAdmitRejects{0};///< 解出来了但**没通过采信前置**的帧数(帧合法 ≠ 该采信)
        uint32_t linkCmdsApplied{0};///< 真正写进共享状态的帧数(收帧在涨而它不涨 ⇒ 全是看不懂的帧)
        uint32_t linkTxFrames{0};   ///< 成功发出的下行帧数
        uint32_t linkTxFail{0};     ///< 发送失败次数
        uint32_t linkTxSkipGps{0};  ///< 轮转到 DL_GPS 而跳过的次数(kGnss=False)
        /** 时分调度的静默、连续有效帧、窗口及上行丢包状态。 */
        uint8_t  linkMuted{0};       ///< 1 = 静默(只收不发): 连续丢主端帧 / 还没攒够恢复帧
        uint8_t  linkGoodStreak{0};  ///< 连续采信的主端帧数(静默后要 ≥ kResyncGoodSlots 才恢复)
        uint8_t  linkWindowArmed{0}; ///< 1 = 当前锚点打开了本机下行窗口(轮得上发车)
        uint8_t  linkUlLossPct{0};   ///< 从端视角的上行丢包 %(按 SEQ 距离算, 不是按时间槽)
        /** v2 诊断: 发送窗口内被丢弃的 RX 字节(半双工换向/串扰瞬变), 以及各位置累计收帧数。 */
        uint32_t linkTxWinBytes{0};
        uint32_t linkRxPos[4]{0u, 0u, 0u, 0u}; ///< U0..U3 累计收帧(定位"丢在哪一帧")
    };

    // 六、小工具
    /** 一条日志经 DLX 打包后的字节数(负载 140 + 帧头 2) */
    constexpr uint16_t kFlightLogPayloadBytes = 140;
    constexpr uint16_t kFlightLogFrameBytes   = kFlightLogPayloadBytes + 2u;

    /** 将 GPIOProfile 转为端口引脚名，输出到调用方提供的缓冲。 */
    inline const char *formatPin(GPIOProfile pin, char *buf, size_t bufBytes)
    {
        const uint8_t raw  = static_cast<uint8_t>(pin);
        const uint8_t port = static_cast<uint8_t>((raw >> 4) & 0x0F); // 0=A, 1=B, ...
        const uint8_t num  = static_cast<uint8_t>(raw & 0x0F);
        snprintf(buf, bufBytes, "P%c%u", static_cast<char>('A' + port), static_cast<unsigned>(num));
        return buf;
    }

    /** 启动、飞行与日志管理流程共用的硬件句柄及运行上下文。 */
    struct ReleaseContext
    {
        USART              *usart;     ///< 调试串口
        DMA                *serialDma; ///< 串口 DMA 发送
        ByteBuffer         *txBuf;     ///< 发送缓冲(与 DMA 绑定)
        RingByteBuffer     *uartRx;    ///< 串口接收环形缓冲
        DLX_ProtocolBuffer *protocol;  ///< DLX 协议(生成代码; 收发地面模式报文与日志)
        FlashManager       *fs;        ///< 文件系统(串口调试模式为 nullptr)
        GPIO               *ledRun;    ///< 运行指示灯
        BootMode            mode;      ///< 当前模式
        /** 非拥有的日志渠道指针；信箱由外部保证生命周期，生产方不访问协议或 DMA。 */
        LogChannel         *channel;   ///< 见 app_log_channel.hpp
    };

    // 全局句柄(裸指针/裸变量: 声明在头, **定义在 release_header.cpp**)
    extern USART          *debugPort;     ///< 文本日志出口(飞行模式置 nullptr = 完全静音)
    extern ReleaseContext *logContext;    ///< 文本日志要用的串口/协议上下文(main 里绑定一次)
    extern bool            flashLedPhase; ///< 擦除进度灯的当前相位(低电平点亮)

    /** 等待前次 DMA 完成后重置并启动本次传输；reset 会清 TC，必须紧接 start。 */
    inline void dmaSendStart(ReleaseContext &ctx)
    {
        ctx.serialDma->wait(); // 回收上一次(没有在跑就立刻返回)
        if (ctx.txBuf->used() == 0) {
            return;
        }
        ctx.serialDma->reset(ctx.txBuf->used());
        ctx.serialDma->start();
    }

    /** 把 txBuf 里的内容阻塞发完(仅地面模式用; 飞行/调试模式的流转发在日志线程里) */
    inline void flushSerial(ReleaseContext &ctx)
    {
        dmaSendStart(ctx);
        ctx.serialDma->wait(); // 没有在跑的传输就立刻返回
        ctx.txBuf->reset();
    }

    inline void flushBootLog(ReleaseContext &ctx)
    {
        ctx.serialDma->wait();
        ctx.txBuf->reset();
        appendLogMailbox(ctx.channel, *ctx.protocol, *ctx.txBuf);
        flushSerial(ctx);
    }

    /**
     * @brief 把 vsnprintf 的结果整理成"可以整条投进文本信箱的一行"
     *
     * 这个函数只做三件现场踩过坑的事(2026-10-18):
     *
     *  ① **超过缓冲就截断, 但必须把行尾换行补回来**. vsnprintf 截断会连末尾的 "\r\n" 一起
     *     切掉, 上位机按 '\n' 切行时这一行就和下一条粘在一起(现场: [Link] 那行 186 字节,
     *     被切成 160 字节后 "...(连" 后面直接粘下一行, 看起来像"日志缺了几行").
     *  ② **截断点必须落在 UTF-8 字符边界上**. 中文一个字 3 字节, 从中间切开会变成替换字符,
     *     而且多出来的半个字符还会影响后续显示.
     *  ③ **不再补"偶数长度"的空格**. 原来那句 `if (n & 1) line[n++] = ' '` 把空格补在
     *     '\r\n' **之后**: 上位机切出来一行 "\r\n" 一条空行 + 一行 " " —— 现场一次启动能看到
     *     26 条莫名其妙的空 [INFO] 行. SimpleLog 帧头里有明确的长度字段, 载荷奇偶不影响解析.
     */
    inline uint16_t sanitizeTextLine(char *line, int n)
    {
        const int cap = static_cast<int>(textmbox::LINE_MAX_BYTES);
        if (n <= 0) {
            return 0;
        }
        int len = (n >= cap) ? (cap - 1) : n; // vsnprintf 最多写了 cap-1 个字符
        if (len != n) {
            // 被截断了: 先把最后一个不完整的 UTF-8 字符丢掉(最多回退 3 个续字节)
            int i = len;
            while (i > 0 && (static_cast<uint8_t>(line[i - 1]) & 0xC0u) == 0x80u) {
                --i;
            }
            if (i > 0) {
                const uint8_t lead = static_cast<uint8_t>(line[i - 1]);
                int           clen = 1;
                if ((lead & 0xE0u) == 0xC0u) {
                    clen = 2;
                } else if ((lead & 0xF0u) == 0xE0u) {
                    clen = 3;
                } else if ((lead & 0xF8u) == 0xF0u) {
                    clen = 4;
                }
                if (i - 1 + clen > len) {
                    len = i - 1; // 末尾这个字符不完整 -> 整个丢掉
                }
            }
            if (len > cap - 3) {
                len = cap - 3; // 给 "\r\n" 腾两字节('缺字符'比缺换行危害小)
            }
            line[len++] = '\r';
            line[len++] = '\n';
        }
        return static_cast<uint16_t>(len);
    }

    inline void postLogLine(const char *fmt, va_list args)
    {
        if (!cfg::feature::kVerboseBootLog) {
            return;
        }
        if (logContext == nullptr || logContext->channel == nullptr ||
            logContext->channel->textMailbox == nullptr) {
            return; // 还没有人接这条渠道: 直接丢弃(绝不自作主张去发)
        }
        // 缓冲在**调用方栈**上: 不再共用那份静态行缓冲 —— 线程/中断同时打也不会踩。
        char line[textmbox::LINE_MAX_BYTES];
        const int n = vsnprintf(line, sizeof(line), fmt, args);
        const uint16_t len = sanitizeTextLine(line, n);
        if (len == 0u) {
            return;
        }
        logContext->channel->textMailbox->post(line, len);
    }

    inline void logLine(const char *fmt, ...) __attribute__((format(printf, 1, 2)))
    {
        va_list args;
        va_start(args, fmt);
        postLogLine(fmt, args);
        va_end(args);
    }

    inline void bootLogLine(const char *fmt, ...) __attribute__((format(printf, 1, 2)))
    {
        va_list args;
        va_start(args, fmt);
        postLogLine(fmt, args);
        va_end(args);
        if (logContext != nullptr) flushBootLog(*logContext);
    }

    //便利函数.

    #define THROWS_R(reason) logLine("[Thrl] %s @%s@%s@%d\n",reason,__FILE__,__func__,__LINE__)

    #define THROWS logLine("[Thrl] %s @%s@%s@%d\n",__FILE__,__func__,__LINE__)

    /** 解析到数据不足为止；已消费的帧或噪声持续前进，半帧留待下次。仅由当前协议拥有者调用。 */
    inline void drainDlx(DLX_ProtocolBuffer &protocol, RingByteBuffer &rx)
    {
        while (rx.available() >= 2u && protocol.check() != DLXCheckResult::FAILED_INSUFFICIENT) {
        }
    }

    /** 发一条地面应答(用 KSP 生成的 GroundReplyW) */
    inline void replyGround(ReleaseContext &ctx, uint16_t op, app::GroundStatus status, uint32_t sessionId,
                            uint32_t value)
    {
        ctx.txBuf->reset();
        ctx.protocol->GroundReplyW(op, static_cast<uint16_t>(status), sessionId, value);
        flushSerial(ctx);
    }

    /** flash 擦除/格式化进度回调: 每擦完一个 64KB 块翻一次擦除指示灯(低电平点亮) */
    inline void onFlashBusy()
    {
        static GPIO led(cfg::hw::led::kFlash);
        static bool inited = false;
        if (!inited) {
            led.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
            led    = 1;
            inited = true;
        }
        flashLedPhase = !flashLedPhase;
        led           = flashLedPhase ? 0 : 1;
    }

    /** 擦除或格式化结束时熄灭进度灯。 */
    inline void onFlashIdle()
    {
        static GPIO led(cfg::hw::led::kFlash);
        static bool inited = false;
        if (!inited) {
            led.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
            inited = true;
        }
        flashLedPhase = false;
        led           = 1; // 熄灭(低电平点亮)
    }

    /** 向当前协议缓冲追加一条飞行日志帧，不重置缓冲；调用前需保留 kFlightLogFrameBytes 空间。 */
    inline bool writeLogEntry(DLX_ProtocolBuffer &protocol, const LogEntry &e)
    {
        return protocol.FlightLogW(e.sessionId, e.seq, e.tickMs, e.flags, e.events, e.linkAgeMs,
                                   e.loopPeriodUs, const_cast<float *>(e.quat),
                                   const_cast<float *>(e.bodyRate), const_cast<float *>(e.accelG),
                                   const_cast<float *>(e.rateSetpoint), const_cast<float *>(e.torque),
                                   e.targetPitchDeg, e.targetRollDeg, e.targetHeightM, e.throttle,
                                   e.manualThrottle, e.heightM, e.vertVelMps, e.baroRelM, e.baroAbsM,
                                   const_cast<float *>(e.gyroBias), const_cast<uint16_t *>(e.motor));
    }

    /**
     * @brief 把一条调试采样编成 DebugSensor 报文, 追加在当前协议缓冲后面(不 reset)。
     *
     * 字段顺序与 KSP 的 kmpfly.Protocol.DebugSensor 一一对应(两边必须同时改)。
     */
    inline bool writeDebugSample(DLX_ProtocolBuffer &protocol, const DebugSample &d)
    {
        return protocol.DebugSensorW(d.tickMs, d.flags, d.flowSqual, d.rangeStatus, d.pressurePa,
                                     d.altitudeM, d.relativeAltitudeM, d.flowDx, d.flowDy, d.flowDtMs,
                                     d.rangeRawMm, d.rangeM, d.ekfHeightM, d.ekfVertVelMps, d.ekfVelXMps,
                                     d.ekfVelYMps);
    }

    /** 按 IMU 相对机体的偏航角重映射传感器轴，支持 0/90/180/270 度。 */
    inline void remapSensorFrame(int32_t yawDeg, float &x, float &y, float &z)
    {
        switch (yawDeg) {
            case 90: {
                const float t = x;
                x             = y;
                y             = -t;
                break;
            }
            case 180: {
                x = -x;
                y = -y;
                break;
            }
            case 270: {
                const float t = x;
                x             = -y;
                y             = t;
                break;
            }
            default:
                break; // 0°
        }
        (void)z;
    }
} // namespace dlx

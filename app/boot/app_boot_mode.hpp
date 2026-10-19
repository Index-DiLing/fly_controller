#pragma once
#include "flight_config.hpp"
#include "app/runtime/release_header.h"
#include "dlx_gpio.hpp"
#include "dlx_delay.hpp"
// 启动模式判定：读取跳线及启动期串口命令。

namespace dlx
{
// 三、上电模式判定(与飞行逻辑解耦的独立函数)
/**
 * @brief 跳线帽(PB8 & PB9)是否接上
 *
 * 两相检测, 兼容两种接法:
 *   - 一个跳线帽把 PB8 与 PB9 短接: 驱动一端为低, 另一端读回低 → 接上;
 *   - 两脚各自跳线到 GND: 任一脚被拉低 → 接上。
 * 检测期间两个脚一会儿输出一会儿输入(带上拉), 不会对外部电路灌大电流(只经过跳线)。
 *
 * @param aLow / bLow 两相各自的读回结果(接上跳线时该脚被拉低), 只用于上电日志:
 *        "两脚都低"= 跳线帽 / "只有一脚低"= 单脚对地 / "都不低"= 没插 —— 插歪/接触不良时靠它区分。
 */
inline bool groundJumpersPresent(bool &aLow, bool &bLow)
{
    GPIO jumpA(cfg::hw::jumper::kA);
    GPIO jumpB(cfg::hw::jumper::kB);

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
    uint16_t op;        ///< 收到的原始 op(非解锁命令时用来报"上位机发的是什么")
    bool    legacy;     ///< true = 来自旧版 UnBlock(param=1 调试 / 2 管理) 报文
};

inline void onBootUnBlock(UnBlock *u, void *raw)
{
    BootCommand *b = static_cast<BootCommand *>(raw);
    b->mode        = u->param();
    b->op          = static_cast<uint16_t>(GroundOp::Unlock);
    b->legacy      = true;
    b->got         = true;
}

inline void onBootGroundCmd(GroundCmd *cmd, void *raw)
{
    BootCommand *b    = static_cast<BootCommand *>(raw);
    const uint16_t op = cmd->op();
    // op == 1 是现在的"解锁"; op == 0 是早期上位机用过的写法, 一并兼容
    if (op == static_cast<uint16_t>(GroundOp::Unlock) || op == 0u) {
        b->mode = static_cast<uint8_t>(cmd->mode());
    } else {
        b->mode = 0xFF; // 非解锁命令: 当作无效(由判定循环报出来)
    }
    b->op     = op;
    b->legacy = false;
    b->got    = true;
    // 启动回调只更新本地状态，不在协议解析回调中复用 TX 缓冲或等待 DMA。
}

/** 模式名(日志用) */
inline const char *bootModeName(BootMode mode)
{
    switch (mode) {
        case BootMode::Flight:
            return "飞行";
        case BootMode::SerialDebug:
            return "串口调试";
        case BootMode::LogManage:
            return "日志管理";
        default:
            return "未知";
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
inline BootDecision decideBootMode(ReleaseContext &ctx)
{
    BootDecision decision = {BootMode::Flight};
    bool         jumperA  = false;
    bool         jumperB  = false;
    /** 跳线状态只用于本次判定，对外只返回启动模式。 */
    const bool groundMode = groundJumpersPresent(jumperA, jumperB);
    if (!groundMode) {
        return decision;
    }

    bootLogLine("\r\n[Boot] 检测到地面模式跳线: PB8(A)=%s PB9(B)=%s, 等待上位机解锁命令...\r\n",
            jumperA ? "低" : "高", jumperB ? "低" : "高");
    bootLogLine("[Boot] 等待策略: 超时=%u ms(0=一直等), 状态灯每 %u ms 翻转(慢闪=能收命令, 但还没解锁)\r\n",
            static_cast<unsigned>(cfg::ground::kWaitTimeoutMs),
            static_cast<unsigned>(cfg::ground::kBlinkMs));
    BootCommand cmd = {false, 0, 0, false};
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
        if ((now - lastBlinkMs) >= cfg::ground::kBlinkMs) {
            lastBlinkMs = now;
            ledOn       = !ledOn;
            *ctx.ledRun = ledOn ? 1 : 0;
        }

        drainDlx(*ctx.protocol, *ctx.uartRx);
        if (cmd.got) {
            cmd.got = false;
            if (cmd.mode <= 2u) {
                decision.mode = static_cast<BootMode>(cmd.mode);
                bootLogLine("[Boot] 收到解锁命令(mode=%u, 来自%s报文) -> 进入%s模式, 等待 %u ms\r\n",
                        static_cast<unsigned>(cmd.mode), cmd.legacy ? "旧版 UnBlock" : "GroundCmd",
                        bootModeName(decision.mode), static_cast<unsigned>(now - startMs));
                return decision;
            }
            bootLogLine("[Boot] 命令无效(param=%u, 期望 0=飞行/1=串口调试/2=日志管理), 继续等待\r\n",
                    static_cast<unsigned>(cmd.mode));
            if (cmd.op != static_cast<uint16_t>(GroundOp::Unlock)) {
                // 不是解锁命令: 上位机版本的 op 表对不上时(或误发了管理命令), 这一行能直接看出来
                bootLogLine("[Boot] 收到的其实是 op=%u(0x%04X), 不是 Unlock —— 检查上位机的 GroundOp 表\r\n",
                        static_cast<unsigned>(cmd.op), static_cast<unsigned>(cmd.op));
            }
            replyGround(ctx, static_cast<uint16_t>(GroundOp::Unlock), app::GroundStatus::BadArgument, 0, 0);
            continue;
        }

        if (cfg::ground::kWaitTimeoutMs != 0u && (now - startMs) > cfg::ground::kWaitTimeoutMs) {
            bootLogLine("[Boot] 等待超时(%u ms), 进入日志管理模式(不动电机)\r\n",
                    static_cast<unsigned>(cfg::ground::kWaitTimeoutMs));
            decision.mode = BootMode::LogManage;
            return decision;
        }
    }
}
} // namespace dlx

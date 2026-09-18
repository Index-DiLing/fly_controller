#pragma once
#include "flight_config_struct.hpp"
#include "W25Q128/dlx_flash_manager.hpp" // 需要 FlashManager 的完整定义
#include "rtthread.h"

//===================================================================================================
// release_log_manage.hpp —— 日志管理模式(与飞行完全分离的一个模块)
//
// 上位机命令(全部走 KSP 生成的 DLX 报文, 见 notebook/2026-09-11_dlx_protocol.md):
//   GroundCmd(op=Unlock, mode)      解锁, 返回目标模式(0 飞行 / 1 串口调试 / 2 日志管理)
//   GroundCmd(op=ListSessions)      逐条回 SessionInfo, 最后回 GroundReply(value=会话数)
//   GroundCmd(op=DeleteOldest,count)丢掉最老的 count 个会话(环形队列语义)
//   GroundCmd(op=DumpSession, id)   把这个会话的日志按 FlightLog 帧流回传(整会话)
//   GroundCmd(op=FormatFlash)       整片擦除重建(几十秒, 丢日志+参数)
//   GroundCmd(op=Ping)              回一条应答, 探链路
//   旧版 UnBlock(param)             等价于 Unlock(mode=param), 兼容首飞版操作习惯
//
// 设计要点:
//   - 解析回调里只"记下命令", 真正的 flash 读写/DMA 发送都放在主循环做 —— 避免在解析回调里
//     做重活(擦除几十秒、回传几千条), 也让流程只有一条主线, 好排查;
//   - 全程单线程, 没有临界区、没有并发访问文件系统的风险;
//   - 本模式不初始化 BMI088/NRF/TIM1/DShot, 物理上不可能输出电机信号。
//
// 为什么是"头文件实现"(而不是 release_log_manage.cpp):
//   EIDE 的"参与编译的文件"列表存在 .eide/(本地工程状态, 不入库), 漏加一个 .cpp 就会在链接期报
//     L6218E: Undefined symbol dlx::runLogManage(dlx::ReleaseContext&)
//   放成头文件后, 只要 release.cpp include 它, 就一定会被编进去, 不会再出现这个错。
//   (DL_LIB 里的 FlashManager / 各驱动也都是纯头文件, 风格一致; 本模块只有下面一个 inline 函数。)
//===================================================================================================
namespace dlx
{

    /** 内部实现细节(命令暂存 / 上下文 / 解析回调), 不是公共接口 */
    namespace log_manage_detail
    {
        /** 上位机命令(由解析回调填, 主循环消费) */
        struct ManageCommand
        {
            volatile bool got;       ///< 有一条待处理命令
            uint16_t      op;        ///< GroundOp
            uint16_t      mode;      ///< Unlock 的目标模式
            uint32_t      sessionId; ///< DumpSession
            uint32_t      count;     ///< DeleteOldest
            bool          legacy;    ///< true = 来自旧版 UnBlock(param)
        };

        inline ManageCommand &cmd()
        {
            static ManageCommand c = {};
            return c;
        }

        inline ReleaseContext *&ctxRef()
        {
            static ReleaseContext *p = nullptr;
            return p;
        }

        // ---- 解析回调: 只记命令, 不做重活 ----
        inline void onGroundCmd(GroundCmd *in, void *)
        {
            ManageCommand &c = cmd();
            c.op             = in->op();
            c.mode           = in->mode();
            c.sessionId      = in->sessionId();
            c.count          = in->count();
            c.legacy         = false;
            c.got            = true;
        }

        inline void onUnBlock(UnBlock *u, void *)
        {
            ManageCommand &c = cmd();
            c.op             = static_cast<uint16_t>(GroundOp::Unlock);
            c.mode           = u->param();
            c.sessionId      = 0;
            c.count          = 0;
            c.legacy         = true;
            c.got            = true;
        }

        // ---- 列会话回调 ----
        struct ListCtx
        {
            uint32_t count;  ///< 已列出几个会话
            uint32_t lastId; ///< 最后一个会话号
        };

        inline bool listSessionCb(const FlashLogSessionInfo &info, void *raw)
        {
            ReleaseContext *ctx = ctxRef();
            ListCtx        *lc  = static_cast<ListCtx *>(raw);
            // 条目数没存在槽头里, 只能扫一遍(地面操作, 慢一点可以接受: 一个会话约 0.2~7s)
            uint32_t      entries = 0;
            ExceptionCode err     = ExceptionCode::FLASH_OK;
            if (ctx->fs->countEntries(info.sessionId, entries, err) != FunctionResult::SUCCESS) {
                entries = 0;
            }
            uint16_t active   = info.active ? 1u : 0u;
            uint16_t reserved = 0u;
            if (info.active) {
                const uint32_t limit = ctx->fs->sessionEntryLimit();
                const uint32_t used  = ctx->fs->currentEntryCount();
                const uint32_t left  = (limit > used) ? (limit - used) : 0u;
                reserved             = (left > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(left);
            }

            // 攒够一批再发, 别一个会话一次 DMA
            if (ctx->txBuf->remaining() < 20u) {
                flushSerial(*ctx);
            }
            ctx->protocol->SessionInfoW(info.sessionId, info.slotCount, entries, active, reserved);

            ++lc->count;
            lc->lastId = info.sessionId;
            return true;
        }

        // ---- 回传会话回调 ----
        struct DumpCtx
        {
            uint32_t sent; ///< 已回传条数
        };

        inline bool dumpEntryCb(const LogEntry &entry, void *raw)
        {
            ReleaseContext *ctx = ctxRef();
            DumpCtx        *dc  = static_cast<DumpCtx *>(raw);
            if (ctx->txBuf->remaining() < kFlightLogFrameBytes) {
                flushSerial(*ctx); // 分批发(单批约 13 条)
            }
            writeLogEntry(*ctx->protocol, entry); // 现场转成 DLX 的 FlightLog 帧
            ++dc->sent;
            return true;
        }

        /** 状态灯: 管理模式慢闪(= 我在跑) */
        inline void tickLed(ReleaseContext &ctx, uint32_t &lastMs, bool &on)
        {
            const uint32_t now = rt_tick_get_millisecond();
            if ((now - lastMs) >= kConfig.manageBlinkMs) {
                lastMs      = now;
                on          = !on;
                *ctx.ledRun = on;
            }
        }
    } // namespace log_manage_detail

    /**
     * @brief 日志管理模式主循环
     * @param ctx 上下文(fs 必须已 init 成功)
     * @return 上位机要求切换到的模式(内部是死循环, 只有收到解锁命令才返回)
     */
    inline BootMode runLogManage(ReleaseContext &ctx)
    {
        using namespace log_manage_detail;

        ctxRef() = &ctx;
        ctx.protocol->setGroundCmdCallbackFunction(onGroundCmd, nullptr);
        ctx.protocol->setUnBlockCallbackFunction(onUnBlock, nullptr);
        ctx.uartRx->reset(); // 进入管理模式时丢掉之前的杂散字节(避免帧错位)

        logLine("\r\n[Manage] 日志管理模式: flash %s, 会话 %u, 已写 %u 条, 预留 %u 槽\r\n",
                ctx.fs->isReady() ? "就绪" : "未就绪",
                static_cast<unsigned>(ctx.fs->currentSessionId()),
                static_cast<unsigned>(ctx.fs->currentEntryCount()),
                static_cast<unsigned>(ctx.fs->sessionReservedSlots()));
        logLine("[Manage] 命令: 2=列会话 3=删最老N个 4=回传会话 5=格式化 1=解锁切模式 6=Ping\r\n");
        replyGround(ctx, static_cast<uint16_t>(GroundOp::Unlock), GroundStatus::Ok,
                    ctx.fs->currentSessionId(), 0);

        uint32_t      lastLedMs = rt_tick_get_millisecond();
        bool          ledOn     = false;
        ExceptionCode err       = ExceptionCode::FLASH_OK;

        while (true) {
            tickLed(ctx, lastLedMs, ledOn);

            // 把收到的帧都解析掉(回调只把命令记进 cmd(); 噪声字节由 drainDlx 负责跳掉)
            drainDlx(*ctx.protocol, *ctx.uartRx);
            ManageCommand &c = cmd();
            if (!c.got) {
                continue;
            }
            // 取走命令
            const uint16_t op        = c.op;
            const uint16_t mode      = c.mode;
            const uint32_t sessionId = c.sessionId;
            const uint32_t count     = c.count;
            const bool     legacy    = c.legacy;
            c.got                    = false;

            if (legacy && mode > 2u) { // 旧版 UnBlock 参数越界
                replyGround(ctx, static_cast<uint16_t>(GroundOp::Unlock), GroundStatus::BadArgument, 0, 0);
                continue;
            }

            switch (static_cast<GroundOp>(op)) {
            case GroundOp::Unlock: {
                if (mode > 2u) {
                    replyGround(ctx, op, GroundStatus::BadArgument, 0, 0);
                    continue;
                }
                replyGround(ctx, op, GroundStatus::Ok, 0, mode);
                logLine("[Manage] 解锁: 切到模式 %u\r\n", static_cast<unsigned>(mode));
                return static_cast<BootMode>(mode);
            }

            case GroundOp::Ping: {
                replyGround(ctx, op, GroundStatus::Ok, ctx.fs->currentSessionId(), 0);
                break;
            }

            case GroundOp::ListSessions: {
                logLine("[Manage] 列会话(条目数要逐个会话扫一遍)...\r\n");
                ListCtx lc = {0, 0};
                ctx.txBuf->reset();
                const FunctionResult r = ctx.fs->forEachSession(listSessionCb, &lc, err);
                flushSerial(ctx);
                replyGround(ctx, op,
                            (r == FunctionResult::SUCCESS) ? GroundStatus::Ok : GroundStatus::FlashFailed,
                            lc.lastId, lc.count);
                logLine("[Manage] 共 %u 个会话\r\n", static_cast<unsigned>(lc.count));
                break;
            }

            case GroundOp::DeleteOldest: {
                if (count == 0u) {
                    replyGround(ctx, op, GroundStatus::BadArgument, 0, 0);
                    continue;
                }
                const uint32_t       reservedBefore = ctx.fs->sessionReservedSlots();
                const FunctionResult r              = ctx.fs->dropSessions(count, onFlashBusy, err);
                const uint32_t       reservedAfter  = ctx.fs->sessionReservedSlots();
                onFlashIdle(); // 擦除结束: 熄灭进度灯
                logLine("[Manage] 删最老 %u 个会话 -> %d err=0x%04X\r\n", static_cast<unsigned>(count),
                        static_cast<int>(r), static_cast<unsigned>(err));
                replyGround(ctx, op,
                            (r == FunctionResult::SUCCESS) ? GroundStatus::Ok : GroundStatus::FlashFailed,
                            ctx.fs->currentSessionId(),
                            (reservedAfter > reservedBefore) ? (reservedAfter - reservedBefore) : 0u);
                break;
            }

            case GroundOp::DumpSession: {
                if (sessionId == 0u) {
                    replyGround(ctx, op, GroundStatus::BadArgument, 0, 0);
                    continue;
                }
                logLine("[Manage] 回传会话 %u ...\r\n", static_cast<unsigned>(sessionId));
                DumpCtx dc = {0};
                ctx.txBuf->reset();
                const FunctionResult r = ctx.fs->forEachEntry(sessionId, dumpEntryCb, &dc, err);
                flushSerial(ctx);
                const GroundStatus st =
                    (r == FunctionResult::SUCCESS)
                        ? GroundStatus::Ok
                        : ((err == ExceptionCode::FLASH_SESSION_NOT_FOUND) ? GroundStatus::NotFound
                                                                          : GroundStatus::FlashFailed);
                // 会话数据流发完后, 再回一条"发完了, 共 N 条"
                replyGround(ctx, op, st, sessionId, dc.sent);
                logLine("[Manage] 会话 %u 回传 %u 条\r\n", static_cast<unsigned>(sessionId),
                        static_cast<unsigned>(dc.sent));
                break;
            }

            case GroundOp::FormatFlash: {
                logLine("[Manage] 整片格式化(几十秒, 会丢全部日志和参数)...\r\n");
                FunctionResult r = ctx.fs->format(onFlashBusy, err);
                if (r == FunctionResult::SUCCESS) {
                    ctx.fs->setFullPolicy(FlashFullPolicy::Halt);
                    ctx.fs->setSessionSlotLimit(kConfig.manageLogSlots);
                    r = ctx.fs->init(FlashFullPolicy::Halt, onFlashBusy, err);
                }
                onFlashIdle(); // 擦除结束: 熄灭进度灯
                logLine("[Manage] 格式化 %s err=0x%04X\r\n",
                        (r == FunctionResult::SUCCESS) ? "完成" : "失败", static_cast<unsigned>(err));
                replyGround(ctx, op,
                            (r == FunctionResult::SUCCESS) ? GroundStatus::Ok : GroundStatus::FlashFailed,
                            ctx.fs->currentSessionId(), ctx.fs->sessionReservedSlots());
                break;
            }

            default:
                logLine("[Manage] 未知命令 op=%u\r\n", static_cast<unsigned>(op));
                replyGround(ctx, op, GroundStatus::BadCommand, 0, 0);
                break;
            }
        }
    }
} // namespace dlx

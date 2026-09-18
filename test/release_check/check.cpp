//===================================================================================================
// check.cpp —— 主机端(不用板子)自检: 发布版的"日志结构体 / DLX 报文(生成侧写法)"
//
//   powershell -ExecutionPolicy Bypass -File .\test\release_check\run.ps1
//
// 检查:
//   1. LogEntry 布局(无填充) 与文件系统几何(步长/每槽条数/单次上限)
//   2. 地面模式 4 条报文: 用 dlx.hpp 里的 XxxW() 打包 -> 长度/类型号/CRC4/字段回读一致
//   3. check() 解析 + 回调(GroundCmd) 与旧版 UnBlock 兼容
//   4. 日志回传帧: 用 writeLogEntry() 把 flash 的 POD 打成 DLX 的 FlightLog 帧, 字段能回读一致
//
// 硬件相关的(SPI/I2C/定时器/线程)不在这里, 只能上板验证。
//===================================================================================================
#include <stdio.h>
#include <string.h>
#include "flight_config_struct.hpp"
#include "W25Q128/dlx_flash_manager_config.h"
#include "dlx.hpp" // 注意: 自动生成的文件没有 include guard, 一个编译单元只包含一次

using namespace dlx;

static int g_pass = 0;
static int g_fail = 0;

static void check(bool ok, const char *what)
{
    if (ok) {
        ++g_pass;
        printf("  [PASS] %s\n", what);
    } else {
        ++g_fail;
        printf("  [FAIL] %s\n", what);
    }
}

/** 取一帧的帧头(大端) */
static uint16_t headOf(const uint8_t *frame)
{
    return (uint16_t)(((uint16_t)frame[0] << 8) | frame[1]);
}

// ---- 线上字段解码(与生成器规则一致: 标量大端, 定长数组原始小端) ----
static uint16_t beU16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }
static uint32_t beU32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static float beF32(const uint8_t *p)
{
    const uint32_t v = beU32(p);
    float          f;
    memcpy(&f, &v, 4);
    return f;
}
static uint16_t leU16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static float leF32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}

int main()
{
    printf("=== release 发布项自检 ===\n");

    // ---------------------------------------------------------------- 1. 结构体布局
    printf("\n[1] 日志结构体布局\n");
    printf("    sizeof(LogEntry)=%u  业务字段=%u  帧步长=%u  每槽=%u 条  单次上限=%u 条\n",
           (unsigned)sizeof(LogEntry), (unsigned)(sizeof(LogEntry) - 12u),
           (unsigned)FLASH_LOG_ENTRY_STRIDE, (unsigned)FLASH_LOG_ENTRIES_PER_SLOT,
           (unsigned)(FLASH_LOG_SESSION_SLOT_LIMIT * FLASH_LOG_ENTRIES_PER_SLOT));
    printf("    50Hz 记一条 -> 单次可记录 %.1f 秒\n",
           (double)(FLASH_LOG_SESSION_SLOT_LIMIT * FLASH_LOG_ENTRIES_PER_SLOT) * 0.02);
    check(sizeof(LogEntry) % 4u == 0u, "LogEntry 尺寸是 4 的整数倍(无填充)");
    check(offsetof(LogEntry, motor) + sizeof(((LogEntry *)0)->motor) == sizeof(LogEntry),
          "业务字段排布无空隙(最后一个字段正好收尾)");
    check(FLASH_LOG_ENTRY_STRIDE == flashAlign4(sizeof(LogEntry) + 2u), "帧步长 = 对齐4(结构体+2)");
    check(FLASH_LOG_SESSION_SLOT_LIMIT == 32u, "默认单次日志预留 32 槽");
    check(kConfig.flightLogSlots == 32u && kConfig.flightLogPeriodMs == 20u,
          "发布配置: 32 槽 / 50Hz");

    // ---------------------------------------------------------------- 2. 生成侧报文
    printf("\n[2] 地面模式报文(用 dlx.hpp 的 XxxW 打包)\n");
    {
        uint8_t txRaw[64];
        ByteBuffer txBuf(txRaw, sizeof(txRaw));
        DLX_ProtocolBuffer proto(txBuf);

        // --- GroundCmd ---
        txBuf.reset();
        check(proto.GroundCmdW((uint16_t)GroundOp::DeleteOldest, 2, 0x1234ABCDu, 3), "GroundCmdW 打包");
        const uint16_t cmdHead = headOf(txRaw);
        check(txBuf.used() == 14u, "GroundCmd 帧长 = 2 + 12");
        check(((cmdHead >> 5) & 0x3FFu) == 11u, "GroundCmd 类型号 = 11");
        check(((cmdHead >> 1) & 0xFu) == proto.crc4_itu(txRaw + 2, 12), "GroundCmd 头部 CRC4 覆盖 12 字节负载");
        {
            ByteBuffer view(txRaw + 2, 12);
            GroundCmd  back(&view);
            check(back.op() == (uint16_t)GroundOp::DeleteOldest && back.mode() == 2u
                      && back.sessionId() == 0x1234ABCDu && back.count() == 3u,
                  "GroundCmd 字段回读一致");
        }

        // --- GroundReply ---
        txBuf.reset();
        check(proto.GroundReplyW((uint16_t)GroundOp::ListSessions, (uint16_t)GroundStatus::Ok, 9u, 42u),
              "GroundReplyW 打包");
        const uint16_t repHead = headOf(txRaw);
        check(txBuf.used() == 14u, "GroundReply 帧长 = 2 + 12");
        check(((repHead >> 5) & 0x3FFu) == 12u, "GroundReply 类型号 = 12");
        check(((repHead >> 1) & 0xFu) == proto.crc4_itu(txRaw + 2, 12), "GroundReply CRC4 正确");
        check(beU16(txRaw + 2) == (uint16_t)GroundOp::ListSessions
                  && beU16(txRaw + 4) == (uint16_t)GroundStatus::Ok && beU32(txRaw + 6) == 9u
                  && beU32(txRaw + 10) == 42u,
              "GroundReply 负载字段正确(标量大端)");

        // --- SessionInfo ---
        txBuf.reset();
        check(proto.SessionInfoW(7u, 3u, 2655u, 1u, 123u), "SessionInfoW 打包");
        const uint16_t siHead = headOf(txRaw);
        check(txBuf.used() == 18u, "SessionInfo 帧长 = 2 + 16");
        check(((siHead >> 5) & 0x3FFu) == 13u, "SessionInfo 类型号 = 13");
        check(beU32(txRaw + 2) == 7u && beU32(txRaw + 6) == 3u && beU32(txRaw + 10) == 2655u
                  && beU16(txRaw + 14) == 1u && beU16(txRaw + 16) == 123u,
              "SessionInfo 负载字段正确");

        // --- PerfMonitor ---
        txBuf.reset();
        check(proto.PerfMonitorW(123456u, 987654u, 11111u, 2222u, 3333u, 4444u, 100u, 2u),
              "PerfMonitorW 打包");
        const uint16_t pmHead = headOf(txRaw);
        check(txBuf.used() == 30u, "PerfMonitor 帧长 = 2 + 28");
        check(((pmHead >> 5) & 0x3FFu) == 14u, "PerfMonitor 类型号 = 14");
        check(beU32(txRaw + 2) == 123456u && beU32(txRaw + 6) == 987654u
                  && beU32(txRaw + 10) == 11111u && beU32(txRaw + 14) == 2222u
                  && beU32(txRaw + 18) == 3333u && beU32(txRaw + 22) == 4444u
                  && beU16(txRaw + 26) == 100u && beU16(txRaw + 28) == 2u,
              "PerfMonitor 负载字段正确");

        // --- FlightCoreStatus(发给遥控器): 布局是遥控器直接按偏移读的, 改动会破坏兼容 */
        txBuf.reset();
        check(proto.FlightCoreStatusW(1.0f, 2.0f, 3.0f, 4.0f, 11u, 22u, 33u, 44u, 5.5f,
                                      (uint8_t)kTelStatusBaroOk, (uint8_t)kTelErrImu),
              "FlightCoreStatusW 打包");
        const uint16_t fcsHead = headOf(txRaw);
        check(txBuf.used() == 32u, "FlightCoreStatus 帧长 = 2 + 30");
        check(((fcsHead >> 5) & 0x3FFu) == 8u, "FlightCoreStatus 类型号 = 8");
        // 下面的偏移都是"负载内偏移"(帧里要 +2 跳过帧头)
        check(beF32(txRaw + 2) == 1.0f && beF32(txRaw + 2 + 12) == 4.0f,
              "FlightCoreStatus 四元数在负载偏移 0~15(标量大端)");
        check(beU16(txRaw + 2 + 16) == 11u && beU16(txRaw + 2 + 18) == 22u
                  && beU16(txRaw + 2 + 20) == 33u && beU16(txRaw + 2 + 22) == 44u,
              "FlightCoreStatus 四路电机在负载偏移 16~23");
        check(beF32(txRaw + 2 + 24) == 5.5f, "FlightCoreStatus height 在负载偏移 24(气压计可用时是垂速)");
        check(txRaw[2 + 28] == (uint8_t)kTelStatusBaroOk && txRaw[2 + 29] == (uint8_t)kTelErrImu,
              "FlightCoreStatus status/error 在负载偏移 28/29, 各 1 字节");
    }

    // ---------------------------------------------------------------- 3. check() + 回调
    printf("\n[3] check() 解析与回调\n");
    {
        uint8_t txRaw[64];
        ByteBuffer txBuf(txRaw, sizeof(txRaw));
        DLX_ProtocolBuffer txProto(txBuf);
        txBuf.reset();
        txProto.GroundCmdW((uint16_t)GroundOp::Unlock, 2u, 0u, 0u);

        uint8_t rxRaw[64];
        ByteBuffer rxByteBuf(rxRaw, sizeof(rxRaw));
        RingByteBuffer rx(rxByteBuf);
        uint8_t    frameRaw[64];
        ByteBuffer frameBuf(frameRaw, sizeof(frameRaw));
        DLX_ProtocolBuffer rxProto(txBuf, &rx, &frameBuf);

        static uint16_t gotOp   = 0;
        static uint16_t gotMode = 0;
        gotOp = gotMode = 0;
        rxProto.setGroundCmdCallbackFunction(
            +[](GroundCmd *c, void *) {
                gotOp   = c->op();
                gotMode = c->mode();
            },
            nullptr);

        const uint8_t junk[3] = {0x00, 0x55, 0x7F}; // 前面塞垃圾, 应能重新同步
        rx.write(junk, 3);
        rx.write(txRaw, txBuf.used());
        drainDlx(rxProto, rx); // 直接用 MCU 侧那个工具函数: 噪声字节会被逐个跳掉
        check(gotOp == (uint16_t)GroundOp::Unlock && gotMode == 2u,
              "收到 GroundCmd(Unlock, mode=2) 并回调");

        // 半帧: 只给一半字节, check() 应该报 INSUFFICIENT 且不动缓冲
        uint8_t rxRaw2[32];
        ByteBuffer rxByteBuf2(rxRaw2, sizeof(rxRaw2));
        RingByteBuffer rx2(rxByteBuf2);
        rx2.write(txRaw, (uint16_t)(txBuf.used() / 2u));
        const uint16_t before = rx2.available();
        check(rxProto.check() == DLXCheckResult::FAILED_INSUFFICIENT && rx2.available() == before,
              "半帧不消费(返回 FAILED_INSUFFICIENT, 等字节攒够)");

        // 噪声: 帧头不合法(bit15=0) 时 check() 只吞 1 个字节并报 FAILED_HEAD
        uint8_t rxRaw4[16];
        ByteBuffer rxByteBuf4(rxRaw4, sizeof(rxRaw4));
        RingByteBuffer rx4(rxByteBuf4);
        DLX_ProtocolBuffer rxProto4(txBuf, &rx4, &frameBuf);
        const uint8_t noise[2] = {0x00, 0x55}; // peek_be -> 0x0055, bit15=0 不是帧头
        rx4.write(noise, 2);
        const uint16_t before4 = rx4.available();
        check(rxProto4.check() == DLXCheckResult::FAILED_HEAD && rx4.available() == before4 - 1u,
              "非法帧头返回 FAILED_HEAD 且只吞 1 个字节");

        // 未知类型: 帧头合法但 type 未定义, 同样只吞 1 个字节并报 FAILED_HEAD
        uint8_t rxRaw6[16];
        ByteBuffer rxByteBuf6(rxRaw6, sizeof(rxRaw6));
        RingByteBuffer rx6(rxByteBuf6);
        DLX_ProtocolBuffer rxProto6(txBuf, &rx6, &frameBuf);
        const uint8_t badType[2] = {0xFC, 0xE0}; // (1<<15)|(999<<5): type=999 未定义
        rx6.write(badType, 2);
        const uint16_t before6 = rx6.available();
        check(rxProto6.check() == DLXCheckResult::FAILED_HEAD && rx6.available() == before6 - 1u,
              "未知类型返回 FAILED_HEAD 且只吞 1 个字节");

        // CRC 错: 整帧被消费掉, 返回 FAILED_CRC(要删掉负载最后一个字节才会错)
        uint8_t rxRaw5[32];
        ByteBuffer rxByteBuf5(rxRaw5, sizeof(rxRaw5));
        RingByteBuffer rx5(rxByteBuf5);
        DLX_ProtocolBuffer rxProto5(txBuf, &rx5, &frameBuf);
        uint8_t badFrame[16];
        memcpy(badFrame, txRaw, txBuf.used());
        badFrame[txBuf.used() - 1u] ^= 0xFFu; // 只改负载, 帧头 CRC4 就对不上了
        rx5.write(badFrame, txBuf.used());
        check(rxProto5.check() == DLXCheckResult::FAILED_CRC && rx5.available() == 0u,
              "CRC 错返回 FAILED_CRC 且整帧已消费");

        // 旧版 UnBlock(param=2) 兼容
        uint8_t ubRaw[8];
        ByteBuffer ubBuf(ubRaw, sizeof(ubRaw));
        uint8_t param = 2;
        const uint16_t ubHead = (uint16_t)((1u << 15) | (1u << 5)
                                           | ((uint16_t)(txProto.crc4_itu(&param, 1) & 0xFu) << 1));
        ubBuf.write<uint16_t>(ubHead);
        ubBuf.write(&param, 1);

        uint8_t rxRaw3[16];
        ByteBuffer rxByteBuf3(rxRaw3, sizeof(rxRaw3));
        RingByteBuffer rx3(rxByteBuf3);
        DLX_ProtocolBuffer rxProto3(txBuf, &rx3, &frameBuf);
        static uint8_t gotParam = 0;
        gotParam = 0;
        rxProto3.setUnBlockCallbackFunction(
            +[](UnBlock *u, void *) { gotParam = u->param(); }, nullptr);
        rx3.write(ubRaw, ubBuf.used());
        drainDlx(rxProto3, rx3);
        check(gotParam == 2u, "旧版 UnBlock(param=2) 仍能解析(等价 Unlock)");
    }

    // ---------------------------------------------------------------- 4. 日志回传帧
    printf("\n[4] 日志回传帧(DLX FlightLog)\n");
    {
        LogEntry e        = {};
        e.sessionId       = 12u;
        e.seq             = 345u;
        e.tickMs          = 6789u;
        e.flags           = kLogFlagLinkOk | kLogFlagMotorEnabled;
        e.events          = kLogEventArm;
        e.linkAgeMs       = 7u;
        e.loopPeriodUs    = 2001u;
        e.quat[0]         = 0.99999f;
        e.quat[1]         = 0.001f;
        e.bodyRate[2]     = -0.25f;
        e.accelG[0]       = 0.01f;
        e.rateSetpoint[1] = 0.5f;
        e.torque[2]       = -0.02f;
        e.targetPitchDeg  = 3.5f;
        e.targetRollDeg   = -1.5f;
        e.throttle        = 0.5f;
        e.manualThrottle  = 0.45f;
        e.heightM         = 1.25f;
        e.vertVelMps      = 0.3f;
        e.baroRelM        = 1.3f;
        e.baroAbsM        = 123.4f;
        e.gyroBias[1]     = 0.002f;
        e.motor[0]        = 1100u;
        e.motor[3]        = 1200u;
        e.crc             = flashStructCrc(e); // flash 侧本来就会带 CRC16

        uint8_t txRaw[256];
        ByteBuffer txBuf(txRaw, sizeof(txRaw));
        DLX_ProtocolBuffer proto(txBuf);
        txBuf.reset();
        check(writeLogEntry(proto, e), "writeLogEntry 打包成功");
        const uint16_t head = headOf(txRaw);
        check(txBuf.used() == kFlightLogFrameBytes && kFlightLogFrameBytes == 142u,
              "FlightLog 帧长 = 2 + 140");
        check(((head >> 5) & 0x3FFu) == 15u, "FlightLog 类型号 = 15");
        check(((head >> 1) & 0xFu) == proto.crc4_itu(txRaw + 2, kFlightLogPayloadBytes),
              "FlightLog 头部 CRC4 覆盖 140 字节负载");
        const uint8_t *p = txRaw + 2; // 负载起点
        check(beU32(p + 0) == 12u && beU32(p + 4) == 345u && beU32(p + 8) == 6789u
                  && beU16(p + 12) == (kLogFlagLinkOk | kLogFlagMotorEnabled)
                  && beU16(p + 14) == kLogEventArm && beU16(p + 16) == 7u && beU16(p + 18) == 2001u,
              "FlightLog 头部字段正确(会话/序号/时间/状态位/事件位)");
        check(leF32(p + 20) == e.quat[0] && leF32(p + 36 + 8) == e.bodyRate[2]
                  && leF32(p + 48) == e.accelG[0] && leF32(p + 60 + 4) == e.rateSetpoint[1]
                  && leF32(p + 72 + 8) == e.torque[2],
              "FlightLog 数组字段正确(姿态/角速度/比力/设定/力矩, 原位小端)");
        check(beF32(p + 84) == e.targetPitchDeg && beF32(p + 88) == e.targetRollDeg
                  && beF32(p + 96) == e.throttle && beF32(p + 104) == e.heightM
                  && beF32(p + 116) == e.baroAbsM,
              "FlightLog 标量字段正确(期望角/油门/高度/气压, 大端)");
        check(leF32(p + 120 + 4) == e.gyroBias[1] && leU16(p + 132) == 1100u
                  && leU16(p + 138) == 1200u,
              "FlightLog 零偏/电机字段正确");
        check(kFlightLogPayloadBytes == 140u && FLASH_LOG_ENTRY_STRIDE == 148u,
              "DLX 帧 142B / flash 记录 148B 两套格式并存");
    }

    // ---------------------------------------------------------------- 5. 上位机(Kotlin)那样的大端负载 → MCU 侧解析
    printf("\n[5] Kotlin 侧写入的 GroundCmd(大端) → MCU 解析\n");
    {
        // Kotlin 生成代码: buf.writeUShort(op); writeUShort(mode); writeUInt(sessionId); writeUInt(count)
        // → 线上就是大端 op(1) mode(1) sid(0) count(0)
        uint8_t payload[12] = {0x00, 0x01, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 0, 0};

        uint8_t txRaw[32];
        ByteBuffer txBuf2(txRaw, sizeof(txRaw));
        DLX_ProtocolBuffer proto2(txBuf2);
        const uint16_t head = (uint16_t)((1u << 15) | (11u << 5)
                                         | ((uint16_t)(proto2.crc4_itu(payload, 12) & 0xFu) << 1));
        uint8_t frame[14];
        frame[0] = (uint8_t)(head >> 8);
        frame[1] = (uint8_t)(head & 0xFFu);
        memcpy(frame + 2, payload, 12);

        uint8_t    rxRaw[32];
        ByteBuffer rxByteBuf(rxRaw, sizeof(rxRaw));
        RingByteBuffer rx(rxByteBuf);
        uint8_t    fRaw[32];
        ByteBuffer fBuf(fRaw, sizeof(fRaw));
        DLX_ProtocolBuffer rxProto(txBuf2, &rx, &fBuf);

        static int parsedOp = -1, parsedMode = -1, parsedSid = -1, parsedCnt = -1;
        parsedOp = parsedMode = parsedSid = parsedCnt = -1;
        rxProto.setGroundCmdCallbackFunction(
            +[](GroundCmd *c, void *) {
                parsedOp   = (int)c->op();
                parsedMode = (int)c->mode();
                parsedSid  = (int)c->sessionId();
                parsedCnt  = (int)c->count();
            },
            nullptr);

        rx.write(frame, sizeof(frame));
        drainDlx(rxProto, rx);
        printf("    MCU 解析结果: op=%d mode=%d sessionId=%d count=%d\n", parsedOp, parsedMode,
               parsedSid, parsedCnt);
        check(parsedOp == 1 && parsedMode == 1 && parsedSid == 0 && parsedCnt == 0,
              "Kotlin 大端负载被 MCU 正确解析(op=1 Unlock, mode=1)");
    }

    // ---------------------------------------------------------------- 6. EkfCompare 对比包
    printf("\n[6] EkfCompare 对比包(类型 16, 144B 负载)\n");
    {
        uint8_t txRaw[192];
        ByteBuffer txBuf(txRaw, sizeof(txRaw));
        DLX_ProtocolBuffer proto(txBuf);

        // 帧内偏移 = 2(头) + 负载偏移: 数组按原始小端, 标量按大端
        float gyro[3]  = {0.125f, -0.5f, 1.0f};
        float accel[3] = {0.01f, 0.02f, 0.999f};
        float mq[4]    = {1.0f, 0.0f, 0.001f, -0.002f};
        float eq[4]    = {0.9999f, 0.001f, 0.0f, 0.003f};
        float vel[3]   = {0.1f, -0.2f, 0.3f};
        float bg[3]    = {0.001f, -0.002f, 0.003f};
        float av[3]    = {1e-4f, 2e-4f, 3e-4f};
        float vv[3]    = {0.25f, 0.26f, 0.27f};
        float bv[3]    = {1e-6f, 2e-6f, 3e-6f};

        txBuf.reset();
        check(proto.EkfCompareW(1234u, 5678u, 2000u, 0x0025u, 4u, 3u, 1u, 0u,
                                gyro, accel, mq, eq, vel, bg, 0.75f, 1.5f,
                                av, vv, bv, 0.09f),
              "EkfCompareW 打包");
        const uint16_t cmpHead = headOf(txRaw);
        check(txBuf.used() == 146u, "EkfCompare 帧长 = 2 + 144");
        check(((cmpHead >> 5) & 0x3FFu) == 16u, "EkfCompare 类型号 = 16");
        check(((cmpHead >> 1) & 0xFu) == proto.crc4_itu(txRaw + 2, 144),
              "EkfCompare CRC4 覆盖 144 字节负载");

        check(beU32(txRaw + 2) == 1234u && beU32(txRaw + 6) == 5678u
                  && beU16(txRaw + 10) == 2000u && beU16(txRaw + 12) == 0x0025u
                  && txRaw[14] == 4u && txRaw[15] == 3u && txRaw[16] == 1u && txRaw[17] == 0u,
              "seq/tickMs/period/flags/计数 偏移与大端正确");
        check(leF32(txRaw + 18) == gyro[0] && leF32(txRaw + 26) == gyro[2]
                  && leF32(txRaw + 30) == accel[0] && leF32(txRaw + 42) == mq[0]
                  && leF32(txRaw + 54) == mq[3] && leF32(txRaw + 58) == eq[0]
                  && leF32(txRaw + 74) == vel[0] && leF32(txRaw + 82) == vel[2]
                  && leF32(txRaw + 86) == bg[0] && leF32(txRaw + 94) == bg[2],
              "输入/姿态/速度/零偏 数组偏移与小端正确");
        check(beF32(txRaw + 98) == 0.75f && beF32(txRaw + 102) == 1.5f,
              "height/baroRel 偏移与大端正确");
        check(leF32(txRaw + 106) == av[0] && leF32(txRaw + 114) == av[2]
                  && leF32(txRaw + 118) == vv[0] && leF32(txRaw + 138) == bv[2]
                  && beF32(txRaw + 142) == 0.09f,
              "协方差对角/posVarZ 偏移与字节序正确");

        printf("    负载布局: 0-15 时序计数标志, 16 gyro, 28 accel, 40 madgwickQuat, 56 ekfQuat,\n");
        printf("              72 vel, 84 gyroBias, 96 height, 100 baroRel, 104 attVar,\n");
        printf("              116 velVar, 128 gyroBiasVar, 140 posVarZ (共 144B)\n");
    }

    printf("\n===== 通过 %d 项, 失败 %d 项 =====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}

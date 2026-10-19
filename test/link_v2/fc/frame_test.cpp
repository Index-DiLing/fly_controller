//===================================================================================================
// test/link_v2/fc/frame_test.cpp —— 飞控链路 v2 帧层主机单测
//
// 依据: docs/链路层v2_协议规格.md §1(帧格式)、§3(逐帧位布局)、§10(黄金向量)。
// 覆盖:
//   1. CRC-16/CCITT-FALSE 自检 + 线上低字节在前;
//   2. §10 黄金向量逐字节对拍(编码 == 向量; 解码(向量) == 原值);
//   3. 13 种帧的编码/解码回环(含 v2 新增 UP_ROUND/HELLO、改动 UP_CMD/DL_ACK、DL_STATE 位图);
//   4. tryDecode: 粘包、裂包(逐字节喂)、坏类型重同步、坏 CRC 只前进 1 字节、随机字节不越界;
//   5. 帧长表 / kMaxFrameBytes / 非法类型。
//
// 构建(主机, 不依赖硬件/RTOS):
//   g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . test/link_v2/fc/frame_test.cpp -o frame_test.exe
//===================================================================================================
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "link/dlx_link_frame.hpp"

using namespace dlx::link;

namespace
{

int g_failures = 0;
int g_checks   = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        ++g_checks;                                                                                \
        if (!(cond)) {                                                                             \
            ++g_failures;                                                                          \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                                 \
        }                                                                                          \
    } while (0)

#define CHECK_EQ_U(a, b)                                                                           \
    do {                                                                                           \
        ++g_checks;                                                                                \
        const unsigned long va_ = (unsigned long)(a);                                              \
        const unsigned long vb_ = (unsigned long)(b);                                              \
        if (va_ != vb_) {                                                                          \
            ++g_failures;                                                                          \
            printf("FAIL %s:%d  %s=%lu != %s=%lu\n", __FILE__, __LINE__, #a, va_, #b, vb_);        \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                                      \
    do {                                                                                           \
        ++g_checks;                                                                                \
        const double va_ = (double)(a);                                                            \
        const double vb_ = (double)(b);                                                            \
        if (!(fabs(va_ - vb_) <= (double)(eps))) {                                                 \
            ++g_failures;                                                                          \
            printf("FAIL %s:%d  %s=%.6f  %s=%.6f  eps=%g\n", __FILE__, __LINE__, #a, va_, #b, vb_, \
                   (double)(eps));                                                                 \
        }                                                                                          \
    } while (0)

/** 整帧是否与期望字节一致(逐字节, 长度也要相等) */
bool bytesEq(const uint8_t *got, const uint8_t *want, uint16_t len)
{
    return memcmp(got, want, (size_t)len) == 0;
}

//===================================================================================================
// 1. CRC-16/CCITT-FALSE
//===================================================================================================
void testCrc()
{
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK_EQ_U(crc16CcittFalse(check, (uint16_t)sizeof(check)), 0x29B1u);

    // 空串: init 值原样输出(CCITT-FALSE 的定义)
    CHECK_EQ_U(crc16CcittFalse(0, 0u), 0xFFFFu);

    // 线上低字节在前 + crc16LeValid 覆盖"整帧, 除最后两字节"
    uint8_t frame[8] = {0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u, 0x00u, 0x00u};
    const uint16_t crc = crc16CcittFalse(frame, 6u);
    frame[6] = (uint8_t)(crc & 0xFFu);
    frame[7] = (uint8_t)((crc >> 8) & 0xFFu);
    CHECK(crc16LeValid(frame, 8u));

    // 翻任意一位都必须报错(顺序: CRC 高/低字节, 数据字节)
    frame[6] ^= 0x01u;
    CHECK(!crc16LeValid(frame, 8u));
    frame[6] ^= 0x01u;
    frame[7] ^= 0x80u;
    CHECK(!crc16LeValid(frame, 8u));
    frame[7] ^= 0x80u;
    frame[2] ^= 0x04u;
    CHECK(!crc16LeValid(frame, 8u));

    // 长度 < 2 直接判非法(不越界读)
    CHECK(!crc16LeValid(frame, 1u));
    CHECK(!crc16LeValid(frame, 0u));
    CHECK(!crc16LeValid(0, 8u));
}

//===================================================================================================
// 2. 帧长表 / 类型合法性
//===================================================================================================
void testFrameTable()
{
    CHECK_EQ_U(kMaxFrameBytes, 21u);
    CHECK_EQ_U(kFrameOverheadBytes, 3u);
    CHECK_EQ_U(sizeof(uint16_t), 2u);

    struct Expect
    {
        FrameType type;
        const char *name;
        uint16_t bytes;
    };
    const Expect table[] = {
        {FrameType::DlAtt, "DL_ATT", 13u},      {FrameType::DlSys, "DL_SYS", 15u},
        {FrameType::DlGps, "DL_GPS", 21u},      {FrameType::DlState, "DL_STATE", 10u},
        {FrameType::DlAck, "DL_ACK", 5u},       {FrameType::UpRound, "UP_ROUND", 11u},
        {FrameType::Hello, "HELLO", 5u},        {FrameType::UpCtrl, "UP_CTRL", 9u},
        {FrameType::UpCmd, "UP_CMD", 6u},       {FrameType::UpVel, "UP_VEL", 6u},
        {FrameType::UpPos, "UP_POS", 7u},       {FrameType::UpSetpoint, "UP_SETPOINT", 6u},
        {FrameType::UpMotor, "UP_MOTOR", 9u},
    };
    for (size_t i = 0u; i < sizeof(table) / sizeof(table[0]); ++i) {
        CHECK_EQ_U(frameBytes(table[i].type), table[i].bytes);
        CHECK_EQ_U(strlen(frameTypeName(table[i].type)), strlen(table[i].name));
        CHECK(strcmp(frameTypeName(table[i].type), table[i].name) == 0);
        CHECK(frameTypeValid((uint8_t)table[i].type));
    }

    // 0x0 与 0xE/0xF 必须是非法类型(自定界的负向判据)
    CHECK(!frameTypeValid(0x0u));
    CHECK(!frameTypeValid(0xEu));
    CHECK(!frameTypeValid(0xFu));
    CHECK_EQ_U(frameBytes(FrameType::DlAtt), 13u); // 编译期表与运行期查询一致
}

//===================================================================================================
// 3. §10 黄金向量(编码 == 向量; 解码(向量) == 原值)
//===================================================================================================
void testGoldenVectors()
{
    uint8_t buf[kMaxFrameBytes];

    // ---- HELLO seq=0 ver=2 caps=0 nonce=0x5A ----
    {
        static const uint8_t want[] = {0x70u, 0x02u, 0x5Au, 0x49u, 0x89u};
        HelloPayload        p       = {};
        p.ver                       = 2u;
        p.caps                      = 0u;
        p.nonce                     = 0x5Au;
        const uint16_t n = encodeHello(p, 0u, buf);
        CHECK_EQ_U(n, sizeof(want));
        CHECK(bytesEq(buf, want, n));
        HelloPayload q = {};
        CHECK(decodeHello(want, q));
        CHECK_EQ_U(q.ver, 2u);
        CHECK_EQ_U(q.caps, 0u);
        CHECK_EQ_U(q.nonce, 0x5Au);
    }

    // ---- UP_ROUND seq=0 杆量 0 thr=1000 flags=1 session=0x3C flags8=1 ----
    {
        static const uint8_t want[] = {0x60u, 0x00u, 0x00u, 0x00u, 0x00u, 0xD0u,
                                       0x17u, 0x3Cu, 0x01u, 0x7Eu, 0xDEu};
        UpRoundPayload      p       = {};
        p.thr                       = 1000u;
        p.flags                     = 0x01u;
        p.session                   = 0x3Cu;
        p.flags8                    = 0x01u;
        const uint16_t n = encodeUpRound(p, 0u, buf);
        CHECK_EQ_U(n, sizeof(want));
        CHECK(bytesEq(buf, want, n));
        UpRoundPayload q = {};
        CHECK(decodeUpRound(want, q));
        CHECK_EQ_U(q.roll, 0u);
        CHECK_EQ_U(q.pitch, 0u);
        CHECK_EQ_U(q.yaw, 0u);
        CHECK_EQ_U(q.thr, 1000u);
        CHECK_EQ_U(q.flags, 1u);
        CHECK_EQ_U(q.session, 0x3Cu);
        CHECK_EQ_U(q.flags8, 1u);
    }

    // ---- UP_CMD seq=1 cmdId=7 op=0x01 arg=0 ----
    {
        static const uint8_t want[] = {0x91u, 0x07u, 0x01u, 0x00u, 0x4Au, 0x82u};
        UpCmdPayload        p       = {};
        p.cmdId                     = 7u;
        p.op                        = 0x01u;
        p.arg                       = 0u;
        const uint16_t n = encodeUpCmd(p, 1u, buf);
        CHECK_EQ_U(n, sizeof(want));
        CHECK(bytesEq(buf, want, n));
        UpCmdPayload q = {};
        CHECK(decodeUpCmd(want, q));
        CHECK_EQ_U(q.cmdId, 7u);
        CHECK_EQ_U(q.op, 0x01u);
        CHECK_EQ_U(q.arg, 0u);
    }

    // ---- DL_ACK seq=2 cmdId=7 result=0 ----
    {
        static const uint8_t want[] = {0x52u, 0x07u, 0x00u, 0xA5u, 0x65u};
        DlAckPayload        p       = {};
        p.cmdId                     = 7u;
        p.result                    = 0u;
        const uint16_t n = encodeDlAck(p, 2u, buf);
        CHECK_EQ_U(n, sizeof(want));
        CHECK(bytesEq(buf, want, n));
        DlAckPayload q = {};
        CHECK(decodeDlAck(want, q));
        CHECK_EQ_U(q.cmdId, 7u);
        CHECK_EQ_U(q.result, 0u);
    }

    // ---- UP_CTRL seq=2 全 0 ----
    {
        static const uint8_t want[] = {0x82u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0xD4u, 0x3Au};
        UpCtrlPayload       p       = {};
        const uint16_t      n       = encodeUpCtrl(p, 2u, buf);
        CHECK_EQ_U(n, sizeof(want));
        CHECK(bytesEq(buf, want, n));
        UpCtrlPayload q = {};
        CHECK(decodeUpCtrl(want, q));
        CHECK_EQ_U(q.roll, 0u);
        CHECK_EQ_U(q.thr, 0u);
        CHECK_EQ_U(q.flags, 0u);
    }

    // ---- UP_CTRL seq=1 roll=-1023 pitch=1023 thr=2047 flags=3 ----
    {
        static const uint8_t want[] = {0x81u, 0x01u, 0xFCu, 0x1Fu, 0x00u,
                                       0xFEu, 0x3Fu, 0xB0u, 0x2Bu};
        UpCtrlPayload       p       = {};
        p.roll                      = -1023;
        p.pitch                     = 1023;
        p.yaw                       = 0;
        p.thr                       = 2047u;
        p.flags                     = 0x03u;
        const uint16_t n = encodeUpCtrl(p, 1u, buf);
        CHECK_EQ_U(n, sizeof(want));
        CHECK(bytesEq(buf, want, n));
        UpCtrlPayload q = {};
        CHECK(decodeUpCtrl(want, q));
        CHECK_EQ_U(q.roll, (unsigned long)(-1023));
        CHECK_EQ_U(q.pitch, 1023u);
        CHECK_EQ_U(q.thr, 2047u);
        CHECK_EQ_U(q.flags, 3u);
    }

    // ---- DL_ATT seq=0 全 0 负载 ----
    {
        static const uint8_t want[] = {0x10u, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x32u, 0x81u};
        DlAttPayload        p       = {};
        const uint16_t      n       = encodeDlAtt(p, 0u, buf);
        CHECK_EQ_U(n, sizeof(want));
        CHECK(bytesEq(buf, want, n));
        DlAttPayload q = {};
        CHECK(decodeDlAtt(want, q));
        CHECK_NEAR(q.q1, 0.0, 1e-9);
        CHECK_NEAR(q.vx, 0.0, 1e-9);
        CHECK_EQ_U(q.status, 0u);
        CHECK_EQ_U(q.mode, 0u);
    }

    // ---- DL_STATE seq=3 全 0 负载 ----
    {
        static const uint8_t want[] = {0x43u, 0, 0, 0, 0, 0, 0, 0, 0x9Bu, 0x0Fu};
        DlStatePayload      p       = {};
        const uint16_t      n       = encodeDlState(p, 3u, buf);
        CHECK_EQ_U(n, sizeof(want));
        CHECK(bytesEq(buf, want, n));
        DlStatePayload q = {};
        CHECK(decodeDlState(want, q));
        CHECK_EQ_U(q.rxBitmap, 0u);
    }
}

//===================================================================================================
// 4. 回环(编码 -> 解码)覆盖每一型; 数值取"定点可精确表示"的点
//===================================================================================================
void testRoundTrip()
{
    uint8_t buf[kMaxFrameBytes];
    Frame   f;

    // ---- DL_ATT ----
    {
        DlAttPayload p = {};
        p.q1 = 0.5f; p.q2 = -0.5f; p.q3 = 0.25f;
        p.vx = 1.0f; p.vy = -2.0f; p.vz = 0.3f;
        p.status = 0xA5u; p.error = 0x5Au; p.arming = 3u; p.mode = 2u;
        const uint16_t n = encodeDlAtt(p, 0u, buf);
        CHECK_EQ_U(n, kDlAttBytes);
        uint16_t consumed = 0u;
        CHECK(tryDecode(buf, n, f, consumed) == kTryDecoded);
        CHECK_EQ_U(consumed, kDlAttBytes);
        CHECK(f.type == FrameType::DlAtt);
        CHECK_NEAR(f.u.att.q1, 0.5, 1.0 / 2047.0);
        CHECK_NEAR(f.u.att.q2, -0.5, 1.0 / 2047.0);
        CHECK_NEAR(f.u.att.vx, 1.0, 0.1);
        CHECK_NEAR(f.u.att.vz, 0.3, 0.1);
        CHECK_EQ_U(f.u.att.status, 0xA5u);
        CHECK_EQ_U(f.u.att.arming, 3u);
    }

    // ---- DL_SYS ----
    {
        DlSysPayload p = {};
        p.x = 1.0f; p.y = -2.5f; p.z = 3.0f;
        p.currentA = 1.2f; p.voltageV = 12.6f;
        p.motor[0] = 0u; p.motor[1] = 1000u; p.motor[2] = 1999u; p.motor[3] = 500u;
        const uint16_t n = encodeDlSys(p, 4u, buf);
        CHECK_EQ_U(n, kDlSysBytes);
        uint16_t consumed = 0u;
        CHECK(tryDecode(buf, n, f, consumed) == kTryDecoded);
        CHECK_EQ_U(consumed, n);
        CHECK_NEAR(f.u.sys.x, 1.0, 0.1);
        CHECK_NEAR(f.u.sys.y, -2.5, 0.1);
        CHECK_NEAR(f.u.sys.voltageV, 12.6, 0.1);
        CHECK_EQ_U(f.u.sys.motor[2], 1999u);
        CHECK_EQ_U(f.u.sys.motor[3], 500u);
    }

    // ---- DL_GPS ----
    {
        DlGpsPayload p = {};
        p.latE7 = 123456789; p.lonE7 = -987654321; p.altMslMm = 12345;
        p.gspeedCms = 500; p.courseCdeg = 35999u; p.sats = 63u; p.fix = 3u; p.hdop = 1.5f;
        const uint16_t n = encodeDlGps(p, 5u, buf);
        CHECK_EQ_U(n, kDlGpsBytes);
        DlGpsPayload q = {};
        CHECK(decodeDlGps(buf, q));
        CHECK(q.latE7 == 123456789);
        CHECK(q.lonE7 == -987654321);
        CHECK_EQ_U(q.courseCdeg, 35999u);
        CHECK_EQ_U(q.sats, 63u);
        CHECK_NEAR(q.hdop, 1.5, 0.1);
    }

    // ---- DL_STATE(含接收位图) ----
    {
        DlStatePayload p = {};
        p.lifecycle = 7u; p.arming = 3u; p.mode = 5u; p.task = 2u;
        p.failsafeLvl = 4u; p.failsafeRsn = 8u; p.health = 0x81u;
        p.ulLoss = 100u; p.dlLoss = 42u; p.rssiDbm = -60;
        p.rxBitmap = 0x0Au;
        const uint16_t n = encodeDlState(p, 6u, buf);
        CHECK_EQ_U(n, kDlStateBytes);
        DlStatePayload q = {};
        CHECK(decodeDlState(buf, q));
        CHECK_EQ_U(q.lifecycle, 7u);
        CHECK_EQ_U(q.mode, 5u);
        CHECK_EQ_U(q.rssiDbm, (unsigned long)(-60));
        CHECK_EQ_U(q.rxBitmap, 0x0Au);
        CHECK_EQ_U(dlStateRxBitmapOf(buf), 0x0Au);

        // 发送前直接改位图: 必须同时重算 CRC, 且不影响其余字段
        dlStateSetRxBitmap(buf, 0x05u);
        CHECK(crc16LeValid(buf, kDlStateBytes));
        CHECK_EQ_U(dlStateRxBitmapOf(buf), 0x05u);
        DlStatePayload r = {};
        CHECK(decodeDlState(buf, r));
        CHECK_EQ_U(r.rxBitmap, 0x05u);
        CHECK_EQ_U(r.health, 0x81u);
        CHECK_EQ_U(r.dlLoss, 42u);

        // 位图只有 4 位: 高位必须被掩掉
        dlStateSetRxBitmap(buf, 0xFFu);
        CHECK_EQ_U(dlStateRxBitmapOf(buf), 0x0Fu);

        // 保留位(bit53..55)必须恒 0
        CHECK_EQ_U(dlx::link::bits::getBits(buf + 1, dlState::kPayloadBytes, dlState::kRxReservedBit,
                                            dlState::kRxReservedBits),
                   0u);
    }

    // ---- UP_ROUND(含负杆量/session 边界) ----
    {
        UpRoundPayload p = {};
        p.roll = -1023; p.pitch = 1023; p.yaw = -500;
        p.thr = 0u; p.flags = 0x01u; p.session = 255u; p.flags8 = 0x07u;
        const uint16_t n = encodeUpRound(p, 7u, buf);
        CHECK_EQ_U(n, kUpRoundBytes);
        UpRoundPayload q = {};
        CHECK(decodeUpRound(buf, q));
        CHECK_EQ_U(q.roll, (unsigned long)(-1023));
        CHECK_EQ_U(q.pitch, 1023u);
        CHECK_EQ_U(q.yaw, (unsigned long)(-500));
        CHECK_EQ_U(q.thr, 0u);
        CHECK_EQ_U(q.session, 255u);
        CHECK_EQ_U(q.flags8, 7u);

        // session 0 是"保留不用", 但帧层不拦(语义在 RoundSlave); 只验证能过线
        p.session = 0u;
        CHECK_EQ_U(encodeUpRound(p, 8u, buf), kUpRoundBytes);
        CHECK(decodeUpRound(buf, q));
        CHECK_EQ_U(q.session, 0u);
    }

    // ---- HELLO ----
    {
        HelloPayload p = {};
        p.ver = 2u; p.caps = 0x07u; p.nonce = 0xFFu;
        CHECK_EQ_U(encodeHello(p, 9u, buf), kHelloBytes);
        HelloPayload q = {};
        CHECK(decodeHello(buf, q));
        CHECK_EQ_U(q.ver, 2u);
        CHECK_EQ_U(q.caps, 7u);
        CHECK_EQ_U(q.nonce, 0xFFu);
    }

    // ---- UP_CMD ----
    {
        UpCmdPayload p = {};
        p.cmdId = 255u; p.op = 0x21u; p.arg = 100u;
        CHECK_EQ_U(encodeUpCmd(p, 10u, buf), kUpCmdBytes);
        UpCmdPayload q = {};
        CHECK(decodeUpCmd(buf, q));
        CHECK_EQ_U(q.cmdId, 255u);
        CHECK_EQ_U(q.op, 0x21u);
        CHECK_EQ_U(q.arg, 100u);
    }

    // ---- UP_VEL ----
    {
        UpVelPayload p = {};
        p.vx = 1.0f; p.vy = -0.5f; p.vz = 2.0f;
        CHECK_EQ_U(encodeUpVel(p, 11u, buf), kUpVelBytes);
        UpVelPayload q = {};
        CHECK(decodeUpVel(buf, q));
        CHECK_NEAR(q.vx, 1.0, 0.05);
        CHECK_NEAR(q.vy, -0.5, 0.05);
        CHECK_NEAR(q.vz, 2.0, 0.05);
    }

    // ---- UP_POS ----
    {
        UpPosPayload p = {};
        p.x = 1.0f; p.y = -2.0f;
        CHECK_EQ_U(encodeUpPos(p, 12u, buf), kUpPosBytes);
        UpPosPayload q = {};
        CHECK(decodeUpPos(buf, q));
        CHECK_NEAR(q.x, 1.0, 0.1);
        CHECK_NEAR(q.y, -2.0, 0.1);
    }

    // ---- UP_SETPOINT ----
    {
        UpSetpointPayload p = {};
        p.altM = 1.23f; p.flags = upSetpoint::kFlagAltValid;
        CHECK_EQ_U(encodeUpSetpoint(p, 13u, buf), kUpSetpointBytes);
        UpSetpointPayload q = {};
        CHECK(decodeUpSetpoint(buf, q));
        CHECK_NEAR(q.altM, 1.23, 0.01);
        CHECK_EQ_U(q.flags, 1u);
    }

    // ---- UP_MOTOR ----
    {
        UpMotorPayload p = {};
        p.throttle[0] = 0u; p.throttle[1] = 1000u; p.throttle[2] = 1999u; p.throttle[3] = 500u;
        CHECK_EQ_U(encodeUpMotor(p, 14u, buf), kUpMotorBytes);
        UpMotorPayload q = {};
        CHECK(decodeUpMotor(buf, q));
        CHECK_EQ_U(q.throttle[1], 1000u);
        CHECK_EQ_U(q.throttle[2], 1999u);
        CHECK(upMotorSpareBitsZero(buf));
    }
}

//===================================================================================================
// 5. tryDecode: 粘包 / 裂包 / 重同步
//===================================================================================================
/** 确定性伪随机(LCG): 随机用例必须可复现, 否则 CI 上偶发失败无法排查 */
uint32_t lcgNext(uint32_t &s)
{
    s = (uint32_t)(s * 1664525u + 1013904223u);
    return s;
}

void testTryDecode()
{
    uint8_t stream[3u * kMaxFrameBytes];
    uint16_t len = 0u;

    UpCtrlPayload ctrl = {};
    ctrl.roll          = 100;
    ctrl.thr           = 1500u;
    len = (uint16_t)(len + encodeUpCtrl(ctrl, 0u, stream + len));

    HelloPayload hello = {};
    hello.ver          = 2u;
    hello.nonce        = 0x5Au;
    len = (uint16_t)(len + encodeHello(hello, 0u, stream + len));

    DlAttPayload att = {};
    att.status       = 0x42u;
    len = (uint16_t)(len + encodeDlAtt(att, 0u, stream + len));

    // ---- 粘包: 一次给三帧, 必须连解三帧 ----
    {
        uint16_t off   = 0u;
        uint8_t  count = 0u;
        while (off < len) {
            Frame    f;
            uint16_t consumed = 0u;
            const int r = tryDecode(stream + off, (uint16_t)(len - off), f, consumed);
            CHECK(r == kTryDecoded);
            if (r != kTryDecoded) {
                break;
            }
            CHECK(consumed == frameBytes(f.type));
            ++count;
            off = (uint16_t)(off + consumed);
            if (count == 1u) {
                CHECK(f.type == FrameType::UpCtrl);
                CHECK_EQ_U(f.u.ctrl.roll, 100u);
            } else if (count == 2u) {
                CHECK(f.type == FrameType::Hello);
                CHECK_EQ_U(f.u.hello.ver, 2u);
            } else {
                CHECK(f.type == FrameType::DlAtt);
                CHECK_EQ_U(f.u.att.status, 0x42u);
            }
        }
        CHECK_EQ_U(count, 3u);
        CHECK_EQ_U(off, len);
    }

    // ---- 裂包: 逐字节到达, 只有"正好到整帧"那一步才解出 ----
    {
        const uint16_t frameLen = kUpCtrlBytes;
        for (uint16_t prefix = 0u; prefix < frameLen; ++prefix) {
            Frame    f;
            uint16_t consumed = 0xFFFFu;
            const int r = tryDecode(stream, prefix, f, consumed);
            CHECK(r == kTryNeedMore);
            CHECK_EQ_U(consumed, 0u);
        }
        Frame    f;
        uint16_t consumed = 0u;
        CHECK(tryDecode(stream, frameLen, f, consumed) == kTryDecoded);
        CHECK_EQ_U(consumed, frameLen);
        CHECK(f.type == FrameType::UpCtrl);
    }

    // ---- 空/空指针 ----
    {
        Frame    f;
        uint16_t consumed = 0xFFFFu;
        CHECK(tryDecode(0, 10u, f, consumed) == kTryNeedMore);
        CHECK_EQ_U(consumed, 0u);
        CHECK(tryDecode(stream, 0u, f, consumed) == kTryNeedMore);
        CHECK_EQ_U(consumed, 0u);
    }

    // ---- 非法类型前缀: 每字节 reject, 一帧内重同步 ----
    {
        uint8_t  junk[5] = {0x00u, 0xE0u, 0xF1u, 0x0Fu, 0xFFu};
        uint8_t  mixed[5u + kMaxFrameBytes];
        uint16_t n = 0u;
        memcpy(mixed, junk, sizeof(junk));
        n = (uint16_t)(n + (uint16_t)sizeof(junk));
        n = (uint16_t)(n + encodeHello(hello, 0u, mixed + n));

        uint16_t off      = 0u;
        uint16_t rejected = 0u;
        bool     got      = false;
        while (off < n) {
            Frame    f;
            uint16_t consumed = 0u;
            const int r = tryDecode(mixed + off, (uint16_t)(n - off), f, consumed);
            if (r == kTryNeedMore) {
                break;
            }
            if (r == kTryReject) {
                CHECK_EQ_U(consumed, 1u); // v2: 坏帧只前进 1 字节
                ++rejected;
            } else {
                CHECK(f.type == FrameType::Hello);
                got = true;
                CHECK_EQ_U(f.u.hello.nonce, 0x5Au);
            }
            off = (uint16_t)(off + consumed);
        }
        CHECK(got);
        CHECK_EQ_U(rejected, sizeof(junk)); // 每个垃圾字节恰好一次 reject
    }

    // ---- 随机坏字节(只取非法类型 nibble): 保证逐字节 reject, 一帧内重同步 ----
    {
        uint32_t seed = 0x12345678u;
        for (uint32_t iter = 0u; iter < 200u; ++iter) {
            const uint16_t junkLen = (uint16_t)(lcgNext(seed) % 32u + 1u);
            uint8_t        mix[32u + kMaxFrameBytes];
            for (uint16_t i = 0u; i < junkLen; ++i) {
                // 高 4 位只取 0x0 / 0xE / 0xF(帧表里的非法类型): 解析器必须每个字节只 reject 1 次
                static const uint8_t badNibble[3] = {0x00u, 0xE0u, 0xF0u};
                mix[i] = (uint8_t)(badNibble[lcgNext(seed) % 3u] | (uint8_t)(lcgNext(seed) & 0x0Fu));
            }
            const uint16_t n = (uint16_t)(junkLen + encodeHello(hello, 0u, mix + junkLen));

            uint16_t off    = 0u;
            bool     got    = false;
            uint16_t rejects = 0u;
            while (off < n) {
                Frame    f;
                uint16_t consumed = 0u;
                const int r = tryDecode(mix + off, (uint16_t)(n - off), f, consumed);
                if (r == kTryReject) {
                    CHECK_EQ_U(consumed, 1u);
                    ++rejects;
                } else if (r == kTryDecoded) {
                    CHECK(f.type == FrameType::Hello);
                    CHECK_EQ_U(f.u.hello.ver, 2u);
                    got = true;
                } else {
                    CHECK(false); // 非法类型 + 完整帧不会出现 NeedMore
                    break;
                }
                off = (uint16_t)(off + consumed);
            }
            CHECK(got);
            CHECK_EQ_U(rejects, junkLen); // 每个坏字节恰好一次, 没有"整帧跳过"
            CHECK_EQ_U(off, n);
        }
    }

    // ---- 坏 CRC: 也必须只前进 1 字节, 后面的真帧还能解出来 ----
    {
        uint8_t  broken[kUpCtrlBytes];
        memcpy(broken, stream, kUpCtrlBytes);
        broken[kUpCtrlBytes - 1u] ^= 0x5Au; // 破坏 CRC 高字节

        uint8_t  mixed[kUpCtrlBytes + kMaxFrameBytes + 32u];
        uint16_t n = 0u;
        memcpy(mixed, broken, kUpCtrlBytes);
        n = (uint16_t)(n + kUpCtrlBytes);
        n = (uint16_t)(n + encodeHello(hello, 0u, mixed + n));
        // 尾部补 0x00(非法类型): 坏帧尾部若恰好"声称"一个更长的帧, 解析器会等更多字节(kTryNeedMore);
        // 真实串口后面永远还有字节, 这里用填充把"等"这一步喂穿, 才能验证最终一定重同步到真帧。
        memset(mixed + n, 0x00, 32u);
        n = (uint16_t)(n + 32u);

        uint16_t off    = 0u;
        uint16_t rejects = 0u;
        bool     got    = false;
        while (off < n) {
            Frame    f;
            uint16_t consumed = 0u;
            const int r = tryDecode(mixed + off, (uint16_t)(n - off), f, consumed);
            if (r == kTryNeedMore) {
                break;
            }
            if (r == kTryReject) {
                CHECK_EQ_U(consumed, 1u);
                ++rejects;
            } else {
                CHECK(f.type == FrameType::Hello);
                got = true;
            }
            off = (uint16_t)(off + consumed);
        }
        CHECK(got);
        CHECK(rejects >= 1u); // 至少坏帧头那一次
    }
}

//===================================================================================================
// 6. 随机字节模糊测试: 不越界、返回码/consumed 自洽
//===================================================================================================
void testFuzz()
{
    const uint16_t cap = 64u;
    uint8_t        buf[cap + 8u];
    uint32_t       seed = 20261018u;

    for (uint32_t iter = 0u; iter < 4000u; ++iter) {
        const uint16_t len = (uint16_t)(lcgNext(seed) % (uint32_t)cap);
        for (uint16_t i = 0u; i < cap + 8u; ++i) {
            buf[i] = 0xAAu; // 哨兵: 越界写会破坏它
        }
        for (uint16_t i = 0u; i < len; ++i) {
            buf[i] = (uint8_t)(lcgNext(seed) & 0xFFu);
        }

        uint16_t off = 0u;
        while (off < len) {
            Frame    f;
            uint16_t consumed = 0u;
            const int r        = tryDecode(buf + off, (uint16_t)(len - off), f, consumed);
            CHECK(r == kTryDecoded || r == kTryNeedMore || r == kTryReject);
            if (r == kTryDecoded) {
                CHECK(consumed == frameBytes(f.type));
                CHECK(f.type == frameTypeOf(buf[off]));
                CHECK(crc16LeValid(buf + off, consumed));
            } else if (r == kTryNeedMore) {
                CHECK_EQ_U(consumed, 0u);
                break; // 半帧: 调用方会等更多字节
            } else {
                CHECK_EQ_U(consumed, 1u);
            }
            if (consumed == 0u) {
                break;
            }
            off = (uint16_t)(off + consumed);
        }

        // 缓冲区尾部哨兵必须原样(没有越界写)
        bool sentinelOk = true;
        for (uint16_t i = cap; i < cap + 8u; ++i) {
            if (buf[i] != 0xAAu) {
                sentinelOk = false;
            }
        }
        CHECK(sentinelOk);
    }
}

} // namespace

int main()
{
    testCrc();
    testFrameTable();
    testGoldenVectors();
    testRoundTrip();
    testTryDecode();
    testFuzz();

    printf("\nchecks=%d failures=%d\n", g_checks, g_failures);
    if (g_failures != 0) {
        printf("RESULT: FAIL\n");
        return 1;
    }
    printf("RESULT: PASS\n");
    return 0;
}

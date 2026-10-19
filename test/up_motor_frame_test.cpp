//===================================================================================================
// test/up_motor_frame_test.cpp —— UP_MOTOR(0xD) 帧的编解码往返
//
// 怎么跑(不需要板子、不需要交叉工具链):
//   g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . test/up_motor_frame_test.cpp -o up_motor_frame_test
//   ./up_motor_frame_test
//
// 为什么值得单独一个测试: 2026-10-12 把 UP_MOTOR 从"单路 index + throttle"(4B) 改成了
// "**四路一次发全**"(8B); 链路 v2(2026-10-18)又因 CRC-16 使整帧 +1B(9B)。位表这种东西
// **差一位就是"1 号电机收到 3 号的值"** ——
// 而这种错在编译期、在类型上、在两边各自的自测里都看不出来, 只有把"编出来的字节"和
// "解出来的值"对拍才能抓住。所以这里做的是**真编码 → 真解码**的往返, 不是对着常量打勾。
//===================================================================================================
#include <stdio.h>
#include <string.h>

#include "link/dlx_link_frame.hpp"

using namespace dlx::link;

static int g_checks   = 0;
static int g_failures = 0;

#define CHECK(condition)                                                 \
    do {                                                                 \
        ++g_checks;                                                      \
        if (!(condition)) {                                              \
            ++g_failures;                                                \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #condition); \
        }                                                                \
    } while (0)

/** 编一帧, 再解码回来 */
static bool roundTrip(const UpMotorPayload &in, UpMotorPayload &out, uint8_t seq = 3u)
{
    uint8_t        buf[kUpMotorBytes + 4];
    const uint16_t n = encodeUpMotor(in, seq, buf);
    if (n != kUpMotorBytes) {
        printf("  encodeUpMotor 返回 %u, 期望 %u\n", (unsigned)n, (unsigned)kUpMotorBytes);
        return false;
    }
    Frame    f;
    uint16_t consumed = 0;
    if (tryDecode(buf, n, f, consumed) != kTryDecoded) {
        printf("  tryDecode 没解出来\n");
        return false;
    }
    if (f.type != FrameType::UpMotor) {
        printf("  解出的类型是 %u, 期望 UpMotor(0xD)\n", (unsigned)f.type);
        return false;
    }
    if (f.seq != seq) {
        printf("  SEQ 是 %u, 期望 %u\n", (unsigned)f.seq, (unsigned)seq);
        return false;
    }
    if (consumed != kUpMotorBytes) {
        printf("  消费了 %u 字节, 期望 %u\n", (unsigned)consumed, (unsigned)kUpMotorBytes);
        return false;
    }
    out = f.u.motor;
    return true;
}

int main()
{
    printf("=== UP_MOTOR 帧往返测试 ===\n");

    // ---- 零、协议常量(帧长/位宽) ----
    printf("-- 常量\n");
    CHECK(upMotor::kMotorCount == 4u);
    CHECK(upMotor::kMotorBits == 11u);
    CHECK(upMotor::kUsedBits == 44u);
    CHECK(upMotor::kSpareBits == 4u);
    CHECK(upMotor::kPayloadBytes == 6u);
    CHECK(kUpMotorBytes == 9u); // 链路 v2: 负载 6B + 头 1B + CRC-16 2B
    CHECK(frameBytes(FrameType::UpMotor) == 9u);
    CHECK(kMaxFrameBytes >= kUpMotorBytes); // 收缓冲按最长帧开
    // 四路的下标必须真正错开 11 位, 否则后一路会覆盖前一路
    CHECK(upMotor::kMotor1Bit == upMotor::kMotor0Bit + upMotor::kMotorBits);
    CHECK(upMotor::kMotor2Bit == upMotor::kMotor1Bit + upMotor::kMotorBits);
    CHECK(upMotor::kMotor3Bit == upMotor::kMotor2Bit + upMotor::kMotorBits);

    // ---- 一、往返: 四个**互不相同**的值 —— 只有这样才能发现"通道串位" ----
    printf("-- 往返(四路各不相同)\n");
    {
        UpMotorPayload in;
        in.throttle[0] = 0u;
        in.throttle[1] = 1u;
        in.throttle[2] = 1024u;
        in.throttle[3] = 1999u;
        UpMotorPayload out;
        memset(&out, 0, sizeof(out));
        CHECK(roundTrip(in, out));
        for (uint8_t i = 0u; i < upMotor::kMotorCount; ++i) {
            if (out.throttle[i] != in.throttle[i]) {
                printf("  第 %u 路: 得到 %u, 期望 %u\n", (unsigned)i, (unsigned)out.throttle[i],
                       (unsigned)in.throttle[i]);
            }
            CHECK(out.throttle[i] == in.throttle[i]);
        }
    }

    // ---- 二、每一位都必须被用上: 单独把某一位置 1, 看它落在哪一路 ----
    // (这一步是"位表对不对"的直接证据: 四路各取一个只属于自己的值)
    printf("-- 逐路独立性\n");
    for (uint8_t ch = 0u; ch < upMotor::kMotorCount; ++ch) {
        UpMotorPayload in;
        for (uint8_t i = 0u; i < upMotor::kMotorCount; ++i) {
            in.throttle[i] = (i == ch) ? 1999u : 0u;
        }
        UpMotorPayload out;
        memset(&out, 0, sizeof(out));
        CHECK(roundTrip(in, out, static_cast<uint8_t>(ch)));
        for (uint8_t i = 0u; i < upMotor::kMotorCount; ++i) {
            const uint16_t want = (i == ch) ? 1999u : 0u;
            if (out.throttle[i] != want) {
                printf("  只给第 %u 路时, 第 %u 路解出 %u\n", (unsigned)ch, (unsigned)i,
                       (unsigned)out.throttle[i]);
            }
            CHECK(out.throttle[i] == want);
        }
    }

    // ---- 三、饱和而不是回绕: 线上 11 位能表示 0..2047, 而 2000..2047 是 DShot 特殊命令区 ----
    printf("-- 夹到 DShot 可用区 0..1999\n");
    {
        UpMotorPayload in;
        in.throttle[0] = 2047u; // 11 位的最大值
        in.throttle[1] = 2000u; // 特殊命令区的下界
        in.throttle[2] = 1999u; // 合法上界
        in.throttle[3] = 0u;
        UpMotorPayload out;
        CHECK(roundTrip(in, out));
        CHECK(out.throttle[0] == 1999u); // 饱和, 不是回绕成 0
        CHECK(out.throttle[1] == 1999u);
        CHECK(out.throttle[2] == 1999u);
        CHECK(out.throttle[3] == 0u);
    }

    // ---- 四、4 个保留位必须恒 0(准入门用它把"不是本工程编出来的帧"挡掉) ----
    printf("-- 保留位\n");
    {
        uint8_t buf[kUpMotorBytes];
        UpMotorPayload in;
        for (uint8_t i = 0u; i < upMotor::kMotorCount; ++i) {
            in.throttle[i] = 0x7FFu; // 11 位全 1, 最大限度"顶"到保留位上
        }
        CHECK(encodeUpMotor(in, 0u, buf) == kUpMotorBytes);
        CHECK(upMotorSpareBitsZero(buf));
        // 手工把 4 个保留位里最高那一位置 1 ⇒ 必须被认出来。
        // 位 44..47 在负载第 6 字节的 bit4..7; v2 整帧 = 头(1) + 负载(6) + CRC(2) ⇒ 该字节是 buf[1+5]。
        buf[1u + 5u] |= 0x80u;
        CHECK(!upMotorSpareBitsZero(buf));
    }

    // ---- 五、CRC: 改一个字节就必须被拒(且只前进 1 字节重找帧头) ----
    printf("-- CRC\n");
    {
        uint8_t buf[kUpMotorBytes];
        UpMotorPayload in;
        in.throttle[0] = 100u;
        in.throttle[1] = 200u;
        in.throttle[2] = 300u;
        in.throttle[3] = 400u;
        encodeUpMotor(in, 5u, buf);
        // 先确认"没改过就能解出来" —— 否则下面那条断言可能只是因为这个帧本来就解不出来
        Frame    f;
        uint16_t consumed = 0;
        CHECK(tryDecode(buf, kUpMotorBytes, f, consumed) == kTryDecoded);
        CHECK(f.u.motor.throttle[2] == 300u);
        // 再动负载中间一个 bit: 类型字节没坏, 所以必须由 **CRC** 把它挡掉
        buf[3] ^= 0x01u;
        consumed = 0;
        CHECK(tryDecode(buf, kUpMotorBytes, f, consumed) == kTryReject);
        CHECK(consumed == 1u); // 坏帧只前进 1 字节(不能整帧跳过, 会顺走真帧的头)
    }

    printf("=== %d 项检查, %d 项失败 ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}

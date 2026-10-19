#pragma once
//===================================================================================================
// DL_LIB/link/dlx_link_frame.hpp —— E220 链路的定长帧表 + 编解码(纯逻辑, 无硬件依赖)
//
// 职责(v2):
//   1. 帧类型 / 帧长常量表(自定界的唯一依据) + frameBytes();
//   2. CRC-16/CCITT-FALSE(poly 0x1021, init 0xFFFF, 不反射, xorout 0, 低字节在前);
//   3. 每种帧一个 POD 负载结构体 + encodeX()/decodeX();
//   4. tryDecode(): 从任意字节流里切帧, 三种结果(解出 / 字节不够 / 噪声坏帧)互不混淆;
//   5. seqGap(): 4 位 SEQ 的丢包计数(含 15→0 回绕); v2 上行另有 seqPos() = SEQ & 3。
//
// ---- 线上格式(v2 · 协议规格 §1.1) ----
//   ┌───────────────┬──────────────────────┬──────────────────┐
//   │ TYPE(4) SEQ(4)│    payload (N-24b)   │ CRC16(低字节在前) │
//   └───────────────┴──────────────────────┴──────────────────┘
//        byte 0             byte 1..             byte N-2, N-1
//   byte0 = (TYPE << 4) | (SEQ & 0x0F);  CRC16 覆盖 byte0 .. CRC 前一字节。
//   负载内位序 = dlx_bits.hpp 的小端位序约定(先定义的字段在低位, 可跨字节)。
//   CRC16 = CCITT-FALSE: poly 0x1021 / init 0xFFFF / 不反射 / xorout 0;
//   校验值 "123456789" → 0x29B1(v2 起假帧通过率从 1/256 降到 1/65536)。
//
//   为什么长度能从 TYPE 推出来(而不是带长度字段, §3.2): 类型少、每型长度恒定 ⇒ 省掉长度字段
//   与"长度字段本身可不可信"这一层; 代价是位宽一改就换帧长, 所以本文件每个长度都由"字段位宽
//   相加"经 static_assert 钉住 —— 改位宽必然编译报错, 不会悄悄换掉线上格式。
//
//   为什么不用 dlx.hpp 既有的 KSP 帧(ControlToFlight/FlightCoreStatus): 它们的帧头是 11 位类型号
//   + CRC4(漏检 1/16), 载荷是"目标姿态"而不是摇杆(§3.4 修正 1)。决策记录 9 已定: 旧的无线 DLX
//   报文保留定义但不再使用。
//
// ---- 长度表(v2 · 13 型; "按位相加"的结果, 就是自定界表) ----
//   TYPE  名字        负载位宽  负载字节  帧长B   算式
//   0x1   DL_ATT      80b       10        13      q1..q3(3×12) + vx..vz(3×8) + status(8) + error(8) + arming(2) + mode(2)   [布局同 v1]
//   0x2   DL_SYS      96b       12        15      x,y,z(3×12) + current(8) + voltage(8) + motor[4](4×11)                    [布局同 v1]
//   0x3   DL_GPS      144b      18        21      lat,lon(2×32) + altMslMm(32) + gspeedCms(16) + courseCdeg(16) + sats(6) + fix(2) + hdop(8) [布局同 v1]
//   0x4   DL_STATE    53b+3空   7         10      lifecycle(3)+arming(2)+mode(3)+task(2)+failsafeLvl(3)+failsafeRsn(4)+health(8)+ulLoss(8)+dlLoss(8)+rssi(8)+rxBitmap(4)+保留(3)
//   0x5   DL_ACK      16b       2         5       cmdId(8) + result(8)                                                          [v2 改]
//   0x6   UP_ROUND    64b       8         11      roll,pitch,yaw(3×11) + thr(11) + flags(4) + session(8) + flags8(8)          [v2 新增]
//   0x7   HELLO       16b       2         5       ver(4) + caps(4) + nonce(8)                                                   [v2 新增, 双向同型]
//   0x8   UP_CTRL     48b       6         9       roll,pitch,yaw(3×11) + thr(11) + flags(4)                                      [布局同 v1, flags 语义改]
//   0x9   UP_CMD      24b       3         6       cmdId(8) + op(8) + arg(8)                                                     [v2 改]
//   0xA   UP_VEL      24b       3         6       vx,vy,vz(3×8)                                                                 [布局同 v1]
//   0xB   UP_POS      32b       4         7       x,y(2×16)                                                                     [布局同 v1, 保留未用]
//   0xC   UP_SETPOINT 24b       3         6       altCm(16) + flags(8)                                                          [布局同 v1]
//   0xD   UP_MOTOR    44b+4空   6         9       motor[4](4×11) + 4 个恒 0 空位                                                [布局同 v1]
//   (帧长 = 帧头 1B + 负载 + CRC16 2B; 最大 21B, 与 §3.1 的 AT+PACKET=32B、"一帧 = 一个射频包"配套)
//
// ---- 2026-10-05 协议冻结(遥控器闭环批次)的三处变更 ----
//   变更 1: UP_VEL 由 vx,vy(16b) 扩成 vx,vy,vz(24b) —— 定速模式要求 XYZ 三轴绝对式,
//     而"一个上行槽只装得下一帧", 让 XY 走 UP_VEL、Z 走 UP_SETPOINT 就得两个类型交替占槽,
//     同一组控制量还会相差一个槽的相位。扩成一帧三轴后: **每种模式只需要一个制导帧类型**。
//     这是**破坏性变更**(4B→5B), 两端必须同时升级; 旧端按 4B 切帧会错位并被 CRC 拒掉(丢 1 字节重同步)。
//   变更 2: 新增 UP_SETPOINT(0xC) —— 定高的**绝对**期望高度。为什么不复用 UP_CTRL.thr:
//     那会让同一个字段在两种模式下有两种量纲(油门/高度), 飞控、日志、UI、上位机四处都要记住
//     "现在这个数是什么", 漏一处就是"飞机按 0.28 的油门往上冲"而不是一个报错。
//   变更 3: 新增 UP_MOTOR(0xD) —— 测试模式直控, 值用 **DShot 计数原样**(与 DL_SYS.motor 同量纲)。
//     只在 mode==Manual 且 state==Armed 时生效。
//   ⚠ 2026-10-12 **破坏性变更**: UP_MOTOR 由"单路 index + throttle"(4B) 改为"**四路一次发全**"(8B)。
//     理由见 upMotor 命名空间上方那段(单路模型里四路不是同一时刻的, 且未定义 index 4..6 会混叠到
//     真实通道上, 需要一条专门的拒绝路径)。两端必须同时升级: 旧端按 4B 切帧会被 CRC 拒掉(丢 1 字节重同步)。
//   ⚠ 同时**修正 UP_CTRL.flags 的含义**: 见 upCtrl::kFlag* —— 旧定义(bit0=ARM 开关 / bit1..2=模式请求 /
//     bit3=KILL)是"命令还走 flags"那版设计的残留, 现在 ARM/模式/KILL 全部走 UP_CMD;
//     而遥控器侧一直在 bit0 上置"请求下行窗口"。两端对同一 bit 有两种理解 = 迟早把窗口位当解锁开关。
//
// ---- v2 变更摘要(2026-10-18; 依据 链路层v2_协议规格 §1.2/§3) ----
//   1. CRC-8 → CRC-16/CCITT-FALSE(低字节在前): 所有帧整帧 +1B, 负载位布局一位不动;
//   2. 新增 0x6 UP_ROUND(= upCtrl 6B + session 8b + flags8 8b, 8B 负载)与 0x7 HELLO(ver4|caps4|nonce8);
//   3. UP_CMD 负载 2B→3B(cmdId 显式携带, 不再靠帧头 SEQ 配对命令)、DL_ACK 负载 op→cmdId;
//   4. DL_STATE 原 7 个恒 0 空位改为"本轮 U0..U3 接收位图"(bit49..52) + 3 个保留位(bit53..55)。
//   除以上四项, 帧类型号 / 位宽 / 位偏移 / 定点换算与 v1 逐位一致: 任何"顺手改"都是协议变更。

// ---- ⚠ 与方案 §3.3 的两处更正(其余帧与 §3.3 完全一致; v2 起仍按 v1 位布局保留) ----
//   更正 1: DL_STATE 负载 **7B 不是 6B**(v1 帧长 9B; v2 加 CRC16 后 10B)。§3.3 表里写 8B, 但按它
//     自己给的位宽相加: 3+2+3 + 2+3 + 4 + 8+8+8+8 = 49 bit ⇒ 需要 ⌈49/8⌉ = 7 字节负载。
//     v2 把 bit49..52 用作上行接收位图、bit53..55 保留恒 0(见 dlState::kRxBitmapBit;
//     encode 时先整帧清零)。
//   更正 2: `mode` 在 DL_ATT 是 2b、在 DL_STATE 是 3b(照抄 §3.3), 而 §7.1 C 说飞行模式只有
//     2 bit(4 个模式: Manual/Stabilize/AltHold/VelHold —— 即如今的 `app::Mode`)。本文件**不改位宽**
//     (保持与 §3.3 帧表逐位一致, 也保持 DL_STATE 的 3b 便于以后扩模式), 但两边都饱和到各自位宽:
//     DL_ATT 的 mode 上限 3, DL_STATE 的 mode 上限 7。要统一的话是改 §3.3/§7.1 的事, 不是改这里。
//
// ---- 定点约定总表(全部换算只在本文件实现一次, 调用方别抄第二份) ----
//   字段                线上表示        换算                    饱和域(超范围**饱和不回绕**)
//   q1,q2,q3            int12           round(x*2047)           ±2047   ⇒ x ∈ ±1.0
//   vx,vy,vz (DL_ATT)   int8  dm/s      round(mps*10)           -128..127 ⇒ -12.8..+12.7 m/s
//   x,y,z    (DL_SYS)   int12 dm        round(m*10)             -2048..2047 ⇒ -204.8..+204.7 m
//   current, voltage    u8    0.1 单位  round(v*10)             0..255  ⇒ 0..25.5 A / 0..25.5 V
//   motor[4]            u11            DShot 计数原样           0..1999 (2000..2047 是 DShot 特殊命令区)
//   lat, lon            int32  1e-7°   原样(源本身就是定点)     int32 全域
//   altMslMm            int32  mm       原样                     int32 全域
//   gspeedCms           int16  cm/s     原样                     int16 全域
//   courseCdeg          u16    0.01°    原样                     0..35999 (359.99°, 不含 360.00°)
//   sats                u6             原样                     0..63
//   fix                 u2             原样                     0..3
//   hdop                u8    0.1       round(x*10)              0..255 ⇒ 0..25.5
//   rssi                int8  dBm       原样(**有符号**)        -128..127
//   ulLoss, dlLoss      u8    %         原样                     0..100
//   roll/pitch/yaw      int11 **补码**  counts 原样               ±1023 (语义域, §3.6)
//   thr                 u11            counts 原样               0..2047
//   vx,vy,vz (UP_VEL)   int8  5cm/s     round(mps*20)            -128..127 ⇒ -6.40..+6.35 m/s
//   x,y      (UP_POS)   int16 1dm       round(m*10)              -32768..32767 ⇒ -3276.8..+3276.7 m
//   altCm    (UP_SETPOINT) int16 cm     round(m*100)             -32767..32767 ⇒ ±327.67 m
//   throttle (UP_MOTOR) u11            DShot 计数原样           0..1999 (与 DL_SYS.motor 同一量纲)
//
//   饱和域的取法有两种, 别混: **归一化量**(q1..q3, 摇杆)饱和到对称满量程 ±(2^(n-1)-1), 因为
//   它们的满量程是"协议约定的 ±1/±1023"; **物理量**(位置/速度/电压)饱和到位宽全域, 因为多出来的
//   那个码点(如 int12 的 -2048)表示的是一个真实的、更远的距离, 没有理由白扔。
//
//   为什么摇杆用 11 位**补码**而不是"偏移二进制"(u11 = counts + 1024): 偏移编码下线上 0 会解出
//   -1024(≈满左舵), 于是 CRC 没兜住的噪声字节、或一块还没写过的 0 缓冲区, 会被解释成"副翼打死";
//   补码下 0 就是中立位, 失控/未初始化时杆量落在中立点, 这是飞控唯一能接受的方向。代价是 -1024
//   这个码点不可用(丢掉 1/2048 行程), 且解码时把它夹回 -1023。依据: §3.6 摇杆 int11 [-1023,1023]。
//
//   为什么负载结构体里 GPS 用整数、姿态/速度/位置用 float: 结构体字段取**数据源的天然表示**。
//   GPS 源(NMEA)给的就是"度×1e7 / mm / cm/s"; 1e-7 度要保精度得 ~31 位有效位, float 只有 24 位,
//   所以要么用 double(M4 上是软件模拟, 几百个周期)要么丢精度 —— 而它本来就是定点整数, 直接过线最省。
//   反过来 q/速度/位置在飞控里本来就是 EKF 出来的 float, 定点化必须由编解码层统一做, 免得调用方
//   各写一份 round(x*10)。
//
//   本文件**不定义**语义枚举(UP_CMD.op 命令表 / DL_ACK.result 结果码 / 状态机 4 维与 Failsafe):
//   它们的唯一真相是 §3.7 命令表与状态机头文件(状态机批次), 链路层只做位搬运。在这里再抄一份
//   枚举, 两边必然漂 —— 而且遥控器工程也在包含本文件, 它连飞控的 cfg:: 生成头都拿不到。
//
//   C++11 / 无异常 / 无 RTTI / 无动态分配 / header-only。
//===================================================================================================

#include <stdint.h>
#include "dlx_bits.hpp" // 同目录: 不用 "link/dlx_bits.hpp", 这样单独 -fsyntax-only 本文件也能过

namespace dlx
{
namespace link
{
    //===============================================================================================
    // 一、帧类型与帧头
    //===============================================================================================
    /** 帧类型(帧头高 4 位)。类型值就是协议表里的 TYPE 号, 不允许改 —— 它决定帧长, 是自定界的根。
     *  v2: 0x6 UP_ROUND / 0x7 HELLO 为新增; 其余类型号与 v1 完全一致(不重排)。 */
    enum class FrameType : uint8_t
    {
        DlAtt   = 0x1, ///< 飞控 → 遥控器: 姿态 + 机体速度 + 状态/错误 + 解锁/模式(13B)
        DlSys   = 0x2, ///< 飞控 → 遥控器: 位置 + 电流电压 + 四路电机(15B)
        DlGps   = 0x3, ///< 飞控 → 遥控器: 经纬高 + 地速航向 + 星数/定位质量(21B)
        DlState = 0x4, ///< 飞控 → 遥控器: 状态机 + 健康 + 丢包/RSSI + 上行接收位图(10B)
        DlAck   = 0x5, ///< 飞控 → 遥控器: 命令应答 cmdId + result(5B)
        UpRound = 0x6, ///< 遥控器 → 飞控: 杆量 + session + flags8, 每轮 U0(11B)
        Hello   = 0x7, ///< 双向: ver4 + caps4 + nonce8, 握手/探测(5B)
        UpCtrl  = 0x8, ///< 遥控器 → 飞控: 摇杆 + flags(9B)
        UpCmd   = 0x9, ///< 遥控器 → 飞控: cmdId + 命令 + 参数(6B)
        UpVel   = 0xA, ///< 遥控器/上位机 → 飞控: 速度指令 vx,vy,vz(6B)
        UpPos   = 0xB, ///< 遥控器/上位机 → 飞控: 位置注入(7B, 保留未用)
        UpSetpoint = 0xC, ///< 遥控器 → 飞控: 定高期望高度 altCm + flags(6B)
        UpMotor    = 0xD, ///< 遥控器 → 飞控: 测试模式**四路** DShot 计数一次发全(9B)
    };

    /** 帧头字节 = (TYPE << 4) | (SEQ & 0x0F)(§3.2) */
    constexpr uint8_t frameHead(FrameType t, uint8_t seq)
    {
        return (uint8_t)(((uint8_t)t << 4) | (seq & 0x0Fu));
    }

    /** 从帧头字节取类型(高 4 位) */
    constexpr FrameType frameTypeOf(uint8_t byte0)
    {
        return (FrameType)(uint8_t)(byte0 >> 4);
    }

    /** 从帧头字节取 SEQ(低 4 位) */
    constexpr uint8_t seqOf(uint8_t byte0)
    {
        return (uint8_t)(byte0 & 0x0Fu);
    }

    //===============================================================================================
    // 二、每帧的位布局(常量表达式串起来 + static_assert 钉住)
    //
    //   位偏移一律**相对负载首字节**(即帧的 byte1), 不含帧头与 CRC。
    //   每一帧用一个独立小 namespace: 偏移由"上一个字段的偏移 + 宽度"推导, 代码里用的是同一个
    //   常量, 所以"表"与"实现"不可能各写一份而对不上 —— 这是 §3.4 要求的"按位相加即可 static_assert"。
    //===============================================================================================

    // ---- DL_ATT: q1,q2,q3 各 12b → vx,vy,vz 各 8b → status 8b → error 8b → arming 2b → mode 2b ----
    namespace dlAtt
    {
        constexpr uint8_t  kQ1Bits     = 12; ///< int12, 四元数虚部 q1
        constexpr uint16_t kQ1Bit      = 0;
        constexpr uint8_t  kQ2Bits     = 12;
        constexpr uint16_t kQ2Bit      = (uint16_t)(kQ1Bit + kQ1Bits); // 12
        constexpr uint8_t  kQ3Bits     = 12;
        constexpr uint16_t kQ3Bit      = (uint16_t)(kQ2Bit + kQ2Bits); // 24
        constexpr uint8_t  kVxBits     = 8; ///< int8, dm/s
        constexpr uint16_t kVxBit      = (uint16_t)(kQ3Bit + kQ3Bits); // 36
        constexpr uint8_t  kVyBits     = 8;
        constexpr uint16_t kVyBit      = (uint16_t)(kVxBit + kVxBits); // 44
        constexpr uint8_t  kVzBits     = 8;
        constexpr uint16_t kVzBit      = (uint16_t)(kVyBit + kVyBits); // 52
        constexpr uint8_t  kStatusBits = 8; ///< u8, 原样
        constexpr uint16_t kStatusBit  = (uint16_t)(kVzBit + kVzBits); // 60
        constexpr uint8_t  kErrorBits  = 8; ///< u8, 原样
        constexpr uint16_t kErrorBit   = (uint16_t)(kStatusBit + kStatusBits); // 68
        constexpr uint8_t  kArmingBits = 2; ///< u2, 语义域 0..3
        constexpr uint16_t kArmingBit  = (uint16_t)(kErrorBit + kErrorBits);   // 76
        constexpr uint8_t  kModeBits   = 2; ///< u2, 语义域 0..3(见文件头"更正 2")
        constexpr uint16_t kModeBit    = (uint16_t)(kArmingBit + kArmingBits); // 78
        constexpr uint16_t kPayloadBits  = (uint16_t)(kModeBit + kModeBits);   // 80
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits);   // 10
    } // namespace dlAtt

    // ---- DL_SYS: x,y,z 各 12b → current 8b → voltage 8b → motor[4] 各 11b ----
    namespace dlSys
    {
        constexpr uint8_t  kXBits       = 12; ///< int12, dm
        constexpr uint16_t kXBit        = 0;
        constexpr uint8_t  kYBits       = 12;
        constexpr uint16_t kYBit        = (uint16_t)(kXBit + kXBits); // 12
        constexpr uint8_t  kZBits       = 12;
        constexpr uint16_t kZBit        = (uint16_t)(kYBit + kYBits); // 24
        constexpr uint8_t  kCurrentBits = 8; ///< u8, 0.1A
        constexpr uint16_t kCurrentBit  = (uint16_t)(kZBit + kZBits); // 36
        constexpr uint8_t  kVoltageBits = 8; ///< u8, 0.1V
        constexpr uint16_t kVoltageBit  = (uint16_t)(kCurrentBit + kCurrentBits); // 44
        constexpr uint8_t  kMotorBits   = 11; ///< u11, DShot 计数
        constexpr uint16_t kMotor0Bit   = (uint16_t)(kVoltageBit + kVoltageBits); // 52
        constexpr uint16_t kMotor1Bit   = (uint16_t)(kMotor0Bit + kMotorBits);    // 63
        constexpr uint16_t kMotor2Bit   = (uint16_t)(kMotor1Bit + kMotorBits);    // 74
        constexpr uint16_t kMotor3Bit   = (uint16_t)(kMotor2Bit + kMotorBits);    // 85
        constexpr uint16_t kPayloadBits  = (uint16_t)(kMotor3Bit + kMotorBits);   // 96
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits);      // 12
        constexpr uint8_t  kMotorCount   = 4; ///< 四路(§3.3 表里就是 motor[4])
    } // namespace dlSys

    // ---- DL_GPS: lat,lon 各 32b → altMslMm 32b → gspeedCms 16b → courseCdeg 16b → sats 6b → fix 2b → hdop 8b ----
    namespace dlGps
    {
        constexpr uint8_t  kLatBits        = 32; ///< int32, 度×1e7
        constexpr uint16_t kLatBit         = 0;
        constexpr uint8_t  kLonBits        = 32;
        constexpr uint16_t kLonBit         = (uint16_t)(kLatBit + kLatBits); // 32
        constexpr uint8_t  kAltBits        = 32; ///< int32, mm
        constexpr uint16_t kAltBit         = (uint16_t)(kLonBit + kLonBits); // 64
        constexpr uint8_t  kGspeedBits     = 16; ///< int16, cm/s
        constexpr uint16_t kGspeedBit      = (uint16_t)(kAltBit + kAltBits); // 96
        constexpr uint8_t  kCourseBits     = 16; ///< u16, 0.01°
        constexpr uint16_t kCourseBit      = (uint16_t)(kGspeedBit + kGspeedBits); // 112
        constexpr uint8_t  kSatsBits       = 6;  ///< u6, 0..63
        constexpr uint16_t kSatsBit        = (uint16_t)(kCourseBit + kCourseBits); // 128
        constexpr uint8_t  kFixBits        = 2;  ///< u2, 0..3
        constexpr uint16_t kFixBit         = (uint16_t)(kSatsBit + kSatsBits);     // 134
        constexpr uint8_t  kHdopBits       = 8;  ///< u8, 0.1
        constexpr uint16_t kHdopBit        = (uint16_t)(kFixBit + kFixBits);       // 136
        constexpr uint16_t kPayloadBits    = (uint16_t)(kHdopBit + kHdopBits);    // 144
        constexpr uint16_t kPayloadBytes   = bits::bytesForBits(kPayloadBits);     // 18
        constexpr uint16_t kCourseMaxCdeg  = 35999; ///< 359.99°; 360.00° 归 0.00°(在编码前做)
        constexpr uint8_t  kSatsMax        = 63;    ///< u6 满值
        constexpr uint8_t  kFixMax         = 3;     ///< u2 满值
    } // namespace dlGps

    // ---- DL_STATE: lifecycle 3b → arming 2b → mode 3b → task 2b → failsafeLvl 3b → failsafeRsn 4b
    //                 → health 8b → ulLoss 8b → dlLoss 8b → rssi 8b → rxBitmap 4b → 保留 3b ----
    //   v2(协议规格 §3.5): v1 的 7 个恒 0 空位改为"本轮 U0..U3 接收位图" + 3 个保留位。
    //   位图由 RoundSlave 填(本轮已收到哪些上行位置), 帧层只做位搬运; 语义在状态机。
    namespace dlState
    {
        constexpr uint8_t  kLifecycleBits   = 3; ///< 0..7(§7.1 A)
        constexpr uint16_t kLifecycleBit    = 0;
        constexpr uint8_t  kArmingBits      = 2; ///< 0..3(§7.1 B)
        constexpr uint16_t kArmingBit       = (uint16_t)(kLifecycleBit + kLifecycleBits); // 3
        constexpr uint8_t  kModeBits        = 3; ///< 0..7(§7.1 C 说 2 bit, 见文件头"更正 2")
        constexpr uint16_t kModeBit         = (uint16_t)(kArmingBit + kArmingBits);       // 5
        constexpr uint8_t  kTaskBits        = 2; ///< 0..3(§7.1 D)
        constexpr uint16_t kTaskBit         = (uint16_t)(kModeBit + kModeBits);           // 8
        constexpr uint8_t  kFailsafeLvlBits = 3; ///< 0..7(§7.1 F)
        constexpr uint16_t kFailsafeLvlBit  = (uint16_t)(kTaskBit + kTaskBits);           // 10
        constexpr uint8_t  kFailsafeRsnBits = 4; ///< 0..15(§7.1 F)
        constexpr uint16_t kFailsafeRsnBit  = (uint16_t)(kFailsafeLvlBit + kFailsafeLvlBits); // 13
        constexpr uint8_t  kHealthBits      = 8; ///< 位域原样(§7.1 E)
        constexpr uint16_t kHealthBit       = (uint16_t)(kFailsafeRsnBit + kFailsafeRsnBits); // 17
        constexpr uint8_t  kUlLossBits      = 8; ///< %
        constexpr uint16_t kUlLossBit       = (uint16_t)(kHealthBit + kHealthBits);   // 25
        constexpr uint8_t  kDlLossBits      = 8; ///< %
        constexpr uint16_t kDlLossBit       = (uint16_t)(kUlLossBit + kUlLossBits);   // 33
        constexpr uint8_t  kRssiBits        = 8; ///< int8 dBm(§3.6 显示行 "R:-60dBm" ⇒ 必须有符号)
        constexpr uint16_t kRssiBit         = (uint16_t)(kDlLossBit + kDlLossBits);   // 41
        // ---- v2: 原空位(bit49..55)的新用途 ----
        constexpr uint16_t kRxBitmapBit     = (uint16_t)(kRssiBit + kRssiBits);       // 49
        constexpr uint8_t  kRxBitmapBits    = 4;   ///< bit49..52 = 本轮 pos0..pos3 是否收到
        constexpr uint8_t  kRxPos0Bit       = 49;
        constexpr uint8_t  kRxPos1Bit       = 50;
        constexpr uint8_t  kRxPos2Bit       = 51;
        constexpr uint8_t  kRxPos3Bit       = 52;
        constexpr uint8_t  kRxBitmapMask    = 0x0Fu;
        constexpr uint16_t kUsedBits        = (uint16_t)(kRxBitmapBit + kRxBitmapBits); // 53
        constexpr uint16_t kRxReservedBit   = kUsedBits;                                 // 53
        constexpr uint8_t  kRxReservedBits  = 3;                                         // 53..55 恒 0
        constexpr uint16_t kSpareBit        = kRxReservedBit; ///< v1 名字的兼容别名(现指保留位起点)
        constexpr uint16_t kPayloadBits     = 56; ///< 53 bit 用量 ⇒ ⌈53/8⌉ = 7B = 56 bit(含 3 个保留位)
        constexpr uint16_t kPayloadBytes    = bits::bytesForBits(kPayloadBits);       // 7
        constexpr uint8_t  kSpareBits       = (uint8_t)(kPayloadBits - kUsedBits);    // 3, 必须恒 0
    } // namespace dlState

    // ---- DL_ACK: cmdId 8b → result 8b ----
    //   v2(协议规格 §3.4): 取消 v1 的"帧头 SEQ 回显被应答命令的 SEQ", 改为显式 cmdId。
    //   这样下行 SEQ 可以严格"方向内自增"(丢包统计不再被 ACK 打断)。
    namespace dlAck
    {
        constexpr uint8_t  kCmdIdBits     = 8; ///< 被应答的 UP_CMD.cmdId 原样回显
        constexpr uint16_t kCmdIdBit      = 0;
        constexpr uint8_t  kResultBits    = 8; ///< 结果码(0=OK … 11=AuthFail; 与 GroundStatus 同值)
        constexpr uint16_t kResultBit     = (uint16_t)(kCmdIdBit + kCmdIdBits); // 8
        constexpr uint16_t kPayloadBits   = (uint16_t)(kResultBit + kResultBits); // 16
        constexpr uint16_t kPayloadBytes  = bits::bytesForBits(kPayloadBits);     // 2
    } // namespace dlAck

    // ---- UP_CTRL: roll,pitch,yaw 各 11b → thr 11b → flags 4b ----
    namespace upCtrl
    {
        constexpr uint8_t  kRollBits      = 11; ///< int11 补码, ±1023
        constexpr uint16_t kRollBit       = 0;
        constexpr uint8_t  kPitchBits     = 11;
        constexpr uint16_t kPitchBit      = (uint16_t)(kRollBit + kRollBits);  // 11
        constexpr uint8_t  kYawBits       = 11;
        constexpr uint16_t kYawBit        = (uint16_t)(kPitchBit + kPitchBits); // 22
        constexpr uint8_t  kThrBits       = 11; ///< uint11, 0..2047
        constexpr uint16_t kThrBit        = (uint16_t)(kYawBit + kYawBits);     // 33
        constexpr uint8_t  kFlagsBits     = 4;  ///< 低 4 位有效
        constexpr uint16_t kFlagsBit      = (uint16_t)(kThrBit + kThrBits);     // 44
        constexpr uint16_t kPayloadBits   = (uint16_t)(kFlagsBit + kFlagsBits); // 48
        constexpr uint16_t kPayloadBytes  = bits::bytesForBits(kPayloadBits);   // 6
        constexpr int32_t  kStickMin      = -1023; ///< 语义域下界(§3.6); 线上还能表示 -1024, 解码时夹回这里
        constexpr int32_t  kStickMax      = 1023;
        constexpr int32_t  kThrMin        = 0;
        constexpr int32_t  kThrMax        = 2047;
        /** flags 位定义(v2 · 协议规格 §3.6)。**2026-10-18 改语义**:
         *    bit0 = ESTOP(锁存 flag): 遥控器每帧重复置位, 飞控收到任一帧该位即锁存 hardStop,
         *           直到新的 ARM(与 v1 的"请求下行窗口"完全不同 —— 下行窗口改由轮次协议保证,
         *           v1 那个 bit0 因此作废, 同一个 bit 两种理解正是 v1 埋下的地雷)。
         *    bit1 = yaw 场有效(本轮恒 0: 无可靠偏航参考, 飞控直接忽略 yaw 场)
         *    bit2..3 保留恒 0(解码端发现非 0 可当坏帧或忽略)
         *  帧里仍是 4 bit(位宽/偏移与 v1 完全一致), 只有 bit0 的含义换成了 ESTOP。 */
        constexpr uint8_t kFlagEstop      = 0x01u; ///< bit0 ESTOP 锁存 flag(每帧重复发)
        constexpr uint8_t kFlagYawValid   = 0x02u; ///< bit1 yaw 场有效(本轮恒 0)
        constexpr uint8_t kFlagsMask      = 0x0Fu; ///< 帧里只有低 4 位; 保留位按原样搬运, 由解码端判断
    } // namespace upCtrl

    // ---- UP_CMD(v2): cmdId 8b → op 8b → arg 8b ----
    //   协议规格 §3.3: cmdId 由遥控器分配(1..255 循环, 0 无效); 重传同一 cmdId,
    //   飞控按 (session, cmdId) 去重(最近 8 条), 命中只重发 ACK、不重复执行。
    //   v1 靠"DL_ACK 回显被应答帧的帧头 SEQ"配对命令, 那条路已取消(帧头 SEQ 要留给丢包统计)。
    namespace upCmd
    {
        constexpr uint8_t  kCmdIdBits    = 8; ///< 1..255 循环, 0=无效
        constexpr uint16_t kCmdIdBit     = 0;
        constexpr uint8_t  kOpBits       = 8; ///< 命令号(命令表语义在 app 层, 本层不解释)
        constexpr uint16_t kOpBit        = (uint16_t)(kCmdIdBit + kCmdIdBits); // 8
        constexpr uint8_t  kArgBits      = 8; ///< 参数(模式号 / 目标高度 dm / 参数 index …)
        constexpr uint16_t kArgBit       = (uint16_t)(kOpBit + kOpBits); // 16
        constexpr uint16_t kPayloadBits  = (uint16_t)(kArgBit + kArgBits); // 24
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits); // 3
        constexpr uint8_t  kCmdIdMin     = 1u;   ///< 0 保留不用
        constexpr uint8_t  kCmdIdMax     = 255u;
    } // namespace upCmd

    // ---- UP_VEL: vx,vy,vz 各 8b(int8 @5cm/s) ----
    //   ⚠ 2026-10-05: 由 2 轴扩到 3 轴(定速模式 XYZ 绝对式)。帧长 4B → 5B, 破坏性变更, 两端同时升级。
    //   坐标系 = **世界系**(x 北 / y 东 / z 上, 向上为正), 与 velocity_controller.hpp 的输入同系;
    //   而 DL_ATT 回传的 vx,vy,vz 是**机体速度** —— 两端做"速度差"曲线时, 遥控器必须先用回传姿态
    //   把机体速度旋到世界系再作差, 否则曲线会随偏航角"转"(看着像控制器坏了)。
    namespace upVel
    {
        constexpr uint8_t  kVxBits       = 8; ///< int8 @5cm/s
        constexpr uint16_t kVxBit        = 0;
        constexpr uint8_t  kVyBits       = 8;
        constexpr uint16_t kVyBit        = (uint16_t)(kVxBit + kVxBits); // 8
        constexpr uint8_t  kVzBits       = 8; ///< int8 @5cm/s, 向上为正
        constexpr uint16_t kVzBit        = (uint16_t)(kVyBit + kVyBits); // 16
        constexpr uint16_t kPayloadBits  = (uint16_t)(kVzBit + kVzBits); // 24
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits); // 3
    } // namespace upVel

    // ---- UP_SETPOINT: altCm 16b → flags 8b ----
    //   定高的**绝对**期望高度(遥控器本地把"增量式"的左摇杆积分成绝对值再发)。
    //   为什么只有一个量: "垂速"已经归 UP_VEL.vz —— 一帧一义, 高度就是高度, 速度就是速度。
    namespace upSetpoint
    {
        constexpr uint8_t  kAltBits      = 16; ///< int16 @1cm, 向上为正
        constexpr uint16_t kAltBit       = 0;
        constexpr uint8_t  kFlagsBits    = 8;
        constexpr uint16_t kFlagsBit     = (uint16_t)(kAltBit + kAltBits); // 16
        constexpr uint16_t kPayloadBits  = (uint16_t)(kFlagsBit + kFlagsBits); // 24
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits);   // 3
        /** bit0 有效; 其余保留恒 0(与 DL_STATE 的空位同一约定: 解码端发现非 0 可按坏帧处理) */
        constexpr uint8_t  kFlagAltValid = 0x01u;
        constexpr uint8_t  kFlagsMask    = 0x01u;
    } // namespace upSetpoint

    // ---- UP_MOTOR: 四路 × 11b = 44b → 6B 负载 → 8B 帧 ----
    //   测试模式直控，**四路一次发全**。throttle 用 **DShot 计数原样**(与 DL_SYS.motor 同一量纲):
    //   测试模式"发什么就转什么", 两端不必再各写一份归一化换算。
    //
    //   为什么从"单路 + index(4B)"改成"一帧四路(8B)"(2026-10-12):
    //     ① 单路模型里四路**不是同一时刻**的 —— 遥控器连发 4 帧期间, 飞控缓存里可能"新三路 + 旧一路";
    //     ② "哪一路刚被寻址"必须靠一个额外的 index 记账(原来的 `motorCmdIndex`), 而协议未定义的
    //        码点 4..6 会被 `index & 0x3` **混叠**到真实通道上, 于是需要一条专门的拒绝路径
    //        (`applyMotorDirect` 的返回值 + `motorDirectRejects` 计数);
    //     ③ 一帧就是**完整状态**之后, 上面那两层都可以删掉: 没有 index、没有半更新、没有混叠。
    //   代价: 帧从 4B 变 8B —— 上行 33Hz 下多 132B/s, 相对 57600bps 的空口可以忽略。
    namespace upMotor
    {
        constexpr uint8_t  kMotorBits   = 11; ///< u11, DShot 计数 0..1999(2000..2047 是特殊命令区)
        constexpr uint8_t  kMotorCount  = 4;  ///< 四路一次发全
        constexpr uint16_t kMotor0Bit   = 0;
        constexpr uint16_t kMotor1Bit   = (uint16_t)(kMotor0Bit + kMotorBits); // 11
        constexpr uint16_t kMotor2Bit   = (uint16_t)(kMotor1Bit + kMotorBits); // 22
        constexpr uint16_t kMotor3Bit   = (uint16_t)(kMotor2Bit + kMotorBits); // 33
        constexpr uint16_t kUsedBits    = (uint16_t)(kMotor3Bit + kMotorBits); // 44
        constexpr uint16_t kPayloadBits = 48; ///< 44b 用量 ⇒ 6B(含 4 个恒 0 空位)
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits);   // 6
        constexpr uint8_t  kSpareBits   = (uint8_t)(kPayloadBits - kUsedBits); // 4, 必须恒 0
        constexpr uint16_t kSpareBit    = kUsedBits;                           // 44
    } // namespace upMotor

    // ---- UP_POS: x,y 各 16b(int16 @1dm) ----
    namespace upPos
    {
        constexpr uint8_t  kXBits        = 16; ///< int16 @1dm
        constexpr uint16_t kXBit         = 0;
        constexpr uint8_t  kYBits        = 16;
        constexpr uint16_t kYBit         = (uint16_t)(kXBit + kXBits); // 16
        constexpr uint16_t kPayloadBits  = (uint16_t)(kYBit + kYBits); // 32
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits); // 4
    } // namespace upPos

    // ---- UP_ROUND(v2 新增, 0x6): upCtrl 6B + session 8b + flags8 8b = 64b ----
    //   协议规格 §3.1: 每轮 U0 必发, 是"轮次开始 + 最新杆量 + session"三合一。
    //   杆量/flags 的量纲与 UP_CTRL **完全一致**(同一套 ±1023 / 0..2047 / 4bit flags),
    //   所以它可以单独维持控制(U0 到了就等于本轮至少有一拍新鲜杆量)。
    namespace upRound
    {
        constexpr uint8_t  kRollBits     = 11; ///< int11 补码, ±1023
        constexpr uint16_t kRollBit      = 0;
        constexpr uint8_t  kPitchBits    = 11;
        constexpr uint16_t kPitchBit     = (uint16_t)(kRollBit + kRollBits);   // 11
        constexpr uint8_t  kYawBits      = 11; ///< yaw 无效时填 0
        constexpr uint16_t kYawBit       = (uint16_t)(kPitchBit + kPitchBits); // 22
        constexpr uint8_t  kThrBits      = 11; ///< u11, 0..2047
        constexpr uint16_t kThrBit       = (uint16_t)(kYawBit + kYawBits);     // 33
        constexpr uint8_t  kFlagsBits    = 4;  ///< 同 UP_CTRL flags(bit0=ESTOP)
        constexpr uint16_t kFlagsBit     = (uint16_t)(kThrBit + kThrBits);     // 44
        constexpr uint8_t  kSessionBits  = 8;  ///< 遥控器本次启动随机 1..255(0 保留)
        constexpr uint16_t kSessionBit   = (uint16_t)(kFlagsBit + kFlagsBits); // 48
        constexpr uint8_t  kFlags8Bits   = 8;
        constexpr uint16_t kFlags8Bit    = (uint16_t)(kSessionBit + kSessionBits); // 56
        constexpr uint16_t kPayloadBits  = (uint16_t)(kFlags8Bit + kFlags8Bits);   // 64
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits);       // 8
        // flags8 位定义(协议规格 §3.1)
        constexpr uint8_t  kFlag8Estop    = 0x01u; ///< bit0 ESTOP(与 flags.bit0 同值, 冗余)
        constexpr uint8_t  kFlag8YawValid = 0x02u; ///< bit1 yaw 有效
        constexpr uint8_t  kFlag8ReqGps   = 0x04u; ///< bit2 请求 DL_GPS
        constexpr uint8_t  kFlags8Mask    = 0x07u; ///< bit3..7 保留恒 0
        constexpr uint8_t  kSessionMin    = 1u;
        constexpr uint8_t  kSessionMax    = 255u;
    } // namespace upRound

    // ---- HELLO(v2 新增, 0x7, 双向同型): ver 4b → caps 4b → nonce 8b = 16b ----
    //   协议规格 §3.2: 请求方填 nonce, 应答方原样回显 nonce、填自己的 ver/caps。
    //   版本值不在帧层定义(那是 link_v2_config.hpp 的 LinkProtoVersion), 这里只管位宽。
    namespace hello
    {
        constexpr uint8_t  kVerBits      = 4;  ///< 0 保留; v2 = 2
        constexpr uint16_t kVerBit       = 0;
        constexpr uint8_t  kCapsBits     = 4;
        constexpr uint16_t kCapsBit      = (uint16_t)(kVerBit + kVerBits); // 4
        constexpr uint8_t  kNonceBits    = 8;
        constexpr uint16_t kNonceBit     = (uint16_t)(kCapsBit + kCapsBits); // 8
        constexpr uint16_t kPayloadBits  = (uint16_t)(kNonceBit + kNonceBits); // 16
        constexpr uint16_t kPayloadBytes = bits::bytesForBits(kPayloadBits);   // 2
        // caps 位定义(协议规格 §3.2)
        constexpr uint8_t  kCapsGnss     = 0x01u; ///< bit0 GNSS/GPS 相关(RC: 想看 DL_GPS; FC: GNSS 已开)
        constexpr uint8_t  kCapsGuidance = 0x02u; ///< bit1 支持制导帧
        constexpr uint8_t  kCapsCmdId    = 0x04u; ///< bit2 支持 cmdId/ACK
        constexpr uint8_t  kCapsMask     = 0x07u; ///< bit3 保留恒 0
    } // namespace hello

    //===============================================================================================
    // 三、帧长表(自定界的唯一依据)
    //
    //   帧长 = 帧头 1B + 负载(向上取整到字节) + CRC16 2B; 每一条都由上面"按位相加"推出来, 不是在
    //   这里另抄一个数 —— 抄的数字与算式一定会漂, 而漂了就是"两端各按各的长度切帧"。
    //===============================================================================================
    constexpr uint16_t kFrameOverheadBytes = 3u; ///< 帧头 1B + CRC16 2B

    constexpr uint16_t kDlAttBytes   = (uint16_t)(dlAtt::kPayloadBytes + kFrameOverheadBytes);   // 13
    constexpr uint16_t kDlSysBytes   = (uint16_t)(dlSys::kPayloadBytes + kFrameOverheadBytes);   // 15
    constexpr uint16_t kDlGpsBytes   = (uint16_t)(dlGps::kPayloadBytes + kFrameOverheadBytes);   // 21
    constexpr uint16_t kDlStateBytes = (uint16_t)(dlState::kPayloadBytes + kFrameOverheadBytes); // 10
    constexpr uint16_t kDlAckBytes   = (uint16_t)(dlAck::kPayloadBytes + kFrameOverheadBytes);   // 5
    constexpr uint16_t kUpRoundBytes = (uint16_t)(upRound::kPayloadBytes + kFrameOverheadBytes); // 11
    constexpr uint16_t kHelloBytes   = (uint16_t)(hello::kPayloadBytes + kFrameOverheadBytes);   // 5
    constexpr uint16_t kUpCtrlBytes  = (uint16_t)(upCtrl::kPayloadBytes + kFrameOverheadBytes);  // 9
    constexpr uint16_t kUpCmdBytes   = (uint16_t)(upCmd::kPayloadBytes + kFrameOverheadBytes);   // 6
    constexpr uint16_t kUpVelBytes   = (uint16_t)(upVel::kPayloadBytes + kFrameOverheadBytes);   // 6
    constexpr uint16_t kUpPosBytes   = (uint16_t)(upPos::kPayloadBytes + kFrameOverheadBytes);   // 7
    constexpr uint16_t kUpSetpointBytes = (uint16_t)(upSetpoint::kPayloadBytes + kFrameOverheadBytes); // 6
    constexpr uint16_t kUpMotorBytes    = (uint16_t)(upMotor::kPayloadBytes + kFrameOverheadBytes);    // 9

    /** 位宽守卫: 每个字段的宽度都必须落在 putBits/getBits 的 1..32 内。
     *  为什么单独写一条: 宽度写错(如 0 或 40)在运行期只表现为"越界计数 + 该字段恒 0", 上板才看得见;
     *  这里是**编译期**报错, 而且报在"表"这一侧, 一眼能看出是哪个字段。 */
    static_assert(bits::widthOk(dlAtt::kQ1Bits) && bits::widthOk(dlAtt::kQ2Bits) && bits::widthOk(dlAtt::kQ3Bits) &&
                      bits::widthOk(dlAtt::kVxBits) && bits::widthOk(dlAtt::kVyBits) && bits::widthOk(dlAtt::kVzBits) &&
                      bits::widthOk(dlAtt::kStatusBits) && bits::widthOk(dlAtt::kErrorBits) &&
                      bits::widthOk(dlAtt::kArmingBits) && bits::widthOk(dlAtt::kModeBits) &&
                      bits::widthOk(dlSys::kXBits) && bits::widthOk(dlSys::kYBits) && bits::widthOk(dlSys::kZBits) &&
                      bits::widthOk(dlSys::kCurrentBits) && bits::widthOk(dlSys::kVoltageBits) &&
                      bits::widthOk(dlSys::kMotorBits) && bits::widthOk(dlGps::kLatBits) &&
                      bits::widthOk(dlGps::kLonBits) && bits::widthOk(dlGps::kAltBits) &&
                      bits::widthOk(dlGps::kGspeedBits) && bits::widthOk(dlGps::kCourseBits) &&
                      bits::widthOk(dlGps::kSatsBits) && bits::widthOk(dlGps::kFixBits) &&
                      bits::widthOk(dlGps::kHdopBits) && bits::widthOk(dlState::kLifecycleBits) &&
                      bits::widthOk(dlState::kArmingBits) && bits::widthOk(dlState::kModeBits) &&
                      bits::widthOk(dlState::kTaskBits) && bits::widthOk(dlState::kFailsafeLvlBits) &&
                      bits::widthOk(dlState::kFailsafeRsnBits) && bits::widthOk(dlState::kHealthBits) &&
                      bits::widthOk(dlState::kUlLossBits) && bits::widthOk(dlState::kDlLossBits) &&
                      bits::widthOk(dlState::kRssiBits) && bits::widthOk(dlState::kRxBitmapBits) &&
                      bits::widthOk(dlAck::kCmdIdBits) &&
                      bits::widthOk(dlAck::kResultBits) && bits::widthOk(upCtrl::kRollBits) &&
                      bits::widthOk(upCtrl::kPitchBits) && bits::widthOk(upCtrl::kYawBits) &&
                      bits::widthOk(upCtrl::kThrBits) && bits::widthOk(upCtrl::kFlagsBits) &&
                      bits::widthOk(upCmd::kOpBits) && bits::widthOk(upCmd::kArgBits) &&
                      bits::widthOk(upVel::kVxBits) && bits::widthOk(upVel::kVyBits) &&
                      bits::widthOk(upVel::kVzBits) && bits::widthOk(upSetpoint::kAltBits) &&
                      bits::widthOk(upSetpoint::kFlagsBits) && bits::widthOk(upMotor::kMotorBits) &&
                      bits::widthOk(upPos::kXBits) && bits::widthOk(upPos::kYBits) &&
                      bits::widthOk(upCmd::kCmdIdBits) && bits::widthOk(upRound::kRollBits) &&
                      bits::widthOk(upRound::kPitchBits) && bits::widthOk(upRound::kYawBits) &&
                      bits::widthOk(upRound::kThrBits) && bits::widthOk(upRound::kFlagsBits) &&
                      bits::widthOk(upRound::kSessionBits) && bits::widthOk(upRound::kFlags8Bits) &&
                      bits::widthOk(hello::kVerBits) && bits::widthOk(hello::kCapsBits) &&
                      bits::widthOk(hello::kNonceBits),
                  "帧表里有字段位宽不在 1..32: putBits/getBits 只支持这个范围");

    // ---- 表 ↔ 结构体: 逐帧断言"按位相加 == §3.3 的帧长" ----
    // 除了 DL_STATE(49b, 见文件头更正 1), 其余帧的负载位宽必须正好是整字节 —— 不是整字节就说明
    // 是"差不多对齐", 而差不多对齐的位表在两端一定会读出不同的字段。
    static_assert(dlAtt::kPayloadBits == 80u && dlAtt::kPayloadBits % 8u == 0u,
                  "DL_ATT 按位相加 = 3×12 + 3×8 + 8 + 8 + 2 + 2 = 80b(10B)");
    static_assert(dlSys::kPayloadBits == 96u && dlSys::kPayloadBits % 8u == 0u,
                  "DL_SYS 按位相加 = 3×12 + 8 + 8 + 4×11 = 96b(12B)");
    static_assert(dlGps::kPayloadBits == 144u && dlGps::kPayloadBits % 8u == 0u,
                  "DL_GPS 按位相加 = 2×32 + 32 + 16 + 16 + 6 + 2 + 8 = 144b(18B)");
    static_assert(dlState::kUsedBits == 53u && dlState::kSpareBits == 3u,
                  "DL_STATE v2 用量 = 49b + rxBitmap(4b) = 53b ⇒ 7B 负载(56b) + 3 个恒 0 保留位");
    static_assert(dlAck::kPayloadBits == 16u && dlAck::kPayloadBits % 8u == 0u, "DL_ACK = cmdId 8 + result 8 = 16b(2B)");
    static_assert(upRound::kPayloadBits == 64u && upRound::kPayloadBits % 8u == 0u,
                  "UP_ROUND = 3×11 + 11 + 4 + 8 + 8 = 64b(8B)");
    static_assert(hello::kPayloadBits == 16u && hello::kPayloadBits % 8u == 0u, "HELLO = 4 + 4 + 8 = 16b(2B)");
    static_assert(upCtrl::kPayloadBits == 48u && upCtrl::kPayloadBits % 8u == 0u,
                  "UP_CTRL = 3×11 + 11 + 4 = 48b(6B)");
    static_assert(upCmd::kPayloadBits == 24u && upCmd::kPayloadBits % 8u == 0u,
                  "UP_CMD v2 = cmdId 8 + op 8 + arg 8 = 24b(3B)");
    static_assert(upVel::kPayloadBits == 24u && upVel::kPayloadBits % 8u == 0u,
                  "UP_VEL = 3×8 = 24b(3B) —— 2026-10-05 由 2 轴扩到 3 轴");
    static_assert(upPos::kPayloadBits == 32u && upPos::kPayloadBits % 8u == 0u, "UP_POS = 2×16 = 32b(4B)");
    static_assert(upSetpoint::kPayloadBits == 24u && upSetpoint::kPayloadBits % 8u == 0u,
                  "UP_SETPOINT = 16 + 8 = 24b(3B)");
    static_assert(upMotor::kUsedBits == 44u && upMotor::kSpareBits == 4u,
                  "UP_MOTOR 用量 = 4 × 11 = 44b ⇒ 6B 负载(48b) + 4 个恒 0 空位");

    // ---- 表 ↔ v2 协议规格 §1.2 标称长度 ----
    static_assert(kDlAttBytes == 13u, "v2: DL_ATT = 12B + CRC16 1B = 13B");
    static_assert(kDlSysBytes == 15u, "v2: DL_SYS = 14B + CRC16 1B = 15B");
    static_assert(kDlGpsBytes == 21u, "v2: DL_GPS = 20B + CRC16 1B = 21B(全表最长)");
    static_assert(kDlStateBytes == 10u, "v2: DL_STATE = 9B + CRC16 1B = 10B");
    static_assert(kDlAckBytes == 5u, "v2: DL_ACK = 4B + CRC16 1B = 5B");
    static_assert(kUpRoundBytes == 11u, "v2: UP_ROUND = 10B + CRC16 1B = 11B");
    static_assert(kHelloBytes == 5u, "v2: HELLO = 4B + CRC16 1B = 5B");
    static_assert(kUpCtrlBytes == 9u, "v2: UP_CTRL = 8B + CRC16 1B = 9B");
    static_assert(kUpCmdBytes == 6u, "v2: UP_CMD = 5B + CRC16 1B = 6B");
    static_assert(kUpVelBytes == 6u, "v2: UP_VEL = 5B + CRC16 1B = 6B");
    static_assert(kUpPosBytes == 7u, "v2: UP_POS = 6B + CRC16 1B = 7B");
    static_assert(kUpSetpointBytes == 6u, "v2: UP_SETPOINT = 5B + CRC16 1B = 6B");
    static_assert(kUpMotorBytes == 9u, "v2: UP_MOTOR = 8B + CRC16 1B = 9B");

    /** 帧长查找表, 下标 = TYPE(高 4 位)。0 = 非法类型 ≈ "这里不是帧头" ⇒ tryDecode 前进 1 字节 */
    constexpr uint16_t kFrameBytesTable[16] = {
        0u,             // 0x0 保留(不用): 全 0 的噪声字节必须被判非法
        kDlAttBytes,    // 0x1 DL_ATT
        kDlSysBytes,    // 0x2 DL_SYS
        kDlGpsBytes,    // 0x3 DL_GPS
        kDlStateBytes,  // 0x4 DL_STATE
        kDlAckBytes,    // 0x5 DL_ACK
        kUpRoundBytes,  // 0x6 UP_ROUND(v2 新增)
        kHelloBytes,    // 0x7 HELLO(v2 新增, 双向同型)
        kUpCtrlBytes,   // 0x8 UP_CTRL
        kUpCmdBytes,    // 0x9 UP_CMD
        kUpVelBytes,    // 0xA UP_VEL
        kUpPosBytes,    // 0xB UP_POS
        kUpSetpointBytes, // 0xC UP_SETPOINT
        kUpMotorBytes,    // 0xD UP_MOTOR
        0u,             // 0xE 非法(v2 保留)
        0u,             // 0xF 非法(v2 保留)
    };

    // 表里的数必须与帧长常量逐项相等(常量改动而表忘改, 是"自定界"最致命的一类错)
    static_assert(kFrameBytesTable[(uint8_t)FrameType::DlAtt] == kDlAttBytes, "帧长表: DL_ATT 与 kDlAttBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::DlSys] == kDlSysBytes, "帧长表: DL_SYS 与 kDlSysBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::DlGps] == kDlGpsBytes, "帧长表: DL_GPS 与 kDlGpsBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::DlState] == kDlStateBytes, "帧长表: DL_STATE 与 kDlStateBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::DlAck] == kDlAckBytes, "帧长表: DL_ACK 与 kDlAckBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::UpRound] == kUpRoundBytes, "帧长表: UP_ROUND 与 kUpRoundBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::Hello] == kHelloBytes, "帧长表: HELLO 与 kHelloBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::UpCtrl] == kUpCtrlBytes, "帧长表: UP_CTRL 与 kUpCtrlBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::UpCmd] == kUpCmdBytes, "帧长表: UP_CMD 与 kUpCmdBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::UpVel] == kUpVelBytes, "帧长表: UP_VEL 与 kUpVelBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::UpPos] == kUpPosBytes, "帧长表: UP_POS 与 kUpPosBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::UpSetpoint] == kUpSetpointBytes,
                  "帧长表: UP_SETPOINT 与 kUpSetpointBytes 不一致");
    static_assert(kFrameBytesTable[(uint8_t)FrameType::UpMotor] == kUpMotorBytes,
                  "帧长表: UP_MOTOR 与 kUpMotorBytes 不一致");

    /** 最长帧(21B = DL_GPS)。收缓冲区按它 + 余量开就够 —— 而"够不够"是自定界能不能工作的前提。
     *  v2 上限 21B: E220 分包 32B, 21B 留给"一帧 = 一个射频包"且仍有余量。 */
    constexpr uint16_t kMaxFrameBytes = kDlGpsBytes;
    static_assert(kMaxFrameBytes >= kDlAttBytes && kMaxFrameBytes >= kDlSysBytes && kMaxFrameBytes >= kDlStateBytes &&
                      kMaxFrameBytes >= kDlAckBytes && kMaxFrameBytes >= kUpRoundBytes && kMaxFrameBytes >= kHelloBytes &&
                      kMaxFrameBytes >= kUpCtrlBytes && kMaxFrameBytes >= kUpCmdBytes &&
                      kMaxFrameBytes >= kUpVelBytes && kMaxFrameBytes >= kUpPosBytes &&
                      kMaxFrameBytes >= kUpSetpointBytes && kMaxFrameBytes >= kUpMotorBytes,
                  "kMaxFrameBytes 必须是帧长表的最大值(新增更长的帧时这里先报)");
    static_assert(kMaxFrameBytes == 21u, "v2 协议规格: 单帧最大 21B(每帧 +1B CRC16 后)");
    static_assert(kMaxFrameBytes <= 32u, "E220 分包 32B: 超过就一帧劈成两个射频包");

    /** 帧长查询: 未知类型返回 0。
     *  constexpr 是为了让上面的 static_assert 能直接用统一入口(表 ↔ 常量 ↔ §3.3 三层对拍)。
     *  为什么类型取值先与 0x0F: 帧头高 4 位天然 ≤15; 万一有人拿一个越界枚举值进来, 掩码后得到的是
     *  一个**定义好的**结果, 而不是数组越界。 */
    constexpr uint16_t frameBytes(FrameType t)
    {
        return kFrameBytesTable[(uint8_t)t & 0x0Fu];
    }

    /** 类型号(0..15)在帧表里是否合法。非法类型是"自定界"的负向判据: 前进 1 字节重找帧头 */
    constexpr bool frameTypeValid(uint8_t typeNibble)
    {
        return (typeNibble & 0xF0u) == 0u && kFrameBytesTable[typeNibble] != 0u;
    }

    /** 类型名(仅用于日志/自检打印)。为什么要它: 帧类型是 4 位数字, 日志里打 "0x4" 没人看得懂,
     *  而这里有 const char* 就不必为它引入 printf 依赖 */
    inline const char *frameTypeName(FrameType t)
    {
        switch (t) {
        case FrameType::DlAtt:   return "DL_ATT";
        case FrameType::DlSys:   return "DL_SYS";
        case FrameType::DlGps:   return "DL_GPS";
        case FrameType::DlState: return "DL_STATE";
        case FrameType::DlAck:   return "DL_ACK";
        case FrameType::UpRound: return "UP_ROUND";
        case FrameType::Hello:   return "HELLO";
        case FrameType::UpCtrl:  return "UP_CTRL";
        case FrameType::UpCmd:   return "UP_CMD";
        case FrameType::UpVel:   return "UP_VEL";
        case FrameType::UpPos:   return "UP_POS";
        case FrameType::UpSetpoint: return "UP_SETPOINT";
        case FrameType::UpMotor:    return "UP_MOTOR";
        default:                 return "INVALID";
        }
    }

    //===============================================================================================
    // 四、CRC-16/CCITT-FALSE
    //===============================================================================================
    /** CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, 不反射(refin/refout 都 false),
     *  xorout 0x0000(协议规格 §1.1)。校验值: "123456789" → 0x29B1(主机单测有这条已知向量)。
     *
     *  为什么不用 256/65536 项查表: 查表版自己也是一份"协议参数"(表抄错一位没有任何编译期手段
     *  发现), 而链路线程是 8.33Hz 下行 + 33.3Hz 上行、每帧 ≤21B, 位运算版每帧才 ~170 次循环。
     *  确定性 > 这点周期。
     *  为什么不反射: 协议规格定死了"不反射", 反射版(CRC-16/IBM 一类)对同一串数据给的是另一个值,
     *  一旦两端各按各的理解实现, 表现是"所有帧都 CRC 错", 而且看起来像硬件问题。
     *  为什么线上低字节在前: 与负载的 LSB-first 位打包同一约定; 若写成"高字节在前",
     *  CRC 本身仍然正确, 但两端必须逐字节一致 —— 这是最容易在两端各写一遍时写反的地方,
     *  所以黄金向量里每种帧都钉了 CRC 的两个字节。 */
    inline uint16_t crc16CcittFalse(const uint8_t *data, uint16_t len)
    {
        uint16_t crc = 0xFFFFu;
        for (uint16_t i = 0u; i < len; ++i) {
            crc ^= (uint16_t)((uint16_t)data[i] << 8);
            for (uint8_t b = 0u; b < 8u; ++b) {
                crc = (crc & 0x8000u) ? (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
            }
        }
        return crc;
    }

    /** 把 CRC16 写到整帧最后两个字节(低字节在前)。写之前整帧已由 clearBytes() 清过, 这里只覆盖。 */
    inline void writeCrc16Le(uint8_t *frame, uint16_t frameLen)
    {
        const uint16_t crc = crc16CcittFalse(frame, (uint16_t)(frameLen - 2u));
        frame[frameLen - 2u] = (uint8_t)(crc & 0x00FFu);
        frame[frameLen - 1u] = (uint8_t)((crc >> 8) & 0x00FFu);
    }

    /** 整帧 CRC16 校验(读最后两字节, 低字节在前)。 */
    inline bool crc16LeValid(const uint8_t *frame, uint16_t frameLen)
    {
        if (frame == 0 || frameLen < 2u) {
            return false;
        }
        const uint16_t got = (uint16_t)((uint16_t)frame[frameLen - 2u] |
                                       (uint16_t)((uint16_t)frame[frameLen - 1u] << 8));
        return crc16CcittFalse(frame, (uint16_t)(frameLen - 2u)) == got;
    }

    //===============================================================================================
    // 五、定点换算(与文件头"定点约定总表"一一对应)
    //
    //   为什么全做成 inline 小函数而不是宏/散在调用处: 单位换算错了在飞行中的表现是"慢了 10 倍"
    //   或"大了 10 倍", 而这类错在代码审查里**看不出来** —— 只有集中一处 + 主机单测覆盖边界值,
    //   才可能被抓住。结构体存什么单位、线上是什么单位, 都在这几个函数里闭合。
    //===============================================================================================
    /** 四元数分量(归一化 ±1) → int12 */
    inline int32_t toInt12Norm(float x) { return bits::quantizeSat(x, 2047.0f, -2047, 2047); }
    /** int12 → 归一化 ±1(夹到 [-1,1]: 线上 ±2047 已是满量程) */
    inline float fromInt12Norm(int32_t raw) { return bits::normalizedSigned(raw, 2047.0f); }

    /** m/s → int8 dm/s(DL_ATT 速度) */
    inline int32_t toInt8DmPerSec(float mps) { return bits::quantizeSat(mps, 10.0f, -128, 127); }
    inline float   fromInt8DmPerSec(int32_t raw) { return bits::dequantize(raw, 0.1f); }

    /** m/s → int8 @5cm/s(UP_VEL 速度注入)。注意与 DL_ATT 的 10cm/s 是**两个标度** */
    inline int32_t toInt8FiveCmPerSec(float mps) { return bits::quantizeSat(mps, 20.0f, -128, 127); }
    inline float   fromInt8FiveCmPerSec(int32_t raw) { return bits::dequantize(raw, 0.05f); }

    /** m → int12 dm(DL_SYS 位置) */
    inline int32_t toInt12Dm(float m) { return bits::quantizeSat(m, 10.0f, -2048, 2047); }
    inline float   fromInt12Dm(int32_t raw) { return bits::dequantize(raw, 0.1f); }

    /** m → int16 dm(UP_POS 位置注入) */
    inline int32_t toInt16Dm(float m) { return bits::quantizeSat(m, 10.0f, -32768, 32767); }
    inline float   fromInt16Dm(int32_t raw) { return bits::dequantize(raw, 0.1f); }

    /** m → int16 cm(UP_SETPOINT 期望高度)。1cm 分辨率、±327.67m, 比 int12 dm 宽得多 ——
     *  定高的高度上限是 10m 量级, 用 dm 会白扔精度, 用 cm 刚好把 int16 用满。 */
    inline int32_t toInt16Centimeters(float m) { return bits::quantizeSat(m, 100.0f, -32767, 32767); }
    inline float   fromInt16Centimeters(int32_t raw) { return bits::dequantize(raw, 0.01f); }

    /** 0.1 单位量(电压 V / 电流 A / hdop) → u8 */
    inline int32_t toU8Tenths(float v) { return bits::quantizeSat(v, 10.0f, 0, 255); }
    inline float   fromU8Tenths(int32_t raw) { return bits::dequantize(raw, 0.1f); }

    /** 0..1 归一化 → u11 油门 counts(§3.6: 0..2047) */
    inline int32_t toU11Throttle(float norm) { return bits::quantizeSat(norm, 2047.0f, 0, 2047); }
    /** -1..+1 归一化 → int11 摇杆 counts(§3.6: ±1023)。11 位补码, 见文件头"为什么摇杆用补码" */
    inline int32_t toInt11Stick(float norm) { return bits::quantizeSat(norm, 1023.0f, -1023, 1023); }

    /** 摇杆 counts(遥控器 ADC 直出的整数) → 合法 counts: 饱和到 ±1023 */
    inline int32_t clampStickCounts(int32_t counts)
    {
        if (counts > upCtrl::kStickMax) {
            return upCtrl::kStickMax;
        }
        if (counts < upCtrl::kStickMin) {
            return upCtrl::kStickMin;
        }
        return counts;
    }
    /** 油门 counts → 合法 counts: 饱和到 0..2047 */
    inline int32_t clampThrottleCounts(int32_t counts)
    {
        if (counts > upCtrl::kThrMax) {
            return upCtrl::kThrMax;
        }
        if (counts < upCtrl::kThrMin) {
            return upCtrl::kThrMin;
        }
        return counts;
    }
    /** 线上 11 位补码 → 语义 counts: 把不可用的 -1024 码点夹回 -1023(见文件头) */
    inline int32_t decodeStick11(int32_t raw)
    {
        return (raw < upCtrl::kStickMin) ? upCtrl::kStickMin : raw;
    }

    /** DShot 计数域(§3.3 "DShot 计数" + 任务书 0..1999)。
     *  为什么上界是 1999 而不是 u11 的 2047: DShot 把 2000..2047 留作特殊命令(beacon/遥测请求),
     *  当成油门发出去会变成"电机唱歌"或触发遥测; 0..47 是停转区, 而 0 是 DISARMED 时明确要发的值,
     *  所以下界留 0 —— 驱动层怎么解释 0..47 是它的事, 链路层只管把值域搬对。 */
    constexpr uint16_t kMotorMin = 0u;
    constexpr uint16_t kMotorMax = 1999u;
    inline uint16_t clampDshot(int32_t v)
    {
        if (v > (int32_t)kMotorMax) {
            return kMotorMax;
        }
        if (v < (int32_t)kMotorMin) {
            return kMotorMin;
        }
        return (uint16_t)v;
    }

    /** 丢包率上限: >100% 不是"更糟", 是数错了 —— 夹住而不是回绕 */
    constexpr uint8_t kLossMaxPercent = 100u;

    //===============================================================================================
    // 六、POD 负载结构体
    //
    //   字段表示法 = **数据源的天然表示**(理由见文件头): 飞控里本来就是 float 物理量的用 float,
    //   本来就是定点整数/位域的用整型原样过线。
    //   这些结构体是 POD(无构造函数/无默认成员初始化), 所以 `DlAttPayload p = {};` 能整体清零,
    //   也能整块 memcpy —— 加了 NSDMI 就不再是聚合体, C++11 下连 `= {}` 都用不了。
    // ===============================================================================================
    /** DL_ATT 负载(§3.3) */
    struct DlAttPayload
    {
        float   q1, q2, q3; ///< 四元数虚部, 归一化 ±1(线上 int12 = round(x*2047))
        float   vx, vy, vz; ///< 机体速度 m/s(线上 int8 = round(mps*10), 单位 dm/s)
        uint8_t status;     ///< u8 原样(语义在事件/状态侧, 本层不解释)
        uint8_t error;      ///< u8 原样
        uint8_t arming;     ///< u2 原样, 0..3(枚举真相在状态机头文件)
        uint8_t mode;       ///< u2 原样, 0..3(见文件头更正 2)
    };

    /** DL_SYS 负载(§3.3) */
    struct DlSysPayload
    {
        float    x, y, z;  ///< 位置 m(线上 int12 = round(m*10), 单位 dm)
        float    currentA; ///< 电流 A(线上 u8 = round(A*10), 即 0.1A 分辨率)
        float    voltageV; ///< 电压 V(线上 u8 = round(V*10), 即 0.1V 分辨率)
        uint16_t motor[dlSys::kMotorCount]; ///< DShot 计数, 0..1999 之外饱和
    };

    /** DL_GPS 负载(§3.3)。
     *  为什么全是整数(见文件头"为什么 GPS 用整数"): 源就是定点, 而 1e-7 度超出 float 的有效精度。 */
    struct DlGpsPayload
    {
        int32_t  latE7;      ///< 纬度 度×1e7(线上 int32, 小端)
        int32_t  lonE7;      ///< 经度 度×1e7
        int32_t  altMslMm;   ///< 海拔(MSL) mm
        int16_t  gspeedCms;  ///< 地速 cm/s
        uint16_t courseCdeg; ///< 航向 0.01°(0..35999)
        uint8_t  sats;       ///< 星数 u6(0..63)
        uint8_t  fix;        ///< 定位类型 u2(0..3)
        float    hdop;       ///< 水平精度因子(线上 u8 = round(x*10))
    };

    /** DL_STATE 负载(v2: v1 布局 + bit49..52 上行接收位图)。
     *  rxBitmap: bit0..3 = 本轮 pos0..3 是否已收到(RoundSlave 填); bit4..7 保留恒 0。 */
    struct DlStatePayload
    {
        uint8_t lifecycle;   ///< u3 原样(§7.1 A)
        uint8_t arming;      ///< u2 原样(§7.1 B)
        uint8_t mode;        ///< u3 原样(§7.1 C 是 2 bit, 见文件头更正 2)
        uint8_t task;        ///< u2 原样(§7.1 D)
        uint8_t failsafeLvl; ///< u3 原样(§7.1 F)
        uint8_t failsafeRsn; ///< u4 原样(§7.1 F)
        uint8_t health;      ///< u8 位域原样(§7.1 E)
        uint8_t ulLoss;      ///< 上行丢包 %, 上限 100
        uint8_t dlLoss;      ///< 下行丢包 %, 上限 100
        int8_t  rssiDbm;     ///< dBm, **有符号**(§3.6 显示行 "R:-60dBm")
        uint8_t rxBitmap;    ///< v2: bit0..3 = 本轮收到 pos0..3(见 dlState::kRxPos*Bit)
    };

    /** DL_ACK 负载(v2 · 协议规格 §3.4): cmdId + result。
     *  v1 靠"帧头 SEQ 回显被应答命令的 SEQ"配对命令; v2 改为显式 cmdId, 帧头 SEQ 留给丢包统计。 */
    struct DlAckPayload
    {
        uint8_t cmdId;  ///< 被应答的 UP_CMD.cmdId 原样回显
        uint8_t result; ///< 结果码(0=OK … 11=AUTH_FAIL), 本层不解释
    };

    /** HELLO 负载(v2 · 协议规格 §3.2): ver4 + caps4 + nonce8。双向同型, 应答回显 nonce。 */
    struct HelloPayload
    {
        uint8_t ver;   ///< 0..15(v2 = 2; 0 保留)
        uint8_t caps;  ///< 能力位, 见 hello::kCaps*
        uint8_t nonce; ///< 请求方随机; 应答方原样回显
    };

    /** UP_ROUND 负载(v2 · 协议规格 §3.1): 杆量 + session + flags8, 每轮 U0 必发。 */
    struct UpRoundPayload
    {
        int16_t  roll, pitch, yaw; ///< int11 补码, 语义域 [-1023, 1023](与 UP_CTRL 同量纲)
        uint16_t thr;              ///< u11, 0..2047
        uint8_t  flags;            ///< 低 4 位, 同 UP_CTRL flags(bit0=ESTOP)
        uint8_t  session;          ///< 遥控器本次启动随机 1..255(0 保留)
        uint8_t  flags8;           ///< bit0=ESTOP(冗余) bit1=yaw 有效 bit2=请求 DL_GPS
    };

    /** UP_CTRL 负载(§3.3/§3.6)。
     *  为什么传 counts 而不是归一化浮点(§3.4 修正 1 只说"上传摇杆量"): 遥控器 ADC 出来的就是
     *  counts, 转成 ±1 再转回来只是多两次乘除和两次舍入 —— 而分辨率恰好是这次改动的目的之一(0.05%)。 */
    struct UpCtrlPayload
    {
        int16_t  roll, pitch, yaw; ///< int11 补码, 语义域 [-1023, 1023](0 = 中立)
        uint16_t thr;              ///< uint11, 0..2047
        uint8_t  flags;            ///< 低 4 位有效(见 upCtrl::kFlag*); 高 4 位不在帧里
    };

    /** UP_CMD 负载(v2 · 协议规格 §3.3): cmdId + op + arg。
     *  cmdId 由遥控器分配(1..255 循环), 重传不变; 飞控按 (session, cmdId) 去重 —— 重复只重发 ACK。 */
    struct UpCmdPayload
    {
        uint8_t cmdId; ///< 1..255 循环, 0=无效
        uint8_t op;    ///< 命令号
        uint8_t arg;   ///< 参数(模式号 / 目标高度 dm / 参数 index …; 用不到时为 0)
    };

    /** UP_VEL 负载(2026-10-05: 三轴)。
     *  坐标系 = 世界系(x 北 / y 东 / z 上)。仍**不能**替代 UP_CTRL: 它没有杆量通道, 也不带油门;
     *  定速模式下 UP_CTRL 作为"降级待命通道"继续每槽发送(见 app_link_thread.hpp)。 */
    struct UpVelPayload
    {
        float vx, vy, vz; ///< 速度指令 m/s(线上 int8 = round(mps*20), 单位 5cm/s)
    };

    /** UP_SETPOINT 负载(2026-10-05 新增): 定高的绝对期望高度 */
    struct UpSetpointPayload
    {
        float   altM;    ///< 期望高度 m, 向上为正(线上 int16 = round(m*100), 单位 cm)
        uint8_t flags;   ///< 见 upSetpoint::kFlag*(bit0 = ALT_VALID)
    };

    /** UP_MOTOR 负载: 测试模式直控, **四路 DShot 计数一次发全** */
    struct UpMotorPayload
    {
        uint16_t throttle[upMotor::kMotorCount]; ///< 逻辑电机 0..3 的 DShot 计数原样 0..1999
    };

    /** UP_POS 负载(§3.3) */
    struct UpPosPayload
    {
        float x, y; ///< 位置指令 m(线上 int16 = round(m*10), 单位 dm)
    };

    //===============================================================================================
    // 七、负载解包(unpack)/ 打包(encode)
    //
    //   unpack 只负责"按布局常量把位搬出来", 不含任何类型/CRC 校验 —— 校验在 decodeX 与 tryDecode
    //   里各做一次。这样 tryDecode 不会为"已经校验过的一帧"重复算一遍 CRC。
    // ===============================================================================================
    inline void unpackDlAtt(const uint8_t *frame, DlAttPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = dlAtt::kPayloadBytes;
        out.q1     = fromInt12Norm(bits::getSignedBits(pl, plLen, dlAtt::kQ1Bit, dlAtt::kQ1Bits));
        out.q2     = fromInt12Norm(bits::getSignedBits(pl, plLen, dlAtt::kQ2Bit, dlAtt::kQ2Bits));
        out.q3     = fromInt12Norm(bits::getSignedBits(pl, plLen, dlAtt::kQ3Bit, dlAtt::kQ3Bits));
        out.vx     = fromInt8DmPerSec(bits::getSignedBits(pl, plLen, dlAtt::kVxBit, dlAtt::kVxBits));
        out.vy     = fromInt8DmPerSec(bits::getSignedBits(pl, plLen, dlAtt::kVyBit, dlAtt::kVyBits));
        out.vz     = fromInt8DmPerSec(bits::getSignedBits(pl, plLen, dlAtt::kVzBit, dlAtt::kVzBits));
        out.status = (uint8_t)bits::getBits(pl, plLen, dlAtt::kStatusBit, dlAtt::kStatusBits);
        out.error  = (uint8_t)bits::getBits(pl, plLen, dlAtt::kErrorBit, dlAtt::kErrorBits);
        out.arming = (uint8_t)bits::getBits(pl, plLen, dlAtt::kArmingBit, dlAtt::kArmingBits);
        out.mode   = (uint8_t)bits::getBits(pl, plLen, dlAtt::kModeBit, dlAtt::kModeBits);
    }

    inline uint16_t encodeDlAtt(const DlAttPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kDlAttBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::DlAtt, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = dlAtt::kPayloadBytes;
        bits::putSignedBits(pl, plLen, dlAtt::kQ1Bit, toInt12Norm(p.q1), dlAtt::kQ1Bits);
        bits::putSignedBits(pl, plLen, dlAtt::kQ2Bit, toInt12Norm(p.q2), dlAtt::kQ2Bits);
        bits::putSignedBits(pl, plLen, dlAtt::kQ3Bit, toInt12Norm(p.q3), dlAtt::kQ3Bits);
        bits::putSignedBits(pl, plLen, dlAtt::kVxBit, toInt8DmPerSec(p.vx), dlAtt::kVxBits);
        bits::putSignedBits(pl, plLen, dlAtt::kVyBit, toInt8DmPerSec(p.vy), dlAtt::kVyBits);
        bits::putSignedBits(pl, plLen, dlAtt::kVzBit, toInt8DmPerSec(p.vz), dlAtt::kVzBits);
        bits::putBits(pl, plLen, dlAtt::kStatusBit, p.status, dlAtt::kStatusBits);
        bits::putBits(pl, plLen, dlAtt::kErrorBit, p.error, dlAtt::kErrorBits);
        // 位域比源窄(u2): 饱和而不是回绕 —— 回绕会把"模式 7"变成"模式 3"
        bits::putBits(pl, plLen, dlAtt::kArmingBit, bits::satUnsigned(p.arming, dlAtt::kArmingBits), dlAtt::kArmingBits);
        bits::putBits(pl, plLen, dlAtt::kModeBit, bits::satUnsigned(p.mode, dlAtt::kModeBits), dlAtt::kModeBits);
        writeCrc16Le(out, n);
        return n;
    }

    inline void unpackDlSys(const uint8_t *frame, DlSysPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = dlSys::kPayloadBytes;
        out.x        = fromInt12Dm(bits::getSignedBits(pl, plLen, dlSys::kXBit, dlSys::kXBits));
        out.y        = fromInt12Dm(bits::getSignedBits(pl, plLen, dlSys::kYBit, dlSys::kYBits));
        out.z        = fromInt12Dm(bits::getSignedBits(pl, plLen, dlSys::kZBit, dlSys::kZBits));
        out.currentA = fromU8Tenths((int32_t)bits::getBits(pl, plLen, dlSys::kCurrentBit, dlSys::kCurrentBits));
        out.voltageV = fromU8Tenths((int32_t)bits::getBits(pl, plLen, dlSys::kVoltageBit, dlSys::kVoltageBits));
        out.motor[0] = (uint16_t)bits::getBits(pl, plLen, dlSys::kMotor0Bit, dlSys::kMotorBits);
        out.motor[1] = (uint16_t)bits::getBits(pl, plLen, dlSys::kMotor1Bit, dlSys::kMotorBits);
        out.motor[2] = (uint16_t)bits::getBits(pl, plLen, dlSys::kMotor2Bit, dlSys::kMotorBits);
        out.motor[3] = (uint16_t)bits::getBits(pl, plLen, dlSys::kMotor3Bit, dlSys::kMotorBits);
    }

    inline uint16_t encodeDlSys(const DlSysPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kDlSysBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::DlSys, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = dlSys::kPayloadBytes;
        bits::putSignedBits(pl, plLen, dlSys::kXBit, toInt12Dm(p.x), dlSys::kXBits);
        bits::putSignedBits(pl, plLen, dlSys::kYBit, toInt12Dm(p.y), dlSys::kYBits);
        bits::putSignedBits(pl, plLen, dlSys::kZBit, toInt12Dm(p.z), dlSys::kZBits);
        bits::putBits(pl, plLen, dlSys::kCurrentBit, (uint32_t)toU8Tenths(p.currentA), dlSys::kCurrentBits);
        bits::putBits(pl, plLen, dlSys::kVoltageBit, (uint32_t)toU8Tenths(p.voltageV), dlSys::kVoltageBits);
        bits::putBits(pl, plLen, dlSys::kMotor0Bit, clampDshot((int32_t)p.motor[0]), dlSys::kMotorBits);
        bits::putBits(pl, plLen, dlSys::kMotor1Bit, clampDshot((int32_t)p.motor[1]), dlSys::kMotorBits);
        bits::putBits(pl, plLen, dlSys::kMotor2Bit, clampDshot((int32_t)p.motor[2]), dlSys::kMotorBits);
        bits::putBits(pl, plLen, dlSys::kMotor3Bit, clampDshot((int32_t)p.motor[3]), dlSys::kMotorBits);
        writeCrc16Le(out, n);
        return n;
    }

    inline void unpackDlGps(const uint8_t *frame, DlGpsPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = dlGps::kPayloadBytes;
        out.latE7      = bits::getSignedBits(pl, plLen, dlGps::kLatBit, dlGps::kLatBits);
        out.lonE7      = bits::getSignedBits(pl, plLen, dlGps::kLonBit, dlGps::kLonBits);
        out.altMslMm   = bits::getSignedBits(pl, plLen, dlGps::kAltBit, dlGps::kAltBits);
        out.gspeedCms  = (int16_t)bits::getSignedBits(pl, plLen, dlGps::kGspeedBit, dlGps::kGspeedBits);
        out.courseCdeg = (uint16_t)bits::getBits(pl, plLen, dlGps::kCourseBit, dlGps::kCourseBits);
        out.sats       = (uint8_t)bits::getBits(pl, plLen, dlGps::kSatsBit, dlGps::kSatsBits);
        out.fix        = (uint8_t)bits::getBits(pl, plLen, dlGps::kFixBit, dlGps::kFixBits);
        out.hdop       = fromU8Tenths((int32_t)bits::getBits(pl, plLen, dlGps::kHdopBit, dlGps::kHdopBits));
    }

    inline uint16_t encodeDlGps(const DlGpsPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kDlGpsBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::DlGps, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = dlGps::kPayloadBytes;
        bits::putSignedBits(pl, plLen, dlGps::kLatBit, p.latE7, dlGps::kLatBits);
        bits::putSignedBits(pl, plLen, dlGps::kLonBit, p.lonE7, dlGps::kLonBits);
        bits::putSignedBits(pl, plLen, dlGps::kAltBit, p.altMslMm, dlGps::kAltBits);
        bits::putSignedBits(pl, plLen, dlGps::kGspeedBit, (int32_t)p.gspeedCms, dlGps::kGspeedBits);
        // 航向 360.00° 与 0.00° 是同一个方向: 先归一化再饱和, 否则 35999 会被当成"超范围"夹掉
        uint32_t course = (uint32_t)p.courseCdeg % 36000u;
        if (course > (uint32_t)dlGps::kCourseMaxCdeg) {
            course = (uint32_t)dlGps::kCourseMaxCdeg;
        }
        bits::putBits(pl, plLen, dlGps::kCourseBit, course, dlGps::kCourseBits);
        bits::putBits(pl, plLen, dlGps::kSatsBit, bits::satUnsigned(p.sats, dlGps::kSatsBits), dlGps::kSatsBits);
        bits::putBits(pl, plLen, dlGps::kFixBit, bits::satUnsigned(p.fix, dlGps::kFixBits), dlGps::kFixBits);
        bits::putBits(pl, plLen, dlGps::kHdopBit, (uint32_t)toU8Tenths(p.hdop), dlGps::kHdopBits);
        writeCrc16Le(out, n);
        return n;
    }

    inline void unpackDlState(const uint8_t *frame, DlStatePayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = dlState::kPayloadBytes;
        out.lifecycle   = (uint8_t)bits::getBits(pl, plLen, dlState::kLifecycleBit, dlState::kLifecycleBits);
        out.arming      = (uint8_t)bits::getBits(pl, plLen, dlState::kArmingBit, dlState::kArmingBits);
        out.mode        = (uint8_t)bits::getBits(pl, plLen, dlState::kModeBit, dlState::kModeBits);
        out.task        = (uint8_t)bits::getBits(pl, plLen, dlState::kTaskBit, dlState::kTaskBits);
        out.failsafeLvl = (uint8_t)bits::getBits(pl, plLen, dlState::kFailsafeLvlBit, dlState::kFailsafeLvlBits);
        out.failsafeRsn = (uint8_t)bits::getBits(pl, plLen, dlState::kFailsafeRsnBit, dlState::kFailsafeRsnBits);
        out.health      = (uint8_t)bits::getBits(pl, plLen, dlState::kHealthBit, dlState::kHealthBits);
        uint8_t ul       = (uint8_t)bits::getBits(pl, plLen, dlState::kUlLossBit, dlState::kUlLossBits);
        uint8_t dl       = (uint8_t)bits::getBits(pl, plLen, dlState::kDlLossBit, dlState::kDlLossBits);
        // 解码侧也夹一次 100: 对端(可能是别的实现/旧版本)发来 200 时, 上层拿到的是"100% 丢包"
        // 而不是一个会溢出显示宽度的数
        out.ulLoss = (ul > kLossMaxPercent) ? kLossMaxPercent : ul;
        out.dlLoss = (dl > kLossMaxPercent) ? kLossMaxPercent : dl;
        out.rssiDbm = (int8_t)bits::getSignedBits(pl, plLen, dlState::kRssiBit, dlState::kRssiBits);
        out.rxBitmap = (uint8_t)(bits::getBits(pl, plLen, dlState::kRxBitmapBit, dlState::kRxBitmapBits) &
                                 dlState::kRxBitmapMask);
    }

    /** v2: 从**原始帧**取 DL_STATE 的上行接收位图(bit0..3 = pos0..3 已收到)。
     *  为什么还要 raw 版: 准入层/日志要在"不重编整帧"的前提下读这一项(与 v1 的 spare 检查同一位置)。 */
    inline uint8_t dlStateRxBitmapOf(const uint8_t *frame)
    {
        if (frame == 0) {
            return 0u;
        }
        return (uint8_t)(bits::getBits(frame + 1, dlState::kPayloadBytes, dlState::kRxBitmapBit,
                                      dlState::kRxBitmapBits) &
                         dlState::kRxBitmapMask);
    }

    /** v2: 直接改写已编码 DL_STATE 帧里的接收位图, 并**重算 CRC**。
     *  供 RoundSlave 在"帧已组好、发送前最后一刻"填入本轮位图(不必把整帧解包再打包)。 */
    inline void dlStateSetRxBitmap(uint8_t *frame, uint8_t mask)
    {
        if (frame == 0) {
            return;
        }
        bits::putBits(frame + 1, dlState::kPayloadBytes, dlState::kRxBitmapBit,
                      (uint32_t)(mask & dlState::kRxBitmapMask), dlState::kRxBitmapBits);
        writeCrc16Le(frame, kDlStateBytes);
    }

    inline uint16_t encodeDlState(const DlStatePayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kDlStateBytes;
        // 整帧先清零: 53 bit 用量之外的 3 个保留位必须恒 0(见 dlState::kRxReservedBit)。不清的话
        // 会把缓冲区里的旧字节带进 CRC, 于是"同一份负载两次编码得到不同字节", 重传/自检失去可比性。
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::DlState, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = dlState::kPayloadBytes;
        bits::putBits(pl, plLen, dlState::kLifecycleBit,
                      bits::satUnsigned(p.lifecycle, dlState::kLifecycleBits), dlState::kLifecycleBits);
        bits::putBits(pl, plLen, dlState::kArmingBit,
                      bits::satUnsigned(p.arming, dlState::kArmingBits), dlState::kArmingBits);
        bits::putBits(pl, plLen, dlState::kModeBit, bits::satUnsigned(p.mode, dlState::kModeBits), dlState::kModeBits);
        bits::putBits(pl, plLen, dlState::kTaskBit, bits::satUnsigned(p.task, dlState::kTaskBits), dlState::kTaskBits);
        bits::putBits(pl, plLen, dlState::kFailsafeLvlBit,
                      bits::satUnsigned(p.failsafeLvl, dlState::kFailsafeLvlBits), dlState::kFailsafeLvlBits);
        bits::putBits(pl, plLen, dlState::kFailsafeRsnBit,
                      bits::satUnsigned(p.failsafeRsn, dlState::kFailsafeRsnBits), dlState::kFailsafeRsnBits);
        bits::putBits(pl, plLen, dlState::kHealthBit, p.health, dlState::kHealthBits);
        const uint8_t ul = (p.ulLoss > kLossMaxPercent) ? kLossMaxPercent : p.ulLoss;
        const uint8_t dl = (p.dlLoss > kLossMaxPercent) ? kLossMaxPercent : p.dlLoss;
        bits::putBits(pl, plLen, dlState::kUlLossBit, ul, dlState::kUlLossBits);
        bits::putBits(pl, plLen, dlState::kDlLossBit, dl, dlState::kDlLossBits);
        bits::putSignedBits(pl, plLen, dlState::kRssiBit, (int32_t)p.rssiDbm, dlState::kRssiBits);
        bits::putBits(pl, plLen, dlState::kRxBitmapBit,
                      (uint32_t)(p.rxBitmap & dlState::kRxBitmapMask), dlState::kRxBitmapBits);
        writeCrc16Le(out, n);
        return n;
    }

    inline void unpackDlAck(const uint8_t *frame, DlAckPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = dlAck::kPayloadBytes;
        out.cmdId  = (uint8_t)bits::getBits(pl, plLen, dlAck::kCmdIdBit, dlAck::kCmdIdBits);
        out.result = (uint8_t)bits::getBits(pl, plLen, dlAck::kResultBit, dlAck::kResultBits);
    }

    /** DL_ACK 编码: seq 是**下行自己的** SEQ(仅用于丢包统计); 配对靠负载里的 cmdId(v2 §3.4) */
    inline uint16_t encodeDlAck(const DlAckPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kDlAckBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::DlAck, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = dlAck::kPayloadBytes;
        bits::putBits(pl, plLen, dlAck::kCmdIdBit, p.cmdId, dlAck::kCmdIdBits);
        bits::putBits(pl, plLen, dlAck::kResultBit, p.result, dlAck::kResultBits);
        writeCrc16Le(out, n);
        return n;
    }

    inline void unpackUpCtrl(const uint8_t *frame, UpCtrlPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = upCtrl::kPayloadBytes;
        // 摇杆走解码夹取: 线上 -1024 是编码端不会发的码点, 收到就当成满舵的左边界
        out.roll  = (int16_t)decodeStick11(bits::getSignedBits(pl, plLen, upCtrl::kRollBit, upCtrl::kRollBits));
        out.pitch = (int16_t)decodeStick11(bits::getSignedBits(pl, plLen, upCtrl::kPitchBit, upCtrl::kPitchBits));
        out.yaw   = (int16_t)decodeStick11(bits::getSignedBits(pl, plLen, upCtrl::kYawBit, upCtrl::kYawBits));
        out.thr   = (uint16_t)bits::getBits(pl, plLen, upCtrl::kThrBit, upCtrl::kThrBits);
        out.flags = (uint8_t)(bits::getBits(pl, plLen, upCtrl::kFlagsBit, upCtrl::kFlagsBits) & upCtrl::kFlagsMask);
    }

    inline uint16_t encodeUpCtrl(const UpCtrlPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kUpCtrlBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::UpCtrl, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = upCtrl::kPayloadBytes;
        bits::putSignedBits(pl, plLen, upCtrl::kRollBit, clampStickCounts(p.roll), upCtrl::kRollBits);
        bits::putSignedBits(pl, plLen, upCtrl::kPitchBit, clampStickCounts(p.pitch), upCtrl::kPitchBits);
        bits::putSignedBits(pl, plLen, upCtrl::kYawBit, clampStickCounts(p.yaw), upCtrl::kYawBits);
        bits::putBits(pl, plLen, upCtrl::kThrBit, (uint32_t)clampThrottleCounts(p.thr), upCtrl::kThrBits);
        bits::putBits(pl, plLen, upCtrl::kFlagsBit, (uint32_t)(p.flags & upCtrl::kFlagsMask), upCtrl::kFlagsBits);
        writeCrc16Le(out, n);
        return n;
    }

    // ---- UP_ROUND(v2): 杆量 + session + flags8 ----

    inline void unpackUpRound(const uint8_t *frame, UpRoundPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = upRound::kPayloadBytes;
        // 摇杆与 UP_CTRL 同一套解码夹取(-1024 码点不可用, 收到就夹到 -1023)
        out.roll    = (int16_t)decodeStick11(bits::getSignedBits(pl, plLen, upRound::kRollBit, upRound::kRollBits));
        out.pitch   = (int16_t)decodeStick11(bits::getSignedBits(pl, plLen, upRound::kPitchBit, upRound::kPitchBits));
        out.yaw     = (int16_t)decodeStick11(bits::getSignedBits(pl, plLen, upRound::kYawBit, upRound::kYawBits));
        out.thr     = (uint16_t)bits::getBits(pl, plLen, upRound::kThrBit, upRound::kThrBits);
        out.flags   = (uint8_t)(bits::getBits(pl, plLen, upRound::kFlagsBit, upRound::kFlagsBits) & upCtrl::kFlagsMask);
        out.session = (uint8_t)bits::getBits(pl, plLen, upRound::kSessionBit, upRound::kSessionBits);
        out.flags8  = (uint8_t)(bits::getBits(pl, plLen, upRound::kFlags8Bit, upRound::kFlags8Bits) &
                                upRound::kFlags8Mask);
    }

    inline uint16_t encodeUpRound(const UpRoundPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kUpRoundBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::UpRound, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = upRound::kPayloadBytes;
        bits::putSignedBits(pl, plLen, upRound::kRollBit, clampStickCounts(p.roll), upRound::kRollBits);
        bits::putSignedBits(pl, plLen, upRound::kPitchBit, clampStickCounts(p.pitch), upRound::kPitchBits);
        bits::putSignedBits(pl, plLen, upRound::kYawBit, clampStickCounts(p.yaw), upRound::kYawBits);
        bits::putBits(pl, plLen, upRound::kThrBit, (uint32_t)clampThrottleCounts(p.thr), upRound::kThrBits);
        bits::putBits(pl, plLen, upRound::kFlagsBit, (uint32_t)(p.flags & upCtrl::kFlagsMask), upRound::kFlagsBits);
        bits::putBits(pl, plLen, upRound::kSessionBit, p.session, upRound::kSessionBits);
        bits::putBits(pl, plLen, upRound::kFlags8Bit, (uint32_t)(p.flags8 & upRound::kFlags8Mask),
                      upRound::kFlags8Bits);
        writeCrc16Le(out, n);
        return n;
    }

    // ---- HELLO(v2): ver + caps + nonce(双向同型) ----

    inline void unpackHello(const uint8_t *frame, HelloPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = hello::kPayloadBytes;
        out.ver   = (uint8_t)bits::getBits(pl, plLen, hello::kVerBit, hello::kVerBits);
        out.caps  = (uint8_t)(bits::getBits(pl, plLen, hello::kCapsBit, hello::kCapsBits) & hello::kCapsMask);
        out.nonce = (uint8_t)bits::getBits(pl, plLen, hello::kNonceBit, hello::kNonceBits);
    }

    inline uint16_t encodeHello(const HelloPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kHelloBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::Hello, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = hello::kPayloadBytes;
        bits::putBits(pl, plLen, hello::kVerBit, (uint32_t)(p.ver & 0x0Fu), hello::kVerBits);
        bits::putBits(pl, plLen, hello::kCapsBit, (uint32_t)(p.caps & hello::kCapsMask), hello::kCapsBits);
        bits::putBits(pl, plLen, hello::kNonceBit, p.nonce, hello::kNonceBits);
        writeCrc16Le(out, n);
        return n;
    }

    inline void unpackUpCmd(const uint8_t *frame, UpCmdPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = upCmd::kPayloadBytes;
        out.cmdId = (uint8_t)bits::getBits(pl, plLen, upCmd::kCmdIdBit, upCmd::kCmdIdBits);
        out.op    = (uint8_t)bits::getBits(pl, plLen, upCmd::kOpBit, upCmd::kOpBits);
        out.arg   = (uint8_t)bits::getBits(pl, plLen, upCmd::kArgBit, upCmd::kArgBits);
    }

    inline uint16_t encodeUpCmd(const UpCmdPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kUpCmdBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::UpCmd, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = upCmd::kPayloadBytes;
        bits::putBits(pl, plLen, upCmd::kCmdIdBit, p.cmdId, upCmd::kCmdIdBits);
        bits::putBits(pl, plLen, upCmd::kOpBit, p.op, upCmd::kOpBits);
        bits::putBits(pl, plLen, upCmd::kArgBit, p.arg, upCmd::kArgBits);
        writeCrc16Le(out, n);
        return n;
    }

    inline void unpackUpVel(const uint8_t *frame, UpVelPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = upVel::kPayloadBytes;
        out.vx = fromInt8FiveCmPerSec(bits::getSignedBits(pl, plLen, upVel::kVxBit, upVel::kVxBits));
        out.vy = fromInt8FiveCmPerSec(bits::getSignedBits(pl, plLen, upVel::kVyBit, upVel::kVyBits));
        out.vz = fromInt8FiveCmPerSec(bits::getSignedBits(pl, plLen, upVel::kVzBit, upVel::kVzBits));
    }

    inline uint16_t encodeUpVel(const UpVelPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kUpVelBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::UpVel, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = upVel::kPayloadBytes;
        bits::putSignedBits(pl, plLen, upVel::kVxBit, toInt8FiveCmPerSec(p.vx), upVel::kVxBits);
        bits::putSignedBits(pl, plLen, upVel::kVyBit, toInt8FiveCmPerSec(p.vy), upVel::kVyBits);
        bits::putSignedBits(pl, plLen, upVel::kVzBit, toInt8FiveCmPerSec(p.vz), upVel::kVzBits);
        writeCrc16Le(out, n);
        return n;
    }

    // ---- UP_SETPOINT(2026-10-05 新增) ----

    inline void unpackUpSetpoint(const uint8_t *frame, UpSetpointPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = upSetpoint::kPayloadBytes;
        out.altM  = fromInt16Centimeters(bits::getSignedBits(pl, plLen, upSetpoint::kAltBit, upSetpoint::kAltBits));
        out.flags = (uint8_t)(bits::getBits(pl, plLen, upSetpoint::kFlagsBit, upSetpoint::kFlagsBits) &
                              upSetpoint::kFlagsMask);
    }

    inline uint16_t encodeUpSetpoint(const UpSetpointPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kUpSetpointBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::UpSetpoint, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = upSetpoint::kPayloadBytes;
        bits::putSignedBits(pl, plLen, upSetpoint::kAltBit, toInt16Centimeters(p.altM), upSetpoint::kAltBits);
        bits::putBits(pl, plLen, upSetpoint::kFlagsBit, (uint32_t)(p.flags & upSetpoint::kFlagsMask),
                      upSetpoint::kFlagsBits);
        writeCrc16Le(out, n);
        return n;
    }

    // ---- UP_MOTOR(2026-10-12: 四路一次发全) ----

    inline void unpackUpMotor(const uint8_t *frame, UpMotorPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = upMotor::kPayloadBytes;
        out.throttle[0] = (uint16_t)bits::getBits(pl, plLen, upMotor::kMotor0Bit, upMotor::kMotorBits);
        out.throttle[1] = (uint16_t)bits::getBits(pl, plLen, upMotor::kMotor1Bit, upMotor::kMotorBits);
        out.throttle[2] = (uint16_t)bits::getBits(pl, plLen, upMotor::kMotor2Bit, upMotor::kMotorBits);
        out.throttle[3] = (uint16_t)bits::getBits(pl, plLen, upMotor::kMotor3Bit, upMotor::kMotorBits);
    }

    /** 4 个保留位必须为 0(与 DL_STATE 的 3 个保留位同一约定)。准入门用它把"不是本工程编出来的帧"挡掉。 */
    inline bool upMotorSpareBitsZero(const uint8_t *frame)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = upMotor::kPayloadBytes;
        return bits::getBits(pl, plLen, upMotor::kSpareBit, upMotor::kSpareBits) == 0u;
    }

    /**
     * @brief 组一帧 UP_MOTOR(四路 DShot 计数)
     *
     * 四路都过 `clampDshot()`: 线上 11 位能表示 0..2047, 而 2000..2047 是 DShot 的**特殊命令区**
     * (不是油门), 夹到 0..1999 才不会把一次"手滑"变成一个电调命令。**饱和而不是回绕** ——
     * 回绕会把 2047 变成 0(满油变停车)。
     */
    inline uint16_t encodeUpMotor(const UpMotorPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kUpMotorBytes;
        bits::clearBytes(out, n); // 顺带把 4 个空位清零(不依赖调用方)
        out[0] = frameHead(FrameType::UpMotor, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = upMotor::kPayloadBytes;
        bits::putBits(pl, plLen, upMotor::kMotor0Bit, (uint32_t)clampDshot((int32_t)p.throttle[0]),
                      upMotor::kMotorBits);
        bits::putBits(pl, plLen, upMotor::kMotor1Bit, (uint32_t)clampDshot((int32_t)p.throttle[1]),
                      upMotor::kMotorBits);
        bits::putBits(pl, plLen, upMotor::kMotor2Bit, (uint32_t)clampDshot((int32_t)p.throttle[2]),
                      upMotor::kMotorBits);
        bits::putBits(pl, plLen, upMotor::kMotor3Bit, (uint32_t)clampDshot((int32_t)p.throttle[3]),
                      upMotor::kMotorBits);
        writeCrc16Le(out, n);
        return n;
    }

    inline void unpackUpPos(const uint8_t *frame, UpPosPayload &out)
    {
        const uint8_t  *pl    = frame + 1;
        const uint16_t  plLen = upPos::kPayloadBytes;
        out.x = fromInt16Dm(bits::getSignedBits(pl, plLen, upPos::kXBit, upPos::kXBits));
        out.y = fromInt16Dm(bits::getSignedBits(pl, plLen, upPos::kYBit, upPos::kYBits));
    }

    inline uint16_t encodeUpPos(const UpPosPayload &p, uint8_t seq, uint8_t *out)
    {
        if (out == 0) {
            return 0u;
        }
        const uint16_t n = kUpPosBytes;
        bits::clearBytes(out, n);
        out[0] = frameHead(FrameType::UpPos, seq);
        uint8_t        *pl    = out + 1;
        const uint16_t plLen = upPos::kPayloadBytes;
        bits::putSignedBits(pl, plLen, upPos::kXBit, toInt16Dm(p.x), upPos::kXBits);
        bits::putSignedBits(pl, plLen, upPos::kYBit, toInt16Dm(p.y), upPos::kYBits);
        writeCrc16Le(out, n);
        return n;
    }

    //===============================================================================================
    // 八、单帧解码(带类型 + CRC 校验)
    //
    //   为什么 decodeX 不做长度检查: 它没有长度参数, 也没必要有 —— 帧长只由类型决定, 调用方要么
    //   先用 frameBytes() 拿到长度, 要么直接走 tryDecode()。多一个 len 参数只会多一个"传错了也
    //   编译得过"的机会。传进来的指针至少要能读 frameBytes(type) 个字节, 这是调用方的前提。
    // ===============================================================================================
    inline bool decodeDlAtt(const uint8_t *frame, DlAttPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::DlAtt) {
            return false;
        }
        if (!crc16LeValid(frame, kDlAttBytes)) {
            return false;
        }
        unpackDlAtt(frame, out);
        return true;
    }

    inline bool decodeDlSys(const uint8_t *frame, DlSysPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::DlSys) {
            return false;
        }
        if (!crc16LeValid(frame, kDlSysBytes)) {
            return false;
        }
        unpackDlSys(frame, out);
        return true;
    }

    inline bool decodeDlGps(const uint8_t *frame, DlGpsPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::DlGps) {
            return false;
        }
        if (!crc16LeValid(frame, kDlGpsBytes)) {
            return false;
        }
        unpackDlGps(frame, out);
        return true;
    }

    inline bool decodeDlState(const uint8_t *frame, DlStatePayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::DlState) {
            return false;
        }
        if (!crc16LeValid(frame, kDlStateBytes)) {
            return false;
        }
        unpackDlState(frame, out);
        return true;
    }

    inline bool decodeDlAck(const uint8_t *frame, DlAckPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::DlAck) {
            return false;
        }
        if (!crc16LeValid(frame, kDlAckBytes)) {
            return false;
        }
        unpackDlAck(frame, out);
        return true;
    }

    inline bool decodeUpCtrl(const uint8_t *frame, UpCtrlPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::UpCtrl) {
            return false;
        }
        if (!crc16LeValid(frame, kUpCtrlBytes)) {
            return false;
        }
        unpackUpCtrl(frame, out);
        return true;
    }

    inline bool decodeUpCmd(const uint8_t *frame, UpCmdPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::UpCmd) {
            return false;
        }
        if (!crc16LeValid(frame, kUpCmdBytes)) {
            return false;
        }
        unpackUpCmd(frame, out);
        return true;
    }

    inline bool decodeUpVel(const uint8_t *frame, UpVelPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::UpVel) {
            return false;
        }
        if (!crc16LeValid(frame, kUpVelBytes)) {
            return false;
        }
        unpackUpVel(frame, out);
        return true;
    }

    inline bool decodeUpPos(const uint8_t *frame, UpPosPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::UpPos) {
            return false;
        }
        if (!crc16LeValid(frame, kUpPosBytes)) {
            return false;
        }
        unpackUpPos(frame, out);
        return true;
    }

    inline bool decodeUpSetpoint(const uint8_t *frame, UpSetpointPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::UpSetpoint) {
            return false;
        }
        if (!crc16LeValid(frame, kUpSetpointBytes)) {
            return false;
        }
        unpackUpSetpoint(frame, out);
        return true;
    }

    /** 注意与 DL_STATE 同一约定: 这里**不**检查 2 个保留位 —— 那属于"这帧可不可信",
     *  由准入门(FrameAdmission, 用 upMotorSpareBitsZero())判, 不在编解码层重复一遍。 */
    inline bool decodeUpMotor(const uint8_t *frame, UpMotorPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::UpMotor) {
            return false;
        }
        if (!crc16LeValid(frame, kUpMotorBytes)) {
            return false;
        }
        unpackUpMotor(frame, out);
        return true;
    }

    // ---- v2 新增帧的单帧解码 ----

    inline bool decodeUpRound(const uint8_t *frame, UpRoundPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::UpRound) {
            return false;
        }
        if (!crc16LeValid(frame, kUpRoundBytes)) {
            return false;
        }
        unpackUpRound(frame, out);
        return true;
    }

    /** HELLO 双向同型: 请求/应答的区分不在帧层(靠谁发起), 这里只解 ver/caps/nonce。 */
    inline bool decodeHello(const uint8_t *frame, HelloPayload &out)
    {
        if (frame == 0 || frameTypeOf(frame[0]) != FrameType::Hello) {
            return false;
        }
        if (!crc16LeValid(frame, kHelloBytes)) {
            return false;
        }
        unpackHello(frame, out);
        return true;
    }

    //===============================================================================================
    // 九、从字节流里切帧: tryDecode()
    //===============================================================================================
    /** 统一解出的帧: 类型 + SEQ + 负载。只有一个负载成员会被填(由 type 决定), 其余是垃圾值 ——
     *  这是 union + tag 的老约定, 也是"零拷贝切帧"能做到无动态分配的原因。 */
    struct Frame
    {
        FrameType type;
        uint8_t   seq;
        union
        {
            DlAttPayload   att;
            DlSysPayload   sys;
            DlGpsPayload   gps;
            DlStatePayload state;
            DlAckPayload   ack;
            UpRoundPayload round;
            HelloPayload   hello;
            UpCtrlPayload  ctrl;
            UpCmdPayload   cmd;
            UpVelPayload   vel;
            UpPosPayload   pos;
            UpSetpointPayload setpoint;
            UpMotorPayload    motor;
        } u;
    };

    /** tryDecode 的三种返回值 —— 必须区分, 因为调用方的动作完全不同:
     *    kTryDecoded : 解出一帧, consumed = 帧长 ⇒ 处理并前进整帧;
     *    kTryNeedMore: 字节还不够, consumed = **0** ⇒ 一个字节都不能丢, 等下一批 DMA/IDLE 数据;
     *    kTryReject  : 这里不是帧头(类型非法 或 CRC 不符), consumed = **1** ⇒ 前进 1 字节重找。 */
    constexpr int kTryDecoded  = 1;
    constexpr int kTryNeedMore = 0;
    constexpr int kTryReject   = -1;

    /** 从 buf[0..len) 里尝试解出一帧。
     *
     * 为什么坏帧只前进 1 字节而不是整帧跳过(§3.2 "一次最多前进 1 字节"):
     *   帧长是从"疑似帧头"推出来的, 而帧头本身可能就是一个噪声字节 —— 按推导长度整帧跳过, 噪声
     *   会顺走后面真帧的开头(尤其噪声字节恰好声称一个 21B 的 DL_GPS 时, 后面的真帧会被整段吃掉)。
     *   只前进 1 字节最坏也只是丢掉真帧的头一个字节, 下一字节立刻能重新定界。少丢帧 > 少算几次 CRC。
     * 为什么"字节不够"时不前进: 这里分不清"这确实是一帧的开头"和"这是噪声、只是凑巧类型合法"。
     *   提前丢弃会把正在到达的真帧尾巴当噪声吃掉 —— 丢帧的代价比"最多多等 20 个字节"大得多。
     *
     * 注意: 半帧等待的代价是**有界**的(kMaxFrameBytes-1 个字节), 不是死等 —— 上层的接收缓冲只要
     * 留够一帧的长度, 这个函数就不会永久停在 kTryNeedMore 上。 */
    inline int tryDecode(const uint8_t *buf, uint16_t len, Frame &out, uint16_t &consumed)
    {
        consumed = 0u;
        if (buf == 0) {
            return kTryNeedMore; // 连缓冲区都没有 = 还没数据, 与"len == 0"同义
        }
        if (len < 1u) {
            return kTryNeedMore;
        }
        const uint8_t   b0   = buf[0];
        const FrameType type = frameTypeOf(b0);
        const uint16_t  need = frameBytes(type);
        if (need == 0u) {
            consumed = 1u; // 0x0/0xE/0xF: 立刻吃 1 字节, 绝不返回"长度 0"让调用方死循环
            return kTryReject;
        }
        if (len < need) {
            return kTryNeedMore; // 半帧: consumed 保持 0
        }
        if (!crc16LeValid(buf, need)) {
            consumed = 1u;
            return kTryReject;
        }
        out.type = type;
        out.seq  = seqOf(b0);
        switch (type) {
        case FrameType::DlAtt:   unpackDlAtt(buf, out.u.att); break;
        case FrameType::DlSys:   unpackDlSys(buf, out.u.sys); break;
        case FrameType::DlGps:   unpackDlGps(buf, out.u.gps); break;
        case FrameType::DlState: unpackDlState(buf, out.u.state); break;
        case FrameType::DlAck:   unpackDlAck(buf, out.u.ack); break;
        case FrameType::UpRound: unpackUpRound(buf, out.u.round); break;
        case FrameType::Hello:   unpackHello(buf, out.u.hello); break;
        case FrameType::UpCtrl:  unpackUpCtrl(buf, out.u.ctrl); break;
        case FrameType::UpCmd:   unpackUpCmd(buf, out.u.cmd); break;
        case FrameType::UpVel:   unpackUpVel(buf, out.u.vel); break;
        case FrameType::UpPos:   unpackUpPos(buf, out.u.pos); break;
        case FrameType::UpSetpoint: unpackUpSetpoint(buf, out.u.setpoint); break;
        case FrameType::UpMotor:    unpackUpMotor(buf, out.u.motor); break;
        default:                 return kTryReject; // 走不到: need==0 已经提前返回了
        }
        consumed = need;
        return kTryDecoded;
    }

    //===============================================================================================
    // 十、SEQ 丢包计数
    //===============================================================================================
    /** 相邻两帧的 SEQ 距离(模 16): 0 = 重复/重放帧, 1 = 连续无丢失, k = 中间丢了 k-1 帧。
     *
     * 为什么是 (cur - prev) & 0x0F 而不是分支写法(if (cur < prev) cur += 16):
     *   15 → 0 是每 16 帧必然发生一次的正常情形, 不是边界特例; 分支写法把它变成一条"只在回绕时
     *   才走到"的路径, 而那种路径在台架上最难复现(要刚好卡在第 16 帧)。取模写法让"连续"永远等于 1,
     *   顺带把方向写反(prev - cur)这种错暴露成 15 而不是一个静默的负值/241。
     * 代价(必须知道): 丢满 16 帧会被看成 0(重复帧)。v2 上行 33.3Hz 下 16 帧 = 480ms,
     *   那时上层(300ms 降级 / 2s 失效保护)早已动作, 所以这里不做"丢包圈数"的额外状态。
     * 依据: 协议规格 §1.3 "SEQ 每发一帧 +1; 上行 SEQ&3 = 轮次位置"。 */
    inline uint8_t seqGap(uint8_t prev, uint8_t cur)
    {
        return (uint8_t)(((cur & 0x0Fu) - (prev & 0x0Fu)) & 0x0Fu);
    }

    //===============================================================================================
    // 十一、轮次位置(v2)
    //===============================================================================================
    /** 上行 SEQ 的低 2 位 = 帧在轮次内的位置(0..3 = U0..U3)。
     *  为什么位置是 2 位而不是超帧号: 轮长/帧距是**时间**概念(22ms/120ms), 帧里只带位置;
     *  飞控靠"收到哪一位置的帧 + 该帧的末字节时刻"反推本轮 t0(协议规格 §2.2), 不需要同步计数器。 */
    constexpr uint8_t kSeqPosMask = 0x03u;
    constexpr uint8_t kSeqPosCount = 4u;

    constexpr uint8_t seqPos(uint8_t seq)
    {
        return (uint8_t)(seq & kSeqPosMask);
    }

} // namespace link
} // namespace dlx

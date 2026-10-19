#pragma once
//===================================================================================================
// app/link/link_v2_config.hpp —— 链路 v2 的全部时序/协议参数(唯一一份, 普通常量)
//
// 规则(owner 2026-10-18 定):
//   1. 名字**不带 k 前缀**, 驼峰; 与遥控器 components/fc_link/include/fc_link_config.h 逐名逐值一致;
//   2. 不引用 cfg::, 不再把链路参数散落在 cfg / 既有宏里;
//   3. P0 台架复测后**只改这里和遥控器同名常量**, 并跑 tools/check_link_config.py;
//   4. 帧长不写在这里: 帧长由帧表按位相加得出, 两处数字必然漂。
//
// 单位: 时间参数除 *Us 外都是 ms; 毫秒常量在代码里乘 1000 用, 不另设 Us 版本(避免两份真值)。
//===================================================================================================
#include <stdint.h>

// ---- 轮次结构 ----
constexpr uint32_t LinkRoundMs           = 120;   ///< 一轮总长: 4 上行 + 1 下行
constexpr uint32_t LinkFrameSpacingMs    = 22;    ///< 相邻上行帧起点间距(U0/U1/U2/U3)
constexpr uint32_t LinkUplinkFrames      = 4;     ///< 每轮上行帧数; 与 SEQ&3 的位置数绑定

// ---- 半双工切换/下行窗口 ----
constexpr uint32_t LinkGuardUs           = 3000;  ///< 末字节 -> 允许换向发送; P0 实测复核
constexpr uint32_t LinkU3JitterUs        = 3000;  ///< U3 未到时, 期望末字节之后再等的抖动余量
constexpr uint32_t LinkDlTimeoutMs       = 117;   ///< 遥控器等本轮下行: U0 后 117ms 仍无 = DL 丢失

// ---- 唤醒/超时 ----
constexpr uint32_t LinkTickMs            = 6;     ///< 飞控链路线程 6ms 唤醒(只做 watchdog/统计)
constexpr uint32_t LinkHelloIntervalMs   = 60;    ///< HELLO 重发间隔
constexpr uint32_t LinkHelloMax          = 5;     ///< 发几次 HELLO 无应答 -> NO_LINK
constexpr uint32_t LinkAcquireMs         = 200;   ///< 飞控多久无合法上行 -> 回 ACQUIRE
constexpr uint32_t LinkDegradeMs         = 360;   ///< 遥控器多久无任何下行 -> 链路降级/命令门控

// ---- 协议版本 ----
constexpr uint32_t LinkProtoVersion      = 2;     ///< HELLO.ver; 不一致时双方 MISMATCH, 不发控制

// ---- 单帧成本模型 F(n) = base + perByte*n (us); 2026-10-18 P0 台架实测后冻结 ----
//   sweep 线性拟合: 斜率 387.25us/B, 截距 13555us (F(8)=16509, F(32)=25365)。
//   注意: 62.5kbps 是射频配置值, 有效吞吐只有 ~20.7kbps —— 差值来自每包固定的 LoRa 前导/编码
//   开销(≈13.5ms), 不是配置错误; 因此帧长不能再涨, 21B 已是预算上限。
constexpr uint32_t LinkAirtimeBaseUs     = 13555;
constexpr uint32_t LinkAirtimePerByteUs  = 387;

// ---- 下行预算: 空中结束后还要把整帧从对端串口排空, 才算"收到" ----
constexpr uint32_t LinkUartByteUs        = 174;   ///< 57600 8N1, 10bit/字节
constexpr uint32_t LinkDlBudgetSlackUs   = 1000;  ///< D0 预算额外调度余量
// 收帧末修正: 模块把整帧收完才开始吐 UART, 且 IDLE 还要再等一个字符 ——
// rawIdle ≈ 空中结束 + frameBytes×LinkUartByteUs + 1×LinkUartByteUs。
constexpr uint32_t LinkRxIdleBytes       = 1;     ///< 帧末 IDLE 判定额外占用的字符数

// ---- 遥控器恢复策略 ----
constexpr uint32_t LinkDlLossToHandshake = 2;     ///< 连续几轮收不到下行 -> 回 HANDSHAKE

//===================================================================================================
// 编译期守卫: 参数之间必须满足的硬关系。改了数却忘了改协议, 在这里先报, 而不是上板看时序。
//===================================================================================================
static_assert(LinkUplinkFrames == 4, "SEQ&3 只有 4 个位置: 上行帧数必须为 4(要改先改协议)");
static_assert(LinkProtoVersion > 0 && LinkProtoVersion < 16, "HELLO.ver 只有 4bit, 且 0 保留");
static_assert(LinkFrameSpacingMs * LinkUplinkFrames < LinkRoundMs,
              "4 帧上行必须整体落在 120ms 轮次内");
static_assert(LinkDlTimeoutMs < LinkRoundMs, "DL 超时必须早于下一轮 U0");
static_assert(LinkGuardUs > 0 && LinkGuardUs < 8000, "guard 是半双工切换余量, 秒级说明单位写错");
static_assert(LinkAirtimeBaseUs > 0 && LinkAirtimePerByteUs > 0, "F(n) 模型参数必须为正");

//===================================================================================================
// test/iic_error_recovery_test.cpp —— I2C 错误标志早退 / 总线恢复的**硬回归门**
//
// 被测对象: **真实的** DL_LIB/dlx_iic.hpp(没有被 shadow, 这一点是本测试的意义所在)。
// 替身:      test/iic_stub/stm32f4xx.h(SPL 函数 + I2C 主状态机 + 调用计数器)。
//
// 这个门锁住 2026-10-18 那三处加固, 任何一条被删掉都会让本测试**失败**(不再是"打印 KNOWN-GAP 也算过"):
//   1. `waitEvent()` 必须每 512 轮(且 i==0 必查)看一次 SR1 的 AF/BERR/ARLO/OVR, 一旦置位**立即**放弃;
//      ⇒ 用 `hmCheckEventCalls` 度量: 有错误标志时一次事务的 I2C_CheckEvent 调用数必须是个位数,
//        而不是空转到 kEventWaitLimit(200000)。
//   2. `recover()` 必须**重跑一次 I2C_Init** + **清一次错误标志**;
//      ⇒ 用 `hmInitCalls`/`hmClearFlagCalls` 的**增量**(不是累计值)度量。
//   3. 恢复之后同一事务必须成功, 且 SR1 的 4 个错误位必须为 0。
//
// 编译(仓库根, g++ 11.2.0 实测):
//   g++ -std=c++11 -Wall -Wextra -Wno-cpp -I test/iic_stub -I DL_LIB -I . `
//       test/iic_error_recovery_test.cpp -o $env:TEMP\iic_error_recovery_test.exe
//   -Wno-cpp 只为屏蔽 DL_LIB/dlx_gpio.hpp 与 DL_LIB/dlx_iic.hpp 里两个既有的
//   `#warning [Experimental]`(本任务的写入范围不允许改 DL_LIB); 除此之外零警告。
//===================================================================================================
#include <stdio.h>
#include "dlx_iic.hpp"

static unsigned checks   = 0u;
static unsigned failures = 0u;

#define CHECK(condition)                                        \
    do {                                                        \
        ++checks;                                               \
        if (!(condition)) {                                     \
            ++failures;                                         \
            printf("FAIL line %d: %s\n", __LINE__, #condition); \
        }                                                       \
    } while (false)

/** SR1 上会让"继续等事件"永远等不到的错误标志, 与 DL_LIB/dlx_iic.hpp 的 kErrorFlags 一一对应 */
static const uint16_t kErrorFlags =
    (uint16_t)(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);

/**
 * "不空转"门限: 有错误标志时, 一次事务里 I2C_CheckEvent 的调用次数上限。
 *
 * waitEvent 的轮询上限是 kEventWaitLimit = 200000。加固后错误标志在 i==0 就被看到, 实测值是个位数。
 * 上限取 200 同时挡住三种回归: 完全没有早退(≈200000)、只在 512 的倍数查但漏了 i==0(≈511)、
 * 以及把"看到标志"当成"继续等"(>200)。
 */
static const uint32_t kNoSpinMaxChecks = 200u;

/** 一次 7 字节寄存器读(read 时序: 写寄存器地址 -> 重复 START -> 连续读), 与 BMP388 的用法一致 */
static bool doRead(dlx::IICDevice &device, uint8_t *output, uint16_t length)
{
    uint8_t commandByte = 0x03u; // STATUS 起 7 字节
    dlx::ByteBuffer writeBuffer(&commandByte, 1u);
    writeBuffer.write(&commandByte, 1u);
    dlx::ByteBuffer readBuffer(output, length);
    return device.read(readBuffer, writeBuffer, length);
}

int main()
{
    // =============================================================================================
    // 场景 A: 注入一次 NACK -> waitEvent 立即放弃(不空转) -> recover() 重跑 I2C_Init + 清标志
    //         -> 下一次同一事务成功
    // =============================================================================================
    dlx::IICBus bus = dlx::IICBus::IIC1_SB6_DB7();
    bus.init(dlx::IICBusModeProfile::ACK_E_DC2_ADDR7, 100000u, 0x51u);
    dlx::IICDevice device(bus, 0x76u);

    // 这条 1 是本测试自己的 bus.init() 产生的(与 recover() 无关, 后面只用增量)
    CHECK(I2C1->hmInitCalls == 1u);
    CHECK((I2C1->CR1 & I2C_CR1_PE) != 0u);

    uint8_t out[7] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};

    // ---- 1. 注入一次地址 NACK: 必须"看到 SR1 错误标志就立即放弃", 不空转 ----
    printf("[1] 注入一次地址 NACK(从机不响应 0x76), 看 waitEvent 是否立即放弃\n");
    I2C1->hmNackNextAddress        = true;
    const uint32_t checkBeforeFail = I2C1->hmCheckEventCalls;
    const uint32_t latchesBefore   = bus.errorLatches();
    const bool firstOk             = doRead(device, out, 7u);
    const uint32_t failSpinChecks  = I2C1->hmCheckEventCalls - checkBeforeFail;

    CHECK(!firstOk);
    CHECK(bus.hasError());
    CHECK(bus.errorLatches() == latchesBefore + 1u); // 走的是"SR1 错误标志提前放弃"这条路
    CHECK(bus.lastFailedEvent() == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    CHECK(failSpinChecks < kNoSpinMaxChecks);        // 核心门: 不是 ~200000
    CHECK((I2C1->SR1 & kErrorFlags) != 0u);          // waitEvent 故意不清标志(留给诊断), 由 recover() 清
    printf("  第一次 read(): ok=%d, errorLatches=%u, eventTimeouts=%u, lastFailedEvent=0x%08X\n",
           firstOk ? 1 : 0,
           (unsigned)bus.errorLatches(),
           (unsigned)bus.eventTimeouts(),
           (unsigned)bus.lastFailedEvent());
    printf("  I2C_CheckEvent 调用=%u (门限 <%u; 未加固时是 ~200000)\n",
           (unsigned)failSpinChecks,
           (unsigned)kNoSpinMaxChecks);

    // ---- 2. 下一次 read(): 入口 recover() 必须重跑 I2C_Init + 清错误标志, 之后同一事务成功 ----
    printf("[2] 下一次 read(): 入口 recover() 必须重跑 I2C_Init + 清错误标志\n");
    const uint32_t initBefore   = I2C1->hmInitCalls;
    const uint32_t clearBefore  = I2C1->hmClearFlagCalls;
    const uint32_t peBefore     = I2C1->hmPeDisableCalls;
    const uint32_t checkBefore2 = I2C1->hmCheckEventCalls;

    const bool secondOk = doRead(device, out, 7u);

    const uint32_t recoverInitDelta  = I2C1->hmInitCalls - initBefore;
    const uint32_t recoverClearDelta = I2C1->hmClearFlagCalls - clearBefore;
    const uint32_t recoverPeDelta    = I2C1->hmPeDisableCalls - peBefore;
    const uint32_t okSpinChecks      = I2C1->hmCheckEventCalls - checkBefore2;

    CHECK(secondOk);                          // 加固后的正确行为: 同一次读成功
    CHECK(!bus.hasError());
    CHECK(recoverPeDelta == 1u);              // recover() 确实跑了(PE 关一次)
    CHECK(recoverInitDelta == 1u);            // 恰好重跑一次 I2C_Init
    CHECK(recoverClearDelta == 1u);           // 恰好清一次错误标志
    CHECK((I2C1->SR1 & kErrorFlags) == 0u);   // AF/BERR/ARLO/OVR 全为 0
    CHECK(okSpinChecks > 0u);                 // 反空洞: 计数器是活的(成功路径也会调 CheckEvent)
    printf("  recover() 期间**增量**: I2C_Cmd(DISABLE) delta=%u, I2C_Init delta=%u, I2C_ClearFlag delta=%u\n",
           (unsigned)recoverPeDelta,
           (unsigned)recoverInitDelta,
           (unsigned)recoverClearDelta);
    printf("  第二次 read(): ok=%d, SR1=0x%04X(错误位应为 0), I2C_CheckEvent 调用=%u\n",
           secondOk ? 1 : 0,
           (unsigned)I2C1->SR1,
           (unsigned)okSpinChecks);

    // =============================================================================================
    // 场景 B: 标志"早就粘住"的情形 —— i==0 必查 ⇒ 一次 I2C_CheckEvent 都不该发
    //         (替身建模: AF=1 时 START 不产生 SB, 见 test/iic_stub/stm32f4xx.h 的 §27.3.7 说明)
    // =============================================================================================
    printf("[3] 反例(标志早粘住): 直接给 SR1 置 AF, 再 start() 一次\n");
    dlx::IICBus busB = dlx::IICBus::IIC2_SB10_DB11();
    busB.init(dlx::IICBusModeProfile::ACK_E_DC2_ADDR7, 100000u, 0x51u);
    CHECK(I2C2->hmAfBlocksNextTransfer); // 建模开关: AF=1 时 START 不产生 SB

    I2C2->SR1 |= I2C_SR1_AF; // 模拟"上一次 NACK 留下的粘住标志"
    const uint32_t checkBefore3 = I2C2->hmCheckEventCalls;
    busB.start();
    const uint32_t latchedSpinChecks = I2C2->hmCheckEventCalls - checkBefore3;

    CHECK(busB.hasError());
    CHECK(busB.errorLatches() == 1u);
    CHECK(latchedSpinChecks < kNoSpinMaxChecks);
    printf("  start(): hasError=%d, errorLatches=%u, I2C_CheckEvent 调用=%u (门限 <%u)\n",
           busB.hasError() ? 1 : 0,
           (unsigned)busB.errorLatches(),
           (unsigned)latchedSpinChecks,
           (unsigned)kNoSpinMaxChecks);

    // =============================================================================================
    printf("iic_error_recovery_test: %u checks, %u failures (mode=gate)\n", checks, failures);
    printf("  -> 硬回归门: 删掉 recover() 里的 I2C_Init/清标志, 或删掉 waitEvent 的提前放弃, 本测试会失败。\n");
    return failures == 0u ? 0 : 1;
}

#pragma once
//===================================================================================================
// test/iic_stub/stm32f4xx.h —— 主机侧寄存器级 I2C/GPIO 替身(只服务于 test/iic_error_recovery_test.cpp)
//
// 目的: 让**真实的** DL_LIB/dlx_iic.hpp 在主机(g++, 无硬件)上编译并运行。
//   - 本文件只提供 SPL 的宏/类型/函数签名 + 一个显式的 I2C 主状态机模型;
//   - GPIO 侧直接用真实的 DL_LIB/dlx_gpio.hpp(不 shadow), 所以 MODER/OSPEEDR/OTYPER/PUPDR
//     与 GPIO_PinAFConfig/SetBits/ResetBits/ReadInputDataBit 都必须是真的可用的最小实现;
//   - 被 shadow 的只有 stm32f4xx.h(本文件); dlx_delay.hpp 想 shadow 但被 GCC 的引号 include
//     规则挡掉, 见下面 SysTick 那一段的说明。
//
// 建模依据(重要: 这是"硬件行为像不像"的关键, 每一条都写出了依据与不确定性):
//
//   [已由参考手册明确的]
//   RM0090 §27.3.1 / §27.3.3 事件序列: 主机发 START -> SB=1, MSL=1, BUSY=1;
//       地址阶段被从机 ACK -> ADDR=1(TX 方向另有 TRA=1/TXE=1); 写 DR 发字节 -> BTF/TXE;
//       接收方向 -> RXNE=1; 发 STOP -> BUSY/MSL 清 0。
//   RM0090 §27.3.7(错误条件): 从机没有应答地址/字节时置 SR1.AF=1, 本次传输作废; AF 是
//       "写 0 清除"的标志位, 必须由软件清 0 才能继续, 硬件不会自动清。因此本模型里
//       `hmAfBlocksNextTransfer == true`(默认)时: AF=1 期间 GenerateSTART 不产生 SB、写 DR 不产生
//       任何事件 —— 对应 dlx_iic.hpp 的 waitEvent(): 它在 i==0 就看到错误标志并**立即放弃**,
//       不再空转到 kEventWaitLimit(这正是 test/iic_error_recovery_test.cpp 的"不空转"门)。
//   RM0090 §27.6.6(I2C_SR1: AF 位 10 / BERR 位 9 / ARLO 位 8 / OVR 位 3): 均为写 0 清除。
//   RM0090 §27.6.1(I2C_CR1 位 0 PE): PE 只是外设使能位; SPL 另有 SWRST(CR1 位 15)才是
//       "软件复位"。见下面的不确定性说明。
//   SPL stm32f4xx_i2c.h 的事件掩码与 SR1/SR2 位定义按原文照抄(值见下方 #define)。
//
//   [刻意保留的不确定性开关]
//   PE=0 到底会不会顺带清掉 SR1.AF, 本替身无法从手册一句话定死(手册把"软件复位"归给 SWRST),
//   所以 hmAfBlocksNextTransfer 同时控制两件事:
//     * true (默认): 按 §27.3.7 -> AF 粘住, PE=0 不清标志, 且 AF=1 时 START/写 DR 不产生事件;
//     * false: 反事实建模 -> PE=0 顺带清掉错误标志, AF 不再阻塞后续传输。
//   驱动现在的修复(recover() 重跑 I2C_Init + 显式 I2C_ClearFlag)在**两种**建模下都成立,
//   所以 iic_error_recovery_test 不再依赖这个开关的取值。
//
//   [明确不建模的部分(与本次故障无关, 保持最小)]
//   BERR/ARLO/OVR 只会由硬件在特定电气条件下置位, 本模型不主动置位; 但它们可以被测试直接写入
//   SR1、也可以被 I2C_ClearFlag 清掉 —— 与驱动的读/清逻辑一致。
//   I2C_Init 只按 SPL 的写法置 PE/记录 CCR/TRISE/OAR, 不清 SR1(SPL 的 I2C_Init 里没有清标志)。
//===================================================================================================
#include <stdint.h>

// ---------------------------------------------------------------------------------------------------
// SPL 基础类型/常量
// ---------------------------------------------------------------------------------------------------
typedef enum
{
    DISABLE = 0,
    ENABLE  = !DISABLE
} FunctionalState;

typedef enum
{
    ERROR   = 0,
    SUCCESS = !ERROR
} ErrorStatus;

// CR1
#define I2C_CR1_PE     ((uint16_t)0x0001)
#define I2C_CR1_START  ((uint16_t)0x0100)
#define I2C_CR1_STOP   ((uint16_t)0x0200)
#define I2C_CR1_ACK    ((uint16_t)0x0400)
// SR1
#define I2C_SR1_SB     ((uint16_t)0x0001)
#define I2C_SR1_ADDR   ((uint16_t)0x0002)
#define I2C_SR1_BTF    ((uint16_t)0x0004)
#define I2C_SR1_STOPF  ((uint16_t)0x0010)
#define I2C_SR1_RXNE   ((uint16_t)0x0040)
#define I2C_SR1_TXE    ((uint16_t)0x0080)
#define I2C_SR1_BERR   ((uint16_t)0x0100)
#define I2C_SR1_ARLO   ((uint16_t)0x0200)
#define I2C_SR1_AF     ((uint16_t)0x0400)
#define I2C_SR1_OVR    ((uint16_t)0x0800)
// SR2
#define I2C_SR2_MSL    ((uint16_t)0x0001)
#define I2C_SR2_BUSY   ((uint16_t)0x0002)
#define I2C_SR2_TRA    ((uint16_t)0x0004)

// 事件掩码(照抄 SPL stm32f4xx_i2c.h; 低位 = SR1, 高位 = SR2)
#define I2C_EVENT_MASTER_MODE_SELECT               ((uint32_t)0x00030001) /* BUSY, MSL, SB */
#define I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ((uint32_t)0x00070082) /* BUSY, MSL, ADDR, TXE, TRA */
#define I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED    ((uint32_t)0x00030002) /* BUSY, MSL, ADDR */
#define I2C_EVENT_MASTER_BYTE_RECEIVED             ((uint32_t)0x00030040) /* BUSY, MSL, RXNE */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING         ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTED          ((uint32_t)0x00070084) /* TRA, BUSY, MSL, TXE, BTF */

// I2C_InitTypeDef 字段值
#define I2C_Mode_I2C                     ((uint16_t)0x0000)
#define I2C_DutyCycle_16_9               ((uint16_t)0x4000)
#define I2C_DutyCycle_2                  ((uint16_t)0xBFFF)
#define I2C_Ack_Enable                   ((uint16_t)0x0400)
#define I2C_Ack_Disable                  ((uint16_t)0x0000)
#define I2C_AcknowledgedAddress_7bit     ((uint16_t)0x4000)
#define I2C_AcknowledgedAddress_10bit    ((uint16_t)0xC000)
#define I2C_Direction_Transmitter        ((uint8_t)0x00)
#define I2C_Direction_Receiver           ((uint8_t)0x01)

// RCC
#define RCC_APB1Periph_I2C1  ((uint32_t)0x00200000)
#define RCC_APB1Periph_I2C2  ((uint32_t)0x00400000)
#define RCC_APB1Periph_I2C3  ((uint32_t)0x00800000)

// GPIO(照抄 SPL stm32f4xx_gpio.h 中用到的位定义)
#define GPIO_MODER_MODER0       ((uint32_t)0x00000003)
#define GPIO_OSPEEDER_OSPEEDR0  ((uint32_t)0x00000003)
#define GPIO_OTYPER_OT_0        ((uint32_t)0x00000001)
#define GPIO_PUPDR_PUPDR0       ((uint32_t)0x00000003)
#define GPIO_Mode_IN            ((uint32_t)0x00)
#define GPIO_Mode_OUT           ((uint32_t)0x01)
#define GPIO_Mode_AF            ((uint32_t)0x02)
#define GPIO_Mode_AN            ((uint32_t)0x03)

// ---------------------------------------------------------------------------------------------------
// 寄存器块
// ---------------------------------------------------------------------------------------------------
typedef struct
{
    volatile uint16_t SR1;   /* 0x00 */
    volatile uint16_t SR2;   /* 0x04 */
    volatile uint16_t CR1;   /* 0x08 */
    volatile uint16_t CR2;   /* 0x0C */
    volatile uint16_t DR;    /* 0x10 */
    volatile uint16_t CCR;   /* 0x14 */
    volatile uint16_t TRISE; /* 0x18 */
    volatile uint16_t OAR1;  /* 0x1C */
    volatile uint16_t OAR2;  /* 0x20 */

    // ---- 以下字段只是主机模型的观测点, 不是硬件寄存器(不参与 dlx_iic.hpp 的逻辑) ----
    bool     hmNackNextAddress;       ///< 测试注入: 下一次地址阶段从机 NACK(一次性)
    bool     hmAfBlocksNextTransfer;  ///< 建模开关: AF=1 时是否阻塞后续传输(见文件头 §27.3.7)
    bool     hmAwaitingAddress;       ///< 模型状态: START 已产生, 正在等地址写入 DR
    bool     hmTransmitter;           ///< 模型状态: 当前方向
    uint32_t hmInitCalls;             ///< I2C_Init 调用次数(recover() 必须重跑恰好一次)
    uint32_t hmClearFlagCalls;        ///< I2C_ClearFlag 调用次数(recover() 必须清恰好一次)
    uint32_t hmPeDisableCalls;        ///< I2C_Cmd(DISABLE) 调用次数(recover() 开头的 PE 关)
    /**
     * I2C_CheckEvent 调用次数 —— 度量"waitEvent 有没有空转"的唯一直接指标。
     * waitEvent 的上限是 kEventWaitLimit(200000), 有错误标志时驱动必须几轮内放弃,
     * 所以这个计数在一次失败事务里应该是**个位数**, 而不是 ~200000。
     */
    uint32_t hmCheckEventCalls;
} I2C_TypeDef;

typedef struct
{
    uint32_t I2C_ClockSpeed;
    uint16_t I2C_Mode;
    uint16_t I2C_DutyCycle;
    uint16_t I2C_OwnAddress1;
    uint16_t I2C_Ack;
    uint16_t I2C_AcknowledgedAddress;
} I2C_InitTypeDef;

// GPIO 寄存器块(偏移与真实 GPIO_TypeDef 一致)
typedef struct
{
    volatile uint32_t MODER;   /* 0x00 */
    volatile uint32_t OTYPER;  /* 0x04 */
    volatile uint32_t OSPEEDR; /* 0x08 */
    volatile uint32_t PUPDR;   /* 0x0C */
    volatile uint32_t IDR;     /* 0x10 */
    volatile uint32_t ODR;     /* 0x14 */
    volatile uint32_t BSRR;    /* 0x18 */
    volatile uint32_t LCKR;    /* 0x1C */
    volatile uint32_t AFR[2];  /* 0x20 */
} GPIO_TypeDef;

// 每个 GPIO 端口占 0x400 字节 —— dlx_gpio.hpp 用 "AHB1PERIPH_BASE + portIndex*0x400" 算地址,
// 所以存储必须按 0x400 对齐/间隔地铺开, 否则不同端口会互相踩。
// 注意类型: 真 stm32f4xx.h 里 AHB1PERIPH_BASE 是 32 位常量(目标机指针 32 位); 主机是 64 位,
// 用 32 位常量会被 -Wint-to-pointer-cast 抓住, 所以这里用 uintptr_t(与指针同宽)。
static uint8_t hmGpioBanks[11u * 0x400u] = {0};
#define AHB1PERIPH_BASE ((uintptr_t)hmGpioBanks)

// 寄存器块初值: 9 个寄存器清零; hmAfBlocksNextTransfer 默认 **true**
// (按 RM0090 §27.3.7 建模: AF 只能由软件写 0 清除, PE 关/开不会顺手清掉它)。
// 字段顺序见 I2C_TypeDef: SR1..OAR2, hmNackNextAddress, hmAfBlocksNextTransfer,
// hmAwaitingAddress, hmTransmitter, hmInitCalls, hmClearFlagCalls, hmPeDisableCalls,
// hmCheckEventCalls
static I2C_TypeDef hmI2cRegs[3] = {
    {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, false, true, false, false, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, false, true, false, false, 0u, 0u, 0u, 0u},
    {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, false, true, false, false, 0u, 0u, 0u, 0u},
};
#define I2C1 (&hmI2cRegs[0])
#define I2C2 (&hmI2cRegs[1])
#define I2C3 (&hmI2cRegs[2])

// ---------------------------------------------------------------------------------------------------
// SysTick / __NOP / SystemCoreClock —— 只为让**真实的** DL_LIB/dlx_delay.hpp 能编
//
// 为什么不能靠 test/iic_stub/dlx_delay.hpp 去 shadow: dlx_iic.hpp 里写的是
// `#include "dlx_delay.hpp"`, 而 GCC 的引号 include **先搜被包含文件所在目录**(DL_LIB/),
// 所以 DL_LIB/dlx_delay.hpp 一定排在 -I test/iic_stub 前面 —— 同目录同名头无法被 -I 顺序遮住。
// 因此这里补上真 dlx_delay.hpp 需要的最小符号: Ctrl 的 ENABLE 位保持 0, delay_ticks 会走
// "SysTick 未使能"的 __NOP 有界循环, 主机上既真实又不会死等。
// ---------------------------------------------------------------------------------------------------
typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} SysTick_Type;

static SysTick_Type hmSysTick = {0u, 0u, 0u, 0u};
#define SysTick (&hmSysTick)
#define SysTick_CTRL_ENABLE_Msk ((uint32_t)0x00000001)

static uint32_t SystemCoreClock = 168000000u; // 与 FLY 目标的 168MHz 一致(只影响空延时的循环次数)

#ifndef __NOP
#define __NOP() ((void)0)
#endif

// ---------------------------------------------------------------------------------------------------
// 模型内部小工具
// ---------------------------------------------------------------------------------------------------
inline bool hmI2cAfBlocks(const I2C_TypeDef *I2Cx)
{
    return I2Cx->hmAfBlocksNextTransfer && ((I2Cx->SR1 & I2C_SR1_AF) != 0u);
}

// ---------------------------------------------------------------------------------------------------
// SPL 函数替身
// ---------------------------------------------------------------------------------------------------
inline void RCC_APB1PeriphClockCmd(uint32_t periph, FunctionalState state)
{
    (void)periph;
    (void)state;
}

inline void RCC_AHB1PeriphClockCmd(uint32_t periph, FunctionalState state)
{
    (void)periph;
    (void)state;
}

inline void I2C_Init(I2C_TypeDef *I2Cx, I2C_InitTypeDef *init)
{
    ++I2Cx->hmInitCalls;
    // 照抄 SPL: 先关 PE, 写配置, 最后开 PE。SPL 的 I2C_Init 里没有任何清标志动作。
    I2Cx->CR1 &= (uint16_t)~I2C_CR1_PE;
    I2Cx->CR2    = (uint16_t)(init->I2C_ClockSpeed / 1000000u); // 模型只记录, 不参与判定
    I2Cx->CCR    = init->I2C_DutyCycle;
    I2Cx->TRISE  = 0u;
    I2Cx->OAR1   = init->I2C_OwnAddress1;
    I2Cx->OAR2   = 0u;
    I2Cx->CR1 |= I2C_CR1_PE;
    (void)init->I2C_Mode;
    (void)init->I2C_Ack;
    (void)init->I2C_AcknowledgedAddress;
}

inline void I2C_Cmd(I2C_TypeDef *I2Cx, FunctionalState state)
{
    if (state != ENABLE) {
        ++I2Cx->hmPeDisableCalls;
        I2Cx->CR1 &= (uint16_t)~I2C_CR1_PE;
        if (!I2Cx->hmAfBlocksNextTransfer) {
            // 反事实建模: 若 PE=0 会顺带清掉错误标志, 则 recover() 的 PE 关/开确实能复位总线。
            I2Cx->SR1 &= (uint16_t)~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);
        }
        // 默认建模假设(文件头已说明): PE=0 不清 SR1 的任何标志位。
        return;
    }
    I2Cx->CR1 |= I2C_CR1_PE;
}

inline void I2C_GenerateSTART(I2C_TypeDef *I2Cx, FunctionalState state)
{
    if (state != ENABLE) {
        I2Cx->CR1 &= (uint16_t)~I2C_CR1_START;
        return;
    }
    I2Cx->CR1 |= I2C_CR1_START;
    if ((I2Cx->CR1 & I2C_CR1_PE) == 0u) {
        return; // 外设没使能: 总线上什么都不会发生
    }
    if (hmI2cAfBlocks(I2Cx)) {
        // RM0090 §27.3.7: AF 必须由软件清 0, 否则本次 START 不产生 SB。
        // 这里刻意保留 CR1.START=1, 模拟"START 位挂住但事件永远不来"。
        return;
    }
    I2Cx->CR1 &= (uint16_t)~I2C_CR1_START; // START 已发出, 硬件自动清 START 位
    I2Cx->SR1 |= I2C_SR1_SB;
    I2Cx->SR2 |= (I2C_SR2_MSL | I2C_SR2_BUSY);
    I2Cx->hmAwaitingAddress = true;
}

inline void I2C_GenerateSTOP(I2C_TypeDef *I2Cx, FunctionalState state)
{
    if (state != ENABLE) {
        return;
    }
    I2Cx->CR1 |= I2C_CR1_STOP;
    // STOP 收尾: 清通信状态位, **但不动 AF/BERR/ARLO/OVR**(它们是写 0 清除的错误标志)
    I2Cx->SR1 &= (uint16_t)~(I2C_SR1_SB | I2C_SR1_ADDR | I2C_SR1_BTF | I2C_SR1_TXE | I2C_SR1_RXNE);
    I2Cx->SR2 &= (uint16_t)~(I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA);
    I2Cx->hmAwaitingAddress = false;
}

inline void I2C_Send7bitAddress(I2C_TypeDef *I2Cx, uint8_t address, uint8_t direction)
{
    if (direction == I2C_Direction_Transmitter) {
        address = (uint8_t)(address & (uint8_t)~I2C_Direction_Receiver);
    } else {
        address = (uint8_t)(address | I2C_Direction_Receiver);
    }
    I2Cx->DR = address;

    if (!I2Cx->hmAwaitingAddress || hmI2cAfBlocks(I2Cx)) {
        return; // 没有 START 在等地址, 或 AF 把总线卡住了: 写 DR 没有任何事件
    }
    I2Cx->hmAwaitingAddress = false;

    if (I2Cx->hmNackNextAddress) {
        I2Cx->hmNackNextAddress = false;
        // 从机 NACK 地址 -> AF=1(一次性注入, 之后 AF 只能靠软件清)
        I2Cx->SR1 |= I2C_SR1_AF;
        I2Cx->SR2 &= (uint16_t)~I2C_SR2_TRA;
        return;
    }

    I2Cx->SR1 |= I2C_SR1_ADDR;
    if (direction == I2C_Direction_Transmitter) {
        I2Cx->hmTransmitter = true;
        I2Cx->SR2 |= I2C_SR2_TRA;
        I2Cx->SR1 |= I2C_SR1_TXE;
    } else {
        I2Cx->hmTransmitter = false;
        I2Cx->SR2 &= (uint16_t)~I2C_SR2_TRA;
        // 模型简化: 认为从机的第一个字节已就绪(RXNE 置位), 让 receiveByte 的 waitEvent 可过
        I2Cx->SR1 |= I2C_SR1_RXNE;
    }
}

inline void I2C_SendData(I2C_TypeDef *I2Cx, uint8_t data)
{
    I2Cx->DR = data;
    if ((I2Cx->CR1 & I2C_CR1_PE) == 0u || hmI2cAfBlocks(I2Cx)) {
        return;
    }
    if (I2Cx->hmTransmitter) {
        I2Cx->SR1 |= (I2C_SR1_TXE | I2C_SR1_BTF);
        I2Cx->SR1 &= (uint16_t)~I2C_SR1_ADDR;
    }
}

inline uint32_t I2C_ReceiveData(I2C_TypeDef *I2Cx)
{
    const uint16_t data = I2Cx->DR;
    I2Cx->SR1 &= (uint16_t)~I2C_SR1_RXNE;
    // 只要 ACK 还开着且总线没被错误标志卡住, 从机会继续送下一个字节
    if (((I2Cx->CR1 & I2C_CR1_ACK) != 0u) && !hmI2cAfBlocks(I2Cx)) {
        I2Cx->SR1 |= I2C_SR1_RXNE;
    }
    return data;
}

inline void I2C_AcknowledgeConfig(I2C_TypeDef *I2Cx, FunctionalState state)
{
    if (state != ENABLE) {
        I2Cx->CR1 &= (uint16_t)~I2C_CR1_ACK;
        return;
    }
    I2Cx->CR1 |= I2C_CR1_ACK;
}

inline ErrorStatus I2C_CheckEvent(I2C_TypeDef *I2Cx, uint32_t event)
{
    ++I2Cx->hmCheckEventCalls; // "waitEvent 有没有空转"的唯一直接指标(见字段说明)
    // 照抄 SPL: 纯组合判据, 只读 SR1/SR2, 不产生任何副作用 —— 这也正是"轮询查不出
    // AF/BERR/ARLO"的原因: 事件掩码里根本不含这几个位。
    const uint32_t lastEvent = ((uint32_t)I2Cx->SR1 | (((uint32_t)I2Cx->SR2) << 16)) & 0x00FFFFFFu;
    return ((lastEvent & event) == event) ? SUCCESS : ERROR;
}

inline void I2C_ClearFlag(I2C_TypeDef *I2Cx, uint32_t flag)
{
    ++I2Cx->hmClearFlagCalls;
    /**
     * 与真实 SPL 的等价性(dlx_iic.hpp 的 recover() 就是用 I2C_ClearFlag 清 AF/BERR/ARLO/OVR 的):
     *
     *   SPL stm32f4xx_i2c.c: I2C_ClearFlag(I2Cx, flag) { flagpos = flag & FLAG_Mask; I2Cx->SR1 = (uint16_t)~flagpos; }
     *   即"把 flag 里的位写 0、其余位写 1"的那一次 16 位写; 而 SR1 上这些错误位是**写 0 清除**(RM0090
     *   §27.6.6): 写 0 的位清零, 写 1 的位保持不变(硬件不会因为写 1 而把标志置起来)。
     *
     *   本替身的 SR1 是普通变量, 不能照抄 `SR1 = ~flag`(那会把没在 flag 里的位**置 1**, 与硬件相反),
     *   所以这里用 `SR1 &= ~flag` —— 对"写 0 清除"的语义而言, 两者对这几个位**完全等价**:
     *     flag 内的位 -> 清 0(与 SPL 的写 0 一致);
     *     flag 外的位 -> 不变(与硬件"写 1 保持不变"一致)。
     *   本测试只用到 SR1 上的 AF/BERR/ARLO/OVR, 不含任何 SR2 标志, 所以不需要处理第二寄存器。
     */
    I2Cx->SR1 &= (uint16_t)~((uint16_t)flag);
}

// ---------------------------------------------------------------------------------------------------
// GPIO 替身(真实 dlx_gpio.hpp 会调用; 不 shadow 它, 所以这里必须给出可用的实现)
// ---------------------------------------------------------------------------------------------------
inline void GPIO_PinAFConfig(GPIO_TypeDef *GPIOx, uint16_t pinSource, uint8_t af)
{
    const uint32_t index = (uint32_t)(pinSource >> 3);
    const uint32_t shift = ((uint32_t)pinSource & 0x07u) * 4u;
    GPIOx->AFR[index] &= ~(0xFu << shift);
    GPIOx->AFR[index] |= ((uint32_t)af & 0xFu) << shift;
}

inline void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    GPIOx->ODR |= pin;
}

inline void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    GPIOx->ODR &= (uint32_t)~pin;
}

inline uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    // 开漏 + 外部上拉的简化: 输出高/未驱动时读回 1, 输出低时读回 0
    if ((GPIOx->ODR & pin) != 0u) {
        return 1u;
    }
    return (uint8_t)((GPIOx->IDR & pin) != 0u ? 1u : 0u);
}

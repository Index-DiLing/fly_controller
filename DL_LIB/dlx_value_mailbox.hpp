#pragma once
//===================================================================================================
// dlx_value_mailbox.hpp —— 定长值投递信箱(生产者: 任意线程/中断; 消费者: 当前 owner)
//
// 用在哪: 异常报文(DLXExceptionW)这种"定长、一次性、丢了就没了"的请求 —— 抛异常的地方可能在
//   中断里, 不能阻塞、更不能去动发送缓冲, 所以只允许往这里 push 一个值, 由 owner 统一成帧。
//
// 为什么不用队列类: 这里的语义就是"投递信箱"而不是"工作队列" ——
//   满的时候**丢新的**(而不是丢最老的、也不是阻塞): 停机现场里最近发生的异常不代表更重要的异常,
//   而且丢新的可以保证已经进去的那批一定被发出去, 计数也简单;
//   全满只留一个 overflowSeen 事实(`EXCEPTION_OVERFLOW` 那条提示), 与记录器的做法一致。
//
// 上下文安全:
//   共享状态(headIndex/tailIndex/storedCount)用 dlx::InterruptLock(CMSIS PRIMASK 的保存-恢复)保护:
//   单核上它对"中断 + 所有线程"一律互斥, 且可嵌套、析构负责恢复。
//
//   为什么**不能**用 rt_enter_critical(): 它只在修改调度锁计数器的几个周期里关中断, 返回时中断已经打开
//   (见 rt-thread/src/scheduler.c:921-936)—— 只挡线程切换, 不挡中断。异常可以从任意上下文抛出(包括
//   中断), 而 push 对 storedCount 是读-改-写, 与"中断里的 push"或"被打断的 take"都会丢更新。
//
//   因此本文件**不含任何 RT-Thread 依赖**: 只 include dlx_interrupt_lock.hpp(进而只依赖 CMSIS)。
//
//   关中断窗口: push/take 都只有十几条指令(<0.5µs @168MHz), 可以忽略。
//===================================================================================================
#include <stdint.h>

#include "dlx_interrupt_lock.hpp" // 中断屏蔽的保存-恢复(不含 RT-Thread 依赖)

namespace dlx
{

/**
 * @brief 定长值投递信箱
 *
 * @tparam ValueType 值类型(要求可平凡拷贝)
 * @tparam SlotCount 槽数(至少 1)
 */
template <typename ValueType, uint16_t SlotCount>
class ValueMailbox
{
public:
    ValueMailbox() = default;
    ValueMailbox(const ValueMailbox &) = delete;
    ValueMailbox &operator=(const ValueMailbox &) = delete;

    /**
     * @brief 生产者: 投一个值(任意上下文)
     *
     * @return true = 收下了; false = 满了(丢弃本值, 并置 overflowSeen)
     */
    bool push(const ValueType &value)
    {
        bool stored = false;
        {
            const InterruptLock lock; // 窗口: 写一个槽 + 两个下标/计数
            if (storedCount < SlotCount) {
                slots[tailIndex] = value;
                tailIndex = static_cast<uint16_t>((tailIndex + 1u) % SlotCount);
                ++storedCount;
                stored = true;
            } else {
                overflowSeen = true;
                ++droppedValueCount;
            }
        }
        return stored;
    }

    /**
     * @brief 消费者: 批量取走
     *
     * @param outValues 目的数组, 必须能放 maxCount 个
     * @param maxCount  本次最多取几个
     * @return 实际取走的个数
     */
    uint8_t take(ValueType *outValues, uint8_t maxCount)
    {
        if (outValues == nullptr || maxCount == 0u) {
            return 0u;
        }
        uint8_t takenCount = 0u;
        {
            const InterruptLock lock; // 窗口: 搬 maxCount 个值 + 两个下标/计数
            while (takenCount < maxCount && storedCount > 0u) {
                outValues[takenCount] = slots[headIndex];
                headIndex = static_cast<uint16_t>((headIndex + 1u) % SlotCount);
                --storedCount;
                ++takenCount;
            }
        }
        return takenCount;
    }

    /**
     * @brief 曾经丢过值吗
     *
     * 消费方在成帧时把这件事补成一条"溢出过"的提示, 然后 clearOverflow() —— 它是**事件**不是状态,
     * 不清掉就会每一轮都报一次。
     */
    bool overflowed() const { const InterruptLock lock; return overflowSeen; }
    void clearOverflow() { const InterruptLock lock; overflowSeen = false; }
    bool takeOverflow() { const InterruptLock lock; const bool seen = overflowSeen; overflowSeen = false; return seen; }

    /** 累计丢掉的个数(诊断用) */
    uint32_t droppedCount() const { const InterruptLock lock; return droppedValueCount; }

private:
    ValueType slots[SlotCount]{};
    uint16_t  headIndex{0};
    uint16_t  tailIndex{0};
    uint16_t  storedCount{0};

    bool     overflowSeen{false};
    uint32_t droppedValueCount{0};
};

} // namespace dlx

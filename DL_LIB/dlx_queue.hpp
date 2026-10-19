#pragma once

//===================================================================================================
// dlx_queue.hpp —— 固定外部存储的 SPSC / MPSC 队列
//
// 线程模型:
//   - 只有一个调用者可以 push();
//   - 只有一个调用者可以 pop()/peek()/clear();
//   - push 与 pop 可以并发, 不需要关中断或 RT-Thread 锁;
//   - push/pop 都是非阻塞的, 满/空时立即返回 false.
//
// 数据槽通过 tail(head) 的 release/acquire 发布。这样生产者写完槽内容后才发布 tail,
// 消费者回收槽内容后才发布 head, 因而下一个循环周期不会覆盖仍在读取的槽。
// MPSCQueue 使用 InterruptLock 串行化多个生产者, 只允许一个消费者。它适合小型事件/命令,
// 不适合大对象或高频大块数据; 这类数据应使用 SPSCQueue。
//
// 存储仍由调用方提供, 不进行动态分配。T 必须可平凡拷贝, 且底层 ByteBuffer 可以不对齐;
// 读写槽统一使用 memcpy。
//===================================================================================================

#include <atomic>
#include <stdint.h>
#include <string.h>
#include <type_traits>

#include "dlx_bytebuffer.hpp"
#include "dlx_interrupt_lock.hpp"

namespace dlx
{

// SPSC 可能被放在 ISR 与线程之间使用, 因而不能接受 std::atomic 的库锁回退。
// ARMClang/GCC 都提供这个编译期判断; 当前 Cortex-M4 目标上的 uint32_t 满足要求。
#if defined(__clang__) || defined(__GNUC__)
static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0),
              "SPSCQueue requires lock-free uint32_t atomics");
#endif

template <typename T>
class SPSCQueue
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "SPSCQueue<T>: T must be trivially copyable");

public:
    explicit SPSCQueue(ByteBuffer &buf)
        : buffer(buf)
        , capacity_(static_cast<uint32_t>(buf.len / sizeof(T)))
        , head_(0u)
        , tail_(0u)
    {
    }

    SPSCQueue(const SPSCQueue &) = delete;
    SPSCQueue &operator=(const SPSCQueue &) = delete;

    /** @brief 生产者接口: 满时返回 false, 不覆盖已有元素。 */
    bool push(const T &item)
    {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        const uint32_t head = head_.load(std::memory_order_acquire);
        if (capacity_ == 0u || (tail - head) >= capacity_) {
            return false;
        }

        const uint32_t slot = tail % capacity_;
        memcpy(buffer.src + static_cast<size_t>(slot) * sizeof(T), &item, sizeof(T));
        // 先发布槽内容, 再发布 tail; 消费者 acquire tail 后才能读取该槽。
        tail_.store(tail + 1u, std::memory_order_release);
        return true;
    }

    /** @brief 消费者接口: 空时返回 false。 */
    bool pop(T &out)
    {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t tail = tail_.load(std::memory_order_acquire);
        if (head == tail) {
            return false;
        }

        const uint32_t slot = head % capacity_;
        memcpy(&out, buffer.src + static_cast<size_t>(slot) * sizeof(T), sizeof(T));
        // 先完成读取, 再归还槽; 生产者 acquire head 后才能复用该槽。
        head_.store(head + 1u, std::memory_order_release);
        return true;
    }

    /** @brief 消费者接口: 查看队首但不移出。 */
    bool peek(T &out) const
    {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t tail = tail_.load(std::memory_order_acquire);
        if (head == tail) {
            return false;
        }

        const uint32_t slot = head % capacity_;
        memcpy(&out, buffer.src + static_cast<size_t>(slot) * sizeof(T), sizeof(T));
        return true;
    }

    /**
     * @brief 丢弃当前已有元素。
     *
     * 这是消费者操作。它只把 head 推到当前已发布的 tail, 因而可以和生产者并发;
     * 在调用期间已经发布的元素会被丢弃, 调用之后新发布的元素仍会保留。
     */
    void clear()
    {
        const uint32_t tail = tail_.load(std::memory_order_acquire);
        head_.store(tail, std::memory_order_release);
    }

    /** @brief 当前元素数; 并发时是某一瞬间的近似快照。 */
    uint16_t size() const
    {
        const uint32_t tail = tail_.load(std::memory_order_acquire);
        const uint32_t head = head_.load(std::memory_order_acquire);
        const uint32_t used = tail - head;
        return static_cast<uint16_t>((used > capacity_) ? capacity_ : used);
    }

    uint16_t capacity() const
    {
        return static_cast<uint16_t>((capacity_ > 0xFFFFu) ? 0xFFFFu : capacity_);
    }

    uint16_t remaining() const
    {
        const uint32_t used = static_cast<uint32_t>(size());
        return static_cast<uint16_t>(capacity() - used);
    }

    bool isEmpty() const { return size() == 0u; }
    bool isFull() const { return capacity_ != 0u && size() >= capacity(); }

    // 显式命名的别名, 便于新代码表达"非阻塞尝试"语义; 原 push/pop 保留兼容性。
    bool tryPush(const T &item) { return push(item); }
    bool tryPop(T &out) { return pop(out); }

private:
    ByteBuffer &buffer;
    const uint32_t capacity_;

    // 每个索引只有对应的一方写入: producer 写 tail, consumer 写 head。
    // 32 位原子在目标 Cortex-M4 上应为 lock-free, 否则不满足 ISR/线程 SPSC 的实时约束。
    std::atomic<uint32_t> head_;
    std::atomic<uint32_t> tail_;
};

//===================================================================================================
// 多生产者/单消费者队列
//
// 这里故意使用 InterruptLock 而不是 CAS:
//   1. 生产者可能来自普通线程, 也可能来自 ISR;
//   2. Cortex-M4 上中断屏蔽能同时保护所有生产者和唯一消费者;
//   3. 队列用于小型事件/命令时, 一次固定槽 memcpy 的时间是有界的, 规则更容易验证。
//
// 所有接口都在短临界区内完成, 不会阻塞。生产者数量不限, 消费者必须只有一个。
//===================================================================================================
template <typename T>
class MPSCQueue
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "MPSCQueue<T>: T must be trivially copyable");

public:
    explicit MPSCQueue(ByteBuffer &buf)
        : buffer(buf)
        , capacity_(static_cast<uint16_t>(buf.len / sizeof(T)))
    {
    }

    MPSCQueue(const MPSCQueue &) = delete;
    MPSCQueue &operator=(const MPSCQueue &) = delete;

    /** @brief 任意生产者调用; 满时丢弃新值并返回 false。 */
    bool push(const T &item)
    {
        const InterruptLock lock;
        if (capacity_ == 0u || count_ >= capacity_) {
            ++dropped_;
            return false;
        }

        const uint16_t tail = static_cast<uint16_t>((head_ + count_) % capacity_);
        memcpy(buffer.src + static_cast<size_t>(tail) * sizeof(T), &item, sizeof(T));
        ++count_;
        return true;
    }

    /** @brief 唯一消费者调用; 空时返回 false。 */
    bool pop(T &out)
    {
        const InterruptLock lock;
        if (count_ == 0u) {
            return false;
        }

        memcpy(&out, buffer.src + static_cast<size_t>(head_) * sizeof(T), sizeof(T));
        head_ = static_cast<uint16_t>((head_ + 1u) % capacity_);
        --count_;
        return true;
    }

    /** @brief 唯一消费者调用; 查看队首但不移出。 */
    bool peek(T &out) const
    {
        const InterruptLock lock;
        if (count_ == 0u) {
            return false;
        }

        memcpy(&out, buffer.src + static_cast<size_t>(head_) * sizeof(T), sizeof(T));
        return true;
    }

    /** @brief 消费者操作: 丢弃当前已有元素。 */
    void clear()
    {
        const InterruptLock lock;
        head_  = 0u;
        count_ = 0u;
    }

    uint16_t size() const
    {
        const InterruptLock lock;
        return count_;
    }

    uint16_t capacity() const { return capacity_; }

    uint16_t remaining() const
    {
        const InterruptLock lock;
        return static_cast<uint16_t>(capacity_ - count_);
    }

    bool isEmpty() const
    {
        const InterruptLock lock;
        return count_ == 0u;
    }

    bool isFull() const
    {
        const InterruptLock lock;
        return capacity_ != 0u && count_ >= capacity_;
    }

    /** @brief 已因队列满而丢弃的新值数量; 读取本身也是线程/ISR 安全的。 */
    uint32_t dropped() const
    {
        const InterruptLock lock;
        return dropped_;
    }

    bool tryPush(const T &item) { return push(item); }
    bool tryPop(T &out) { return pop(out); }

private:
    ByteBuffer &buffer;
    const uint16_t capacity_;
    uint16_t       head_{0u};
    uint16_t       count_{0u};
    uint32_t       dropped_{0u};
};

/**
 * 旧名称兼容别名。
 *
 * Queue 过去是未保护的裸环形队列; 从现在开始它明确表示 SPSCQueue。
 * 多生产者/多消费者代码不得继续使用这个别名假定安全。
 */
template <typename T>
using Queue = SPSCQueue<T>;

} // namespace dlx

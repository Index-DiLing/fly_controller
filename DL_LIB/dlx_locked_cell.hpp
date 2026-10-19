#pragma once

//===================================================================================================
// dlx_locked_cell.hpp —— 用 InterruptLock 保护的小型共享值
//
// 适用:
//   - ISR/线程或多个线程需要读写同一份小型状态;
//   - T 可平凡拷贝, 拷贝时间有界;
//   - 操作必须立即完成, 不允许在 update() 回调里阻塞或调用外部慢路径。
//
// 不适用:
//   - 大对象、大块缓冲区或硬件访问;
//   - 需要等待资源的操作;
//   - 多消费者 FIFO(请使用队列)。
//
// 这个类型把锁策略封装在数据结构里, 避免调用方忘记给某个字段加锁。它不依赖 RT-Thread,
// 但依赖现有的 CMSIS InterruptLock, 因而可从线程和中断上下文使用。
//===================================================================================================

#include <string.h>
#include <type_traits>

#include "dlx_interrupt_lock.hpp"

namespace dlx
{

template <typename T>
class LockedCell
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "LockedCell<T>: T must be trivially copyable");

public:
    LockedCell()
        : value{}
    {
    }

    explicit LockedCell(const T &initial)
        : value(initial)
    {
    }

    LockedCell(const LockedCell &) = delete;
    LockedCell &operator=(const LockedCell &) = delete;

    /** @brief 在锁内复制出一份值。 */
    T load() const
    {
        T out{};
        const InterruptLock lock;
        memcpy(&out, &value, sizeof(T));
        return out;
    }

    /** @brief 在锁内替换值。 */
    void store(const T &next)
    {
        const InterruptLock lock;
        memcpy(&value, &next, sizeof(T));
    }

    /** @brief 在锁内替换值并返回旧值。 */
    T exchange(const T &next)
    {
        T old{};
        const InterruptLock lock;
        memcpy(&old, &value, sizeof(T));
        memcpy(&value, &next, sizeof(T));
        return old;
    }

    /**
     * @brief 在锁内做一次短小的读-改-写。
     *
     * 回调不得阻塞、访问慢速外设、调用可能再次等待该资源的函数, 否则会把这些操作放进关中断窗口。
     */
    template <typename Function>
    void update(Function function)
    {
        const InterruptLock lock;
        function(value);
    }

private:
    T value;
};

} // namespace dlx

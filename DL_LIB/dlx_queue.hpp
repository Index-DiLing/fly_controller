#pragma once

#include <stdint.h>
#include <string.h>
#include <type_traits>
#include "dlx_bytebuffer.hpp"

namespace dlx
{
    /**
     * @brief 定长队列模板
     *
     * 底层内存由外部传入的 ByteBuffer 管理(在构造函数中传入),
     * 本类只维护读写下标与元素个数, 不负责分配/释放.
     * 元素按字节拷贝, 不要求对齐, 但 T 必须可平凡拷贝(trivially copyable).
     *
     * @tparam T 元素类型, 如 AccelerometerRaw / GyroscopeRaw / uint8_t ...
     */
    template <typename T>
    class Queue
    {
        static_assert(std::is_trivially_copyable<T>::value,
                      "Queue<T>: T 必须可平凡拷贝(trivially copyable)");

    private:
        ByteBuffer &buffer; // 底层内存(构造时传入)
        uint16_t head;      // 队首元素下标
        uint16_t count;     // 当前元素个数
        uint16_t cap;       // 容量(元素个数) = buffer.len / sizeof(T)

    public:
        explicit Queue(ByteBuffer &buf)
            : buffer(buf), head(0), count(0), cap(buf.len / sizeof(T))
        {
        }

        /** @brief 入队, 队列满则返回 false */
        bool push(const T &item)
        {
            if (cap == 0 || count >= cap) {
                return false;
            }
            const uint16_t tail = (head + count) % cap;
            memcpy(buffer.src + (size_t)tail * sizeof(T), &item, sizeof(T));
            ++count;
            return true;
        }

        /** @brief 出队, 队列空则返回 false */
        bool pop(T &out)
        {
            if (count == 0) {
                return false;
            }
            memcpy(&out, buffer.src + (size_t)head * sizeof(T), sizeof(T));
            head = (head + 1) % cap;
            --count;
            return true;
        }

        /** @brief 查看队首元素, 不移出队列 */
        bool peek(T &out) const
        {
            if (count == 0) {
                return false;
            }
            memcpy(&out, buffer.src + (size_t)head * sizeof(T), sizeof(T));
            return true;
        }

        /** @brief 清空队列(不释放内存) */
        void clear()
        {
            head  = 0;
            count = 0;
        }

        uint16_t size() const { return count; }
        uint16_t capacity() const { return cap; }
        uint16_t remaining() const { return cap - count; }
        bool isEmpty() const { return count == 0; }
        bool isFull() const { return count == cap; }
    };
} // namespace dlx

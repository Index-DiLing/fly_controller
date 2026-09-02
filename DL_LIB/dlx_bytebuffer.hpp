#pragma once
#include <string.h>
#include <type_traits>
namespace dlx
{
    /***
     * @note ByteBuffer不负责任何安全处理问题,除了静态断言!
     */
    struct ByteBuffer {
    private:
        template <typename T>
        inline T bswap(T val)
        {
            static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8,
                          "Unsupported size for byteswap");
            if (sizeof(T) == 1) { return val; }
            if (sizeof(T) == 2) {
                uint16_t v;
                memcpy(&v, &val, 2);
                v = __builtin_bswap16(v);
                memcpy(&val, &v, 2);
            } else if (sizeof(T) == 4) {
                uint32_t v;
                memcpy(&v, &val, 4);
                v = __builtin_bswap32(v);
                memcpy(&val, &v, 4);
            } else if (sizeof(T) == 8) {
                uint64_t v;
                memcpy(&v, &val, 8);
                v = __builtin_bswap64(v);
                memcpy(&val, &v, 8);
            }
            return val;
        }

    public:
        uint16_t len; // 长度,指的是分配长度,当前当都由cur-src得到
        uint8_t *cur; // 当前指针,可修改
        uint8_t *src; // 原始指针,不修改,是规定const的,但为了避免转换,不使用const
        inline uint16_t used()
        {
            return cur - src;
        }
        inline uint16_t remaining()
        {
            return len - (cur - src);
        }

        ByteBuffer(uint8_t *buffer, uint16_t bufferSize)
        {
            this->src = buffer;
            this->cur = buffer;
            this->len = bufferSize;
        }

        ByteBuffer &reset()
        {
            cur = src;
            return *this;
        }
        ByteBuffer &operator!()
        {
            return reset();
        }

        template <typename T>
        T read_le()
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "MessageManager T must be trivially copyable");
            T value;
            memcpy(&value, cur, sizeof(T));
            cur += sizeof(T);
            return value;
        }

        template <typename T>
        T read()
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "MessageManager T must be trivially copyable");
            T value;
            memcpy(&value, cur, sizeof(T));
            cur += sizeof(T);
            value = bswap(value);
            return value;
        }

        template <typename T>
        T read_le_at(uint16_t offset)
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "MessageManager T must be trivially copyable");
            T value;
            memcpy(&value, src+offset, sizeof(T));
            return value;
        }

        template <typename T>
        T read_at(uint16_t offset)
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "MessageManager T must be trivially copyable");
            T value;
            memcpy(&value, src+offset, sizeof(T));
            value = bswap(value);
            return value;
        }


        template <typename T>
        void write_le(T value)
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "ByteBuffer write requires trivially copyable type");
            memcpy(cur, &value, sizeof(T));
            cur += sizeof(T);
            return;
        }

        template <typename T>
        void write(T value)
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "MessageManager T must be trivially copyable");
            value = bswap(value);
            memcpy(cur, &value, sizeof(T));
            cur += sizeof(T);
            return;
        }

        void write(uint8_t *data, uint16_t len)
        {
            memcpy(cur, data, len);
            cur += len;
            return;
        }
        void write(const uint8_t *data, uint16_t len)
        {
            memcpy(cur, data, len);
            cur += len;
            return;
        }

        //这里可能存在对齐引发的崩溃,如果没对齐可能会出事.
        template <typename T>
        T *toArray(uint16_t *size)const
        {
            // static_assert(len%sizeof(T)==0,"ByteBuffer存在浪费内存的的类型转换")
            if (size != nullptr) {

                *size = len / sizeof(T);
            }

            return (T *)src;
        }

        ByteBuffer slice(uint16_t size){
            return ByteBuffer(src,size);
        }
    };



struct RingByteBuffer {
private:
    uint8_t* data;
    uint16_t maxLen;
    uint16_t head = 0;
    uint16_t tail = 0;
    uint16_t count = 0;
    bool allowOverwrite = false;

public:
    RingByteBuffer(ByteBuffer& buffer)
        : data(buffer.src), maxLen(buffer.len) {}

    void setOverwrite(bool enable) { allowOverwrite = enable; }

    // ---------- 单字节读写 ----------
    bool write(uint8_t b) {
        if (count == maxLen) {
            if (!allowOverwrite) return false;
            head = (head + 1) % maxLen;
            --count;
        }
        data[tail] = b;
        tail = (tail + 1) % maxLen;
        ++count;
        return true;
    }

    bool read(uint8_t& b) {
        if (count == 0) return false;
        b = data[head];
        head = (head + 1) % maxLen;
        --count;
        return true;
    }

    // ---------- 批量读写 ----------
    bool write(const uint8_t* src, uint16_t len) {
        if (len > maxLen) return false;
        if (len > maxLen - count) {
            if (!allowOverwrite) return false;
            uint16_t needDiscard = len - (maxLen - count);
            head = (head + needDiscard) % maxLen;
            count -= needDiscard;
        }
        uint16_t first = maxLen - tail;
        if (first > len) first = len;
        memcpy(data + tail, src, first);
        tail = (tail + first) % maxLen;
        if (len > first) {
            memcpy(data + tail, src + first, len - first);
            tail = (tail + (len - first)) % maxLen;
        }
        count += len;
        return true;
    }

    bool read(uint8_t* dst, uint16_t len) {
        if (len > count) return false;
        uint16_t first = maxLen - head;
        if (first > len) first = len;
        memcpy(dst, data + head, first);
        head = (head + first) % maxLen;
        if (len > first) {
            memcpy(dst + first, data + head, len - first);
            head = (head + (len - first)) % maxLen;
        }
        count -= len;
        return true;
    }

    // ---------- Peek 功能（只读，不移动读指针） ----------
    // 查看单个字节
    bool peek(uint8_t& b) const {
        if (count == 0) return false;
        b = data[head];
        return true;
    }

    // 查看指定长度的数据到 dst，不消耗数据
    bool peek(uint8_t* dst, uint16_t len) const {
        if (len > count) return false;
        uint16_t h = head;               // 用局部变量模拟读取过程
        uint16_t first = maxLen - h;
        if (first > len) first = len;
        memcpy(dst, data + h, first);
        h = (h + first) % maxLen;
        if (len > first) {
            memcpy(dst + first, data + h, len - first);
            // h 不再需要更新，因为只是查看
        }
        return true;
    }

    // 查看一个小端序数值,不检查可用字节数(由调用方保证,如断言 remaining/available)
    template <typename T>
    T peek_le() const {
        static_assert(std::is_trivially_copyable<T>::value,
                      "T must be trivially copyable");
        uint8_t tmp[sizeof(T)];
        peek(tmp, sizeof(T));
        T value;
        memcpy(&value, tmp, sizeof(T));
        return value;
    }

    // 查看一个大端序数值,不检查可用字节数(由调用方保证)
    template <typename T>
    T peek_be() const {
        static_assert(std::is_trivially_copyable<T>::value,
                      "T must be trivially copyable");
        // 对多字节类型交换字节序
        return bswap(peek_le<T>());
    }

    // ---------- Skip 功能（直接丢弃，不拷贝） ----------
    // 跳过指定长度的数据,不检查可用字节数(由调用方保证 count >= len)
    void skip(uint16_t len) {
        head = (head + len) % maxLen;
        count -= len;
    }

    // ---------- 其他公共接口 ----------
    uint16_t available() const { return count; }
    uint16_t remaining() const { return maxLen - count; }
    bool isEmpty() const { return count == 0; }
    bool isFull() const { return count == maxLen; }
    void reset() { head = tail = count = 0; }

    template <typename T>
    T read_le() {
        static_assert(std::is_trivially_copyable<T>::value,
                      "T must be trivially copyable");
        T value{};
        if (count < sizeof(T)) return value;
        uint8_t tmp[sizeof(T)];

        if (!read(tmp, sizeof(T))) return value;

        memcpy(&value, tmp, sizeof(T));

        return value;
    }
    template <typename T>
    bool write_le(const T& value) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "T must be trivially copyable");
        uint8_t tmp[sizeof(T)];
        memcpy(tmp, &value, sizeof(T));
        return write(tmp, sizeof(T));
    }
    template <typename T>
    T read_be() {
        static_assert(std::is_trivially_copyable<T>::value,
                      "T must be trivially copyable");
        T value = read_le<T>();
        value = bswap(value);
        return value;
    }
    template <typename T>
    bool write_be(const T& value) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "T must be trivially copyable");
        T swapped = bswap(value);
        return write_le(swapped);
    }

private:
    template <typename T>
    static inline T bswap(T val) {
        static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8,
                      "Unsupported size for byteswap");
        if (sizeof(T) == 1) return val;
        if (sizeof(T) == 2) {
            uint16_t v;
            memcpy(&v, &val, 2);
            v = __builtin_bswap16(v);
            memcpy(&val, &v, 2);
        } else if (sizeof(T) == 4) {
            uint32_t v;
            memcpy(&v, &val, 4);
            v = __builtin_bswap32(v);
            memcpy(&val, &v, 4);
        } else if (sizeof(T) == 8) {
            uint64_t v;
            memcpy(&v, &val, 8);
            v = __builtin_bswap64(v);
            memcpy(&val, &v, 8);
        }
        return val;
    }
};


    template <int SIZE>
    class StaticByteBufferAllocator // 一次性栈分配
    {
    private:
        uint8_t pool[SIZE];
        uint16_t current = 0;
        StaticByteBufferAllocator(/* args */)
        {

        }
        ~StaticByteBufferAllocator() = default;

    public:
        static StaticByteBufferAllocator &instance()
        {
            static StaticByteBufferAllocator ins;
            return ins;
        }

        uint16_t getFreeSize()
        {
            return SIZE - current;
        }

        /**
         * @brief 静态分配一片内存,不回收,用于长期存储段
         * @param size 分配的内存字节数
         * @return ByteBuffer 如果程序在此卡死(超额)则直接进入死循环.
         */
        
        ByteBuffer allocate(const uint16_t size)
        {
            if (size > getFreeSize()) {
                while (true);
            }
            
            current += size;
            return ByteBuffer(pool + current, size);
        }
    };
} // namespace dl

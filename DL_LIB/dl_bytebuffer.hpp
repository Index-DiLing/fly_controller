#pragma once
#include "stm32f4xx.h"
#include <memory>
namespace dl
{
    struct ByteBuffer {
    private:
    
        uint16_t len; // 长度,指的是分配长度,当前当都由cur-src得到
        template <typename T>
        inline T bswap(T val)
        {
            static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8,
                          "Unsupported size for byteswap");
            if (sizeof(T) == 1) { return val; }
            if (sizeof(T) == 2) {
                uint16_t v;
                std::memcpy(&v, &val, 2);
                v = __builtin_bswap16(v);
                std::memcpy(&val, &v, 2);
            } else if (sizeof(T) == 4) {
                uint32_t v;
                std::memcpy(&v, &val, 4);
                v = __builtin_bswap32(v);
                std::memcpy(&val, &v, 4);
            } else if (sizeof(T) == 8) {
                uint64_t v;
                std::memcpy(&v, &val, 8);
                v = __builtin_bswap64(v);
                std::memcpy(&val, &v, 8);
            }
            return val;
        }
    
    public:
        uint8_t *cur; // 当前指针,可修改
        uint8_t *src; // 原始指针,不修改
        inline uint16_t length()
        {
            return cur - src;
        }
        inline uint16_t remaining(){
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
            std::memcpy(&value, cur, sizeof(T));
            cur += sizeof(T);
            return value;
        }

        template <typename T>
        T read()
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "MessageManager T must be trivially copyable");
            T value;
            std::memcpy(&value, cur, sizeof(T));
            cur += sizeof(T);
            value = bswap(value);
            return value;
        }

        template <typename T>
        void write_le(T value)
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "ByteBuffer write requires trivially copyable type");
            std::memcpy(cur, &value, sizeof(T));
            cur += sizeof(T);
            return;
        }

        template <typename T>
        void write(T value)
        {
            static_assert(std::is_trivially_copyable<T>::value,
                          "MessageManager T must be trivially copyable");
            value = bswap(value);
            std::memcpy(cur, &value, sizeof(T));
            cur += sizeof(T);
            return;
        }

        void write(uint8_t *data, uint16_t len)
        {
            std::memcpy(cur, data, len);
            cur += len;
            return;
        }
    };
} // namespace dl

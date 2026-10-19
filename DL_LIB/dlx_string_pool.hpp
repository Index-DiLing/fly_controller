#pragma once

#include <stdint.h>
#include <stdarg.h> // va_list
#include <stdio.h>  // vsnprintf(与工程其余部分一致: 走 newlib)
#include <string.h>

#include "dlx_bytebuffer.hpp"

namespace dlx
{
    class StringPool
    {
    private:
        // buffer 平凡可拷贝, StringPool 本身也是缓冲区, 外部没必要修改 buffer,
        // 但外部需要保证 buffer 可用, 因此保留引用语义而不是拷贝。对外声明: 不持有数据。
        // 本身实质是对 buffer 的一个包装和裁剪。
        ByteBuffer &buffer;

    public:
        explicit StringPool(ByteBuffer &buffer)
            : buffer(buffer)
        {
        }

        // 池是"某块内存的视图", 复制它只会得到两个指向同一块内存的同名对象, 容易误判成"各有一份"。
        StringPool(const StringPool &)            = delete;
        StringPool &operator=(const StringPool &) = delete;

        // 析构: 不持有数据, 无事可做 ⇒ 用编译器生成的即可(手写空的反而会抑制隐式移动)。

        //-------------------------------------------------------------------------------------------
        // 容量查询(转发到 buffer, 免得调用方伸手进去拿 ByteBuffer)
        //-------------------------------------------------------------------------------------------
        uint16_t used() const { return buffer.used(); }
        uint16_t remaining() const { return buffer.remaining(); }
        uint16_t capacity() const { return buffer.len; } ///< 分配长度(不是已用长度)
        bool     empty() const { return buffer.used() == 0u; }

        /** 已攒内容的只读指针(别名, 见文件头 ①) */
        const uint8_t *data() const { return buffer.src; }


        bool write(const char *str)
        {
            if (str == nullptr) {
                return false;
            }
            const size_t n = strlen(str);
            return write(str, static_cast<uint16_t>(n));
        }

        bool write(const char *str, uint16_t len)
        {
            return write(reinterpret_cast<const uint8_t *>(str), len);
        }

        /** 追加一段定长字节(日志负载常常不是 C 串, 用这个) */
        bool write(const uint8_t *bytes, uint16_t len)
        {
            if (len == 0u) {
                return true; 
            }
            if (bytes == nullptr || buffer.remaining() < len) {
                return false; // 整条放弃, 不做部分写入
            }
            buffer.write(bytes, len);
            return true;
        }


        // 注意: format 属性必须挂在**定义**这一处, 不能另起一行只写前置声明。
        // 写成"声明带属性 + 定义不带"时 GCC 会报 "cannot be overloaded with"(AC6 容忍, 但主机侧测试就编不过了)。
        int writef(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
        {
            const uint16_t room = buffer.remaining();
            if (fmt == nullptr || room == 0u) {
                return 0;
            }
            // vsnprintf 最多写 room-1 个字符 + 结尾 '\0', 正好给它留出放结尾的位置;
            // 而 cur 只前进"真正的字符数", 那个 '\0' 留在缓冲里等下一次 write 覆盖它。
            va_list args;
            va_start(args, fmt);
            const int n = vsnprintf(reinterpret_cast<char *>(buffer.cur), room, fmt, args);
            va_end(args);
            if (n <= 0) {
                return 0;
            }
            const uint16_t written = (static_cast<size_t>(n) < static_cast<size_t>(room))
                                         ? static_cast<uint16_t>(n)
                                         : static_cast<uint16_t>(room - 1u);
            buffer.cur += written;
            return static_cast<int>(written);
        }

        ByteBuffer view() { return buffer.slice(buffer.used()); }

        void reset() { buffer.reset(); }
    };

} // namespace dlx

#pragma once
//===================================================================================================
// DL_LIB/link/dlx_bits.hpp —— 链路帧的位打包/解包小工具(纯逻辑, 无硬件依赖)
//
// 职责(只有四件, 多了不干):
//   1. putBits/getBits            往字节缓冲的**任意位偏移**写/读 n 位(1..32), 允许跨字节;
//   2. signExtend/putSignedBits/getSignedBits  n 位补码 <-> int32, 手工符号扩展;
//   3. quantizeSat/dequantize/normalizedSigned 定点 <-> 浮点, **饱和**不回绕;
//   4. 越界防护                   位偏移/位宽越界时既不读也不写(只计数上报), 编译期另有守卫。
//
// ---- 位序约定(与 §3.3 帧表配套; 全仓库只在这里定义一次) ----
//   位偏移从**缓冲区第 0 字节的 bit0** 起连续编号, 先放的字段占**低位**。
//   字段 f 在偏移 o、宽 n 时, 它的第 i 位(0 = LSB)落在字节 buf[(o+i)>>3] 的 bit ((o+i)&7)。
//   等价说法: 把负载当成一个小端大整数(第 0 字节是低 8 位), 字段从低位往高位塞。
//
//   为什么不按 ARM 位域默认的"大端位序"(bit31 先): 帧里最长的字段是 int32
//   (lat/lon/altMslMm), 大端位序会让它们的字节序与 MCU 内存里的 int32 相反, 每次收发都要
//   逐字节拆装; 小端位序下"32 位且字节对齐"的字段就等于一次 memcpy, 而且与 §3.3 表里
//   "从左到右相加"的阅读顺序一致(左边 = 低位)。依据: §3.2 帧格式 / §3.3 帧表。
//
//   为什么不用 C 位域(struct { uint32_t a : 12; }): 位域布局是 implementation-defined
//   (字节内先低还是先高、跨不跨存储单元、能不能跨), AC6 与 g++ 都不保证一致; 而且它给不了
//   "任意位偏移"的读写能力。协议解析必须自己拿住位序, 这是能同时满足"两端一致 + 主机可测"的
//   唯一写法。依据: 本库要在 armclang AC6 与 g++ 11.2 上编出**同一份**线上格式。
//
//   为什么用逐位循环而不是"掩码 + 移位"的快路径: 掩码版要按 (offset&7) 把 n 位拆成
//   1~3 段, 还要特判 n=32(1u<<32 是 UB)与 offset 恰好字节对齐的退化情形 —— 分支比循环还多。
//   链路线程是 20Hz 量级、每帧 ≤20B, 逐位循环几十个周期, 换来确定性, 值。
//
//   C++11 / 无异常 / 无 RTTI / 无动态分配 / header-only。
//===================================================================================================

#include <stdint.h>

namespace dlx
{
namespace link
{
namespace bits
{
    //===============================================================================================
    // 一、越界防护
    //===============================================================================================
    /** 位偏移越界计数(只增不减, 不 abort)。
     *
     * 为什么不 assert(): 飞控上 assert 失败就是停机, 在飞行中比"少写一个字段"危险得多; 这里退化成
     *   "不写/读 0 + 计数", 由调用方决定是闪码、记 ExceptionCode 还是忽略。
     * 为什么不用文件级全局变量: 头文件里写 static 全局, 每个编译单元各持一份, 计数永远看不全;
     *   C++11 又没有 inline 变量。inline 函数内的静态量是**唯一**实体, 且常量初始化(POD + 常量初值),
     *   不会生成线程安全的 guard 变量 —— 在 RT-Thread 多线程下也安全。 */
    inline uint32_t &violationCount()
    {
        static uint32_t n = 0u;
        return n;
    }

    /** 清空越界计数(主机单测在每个用例前调用, 才能断言"这一小段没越界") */
    inline void clearViolations()
    {
        violationCount() = 0u;
    }

    /** 位宽是否合法: 1..32。0 位字段没有语义, 静默放过会让"宽度写错"退化成"字段恒 0" */
    constexpr bool widthOk(uint8_t n)
    {
        return n >= 1u && n <= 32u;
    }

    /** 位偏移 + 位宽是否落在 [0, byteLen*8] 内; 越界则计数并返回 false(调用方据此不读不写) */
    inline bool rangeOk(uint16_t byteLen, uint16_t bitOffset, uint8_t n)
    {
        if (!widthOk(n)) {
            ++violationCount();
            return false;
        }
        // 用 32 位算: byteLen 最大 65535 ⇒ byteLen*8 会溢出 16 位
        if ((uint32_t)bitOffset + (uint32_t)n > (uint32_t)byteLen * 8u) {
            ++violationCount();
            return false;
        }
        return true;
    }

    /** 位宽合计 -> 字节数(向上取整)。帧长表就是拿它把"按位相加"变成"字节数"的 */
    constexpr uint16_t bytesForBits(uint16_t totalBits)
    {
        return (uint16_t)((totalBits + 7u) / 8u);
    }

    /** 整段清零(整帧编码的第一步)。
     *  为什么必须清: 位域之外的空位(如 DL_STATE 的 7 个空位)如果带着缓冲区里的旧字节进入 CRC,
     *  同一份负载两次编码会得到不同字节, "重传同一帧"与"自检比对"就都失去意义了。 */
    inline void clearBytes(uint8_t *buf, uint16_t len)
    {
        for (uint16_t i = 0u; i < len; ++i) {
            buf[i] = 0u;
        }
    }

    //===============================================================================================
    // 二、无符号位读写(允许跨字节)
    //===============================================================================================
    /** 把 value 的低 n 位写到 buf 的 bitOffset 处(小端位序)。
     *  out-of-range 时**整段不写** —— 部分写会让缓冲区落在"半新半旧"的状态, 比不写更难查。 */
    inline void putBits(uint8_t *buf, uint16_t byteLen, uint16_t bitOffset, uint32_t value, uint8_t n)
    {
        if (buf == 0 || !rangeOk(byteLen, bitOffset, n)) {
            return;
        }
        for (uint8_t i = 0u; i < n; ++i) {
            const uint32_t idx  = (uint32_t)bitOffset + (uint32_t)i; // 32 位算, 见 rangeOk 注释
            uint8_t       &cell = buf[idx >> 3];
            const uint8_t  mask = (uint8_t)(1u << (idx & 7u));
            if ((value >> i) & 1u) {
                cell = (uint8_t)(cell | mask);
            } else {
                cell = (uint8_t)(cell & (uint8_t)~mask);
            }
        }
    }

    /** 读 buf 的 bitOffset 处 n 位(小端位序, 结果在低 n 位)。
     *  out-of-range 时返回 0 且计数: 返回"缓冲区里恰好有的那几位"会让调用方拿到一个
     *  看似合法的半截字段, 比全 0 更难发现。 */
    inline uint32_t getBits(const uint8_t *buf, uint16_t byteLen, uint16_t bitOffset, uint8_t n)
    {
        if (buf == 0 || !rangeOk(byteLen, bitOffset, n)) {
            return 0u;
        }
        uint32_t v = 0u;
        for (uint8_t i = 0u; i < n; ++i) {
            const uint32_t idx = (uint32_t)bitOffset + (uint32_t)i;
            if (buf[idx >> 3] & (uint8_t)(1u << (idx & 7u))) {
                v |= (uint32_t)1u << i;
            }
        }
        return v;
    }

    //===============================================================================================
    // 三、有符号位读写(n 位补码)
    //===============================================================================================
    /** n 位补码 -> int32(n = 32 时原样返回)。
     *
     * 为什么不用教科书那行 `(int32_t)(v << (32 - n)) >> (32 - n)`: 左移有符号数是 UB;
     *   而负值右移在 C++11 里是 implementation-defined(AC6/g++ 都是算术移位, 但标准不保证)。
     *   本文件要"AC6 与 g++ 编出同一位序 + 同一数值", 所以手工补高位, 不给实现定义留口子。
     * 为什么 n = 0 返回 0 而不是报错: rangeOk 已经在读写入口拦掉了; 这里只是保证任何输入都有定义值。 */
    inline int32_t signExtend(uint32_t v, uint8_t n)
    {
        if (n == 0u) {
            return 0;
        }
        if (n >= 32u) {
            return (int32_t)v;
        }
        const uint32_t mask = ((uint32_t)1u << n) - 1u;
        v &= mask;
        if (v & ((uint32_t)1u << (n - 1u))) {
            return (int32_t)(v | ~mask); // 高位全补 1
        }
        return (int32_t)v;
    }

    /** 写 n 位有符号值(只取低 n 位; 超范围的截断由调用方的饱和函数负责, 见 quantizeSat) */
    inline void putSignedBits(uint8_t *buf, uint16_t byteLen, uint16_t bitOffset, int32_t value, uint8_t n)
    {
        const uint32_t mask = (n >= 32u) ? 0xFFFFFFFFu : (((uint32_t)1u << n) - 1u);
        putBits(buf, byteLen, bitOffset, (uint32_t)value & mask, n);
    }

    /** 读 n 位有符号值(符号扩展) */
    inline int32_t getSignedBits(const uint8_t *buf, uint16_t byteLen, uint16_t bitOffset, uint8_t n)
    {
        return signExtend(getBits(buf, byteLen, bitOffset, n), n);
    }

    //===============================================================================================
    // 四、定点 <-> 浮点(全部饱和, 不回绕)
    //===============================================================================================
    /** v*scale 四舍五入后续饱和到 [lo, hi]。
     *
     * 为什么不是 (int32_t)(v*scale + 0.5f): ① 负数会朝 0 偏(round(-0.5) 应为 -1, 实际得 0);
     *   ② 超范围浮点转整数是 UB, 而"链路字段越界"恰恰是最该有定义行为的地方。
     * 为什么不是取模回绕: 回绕把"油门超一点"变成"怠速", 把"姿态超一点"变成反向姿态 —— 这是
     *   饱和与回绕唯一不能互换的地方。依据: 任务书"超范围要饱和, 不许回绕"。
     * NaN 为什么返回 0 而不是饱和到边界: 边界饱和会把"上游算坏了"变成"满舵/满速", 而 0 在
     *   杆量与速度指令上都是中立值。±inf 仍按超范围饱和到对应边界(它们的"方向"是明确的)。 */
    inline int32_t quantizeSat(float v, float scale, int32_t lo, int32_t hi)
    {
        const float s = v * scale;
        if (s != s) {
            return 0; // NaN
        }
        // 先夹一次浮点: 保证下面的 (int32_t) 转换永远在安全范围内(含 ±inf)
        if (!(s > (float)lo - 0.5f)) {
            return lo;
        }
        if (s > (float)hi + 0.5f) {
            return hi;
        }
        const int32_t r = (int32_t)((s >= 0.0f) ? (s + 0.5f) : (s - 0.5f));
        if (r < lo) {
            return lo;
        }
        if (r > hi) {
            return hi;
        }
        return r;
    }

    /** 定点 -> 浮点(纯缩放)。不夹范围: 定点域本身有限, 夹了反而掩盖上游的缩放写错 */
    inline float dequantize(int32_t raw, float scale)
    {
        return (float)raw * scale;
    }

    /** 有符号定点 -> 归一化 float, 夹到 [-1, 1]。
     *  分母用 fullScale 而不是 fullScale+1: n 位补码能表示的最大正数是 2^(n-1)-1(如 12 位的 2047),
     *  用它做分母才能让"满量程 raw"恰好落在 +1.000。 */
    inline float normalizedSigned(int32_t raw, float fullScale)
    {
        const float v = (float)raw / fullScale;
        if (v > 1.0f) {
            return 1.0f;
        }
        if (v < -1.0f) {
            return -1.0f;
        }
        return v;
    }

    /** 无符号整数塞进 n 位域前的饱和(u6/u3/u2 这类窄位域用)。
     *  为什么不是 & mask: 回绕会把"17 颗星"变成"0 颗星"、"模式 9"变成"模式 1"。 */
    inline uint32_t satUnsigned(uint32_t v, uint8_t n)
    {
        if (n >= 32u) {
            return v;
        }
        const uint32_t max = ((uint32_t)1u << n) - 1u;
        return (v > max) ? max : v;
    }

    /** 有符号整数塞进 n 位域前的饱和(对称满量程 ±(2^(n-1)-1), 与 quantizeSat 的归一化约定一致) */
    inline int32_t satSigned(int32_t v, uint8_t n)
    {
        if (n >= 32u) {
            return v;
        }
        const int32_t max = (int32_t)(((uint32_t)1u << (n - 1u)) - 1u);
        if (v > max) {
            return max;
        }
        if (v < -max) {
            return -max;
        }
        return v;
    }

} // namespace bits
} // namespace link
} // namespace dlx

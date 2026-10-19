#pragma once
//===================================================================================================
// DL_LIB/dlx_latest.hpp —— "最新一份快照"的无锁传递(单写者 / 单读者)
//   数据成员不是 `volatile`/`std::atomic`, 严格按 C++11 内存模型这是数据竞争(UB)。实践中靠
//   两侧的内存屏障(`__DMB()` / `atomic_thread_fence`)保证顺序, 在 Cortex-M4 与 x86 上都是
//   "有序但不原子"的读写, 而本类型的读者只用"序号没变"来确认一致性 —— 这正是 seqlock 的
//   定义用法。要彻底消除 UB 得把数据成员做成 atomic 并逐字段拷贝, 代价是代码量翻倍而语义不变。
//===================================================================================================

#include <stdint.h>

#if defined(__arm__) || defined(__ARM_ARCH) || defined(__ARM_ARCH_7M__)
#define DLX_SEQ_FENCE() __DMB()
#else
// 主机(单测)路径: 用编译器屏障。x86 的存储是强序的, 这里主要是拦住编译器重排。
#define DLX_SEQ_FENCE() __asm__ __volatile__("" ::: "memory")
#endif

namespace dlx
{

/** 序号计数(便于测试与诊断: 写了几次) */
inline uint32_t latestWrites(uint32_t seq)
{
    return seq / 2u;
}

/**
 * @brief 一份"最新值"快照槽
 *
 * @tparam T 快照类型。要求可平凡拷贝(逐字节拷贝即等价, 不含指针/虚表)。
 *           这不是洁癖: seqlock 的一致性检查靠"拷贝期间没人写", 只有平凡拷贝才能保证
 *           "读出来的字节是某一时刻的完整状态"这句话有意义。
 */
template <typename T>
class Latest
{
public:
    Latest() = default;

    /** 发布一份新快照(只许一个写者调用) */
    void publish(const T &v)
    {
        const uint32_t s = _seq;
        _seq             = s + 1u; // 奇数 = 正在写
        DLX_SEQ_FENCE();
        _value = v;
        DLX_SEQ_FENCE();
        _seq = s + 2u; // 偶数 = 写完
    }

    /**
     * @brief 读一份快照
     * @param out 读到的一致快照
     * @return false = 读到一半被写者打断(或本槽从来没被写过), **out 不可用**, 请沿用上一次的值
     */
    bool read(T &out) const
    {
        const uint32_t s1 = _seq;
        if ((s1 & 1u) != 0u) {
            return false; // 写者正在写
        }
        if (s1 == 0u) {
            return false; // 从未发布过: 别把"没数据"当成"数据是 0"
        }
        DLX_SEQ_FENCE();
        out = _value;
        DLX_SEQ_FENCE();
        return _seq == s1;
    }

    /** 已经发布过几次(诊断用) */
    uint32_t written() const { return latestWrites(_seq); }

private:
    volatile uint32_t _seq{0};
    T                 _value{};
};

} // namespace dlx

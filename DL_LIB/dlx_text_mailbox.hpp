#pragma once
//===================================================================================================
// dlx_text_mailbox.hpp —— 文本行投递信箱(生产者: 任意线程/中断; 消费者: 当前 owner)
//
// 为什么需要它:
//   文本日志和"固定长报文"走的是同一条串口 DMA 和同一块发送缓冲。谁在调用点直接动 txBuf + 协议 +
//   DMA, 谁就和日志线程抢同一块内存: reset() 可能踩在 DMA 正在读的缓冲上, 帧头也会互相覆盖。
//   所以生产者只允许做一件事: "把一整行字放进池子", 成帧与发送统统交给 owner。
//
// 为什么是双缓冲:
//   生产者一直在写, 消费者要把它整批成帧(SimpleLogW 是**拷贝**进 txBuf)。单只池只有两条路:
//   要么消费者长时间关中断做搬移, 要么把"消费到哪了"变成消费者必须维护的状态。双只池把这两件事
//   彻底分开:
//     - 生产者写 writeIndex 那只;
//     - 消费者 acquire() 只交换两个下标(临界区里几个周期), 换完在临界区外从容成帧;
//     - release() 清空归还 —— 成帧已经把内容拷走了, 不必等 DMA 完成。
//
// 不变量(实现与测试都按这三条来):
//   ① post() 只认 writeIndex, acquire()/release() 只认 readIndex, 三者都在短临界区里, 所以"交换"
//      对生产者是原子的;
//   ② acquire() 的封口点永远落在行边界上(整条 post 在临界区内完成)⇒ 绝不会切出半行;
//   ③ readIndex 那只在下一次 acquire() 之前必须为空。若消费者漏调 release(), acquire() 会把那一批
//      计入丢弃并清掉, 而不是让它和下一批粘成一条(脏池绝不能变成生产者的目标池)。
//
// 计数器语义(给 [Stat] 行看):
//   droppedLineCount —— post() 因池满被**整条拒掉**的行数。这是唯一确切知道"行数"的地方;
//   droppedByteCount —— 所有被丢弃的字节数(post 拒掉的 + 消费者整批丢弃/截断的)。
//   池里只有字节、没有行边界, 所以消费者丢弃时只能计字节 —— 这是刻意的取舍, 不要在这里假装知道行数。
//
// 上下文安全:
//   所有共享状态都用 dlx::InterruptLock(CMSIS PRIMASK 的保存-恢复)保护: 单核上它对"中断 + 所有线程"
//   一律互斥, 可嵌套, 而且析构负责恢复 —— 不会漏掉某条 return。
//
//   为什么**不能**用 rt_enter_critical(): 它只在修改调度锁计数器的几个周期里关中断, 返回时中断已经
//   打开(见 rt-thread/src/scheduler.c:921-936)—— 它只阻止线程切换, 挡不住中断。生产方一旦可能来自
//   中断(本工程明确允许), 它和被中断打断的线程之间仍然会撞: 计数器是读-改-写, StringPool 的 cur 也是。
//
//   因此本文件**不含任何 RT-Thread 依赖**: 只 include dlx_interrupt_lock.hpp(进而只依赖 CMSIS)。
//
//   关中断窗口(量级, @168MHz): post() 里包含 160B 的 memcpy ≈ 1~2µs; acquire()/release()/noteDropped()
//   只有十几个周期(<0.5µs)。对照: 控制台 2Mbaud 每字节 5.2µs, IMU 数据就绪 1kHz, DShot 由定时器+DMA
//   驱动(不吃中断)。行文本的格式化在调用方栈上做(见 release_header.h 的 logLine), 不占窗口。
//===================================================================================================
#include <stdint.h>
#include <string.h>

#include "dlx_interrupt_lock.hpp" // 中断屏蔽的保存-恢复(不含 RT-Thread 依赖)
#include "dlx_string_pool.hpp"

namespace dlx
{

namespace textmbox
{
/** 池数: 一只收、一只发 */
constexpr uint8_t POOL_COUNT = 2;

/** 单只池的**推荐**容量(字节)。512 ≈ 3 行, 5ms 的窗口足够。
 *  实际容量由拥有者传入的 ByteBuffer 决定 —— 只要满足: 单池容量 + 4(SimpleLog 帧头) ≤ 保底空间。 */
constexpr uint16_t POOL_BYTES = 512; // TODO(cfg): 新配置项(文本池容量, 仅作推荐值/编译期核对)

/** 单行上限(字节)。与调用方 vsnprintf 的缓冲一致 */
constexpr uint16_t LINE_MAX_BYTES = 160; // TODO(cfg): cfg::hw::buf::kTextLine
} // namespace textmbox

/**
 * @brief 文本投递信箱
 *
 * **不持有存储**: 两只池的存储与 ByteBuffer 都由拥有者创建并传入(与工程里 Queue / StringPool 的惯例
 * 一致 —— 谁管生命周期谁给缓冲)。本对象通常也是栈上变量, 通过上下文里的指针交给生产方与消费方,
 * 所以它可以在多个 owner 之间转移。
 */
class TextMailbox
{
public:
    /**
     * @brief 外部传入两只池
     *
     * @param firstPoolBuffer  第一只池(生产者的目标池之一)
     * @param secondPoolBuffer 第二只池
     *
     * 两只池建议等容量、且不小于一行; 容量不等不会写坏任何东西(只是先满的那只更容易拒收),
     * 但**必须**满足"单池容量 + 4 ≤ 保底空间", 否则文本帧会被截断(见 app_log_thread.hpp 的 static_assert)。
     */
    TextMailbox(ByteBuffer &firstPoolBuffer, ByteBuffer &secondPoolBuffer)
        : textPoolA(firstPoolBuffer), textPoolB(secondPoolBuffer), textPool{&textPoolA, &textPoolB}
    {
    }
    TextMailbox(const TextMailbox &) = delete;
    TextMailbox &operator=(const TextMailbox &) = delete;

    /**
     * @brief 生产者: 追加一整行
     *
     * 空间不足 ⇒ **整条放弃**(不做部分写入)并计数; 绝不阻塞。
     * @return true = 整行都进池了
     */
    bool post(const char *text, uint16_t length)
    {
        if (text == nullptr || length == 0u || length > textmbox::LINE_MAX_BYTES) {
            return false;
        }
        bool stored = false;
        {
            const InterruptLock lock; // 窗口: 这一次写入(含整条 memcpy), 窗口外不做别的事
            stored = textPool[writeIndex]->write(text, length);
            if (!stored) {
                ++droppedLineCount;
                droppedByteCount += length;
            }
        }
        return stored;
    }

    /**
     * @brief 消费者: 交换两只池, 返回本批(只读)
     *
     * @return nullptr = 本批为空(此时什么都没动, 不需要 release)
     */
    const StringPool *acquire()
    {
        const StringPool *batch = nullptr;
        {
            const InterruptLock lock; // 窗口: 只读几个 used() + 交换两个下标
            if (textPool[writeIndex]->used() != 0u) {
                if (textPool[readIndex]->used() != 0u) {
                    // 上一批忘了归还: 记进丢弃并清掉 —— 脏池绝不能变成生产者的目标池
                    droppedByteCount += textPool[readIndex]->used();
                    textPool[readIndex]->reset();
                }
                const uint8_t previousWriteIndex = writeIndex;
                writeIndex = readIndex;
                readIndex = previousWriteIndex;
                batch = textPool[readIndex];
            }
        }
        return batch;
    }

    /** @brief 消费者: 归还并清空(成帧之后立刻调; 内容已被 SimpleLogW 拷进发送缓冲) */
    void release()
    {
        const InterruptLock lock; // 窗口: cur = src 一条指令
        textPool[readIndex]->reset();
    }

    /** @brief 消费者: 记账"本批有这么多字节没发出去"(尽力而为成帧时被截断的部分) */
    void noteDropped(uint16_t bytes)
    {
        if (bytes == 0u) {
            return;
        }
        const InterruptLock lock; // 窗口: 一次 32 位累加
        droppedByteCount += bytes;
    }

    /**
     * @brief 消费者: 整批丢弃(没有文本出口时用)
     *
     * 必须调用: 投递方一直在投, 不排空池子就会永久满, 投递方永远失败。
     * @return 丢掉的字节数
     */
    uint16_t discardAll()
    {
        const StringPool *batch = acquire();
        if (batch == nullptr) {
            return 0u;
        }
        const uint16_t discardedBytes = batch->used();
        noteDropped(discardedBytes);
        release();
        return discardedBytes;
    }

    uint32_t droppedLines() const { return droppedLineCount; }
    uint32_t droppedBytes() const { return droppedByteCount; }

private:
    // 两个 StringPool 写成具名成员而不是数组: StringPool 的构造函数是 explicit, 而"数组元素"的列表
    // 初始化走的是拷贝初始化, 用不了 explicit 构造函数(C++11 实测编不过)。下标访问靠下面的指针数组。
    StringPool textPoolA;
    StringPool textPoolB;

    /** 下标访问用: 交换下标即交换两只池 */
    StringPool *textPool[textmbox::POOL_COUNT];

    uint8_t  writeIndex{0}; ///< 生产者写这只
    uint8_t  readIndex{1};  ///< 消费者读这只; 空闲时必须为空

    uint32_t droppedLineCount{0};
    uint32_t droppedByteCount{0};
};

} // namespace dlx

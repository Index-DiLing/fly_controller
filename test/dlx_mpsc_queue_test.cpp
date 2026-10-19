//===================================================================================================
// test/dlx_mpsc_queue_test.cpp —— MPSCQueue 主机侧功能测试
//
// 这里验证的是队列协议(多个生产者交错投递、满时丢新、单消费者取走)。
// 真正的线程/ISR 竞态由 InterruptLock 在目标单核上串行化; 主机 stub 不模拟中断。
//===================================================================================================
#include <stdio.h>

#include "dlx_queue.hpp"

using namespace dlx;

static int g_checks   = 0;
static int g_failures = 0;

#define CHECK(condition)                                                     \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(condition)) {                                                  \
            ++g_failures;                                                    \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #condition);   \
        }                                                                    \
    } while (0)

static void producerA(MPSCQueue<uint16_t> &queue)
{
    CHECK(queue.push(11u));
    CHECK(queue.push(12u));
}

static void producerB(MPSCQueue<uint16_t> &queue)
{
    CHECK(queue.push(21u));
}

static void testQueue()
{
    uint8_t storage[sizeof(uint16_t) * 3u] = {};
    ByteBuffer buffer(storage, sizeof(storage));
    MPSCQueue<uint16_t> queue(buffer);

    producerA(queue);
    producerB(queue);
    CHECK(queue.size() == 3u);
    CHECK(!queue.push(31u)); // 满时丢新
    CHECK(queue.dropped() == 1u);

    uint16_t value = 0u;
    CHECK(queue.peek(value) && value == 11u);
    CHECK(queue.pop(value) && value == 11u);
    CHECK(queue.push(31u)); // 释放一个槽后可以环绕复用
    CHECK(queue.pop(value) && value == 12u);
    CHECK(queue.pop(value) && value == 21u);
    CHECK(queue.pop(value) && value == 31u);
    CHECK(!queue.pop(value));

    CHECK(queue.push(41u));
    CHECK(queue.push(42u));
    queue.clear();
    CHECK(queue.isEmpty());
    CHECK(!queue.pop(value));
}

static void testEmptyStorage()
{
    uint8_t storage[1] = {};
    ByteBuffer buffer(storage, sizeof(storage));
    MPSCQueue<uint16_t> queue(buffer);
    CHECK(queue.capacity() == 0u);
    CHECK(!queue.push(1u));
    CHECK(queue.dropped() == 1u);
}

int main()
{
    printf("=== MPSCQueue test ===\n");
    testQueue();
    testEmptyStorage();
    printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}

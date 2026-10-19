//===================================================================================================
// test/dlx_queue_test.cpp —— SPSCQueue 主机侧测试
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

static void testQueue()
{
    uint8_t storage[sizeof(uint16_t) * 3u] = {};
    ByteBuffer buffer(storage, sizeof(storage));
    SPSCQueue<uint16_t> queue(buffer);

    CHECK(queue.capacity() == 3u);
    CHECK(queue.isEmpty());
    CHECK(queue.remaining() == 3u);
    CHECK(queue.push(11u));
    CHECK(queue.push(22u));
    CHECK(queue.push(33u));
    CHECK(!queue.push(44u));
    CHECK(queue.isFull());

    uint16_t value = 0u;
    CHECK(queue.peek(value) && value == 11u);
    CHECK(queue.pop(value) && value == 11u);
    CHECK(queue.pop(value) && value == 22u);
    CHECK(queue.push(44u));
    CHECK(queue.pop(value) && value == 33u);
    CHECK(queue.pop(value) && value == 44u);
    CHECK(!queue.pop(value));

    CHECK(queue.push(55u));
    CHECK(queue.push(66u));
    queue.clear();
    CHECK(queue.isEmpty());
    CHECK(!queue.pop(value));
}

static void testEmptyStorage()
{
    uint8_t storage[1] = {};
    ByteBuffer buffer(storage, sizeof(storage));
    SPSCQueue<uint16_t> queue(buffer);
    CHECK(queue.capacity() == 0u);
    CHECK(!queue.push(1u));
    uint16_t value = 0u;
    CHECK(!queue.pop(value));
    CHECK(queue.remaining() == 0u);
}

int main()
{
    printf("=== SPSCQueue test ===\n");
    testQueue();
    testEmptyStorage();
    printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}

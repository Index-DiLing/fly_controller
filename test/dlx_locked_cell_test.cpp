//===================================================================================================
// test/dlx_locked_cell_test.cpp —— LockedCell 主机侧测试
//===================================================================================================
#include <stdio.h>

#include "dlx_locked_cell.hpp"

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

struct Sample
{
    uint32_t sequence;
    int16_t  value;
};

static void testCell()
{
    LockedCell<Sample> cell(Sample{1u, 10});
    Sample sample = cell.load();
    CHECK(sample.sequence == 1u && sample.value == 10);

    cell.store(Sample{2u, 20});
    sample = cell.load();
    CHECK(sample.sequence == 2u && sample.value == 20);

    const Sample old = cell.exchange(Sample{3u, 30});
    CHECK(old.sequence == 2u && old.value == 20);
    sample = cell.load();
    CHECK(sample.sequence == 3u && sample.value == 30);

    cell.update([](Sample &current) {
        ++current.sequence;
        current.value += 5;
    });
    sample = cell.load();
    CHECK(sample.sequence == 4u && sample.value == 35);
}

int main()
{
    printf("=== LockedCell test ===\n");
    testCell();
    printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}

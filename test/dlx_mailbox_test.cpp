//===================================================================================================
// test/dlx_mailbox_test.cpp —— 两个投递信箱的主机侧测试
//
// 怎么跑(不需要板子、不需要交叉工具链):
//   g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . test/dlx_mailbox_test.cpp -o mbox_test
//   ./mbox_test
//
// 为什么不放在 DL_LIB 里: 构建工程(.eide/eide.yml)的 srcDirs 只有 DL_LIB 与 rt-thread, 放在那里的
// 测试文件必须靠 excludeList 一条条排除(flight_control_test.cpp 就是靠那条规则才没被编进固件);
// 放在 test/ 下则天然不在构建范围内, 零配置改动、也不可能污染固件。
//
// 覆盖的是"状态机"而不是并发: 双缓冲的交换、封口点落在行边界、漏 release 的防御、满丢弃计数、
// 定长信箱的环绕与溢出。真正的并发(生产者线程 + 消费者线程)只能上台架验。
//===================================================================================================
#include <stdio.h>
#include <string.h>

#include "app/channels/app_log_channel.hpp" // LogChannel / ExceptionMailbox(顺带验证这个头本身能编)
#include "dlx_text_mailbox.hpp"
#include "dlx_value_mailbox.hpp"

using namespace dlx;

static int g_checks   = 0;
static int g_failures = 0;

#define CHECK(condition)                                                        \
    do {                                                                        \
        ++g_checks;                                                             \
        if (!(condition)) {                                                     \
            ++g_failures;                                                       \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #condition);        \
        }                                                                       \
    } while (0)

/**
 * @brief 测试夹子: 存储 → 两只 ByteBuffer → 信箱
 *
 * 顺序就是构造顺序, 也就是"拥有者先给缓冲、再给信箱"这件事本身 —— 信箱不持有存储,
 * 所以缓冲必须在信箱之前构造、在信箱之后析构。
 */
struct MailboxFixture
{
    uint8_t     storageA[textmbox::POOL_BYTES]{};
    uint8_t     storageB[textmbox::POOL_BYTES]{};
    ByteBuffer  bufferA{storageA, textmbox::POOL_BYTES};
    ByteBuffer  bufferB{storageB, textmbox::POOL_BYTES};
    TextMailbox mailbox{bufferA, bufferB};
};

//---------------------------------------------------------------------------------------------------
// 一、基本往返: 投递 -> 取走 -> 归还 -> 又是空的
//---------------------------------------------------------------------------------------------------
static void testTextBasic()
{
    printf("[1] 文本信箱基本往返\n");
    MailboxFixture fixture;
    TextMailbox   &box = fixture.mailbox;

    CHECK(box.acquire() == nullptr); // 空: 返回 nullptr, 且什么都没动
    CHECK(box.post("hello", 5u));
    const StringPool *batch = box.acquire();
    CHECK(batch != nullptr);
    CHECK(batch->used() == 5u);
    CHECK(batch != nullptr && memcmp(batch->data(), "hello", 5u) == 0);
    box.release();
    CHECK(box.acquire() == nullptr);
    CHECK(box.droppedLines() == 0u);
    CHECK(box.droppedBytes() == 0u);
}

//---------------------------------------------------------------------------------------------------
// 二、双缓冲: 消费者手里拿着一批时, 生产者继续投必须落到另一只池, 且两批互不污染
//---------------------------------------------------------------------------------------------------
static void testTextPingPong()
{
    printf("[2] 双缓冲: 消费者成帧期间生产者继续投\n");
    MailboxFixture fixture;
    TextMailbox   &box = fixture.mailbox;

    CHECK(box.post("AAAA", 4u));
    const StringPool *firstBatch = box.acquire();
    CHECK(firstBatch != nullptr && firstBatch->used() == 4u);

    // 这一批还没归还: 新的投递必须进另一只池, 绝不能续在第一批后面
    CHECK(box.post("BB", 2u));
    CHECK(box.post("CCCC", 4u));
    CHECK(firstBatch != nullptr && firstBatch->used() == 4u); // 第一批纹丝不动

    box.release();
    const StringPool *secondBatch = box.acquire();
    CHECK(secondBatch != nullptr);
    CHECK(secondBatch != nullptr && secondBatch->used() == 6u);
    CHECK(secondBatch != nullptr && memcmp(secondBatch->data(), "BBCCCC", 6u) == 0);
    box.release();
    CHECK(box.acquire() == nullptr);
}

//---------------------------------------------------------------------------------------------------
// 三、封口点落在行边界: 池满时被拒的一定是"整条", 已进池的内容一个字节都不少
//---------------------------------------------------------------------------------------------------
static void testTextLineBoundaryAndFull()
{
    printf("[3] 行边界与池满整条丢弃\n");
    MailboxFixture fixture;
    TextMailbox   &box = fixture.mailbox;

    char line[textmbox::LINE_MAX_BYTES];
    memset(line, 'x', sizeof(line));

    uint16_t acceptedCount = 0u;
    while (box.post(line, textmbox::LINE_MAX_BYTES)) {
        ++acceptedCount;
    }
    // 512 / 160 = 3 条; 第 4 条整条被拒
    CHECK(acceptedCount == static_cast<uint16_t>(textmbox::POOL_BYTES / textmbox::LINE_MAX_BYTES));
    CHECK(box.droppedLines() == 1u);
    CHECK(box.droppedBytes() == textmbox::LINE_MAX_BYTES);

    const StringPool *batch = box.acquire();
    CHECK(batch != nullptr &&
          batch->used() == static_cast<uint16_t>(acceptedCount * textmbox::LINE_MAX_BYTES));
    box.release();

    // 排空之后池子又能用: 这是"投递方永远不会因为一次失败就永久坏掉"的前提
    CHECK(box.post(line, 100u));
    CHECK(box.acquire() != nullptr);
    box.release();
}

//---------------------------------------------------------------------------------------------------
// 四、漏调 release 的防御: 上一批必须计入丢弃并被清掉, 绝不能和下一批粘成一条
//---------------------------------------------------------------------------------------------------
static void testTextForgottenRelease()
{
    printf("[4] 漏调 release 的防御分支\n");
    MailboxFixture fixture;
    TextMailbox   &box = fixture.mailbox;

    CHECK(box.post("AAAA", 4u));
    CHECK(box.acquire() != nullptr); // 故意不 release
    CHECK(box.post("BBBBBB", 6u));

    const StringPool *batch = box.acquire();
    CHECK(batch != nullptr && batch->used() == 6u);
    CHECK(batch != nullptr && memcmp(batch->data(), "BBBBBB", 6u) == 0);
    CHECK(box.droppedBytes() == 4u); // 丢掉的是上一批的 4 字节
    box.release();
}

//---------------------------------------------------------------------------------------------------
// 五、整批丢弃(没有文本出口时)
//---------------------------------------------------------------------------------------------------
static void testTextDiscardAll()
{
    printf("[5] 整批丢弃\n");
    MailboxFixture fixture;
    TextMailbox   &box = fixture.mailbox;

    CHECK(box.post("AAAA", 4u));
    CHECK(box.post("BBBB", 4u));
    CHECK(box.discardAll() == 8u);
    CHECK(box.droppedBytes() == 8u);
    CHECK(box.discardAll() == 0u);
    CHECK(box.post("CC", 2u)); // 丢完还能继续用
    box.release();
}

//---------------------------------------------------------------------------------------------------
// 六、参数非法不算"丢日志"(免得计数被误读成"日志系统坏了")
//---------------------------------------------------------------------------------------------------
static void testTextInvalidArguments()
{
    printf("[6] 参数校验\n");
    MailboxFixture fixture;
    TextMailbox   &box = fixture.mailbox;

    char tooLong[textmbox::LINE_MAX_BYTES + 1];
    memset(tooLong, 'x', sizeof(tooLong));
    CHECK(!box.post(tooLong, textmbox::LINE_MAX_BYTES + 1u));
    CHECK(!box.post(nullptr, 4u));
    CHECK(!box.post("x", 0u));
    CHECK(box.droppedLines() == 0u);
    CHECK(box.droppedBytes() == 0u);
}

//---------------------------------------------------------------------------------------------------
// 七、定长信箱: 环绕、批量取、满时丢新的并置溢出
//---------------------------------------------------------------------------------------------------
static void testValueMailbox()
{
    printf("[7] 定长信箱\n");
    ValueMailbox<uint16_t, 4> box;
    uint16_t                  values[4] = {};

    CHECK(box.take(values, 4u) == 0u);
    CHECK(box.push(11u) && box.push(22u) && box.push(33u));
    CHECK(box.take(values, 2u) == 2u);
    CHECK(values[0] == 11u && values[1] == 22u);

    CHECK(box.push(44u) && box.push(55u) && box.push(66u)); // 33,44,55,66 = 满
    CHECK(!box.push(77u));                                  // 满: 丢新的
    CHECK(box.overflowed());
    CHECK(box.droppedCount() == 1u);

    CHECK(box.take(values, 4u) == 4u);
    CHECK(values[0] == 33u && values[1] == 44u && values[2] == 55u && values[3] == 66u);
    CHECK(box.take(values, 4u) == 0u);

    box.clearOverflow(); // 溢出是"事件", 消费方报过一次就要清, 否则每轮都报
    CHECK(!box.overflowed());
    CHECK(box.push(88u));
    CHECK(box.take(values, 4u) == 1u && values[0] == 88u);
}

//---------------------------------------------------------------------------------------------------
// 八、异常渠道用的真实类型(ExceptionMailbox = ValueMailbox<ExceptionCode, 16>)
//---------------------------------------------------------------------------------------------------
static void testExceptionMailbox()
{
    printf("[8] 异常渠道类型\n");
    ExceptionMailbox box;
    ExceptionCode    codes[2] = {};

    CHECK(box.push(ExceptionCode::FLASH_WRITE_FAILED));
    CHECK(box.push(ExceptionCode::LOG_WRITE_FAILED));
    CHECK(box.take(codes, 2u) == 2u);
    CHECK(codes[0] == ExceptionCode::FLASH_WRITE_FAILED);
    CHECK(codes[1] == ExceptionCode::LOG_WRITE_FAILED);
    CHECK(box.take(codes, 2u) == 0u);
}

int main()
{
    printf("=== dlx mailbox test (host) ===\n");
    testTextBasic();
    testTextPingPong();
    testTextLineBoundaryAndFull();
    testTextForgottenRelease();
    testTextDiscardAll();
    testTextInvalidArguments();
    testValueMailbox();
    testExceptionMailbox();

    printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}

#include <stdio.h>
#include <string.h>
#include "dlx_flash_manager.hpp"

using namespace dlx;

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++g_checks;                                                              \
        if (!(cond)) {                                                           \
            ++g_fails;                                                           \
            printf("  [FAIL] line %d: %s\n", __LINE__, #cond);                   \
        }                                                                        \
    } while (0)

static sim::Chip chip;
static W25Q128 flash(chip);

// ---------------- 回调用的收集器 ----------------
struct SessionCollect
{
    uint32_t count;
    uint32_t activeCount;
    uint32_t maxSession;
    uint32_t slotSum;
};

static bool sessionCb(const FlashLogSessionInfo &info, void *ctx)
{
    SessionCollect *c = static_cast<SessionCollect *>(ctx);
    ++c->count;
    if (info.active) {
        ++c->activeCount;
    }
    if (info.sessionId > c->maxSession) {
        c->maxSession = info.sessionId;
    }
    c->slotSum += info.slotCount;
    return true;
}

struct EntryCollect
{
    uint32_t count;
    uint32_t orderOk;
    float lastThrottle;
    uint32_t lastTick;
};

static bool entryCb(const LogEntry &e, void *ctx)
{
    EntryCollect *c = static_cast<EntryCollect *>(ctx);
    if (e.seq == c->count + 1u) {
        ++c->orderOk;
    }
    ++c->count;
    c->lastThrottle = e.throttle;
    c->lastTick = e.tickMs;
    return true;
}

static uint32_t metaSlotAddress(uint32_t ordinal)
{
    return FLASH_META_OFFSET
         + (ordinal / FLASH_META_SLOTS_PER_SECTOR) * FLASH_SECTOR_SIZE
         + (ordinal % FLASH_META_SLOTS_PER_SECTOR) * FLASH_META_SLOT_BYTES;
}

/** 找出当前最新的元数据记录所在地址(测试里直接改内存用) */
static uint32_t newestMetaAddr()
{
    uint32_t bestAddr = 0;
    uint32_t bestSeq = 0;
    bool found = false;
    for (uint32_t i = 0; i < FLASH_META_SLOT_COUNT; ++i) {
        const uint32_t addr = metaSlotAddress(i);
        FlashMetaRecord r;
        memcpy(&r, chip.mem + addr + 1, sizeof(r));
        if (chip.mem[addr] != FLASH_FRAME_HEAD || chip.mem[addr + sizeof(r) + 1] != FLASH_FRAME_TAIL) {
            continue;
        }
        if (r.crc != flashStructCrc(r) || r.type != static_cast<uint8_t>(FlashRecordType::Meta)) {
            continue;
        }
        if (!found || r.seq > bestSeq) {
            found = true;
            bestSeq = r.seq;
            bestAddr = addr;
        }
    }
    return bestAddr;
}

static LogEntry makeEntry(uint32_t tick, float throttle, uint16_t flags)
{
    LogEntry e;
    memset(&e, 0, sizeof(e));
    e.tickMs = tick;
    e.throttle = throttle;
    e.flags = flags;
    e.rollRad = 0.001f * static_cast<float>(tick);
    return e;
}

int main()
{
    printf("=== FlashManager 主机逻辑测试 ===\n");
    printf("sizeof(LogEntry)=%u  sizeof(FlashParamData)=%u  sizeof(FlashMetaRecord)=%u\n",
           (unsigned)sizeof(LogEntry), (unsigned)sizeof(FlashParamData), (unsigned)sizeof(FlashMetaRecord));
    printf("条目步长=%u  每槽条目=%u  槽数=%u  单次预留槽=%u\n",
           (unsigned)FLASH_LOG_ENTRY_STRIDE, (unsigned)FLASH_LOG_ENTRIES_PER_SLOT,
           (unsigned)FLASH_LOG_SLOT_COUNT, (unsigned)FLASH_LOG_SESSION_SLOT_LIMIT);
    printf("单次日志上限 = %u 条 (预留区 %.2f MB, 按需擦除: 通常只擦上次用掉的槽)\n",
           (unsigned)(FLASH_LOG_SESSION_SLOT_LIMIT * FLASH_LOG_ENTRIES_PER_SLOT),
           (double)FLASH_LOG_SESSION_SLOT_LIMIT * (double)FLASH_LOG_SLOT_SIZE / (1024.0 * 1024.0));
    printf("元数据: 每扇区槽=%u 总槽=%u | 参数: 每扇区槽=%u 总槽=%u\n",
           (unsigned)FLASH_META_SLOTS_PER_SECTOR, (unsigned)FLASH_META_SLOT_COUNT,
           (unsigned)FLASH_PARAM_SLOTS_PER_SECTOR, (unsigned)FLASH_PARAM_SLOT_COUNT);

    ExceptionCode err = ExceptionCode::FLASH_OK;
    uint32_t busySteps = 0;
    auto onBusy = [&busySteps]() { ++busySteps; };   // 模拟"点灯"的进度回调

    // ---------------- 1. 未建立系统 -> 自动新建 ----------------
    printf("\n[1] 空芯片首次初始化(整片擦除 + 新会话预留 %u 个槽)\n",
           (unsigned)FLASH_LOG_SESSION_SLOT_LIMIT);
    chip.eraseAll();
    FlashManager fs(flash);
    CHECK(fs.appendLog(makeEntry(1, 0.1f, 0), err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_NOT_READY);
    FlashParamData probe;
    memset(&probe, 0, sizeof(probe));
    CHECK(fs.peekParams(probe, err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_PARAM_NOT_FOUND);
    busySteps = 0;
    CHECK(fs.init(FlashFullPolicy::Halt, onBusy, err) == FunctionResult::SUCCESS);
    CHECK(err == ExceptionCode::FLASH_SYSTEM_CREATED);
    CHECK(fs.currentSessionId() == 1);
    CHECK(fs.currentEntryCount() == 0);
    CHECK(fs.sessionReservedSlots() == FLASH_LOG_SESSION_SLOT_LIMIT);
    CHECK(fs.sessionEntryLimit() == FLASH_LOG_SESSION_SLOT_LIMIT * FLASH_LOG_ENTRIES_PER_SLOT);
    CHECK(busySteps == W25Q128_CAPACITY / W25Q128_BLOCK64K_SIZE);   // 整片擦除的进度回调次数
    printf("  整片擦除进度回调 %u 次(每 64KB 一次)\n", (unsigned)busySteps);

    // ---------------- 2. 再次上电 = 新会话 ----------------
    printf("\n[2] 第二次上电\n");
    busySteps = 0;
    CHECK(fs.init(FlashFullPolicy::Halt, onBusy, err) == FunctionResult::SUCCESS);
    CHECK(err == ExceptionCode::FLASH_OK);
    CHECK(fs.currentSessionId() == 2);
    // 上一次会话只用了 1 个槽, 预留区剩下 7 个槽上次已经擦好、没写过 -> 这次只擦 1 个槽
    CHECK(busySteps == 1u * (FLASH_LOG_SLOT_SIZE / W25Q128_BLOCK64K_SIZE));
    CHECK(fs.sessionUsedSlots() == 1);

    // ---------------- 3. 写日志 + 读回 ----------------
    printf("\n[3] 日志写入与回读\n");
    for (uint32_t i = 1; i <= 5; ++i) {
        CHECK(fs.appendLog(makeEntry(100u * i, 0.25f * i, static_cast<uint16_t>(i)), err)
              == FunctionResult::SUCCESS);
        CHECK(err == ExceptionCode::FLASH_OK);
    }
    CHECK(fs.currentEntryCount() == 5);
    for (uint32_t i = 1; i <= 5; ++i) {
        LogEntry e;
        CHECK(fs.readEntry(2, i, e, err) == FunctionResult::SUCCESS);
        CHECK(e.seq == i);
        CHECK(e.sessionId == 2);
        CHECK(e.tickMs == 100u * i);
        CHECK(e.flags == static_cast<uint16_t>(i));
        CHECK(e.type == static_cast<uint8_t>(FlashRecordType::Entry));
    }
    LogEntry tmp;
    CHECK(fs.readEntry(2, 6, tmp, err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_ENTRY_NOT_FOUND);
    CHECK(fs.readEntry(77, 1, tmp, err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_SESSION_NOT_FOUND);

    EntryCollect entries = {0, 0, 0.0f, 0};
    CHECK(fs.forEachEntry(2, entryCb, &entries, err) == FunctionResult::SUCCESS);
    CHECK(entries.count == 5 && entries.orderOk == 5);
    CHECK(entries.lastTick == 500u);

    SessionCollect sessions = {0, 0, 0, 0};
    CHECK(fs.forEachSession(sessionCb, &sessions, err) == FunctionResult::SUCCESS);
    CHECK(sessions.count == 2);
    CHECK(sessions.activeCount == 1);
    CHECK(sessions.maxSession == 2);
    CHECK(sessions.slotSum == 2);
    uint32_t cnt1 = 99;
    CHECK(fs.countEntries(1, cnt1, err) == FunctionResult::SUCCESS && cnt1 == 0);
    uint32_t cnt2 = 0;
    CHECK(fs.countEntries(2, cnt2, err) == FunctionResult::SUCCESS && cnt2 == 5);

    // ---------------- 4. 参数保存/读取(含环形绕回与扇区擦除) ----------------
    printf("\n[4] 持久化参数\n");
    FlashParamData p;
    memset(&p, 0, sizeof(p));
    CHECK(fs.loadParams(p, err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_PARAM_NOT_FOUND);

    memset(&p, 0, sizeof(p));
    p.version = 1;
    p.value[0] = 3.14f;
    p.value[10] = -2.5f;
    CHECK(fs.saveParams(p, err) == FunctionResult::SUCCESS && err == ExceptionCode::FLASH_OK);
    FlashParamData q;
    memset(&q, 0, sizeof(q));
    CHECK(fs.loadParams(q, err) == FunctionResult::SUCCESS);
    CHECK(q.version == 1 && q.value[0] == 3.14f && q.value[10] == -2.5f);

    const long eraseBefore = chip.eraseCalls;
    const uint32_t rounds = FLASH_PARAM_SLOT_COUNT * 3u + 5u;
    for (uint32_t i = 0; i < rounds; ++i) {
        p.value[1] = static_cast<float>(i);
        CHECK(fs.saveParams(p, err) == FunctionResult::SUCCESS);
    }
    CHECK(fs.loadParams(q, err) == FunctionResult::SUCCESS);
    CHECK(q.value[1] == static_cast<float>(rounds - 1u));
    CHECK(q.value[0] == 3.14f);
    printf("  参数写 %u 次触发扇区擦除 %ld 次(槽位轮转, 磨损均衡)\n",
           (unsigned)rounds, chip.eraseCalls - eraseBefore);

    // 不用 init() 也能读到参数(上电早期决定日志档位时用)
    FlashManager fsPeek(flash);
    FlashParamData peeked;
    memset(&peeked, 0, sizeof(peeked));
    CHECK(fsPeek.peekParams(peeked, err) == FunctionResult::SUCCESS);
    CHECK(peeked.value[1] == static_cast<float>(rounds - 1u));

    // ---------------- 5. Halt 策略写满 + 切覆写策略 ----------------
    printf("\n[5] 写满策略(单次日志预留 2 个槽)\n");
    chip.eraseAll();
    FlashManager fs2(flash);
    fs2.setSessionSlotLimit(2);
    CHECK(fs2.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
    CHECK(err == ExceptionCode::FLASH_SYSTEM_CREATED);
    CHECK(fs2.sessionReservedSlots() == 2);
    const uint32_t limit2 = 2u * FLASH_LOG_ENTRIES_PER_SLOT;
    CHECK(fs2.sessionEntryLimit() == limit2);
    for (uint32_t i = 0; i < limit2; ++i) {
        CHECK(fs2.appendLog(makeEntry(i, 0.5f, 0), err) == FunctionResult::SUCCESS);
        // 预留区内部换槽(不需要擦除)不该报"覆写"异常
        CHECK(err == ExceptionCode::FLASH_OK);
    }
    CHECK(fs2.currentEntryCount() == limit2);
    CHECK(fs2.sessionUsedSlots() == 2);        // 一个会话连用两个槽, 中途不报错
    CHECK(fs2.isLogFull());
    CHECK(fs2.appendLog(makeEntry(9999, 0.5f, 0), err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_LOG_FULL);

    // 切到覆写策略: 现场擦掉后面的槽继续写同一个会话
    fs2.setFullPolicy(FlashFullPolicy::OverwriteOldest);
    CHECK(fs2.appendLog(makeEntry(9999, 0.75f, 7), err) == FunctionResult::SUCCESS);
    CHECK(err == ExceptionCode::FLASH_LOG_OVERWRITTEN);
    CHECK(fs2.sessionUsedSlots() == 3);
    CHECK(fs2.currentEntryCount() == limit2 + 1u);
    uint32_t cnt = 0;
    CHECK(fs2.countEntries(fs2.currentSessionId(), cnt, err) == FunctionResult::SUCCESS);
    CHECK(cnt == limit2 + 1u);   // 跨 3 个槽也要能完整读回来
    LogEntry last;
    CHECK(fs2.readEntry(fs2.currentSessionId(), cnt, last, err) == FunctionResult::SUCCESS);
    CHECK(last.tickMs == 9999u && last.flags == 7u);
    LogEntry mid;
    CHECK(fs2.readEntry(fs2.currentSessionId(), FLASH_LOG_ENTRIES_PER_SLOT + 1u, mid, err)
          == FunctionResult::SUCCESS);
    CHECK(mid.seq == FLASH_LOG_ENTRIES_PER_SLOT + 1u);   // 槽边界上的条目
    SessionCollect sc2 = {0, 0, 0, 0};
    CHECK(fs2.forEachSession(sessionCb, &sc2, err) == FunctionResult::SUCCESS);
    CHECK(sc2.count == 1 && sc2.slotSum == 3);

    // ---------------- 6. 掉电: 写一半的条目 ----------------
    printf("\n[6] 掉电中断记录\n");
    chip.eraseAll();
    FlashManager fs3(flash);
    CHECK(fs3.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
    for (uint32_t i = 1; i <= 3; ++i) {
        CHECK(fs3.appendLog(makeEntry(i, 0.5f, 0), err) == FunctionResult::SUCCESS);
    }
    chip.killBudget = 12;   // 第 4 条只写进去 12 个字节就"掉电"
    CHECK(fs3.appendLog(makeEntry(4, 0.5f, 0), err) == FunctionResult::FAILED);
    CHECK(chip.killed);
    const uint32_t slot0 = FLASH_LOG_OFFSET;
    CHECK(chip.mem[slot0 + FLASH_LOG_ENTRY_AREA_OFFSET + 3u * FLASH_LOG_ENTRY_STRIDE]
          != FLASH_FRAME_TAIL); // 半条记录的提交标记没有写进去

    chip.killBudget = -1;
    chip.killed = false;
    FlashManager fs4(flash);   // 重新上电
    CHECK(fs4.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
    CHECK(err == ExceptionCode::FLASH_OK);
    CHECK(fs4.currentSessionId() == 2);
    uint32_t c1 = 0;
    CHECK(fs4.countEntries(1, c1, err) == FunctionResult::SUCCESS);
    CHECK(c1 == 3);            // 半条记录被正确丢弃, 之前的 3 条完好
    LogEntry e3;
    CHECK(fs4.readEntry(1, 3, e3, err) == FunctionResult::SUCCESS && e3.tickMs == 3u);
    CHECK(fs4.appendLog(makeEntry(11, 0.5f, 0), err) == FunctionResult::SUCCESS);

    // ---------------- 7. 数据损坏 ----------------
    printf("\n[7] 记录损坏检测\n");
    const uint32_t rec1 = slot0 + FLASH_LOG_ENTRY_AREA_OFFSET + 1u * FLASH_LOG_ENTRY_STRIDE;
    chip.mem[rec1 + 1u + offsetof(LogEntry, tickMs)] ^= 0x01u;   // 翻转第 2 条数据里的 1 个 bit
    uint32_t c1b = 77;
    CHECK(fs4.countEntries(1, c1b, err) == FunctionResult::SUCCESS);
    CHECK(c1b == 1);           // 损坏处之后不再返回, 之前的仍然可读
    LogEntry e1;
    CHECK(fs4.readEntry(1, 1, e1, err) == FunctionResult::SUCCESS && e1.tickMs == 1u);

    // ---------------- 8. 布局指纹不匹配 -> 拒绝使用 ----------------
    printf("\n[8] 布局指纹校验\n");
    const uint32_t metaAddr = newestMetaAddr();
    CHECK(metaAddr != 0);
    FlashMetaRecord mr;
    memcpy(&mr, chip.mem + metaAddr + 1, sizeof(mr));
    mr.entrySize += 1u;
    mr.crc = 0;
    mr.crc = flashStructCrc(mr);
    memcpy(chip.mem + metaAddr + 1, &mr, sizeof(mr));
    FlashManager fs5(flash);
    CHECK(fs5.init(FlashFullPolicy::Halt, err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_GEOMETRY_MISMATCH);

    // ---------------- 9. 环形使用: 上电次数超过槽数 ----------------
    printf("\n[9] 日志槽环形覆写(130 次上电 / 127 个槽)\n");
    chip.eraseAll();
    const long erase0 = chip.eraseCalls;
    const long program0 = chip.programCalls;
    FlashManager fs6(flash);
    fs6.setSessionSlotLimit(1);
    CHECK(fs6.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
    for (uint32_t i = 0; i < 129; ++i) {
        CHECK(fs6.appendLog(makeEntry(i + 1, 0.5f, 0), err) == FunctionResult::SUCCESS);
        CHECK(fs6.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
        CHECK(err == ExceptionCode::FLASH_OK);
    }
    SessionCollect sc3 = {0, 0, 0, 0};
    CHECK(fs6.forEachSession(sessionCb, &sc3, err) == FunctionResult::SUCCESS);
    CHECK(sc3.count == FLASH_LOG_SLOT_COUNT);          // 只保留最近 127 次上电
    CHECK(sc3.slotSum == FLASH_LOG_SLOT_COUNT);
    CHECK(sc3.maxSession == 130u);
    CHECK(sc3.activeCount == 1);
    printf("  130 次上电: 扇区擦除 %ld 次(每次上电 2 个块擦除 + 偶尔元数据扇区擦除), 编程 %ld 次\n",
           chip.eraseCalls - erase0, chip.programCalls - program0);

    // ---------------- 10. 显式 format() ----------------
    printf("\n[10] 显式格式化\n");
    FlashManager fs7(flash);
    CHECK(fs7.format(err) == FunctionResult::SUCCESS && err == ExceptionCode::FLASH_OK);
    CHECK(fs7.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
    CHECK(err == ExceptionCode::FLASH_OK);   // 元数据已经存在, 不算"新建"
    CHECK(fs7.currentSessionId() == 1u);     // 会话号从头开始
    SessionCollect sc4 = {0, 0, 0, 0};
    CHECK(fs7.forEachSession(sessionCb, &sc4, err) == FunctionResult::SUCCESS);
    CHECK(sc4.count == 1);

    // ---------------- 11. 擦除量优化: 只擦需要回收的槽 ----------------
    printf("\n[11] 上电擦除量(只擦上次真正用掉的槽)\n");
    chip.eraseAll();
    FlashManager fs8(flash);
    fs8.setSessionSlotLimit(8);
    busySteps = 0;
    CHECK(fs8.init(FlashFullPolicy::Halt, onBusy, err) == FunctionResult::SUCCESS);
    CHECK(busySteps == W25Q128_CAPACITY / W25Q128_BLOCK64K_SIZE);   // 新建: 整片擦
    CHECK(fs8.appendLog(makeEntry(1, 0.5f, 0), err) == FunctionResult::SUCCESS);
    busySteps = 0;
    CHECK(fs8.init(FlashFullPolicy::Halt, onBusy, err) == FunctionResult::SUCCESS);
    CHECK(busySteps == 1u * (FLASH_LOG_SLOT_SIZE / W25Q128_BLOCK64K_SIZE));
    printf("  上次用 1 个槽 -> 本次只擦 %u 个 64KB 块(原来要擦 16 个)\n", (unsigned)busySteps);
    // 这次写满 1 个槽再多 1 条, 用掉 2 个槽
    for (uint32_t i = 0; i <= FLASH_LOG_ENTRIES_PER_SLOT; ++i) {
        CHECK(fs8.appendLog(makeEntry(i, 0.5f, 0), err) == FunctionResult::SUCCESS);
    }
    CHECK(fs8.sessionUsedSlots() == 2);
    busySteps = 0;
    CHECK(fs8.init(FlashFullPolicy::Halt, onBusy, err) == FunctionResult::SUCCESS);
    CHECK(busySteps == 2u * (FLASH_LOG_SLOT_SIZE / W25Q128_BLOCK64K_SIZE));
    printf("  上次用 2 个槽 -> 本次擦 %u 个 64KB 块\n", (unsigned)busySteps);

    // ---------------- 12. 清理最老的会话: 腾出的空间直接给当前会话用 ----------------
    printf("\n[12] 清理最老的会话\n");
    chip.eraseAll();
    FlashManager fs9(flash);
    fs9.setSessionSlotLimit(2);
    CHECK(fs9.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
    for (uint32_t boot = 0; boot < 4; ++boot) {          // 4 次上电: 会话 1..4, 每次 3 条(各占 1 个槽)
        for (uint32_t i = 1; i <= 3; ++i) {
            CHECK(fs9.appendLog(makeEntry(i, 0.5f, static_cast<uint16_t>(boot)), err)
                  == FunctionResult::SUCCESS);
        }
        if (boot < 3) {
            CHECK(fs9.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
        }
    }
    SessionCollect sc5 = {0, 0, 0, 0};
    CHECK(fs9.forEachSession(sessionCb, &sc5, err) == FunctionResult::SUCCESS);
    CHECK(sc5.count == 4u && sc5.maxSession == 4u);
    CHECK(fs9.currentSessionId() == 4u);
    CHECK(fs9.sessionReservedSlots() == 2u);

    // 丢掉最老的 2 个会话(1、2): 队头出队, 写指针不动
    busySteps = 0;
    CHECK(fs9.dropSessions(2, onBusy, err) == FunctionResult::SUCCESS && err == ExceptionCode::FLASH_OK);
    CHECK(busySteps == 2u * (FLASH_LOG_SLOT_SIZE / W25Q128_BLOCK64K_SIZE));   // 它们各占 1 个槽
    const uint32_t dropBlocks = busySteps;
    SessionCollect sc6 = {0, 0, 0, 0};
    CHECK(fs9.forEachSession(sessionCb, &sc6, err) == FunctionResult::SUCCESS);
    CHECK(sc6.count == 2u);              // 只剩会话 3、4
    CHECK(sc6.maxSession == 4u);
    CHECK(fs9.currentSessionId() == 4u); // 当前会话没被重开, 会话号也没变
    CHECK(fs9.sessionReservedSlots() == 4u);   // 腾出来的 2 个槽并进来了
    CHECK(fs9.sessionEntryLimit() == 4u * FLASH_LOG_ENTRIES_PER_SLOT);
    uint32_t cnt9 = 0;
    CHECK(fs9.countEntries(1u, cnt9, err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_SESSION_NOT_FOUND);   // 老会话确实没了
    CHECK(fs9.countEntries(3u, cnt9, err) == FunctionResult::SUCCESS && cnt9 == 3u);
    CHECK(fs9.countEntries(4u, cnt9, err) == FunctionResult::SUCCESS && cnt9 == 3u);

    // 接着写: 现在能一直写到 4 个槽(以前 2 个槽就 FLASH_LOG_FULL 了)
    uint32_t extra = 0;
    for (;;) {
        if (fs9.appendLog(makeEntry(100u + extra, 0.5f, 9), err) != FunctionResult::SUCCESS) {
            break;
        }
        ++extra;
        CHECK(extra <= 8u * FLASH_LOG_ENTRIES_PER_SLOT);
    }
    CHECK(err == ExceptionCode::FLASH_LOG_FULL);
    CHECK(fs9.currentEntryCount() == 4u * FLASH_LOG_ENTRIES_PER_SLOT);   // 预留区被完整用掉
    printf("  丢 2 个会话擦 %u 个块, 本次会话上限从 2 槽涨到 4 槽, 共写了 %u 条\n",
           (unsigned)dropBlocks, (unsigned)fs9.currentEntryCount());

    // 清空全部历史(只留当前会话)
    busySteps = 0;
    CHECK(fs9.clearHistory(onBusy, err) == FunctionResult::SUCCESS && err == ExceptionCode::FLASH_OK);
    SessionCollect sc7 = {0, 0, 0, 0};
    CHECK(fs9.forEachSession(sessionCb, &sc7, err) == FunctionResult::SUCCESS);
    CHECK(sc7.count == 1u);
    CHECK(fs9.currentSessionId() == 4u);
    CHECK(fs9.currentEntryCount() == 4u * FLASH_LOG_ENTRIES_PER_SLOT);   // 当前会话数据一条不少
    CHECK(fs9.sessionReservedSlots() == 5u);   // 会话 3 占的 1 个槽也收回来了
    CHECK(fs9.appendLog(makeEntry(7, 0.5f, 4), err) == FunctionResult::SUCCESS);  // 还能继续往下写
    CHECK(err == ExceptionCode::FLASH_OK);
    printf("  全清历史擦 %u 个块(丢弃最后 1 个老会话), 当前会话数据保留且上限又涨了 1 槽\n",
           (unsigned)busySteps);

    // ---------------- 13. 丢掉一个跨多个槽的最老会话 ----------------
    printf("\n[13] 丢跨多个槽的最老会话\n");
    chip.eraseAll();
    FlashManager fsA(flash);
    fsA.setSessionSlotLimit(2);
    CHECK(fsA.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);      // 会话 1
    for (uint32_t i = 0; i <= FLASH_LOG_ENTRIES_PER_SLOT; ++i) {                 // 用它 2 个槽
        CHECK(fsA.appendLog(makeEntry(i, 0.5f, 1), err) == FunctionResult::SUCCESS);
    }
    CHECK(fsA.sessionUsedSlots() == 2u);
    CHECK(fsA.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);      // 会话 2(当前)
    CHECK(fsA.appendLog(makeEntry(1, 0.5f, 2), err) == FunctionResult::SUCCESS);

    busySteps = 0;
    CHECK(fsA.dropSessions(1, onBusy, err) == FunctionResult::SUCCESS);          // 丢掉最老的会话 1
    CHECK(busySteps == 2u * (FLASH_LOG_SLOT_SIZE / W25Q128_BLOCK64K_SIZE));      // 它占了 2 个槽
    SessionCollect scA = {0, 0, 0, 0};
    CHECK(fsA.forEachSession(sessionCb, &scA, err) == FunctionResult::SUCCESS);
    CHECK(scA.count == 1u && scA.maxSession == 2u);
    CHECK(fsA.currentSessionId() == 2u);
    CHECK(fsA.sessionReservedSlots() == 4u);
    uint32_t cntA = 0;
    CHECK(fsA.countEntries(1u, cntA, err) == FunctionResult::FAILED);
    CHECK(err == ExceptionCode::FLASH_SESSION_NOT_FOUND);
    CHECK(fsA.countEntries(2u, cntA, err) == FunctionResult::SUCCESS && cntA == 1u);

    // 丢完接着写 + 下一次上电仍能算对起点
    CHECK(fsA.appendLog(makeEntry(2, 0.5f, 2), err) == FunctionResult::SUCCESS);
    CHECK(fsA.init(FlashFullPolicy::Halt, err) == FunctionResult::SUCCESS);
    CHECK(err == ExceptionCode::FLASH_OK);
    CHECK(fsA.currentSessionId() == 3u);
    uint32_t cntB = 0;
    CHECK(fsA.countEntries(2u, cntB, err) == FunctionResult::SUCCESS && cntB == 2u);

    printf("\n===== 检查 %d 项, 失败 %d 项 =====\n", g_checks, g_fails);
    printf("读字节 %ld / 编程 %ld 次 / 擦除 %ld 次\n", chip.readCalls, chip.programCalls, chip.eraseCalls);
    return (g_fails == 0) ? 0 : 1;
}

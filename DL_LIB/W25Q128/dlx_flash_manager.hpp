#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "dlx_w25q128.hpp"
#include "dlx_exception.h"
#include "dlx_flash_manager_config.h"

namespace dlx
{
    /**
     * @brief 简易日志 / 持久化参数文件系统(基于 W25Q128)
     *
     * 不做通用文件系统那套目录、变长文件、碎片整理, 只解决两件事:
     *   1. 把运行过程中的日志按"一次上电 = 一个日志会话"存下来, 断电重启后能接着存、能按会话回读;
     *   2. 把一组数值参数持久化, 掉电不丢, 支持反复改写(槽位轮转 + 磨损均衡)。
     *
     * 存储结构、帧格式、结构体约定全在 dlx_flash_manager_config.h 里, 本文件只负责行为。
     *
     * 使用流程:
     *     SPI spi = SPI::SPI2_SB10_MIC2_MOC3(cs);  spi.init(mode);
     *     W25Q128 flash(spi);
     *     flash.init();                            // 必须先做(校验 JEDEC ID)
     *     FlashManager fs(flash);
     *     ExceptionCode err;
     *     // 擦除过程比较长, 可以传一个空参 lambda 当"我正在干活"的提示(点灯/翻 IO/喂狗):
     *     fs.init(FlashFullPolicy::Halt, [&]{ led = !led; }, err);
     *     // 没建立过文件系统时: 整片擦除并新建, err = FLASH_SYSTEM_CREATED
     *     LogEntry e = {};  e.tickMs = 100;  fs.appendLog(e, err);
     *     fs.saveParams(params, err);  fs.loadParams(params, err);
     *
     *     fs.dropSessions(1, err);                 // 丢掉最老 1 个会话, 槽并入本次会话(可以写更久)
     *     fs.clearHistory(onBusy, err);            // 清空全部历史日志(保留当前会话), 接着往下写
     *
     * 一次上电 = 一个日志会话, 默认可以占用 8 个槽(1MB, 约 17472 条), 写满一个槽自动接着用下一个,
     * 不会中途反复擦除; 具体尺寸和上限都在 dlx_flash_manager_config.h 里调。
     *
     * 注意事项:
     *  - 非线程安全: 请在同一个上下文(主循环 / 同一线程)里调用;
     *  - 擦除是阻塞的: 一个 128KB 的槽约 0.3s(最坏 4s)。init() 只擦"需要回收的槽"——上次会话没写到的
     *    预留槽上次已经擦好, 不再重复擦; format() 擦整片, dropSessions() 擦被丢掉的那些槽。
     *    每擦完一个 64KB 块会回调一次进度函数(可以点灯/喂狗), 但擦除本身期间 CPU 是等在那里的。
     */
    class FlashManager
    {
    public:
        //由外部保证其生命周期
        FlashManager(W25Q128 &flashRef)
            : flash(flashRef)
        {
        }

        ~FlashManager()
        {
        }

        //===========================================================================================
        // 初始化 / 格式化
        //===========================================================================================
        /**
         * @brief 初始化: 判断文件系统是否已经建立, 并开始一次新的日志会话
         *
         * 流程:
         *  1. 读元数据段, 找最新的一条元数据记录;
         *  2. 找不到 -> 认为还没有文件系统: 整片擦除 + 写元数据(耗时几十秒), 随后照样建立会话,
         *     最终返回 SUCCESS 且 err = FLASH_SYSTEM_CREATED(提示"新建了", 这属于异常返回);
         *  3. 找到但布局/结构体大小对不上 -> 返回 FAILED 且 err = FLASH_GEOMETRY_MISMATCH(不动数据);
         *  4. 扫日志段槽头找到最新写过的槽, 把它后面的"预留区"(sessionReservedSlots() 个槽)
         *     一次性擦干净, 起点槽写槽头, 作为本次上电的会话;
         *  5. 元数据追加一条记录(上电次数 +1)。
         *
         * @param fullPolicy 写满策略(Halt / OverwriteOldest)
         * @param onProgress 空参回调, 每擦完一个 64KB 块调用一次(点灯/翻 IO/喂狗用);
         *                   不需要可以调用不传回调的那个重载。
         * @note  调用前必须先做过 flash.init()(否则返回 FLASH_HARDWARE_ERROR)。
         */
        template <typename ProgressFn>
        FunctionResult init(FlashFullPolicy fullPolicy, ProgressFn onProgress, ExceptionCode &err)
        {
            policy = fullPolicy;
            ready = false;

            if (!checkHardware(err)) {
                return FunctionResult::FAILED;
            }

            FlashMetaRecord meta;
            uint32_t metaOrdinal = 0;
            bool haveMeta = false;
            if (readNewestMeta(meta, metaOrdinal, haveMeta, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }

            bool created = false;
            if (!haveMeta) {
                // 还没有文件系统: 整片擦除后新建(整片都擦过了, 下面的预留区就不用再擦)
                if (format(onProgress, err) != FunctionResult::SUCCESS) {
                    return FunctionResult::FAILED;
                }
                if (readNewestMeta(meta, metaOrdinal, haveMeta, err) != FunctionResult::SUCCESS || !haveMeta) {
                    err = ExceptionCode::FLASH_WRITE_FAILED;
                    return FunctionResult::FAILED;
                }
                created = true;
            }

            // 布局/结构体大小必须与本次编译一致, 否则宁可报错也不按新格式去解析旧数据
            if (meta.magic != FLASH_MAGIC
                || meta.version != FLASH_FORMAT_VERSION
                || meta.metaSectors != FLASH_META_SECTOR_COUNT
                || meta.paramSectors != FLASH_PARAM_SECTOR_COUNT
                || meta.logSlotSectors != FLASH_LOG_SLOT_SECTOR_COUNT
                || meta.logSlots != FLASH_LOG_SLOT_COUNT
                || meta.entrySize != sizeof(LogEntry)
                || meta.paramSize != sizeof(FlashParamData)) {
                err = ExceptionCode::FLASH_GEOMETRY_MISMATCH;
                return FunctionResult::FAILED;
            }

            // 本次上电 = 一个新会话, 从"最新槽的下一个槽"开始预留
            SlotRef oldest;
            SlotRef newest;
            bool empty = true;
            if (findRingBounds(oldest, newest, empty, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }
            bootCount = meta.bootCount + 1u;
            sessionCounter = flashMaxU32(meta.sessionCounter, empty ? 0u : newest.sessionId);
            const uint32_t startSlot = empty ? 0u : ((newest.index + 1u) % FLASH_LOG_SLOT_COUNT);
            const uint32_t startSeq = (empty ? 0u : newest.seq) + 1u;
            const uint32_t session = sessionCounter + 1u;

            // 判断"上次预留区里还有多少槽是擦好的、没写过的" -> 这部分本次不用再擦
            uint32_t erasedPrefix = 0;
            if (created) {
                // 刚刚整片擦过, 整个预留区都是干净的
                erasedPrefix = FLASH_LOG_SLOT_COUNT;
            } else if (!empty && meta.reserveStartSeq <= newest.seq) {
                const uint32_t used = newest.seq - meta.reserveStartSeq + 1u; // 上次会话实际用了几个槽
                if (meta.reservedSlots >= 1u && used >= 1u && used <= meta.reservedSlots) {
                    erasedPrefix = meta.reservedSlots - used;
                }
            }

            if (startSession(startSlot, startSeq, session, erasedPrefix, onProgress, err)
                != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }
            err = created ? ExceptionCode::FLASH_SYSTEM_CREATED : ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        /** @brief 不带进度回调的 init() */
        FunctionResult init(FlashFullPolicy fullPolicy, ExceptionCode &err)
        {
            return init(fullPolicy, &flashNoProgress, err);
        }

        /**
         * @brief 强制重建文件系统: 整片擦除 + 写元数据
         * @param onProgress 空参回调, 每擦完一个 64KB 块调用一次(共 256 次, 点灯/喂狗用); 可以不传
         * @warning 会丢掉全部日志与参数; 整片擦除需要几十秒(16MB / 256 个 64KB 块)。
         * @note    调用之后还需要再 init() 才会开始新的会话。
         */
        template <typename ProgressFn>
        FunctionResult format(ProgressFn onProgress, ExceptionCode &err)
        {
            ready = false;
            if (!checkHardware(err)) {
                return FunctionResult::FAILED;
            }
            if (!eraseRange(FLASH_META_OFFSET, W25Q128_CAPACITY, onProgress)) {
                err = ExceptionCode::FLASH_ERASE_FAILED;
                return FunctionResult::FAILED;
            }
            bootCount = 1u;
            sessionCounter = 0u;
            // 记一条"空白"元数据: 会话号从 0 开始, 预留区信息为空(整片都是干净的)
            return appendMeta(bootCount, sessionCounter, 0u, 0u, 0u, err);
        }

        /** @brief 不带进度回调的 format() */
        FunctionResult format(ExceptionCode &err)
        {
            return format(&flashNoProgress, err);
        }

        //===========================================================================================
        // 清理日志(从最后一次上电往前丢)
        //===========================================================================================
        /**
         * @brief 丢掉**最老**的若干个日志会话, 把腾出来的槽直接并给"本次正在写的会话"
         *
         * 环形队列语义(不是栈): 写指针不动, 丢的永远是队头(最老)的那几次上电。
         * 这些槽擦干净后立刻并入本次会话的预留区, 所以:
         *  - 本次会话的上限变大, 可以一直往下写, **飞行中依然不会发生擦除**;
         *  - 空间账不变: 丢掉多少槽, 本次会话就能多写多少槽。
         *
         * @param count 丢几个最老的会话; 传 FLASH_DROP_ALL_SESSIONS = 只留当前会话(清空全部历史)
         * @param onProgress 空参回调, 每擦完一个 64KB 块调用一次(点灯/喂狗用); 可以不传
         * @note  - 当前正在写的这次上电是队尾, 永远不会被丢(count 超过历史会话数时按实际能丢的来);
         *        - 阻塞: 大约 被丢掉的槽数 × 0.3s;
         *        - 想连当前会话的数据一起清掉(恢复出厂)请用 format()。
         */
        template <typename ProgressFn>
        FunctionResult dropSessions(uint32_t count, ProgressFn onProgress, ExceptionCode &err)
        {
            if (!ready) {
                err = ExceptionCode::FLASH_NOT_READY;
                return FunctionResult::FAILED;
            }
            if (count == 0u) {
                err = ExceptionCode::FLASH_OK;
                return FunctionResult::SUCCESS;
            }

            SlotRef oldest;
            SlotRef newest;
            bool empty = true;
            if (findRingBounds(oldest, newest, empty, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }
            if (empty) {
                err = ExceptionCode::FLASH_OK; // 本来就没有会话
                return FunctionResult::SUCCESS;
            }

            // 从最老的一端往后数 count 个会话(一个会话在环里是连续占槽的), 不越过当前会话
            uint32_t dropSlots = 0;      // 这一段一共几个槽
            uint32_t droppedSessions = 0;
            uint32_t lastSession = 0;
            SlotRef cur = oldest;
            for (uint32_t step = 0; step < FLASH_LOG_SLOT_COUNT; ++step) {
                if (cur.sessionId == sessionId) {
                    break; // 到当前会话了, 它是队尾, 不丢
                }
                if (cur.sessionId != lastSession) {
                    if (droppedSessions >= count) {
                        break; // 已经凑够 count 个会话
                    }
                    lastSession = cur.sessionId;
                    ++droppedSessions;
                }
                ++dropSlots;

                const uint32_t nextIndex = (cur.index + 1u) % FLASH_LOG_SLOT_COUNT;
                SlotRef next;
                if (!readSlotRef(nextIndex, next) || next.seq != (cur.seq + 1u)) {
                    break; // 环里的数据到头了
                }
                cur = next;
            }
            if (dropSlots == 0u) {
                err = ExceptionCode::FLASH_OK; // 没有可丢的历史会话
                return FunctionResult::SUCCESS;
            }

            // 擦掉这一段: 它们紧跟在"本次预留区"后面, 擦完就与预留区连成一片干净区域
            for (uint32_t i = 0; i < dropSlots; ++i) {
                const uint32_t slot = (oldest.index + i) % FLASH_LOG_SLOT_COUNT;
                if (!eraseRange(logSlotAddr(slot), FLASH_LOG_SLOT_SIZE, onProgress)) {
                    err = ExceptionCode::FLASH_ERASE_FAILED;
                    return FunctionResult::FAILED;
                }
            }

            // 把腾出来的槽并入本次会话的预留区(写指针不动), 并补一条元数据,
            // 让下次上电也知道这段是干净的、不用重复擦。
            reservedSlots = flashMinU32(reservedSlots + dropSlots, FLASH_LOG_SLOT_COUNT);
            if (appendMeta(bootCount, sessionCounter, sessionStartSlot, sessionStartSeq, reservedSlots, err)
                != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        /** @brief 不带进度回调的 dropSessions() */
        FunctionResult dropSessions(uint32_t count, ExceptionCode &err)
        {
            return dropSessions(count, &flashNoProgress, err);
        }

        /**
         * @brief 清空全部历史日志(只保留当前正在写的这次会话), 腾出来的槽并入当前会话预留区
         * @note 等价于 dropSessions(FLASH_DROP_ALL_SESSIONS, ...); 想连当前会话一起清掉用 format()。
         */
        template <typename ProgressFn>
        FunctionResult clearHistory(ProgressFn onProgress, ExceptionCode &err)
        {
            return dropSessions(FLASH_DROP_ALL_SESSIONS, onProgress, err);
        }

        /** @brief 不带进度回调的 clearHistory() */
        FunctionResult clearHistory(ExceptionCode &err)
        {
            return dropSessions(FLASH_DROP_ALL_SESSIONS, &flashNoProgress, err);
        }

        //===========================================================================================
        // 日志写入
        //===========================================================================================
        /**
         * @brief 追加一条日志(自动带上当前会话号与序号)
         * @param entry 业务数据; 其中 FlashManager 使用的字段(type/reserved/crc/sessionId/seq)会被覆盖
         * @return 成功返回 SUCCESS; err = FLASH_LOG_OVERWRITTEN 表示本次已经环绕覆写了最旧的槽(写入本身成功)
         *
         * Halt 策略: 到达单次上限后返回 FAILED 且 err = FLASH_LOG_FULL, 之后继续调用仍是这个错误,
         *            直到下次 init() 或者把策略切换成 OverwriteOldest。
         */
        FunctionResult appendLog(const LogEntry &entry, ExceptionCode &err)
        {
            if (!ready) {
                err = ExceptionCode::FLASH_NOT_READY;
                return FunctionResult::FAILED;
            }

            bool overwroteOldest = false;
            if (slotEntries >= FLASH_LOG_ENTRIES_PER_SLOT) {
                if (!canAdvanceSlot()) {
                    // Halt 策略下的"单次日志上限": 预留区已经用满
                    err = ExceptionCode::FLASH_LOG_FULL;
                    return FunctionResult::FAILED;
                }
                // 预留区用完了还要往下走, 这一步会擦掉最旧的会话(OverwriteOldest 策略)
                overwroteOldest = (slotUsed >= reservedSlots);
                if (advanceSlot(err) != FunctionResult::SUCCESS) {
                    return FunctionResult::FAILED;
                }
            }

            LogEntry rec = entry;
            rec.type = static_cast<uint8_t>(FlashRecordType::Entry);
            rec.reserved = 0;
            rec.sessionId = sessionId;
            rec.seq = entryCount + 1u;
            rec.crc = flashStructCrc(rec);

            if (!writeFrame(logEntryAddr(curSlot, slotEntries), &rec, sizeof(rec))) {
                err = ExceptionCode::FLASH_WRITE_FAILED;
                return FunctionResult::FAILED;
            }
            ++slotEntries;
            ++entryCount;
            err = overwroteOldest ? ExceptionCode::FLASH_LOG_OVERWRITTEN : ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        //===========================================================================================
        // 日志读取(低频: 导出 / 回放时用)
        //===========================================================================================
        /**
         * @brief 遍历日志段里现存的所有会话(按时间从旧到新)
         * @note  只扫槽头, 不做全段读取, 所以很快; 具体条目数请用 countEntries()。
         *        回调返回 false 可以提前结束遍历。
         */
        FunctionResult forEachSession(FlashSessionCallback callback, void *ctx, ExceptionCode &err)
        {
            if (!ready) {
                err = ExceptionCode::FLASH_NOT_READY;
                return FunctionResult::FAILED;
            }
            if (callback == nullptr) {
                err = ExceptionCode::FLASH_INVALID_ARGUMENT;
                return FunctionResult::FAILED;
            }

            SlotRef oldest;
            SlotRef newest;
            bool empty = true;
            if (findRingBounds(oldest, newest, empty, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }
            if (empty) {
                err = ExceptionCode::FLASH_OK;
                return FunctionResult::SUCCESS;
            }

            FlashLogSessionInfo info;
            info.sessionId = 0;
            info.firstSlot = 0;
            info.slotCount = 0;
            info.regionBytes = 0;
            info.active = false;
            uint32_t lastSlot = 0;

            SlotRef cur = oldest;
            for (;;) {
                if (info.slotCount == 0) {
                    info.sessionId = cur.sessionId;
                    info.firstSlot = cur.index;
                    info.slotCount = 1;
                } else if (cur.sessionId == info.sessionId) {
                    info.slotCount += 1;
                } else {
                    info.regionBytes = info.slotCount * FLASH_LOG_SLOT_SIZE;
                    info.active = (lastSlot == newest.index);
                    if (!callback(info, ctx)) {
                        err = ExceptionCode::FLASH_OK;
                        return FunctionResult::SUCCESS;
                    }
                    info.sessionId = cur.sessionId;
                    info.firstSlot = cur.index;
                    info.slotCount = 1;
                }
                lastSlot = cur.index;

                if (cur.index == newest.index) {
                    info.regionBytes = info.slotCount * FLASH_LOG_SLOT_SIZE;
                    info.active = true;
                    callback(info, ctx);
                    break;
                }
                SlotRef next;
                const uint32_t nextIndex = (cur.index + 1u) % FLASH_LOG_SLOT_COUNT;
                if (!readSlotRef(nextIndex, next) || next.seq != (cur.seq + 1u)) {
                    info.regionBytes = info.slotCount * FLASH_LOG_SLOT_SIZE;
                    info.active = (lastSlot == newest.index);
                    callback(info, ctx);
                    break;
                }
                cur = next;
            }

            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        /** @brief 某个会话里存了多少条日志 */
        FunctionResult countEntries(uint32_t targetSession, uint32_t &count, ExceptionCode &err)
        {
            EntryCounter counter;
            counter.count = 0;
            const FunctionResult result = forEachEntry(targetSession, countEntryCallback, &counter, err);
            if (result == FunctionResult::SUCCESS) {
                count = counter.count;
            }
            return result;
        }

        /**
         * @brief 按序号读取一条日志
         * @param seq 会话内序号(从 1 开始)
         */
        FunctionResult readEntry(uint32_t targetSession, uint32_t seq, LogEntry &out, ExceptionCode &err)
        {
            if (seq == 0u) {
                err = ExceptionCode::FLASH_INVALID_ARGUMENT;
                return FunctionResult::FAILED;
            }
            EntryFinder finder;
            finder.target = seq;
            finder.out = &out;
            finder.found = false;
            const FunctionResult result = forEachEntry(targetSession, findEntryCallback, &finder, err);
            if (result != FunctionResult::SUCCESS) {
                return result;
            }
            if (!finder.found) {
                err = ExceptionCode::FLASH_ENTRY_NOT_FOUND;
                return FunctionResult::FAILED;
            }
            return FunctionResult::SUCCESS;
        }

        /**
         * @brief 遍历某个会话的全部日志条目(按写入顺序)
         * @note  遇到"没写完 / 校验失败"的记录就停(正常情况下就是上次掉电时那一条),
         *        它之前的数据仍然是完整可用的。回调返回 false 可以提前结束。
         */
        FunctionResult forEachEntry(uint32_t targetSession, FlashEntryCallback callback, void *ctx, ExceptionCode &err)
        {
            if (!ready) {
                err = ExceptionCode::FLASH_NOT_READY;
                return FunctionResult::FAILED;
            }
            if (callback == nullptr) {
                err = ExceptionCode::FLASH_INVALID_ARGUMENT;
                return FunctionResult::FAILED;
            }

            SlotRef first;
            uint32_t slotCount = 0;
            if (findSession(targetSession, first, slotCount, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }

            SlotRef cur = first;
            for (uint32_t s = 0; s < slotCount; ++s) {
                for (uint32_t i = 0; i < FLASH_LOG_ENTRIES_PER_SLOT; ++i) {
                    LogEntry entry;
                    if (!readFrame(logEntryAddr(cur.index, i), entry)) {
                        err = ExceptionCode::FLASH_OK;
                        return FunctionResult::SUCCESS; // 读到没写完的记录(或槽尾空区): 本次会话到此为止
                    }
                    if (entry.type != static_cast<uint8_t>(FlashRecordType::Entry)
                        || entry.sessionId != targetSession) {
                        err = ExceptionCode::FLASH_OK;
                        return FunctionResult::SUCCESS;
                    }
                    if (!callback(entry, ctx)) {
                        err = ExceptionCode::FLASH_OK;
                        return FunctionResult::SUCCESS;
                    }
                }
                if ((s + 1u) < slotCount) {
                    SlotRef next;
                    const uint32_t nextIndex = (cur.index + 1u) % FLASH_LOG_SLOT_COUNT;
                    if (!readSlotRef(nextIndex, next)) {
                        break;
                    }
                    cur = next;
                }
            }
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        //===========================================================================================
        // 持久化参数
        //===========================================================================================
        /** @brief 读出最新一次保存的参数; 一条都没有时返回 FLASH_PARAM_NOT_FOUND(上层用默认值) */
        FunctionResult loadParams(FlashParamData &out, ExceptionCode &err)
        {
            if (!ready) {
                err = ExceptionCode::FLASH_NOT_READY;
                return FunctionResult::FAILED;
            }
            FlashParamSlot newest;
            uint32_t ordinal = 0;
            if (!findNewestParam(newest, ordinal)) {
                err = ExceptionCode::FLASH_PARAM_NOT_FOUND;
                return FunctionResult::FAILED;
            }
            out = newest.data;
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        /**
         * @brief 只读探测一次参数(不需要 init(), 也不会建立新会话)
         * @note  用在"上电早期按参数决定日志档位"这类场景:
         *        peekParams() 看看上次存的是不是"全量日志"模式 -> setSessionSlotLimit() -> init()。
         *        文件系统还没建立时返回 FLASH_PARAM_NOT_FOUND。
         */
        FunctionResult peekParams(FlashParamData &out, ExceptionCode &err)
        {
            if (!checkHardware(err)) {
                return FunctionResult::FAILED;
            }
            FlashParamSlot newest;
            uint32_t ordinal = 0;
            if (!findNewestParam(newest, ordinal)) {
                err = ExceptionCode::FLASH_PARAM_NOT_FOUND;
                return FunctionResult::FAILED;
            }
            out = newest.data;
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        /**
         * @brief 保存参数(追加一个新槽, 不擦旧数据, 所以掉电最多丢这一次的写入)
         * @note  参数段写满一个扇区才擦除那个扇区, 擦除在扇区之间轮转(磨损均衡)。
         */
        FunctionResult saveParams(const FlashParamData &in, ExceptionCode &err)
        {
            if (!ready) {
                err = ExceptionCode::FLASH_NOT_READY;
                return FunctionResult::FAILED;
            }

            FlashParamSlot newest;
            uint32_t newestOrdinal = 0;
            const bool found = findNewestParam(newest, newestOrdinal);
            const uint32_t nextOrdinal = found ? ((newestOrdinal + 1u) % FLASH_PARAM_SLOT_COUNT) : 0u;
            const uint32_t nextSeq = found ? (newest.seq + 1u) : 1u;
            const uint32_t addr = paramSlotAddr(nextOrdinal);

            if ((nextOrdinal % FLASH_PARAM_SLOTS_PER_SECTOR) == 0u) {
                if (!flash.eraseSector(addr)) {
                    err = ExceptionCode::FLASH_ERASE_FAILED;
                    return FunctionResult::FAILED;
                }
            }

            FlashParamSlot slot;
            slot.type = static_cast<uint8_t>(FlashRecordType::Param);
            slot.version = static_cast<uint8_t>(in.version & 0xFFu);
            slot.crc = 0;
            slot.seq = nextSeq;
            slot.data = in;
            slot.crc = flashStructCrc(slot);

            if (!writeFrame(addr, &slot, sizeof(slot))) {
                err = ExceptionCode::FLASH_WRITE_FAILED;
                return FunctionResult::FAILED;
            }
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        //===========================================================================================
        // 状态查询
        //===========================================================================================
        bool isReady() const { return ready; }
        /** @brief 当前会话号(每次上电 +1, 用于把条目区分到不同次上电) */
        uint32_t currentSessionId() const { return sessionId; }
        /** @brief 当前会话已经写入的日志条数 */
        uint32_t currentEntryCount() const { return entryCount; }
        /** @brief 本次会话预留了几个槽(这些槽在 init 时已经一次性擦干净) */
        uint32_t sessionReservedSlots() const { return reservedSlots; }
        /** @brief 本次会话已经用掉几个槽 */
        uint32_t sessionUsedSlots() const { return slotUsed; }
        /** @brief 本次会话的条目上限(= 预留槽数 × 每个槽的条目数) */
        uint32_t sessionEntryLimit() const { return reservedSlots * FLASH_LOG_ENTRIES_PER_SLOT; }
        /** @brief 设置单次日志最多占用几个槽(1 ~ FLASH_LOG_SLOT_COUNT), 下一次 init() 生效 */
        void setSessionSlotLimit(uint32_t slots)
        {
            sessionSlotLimit = flashClampU32(slots, 1u, FLASH_LOG_SLOT_COUNT);
        }
        /** @brief Halt 策略下本次日志是否已经写满(写满后 appendLog 一直返回 FLASH_LOG_FULL) */
        bool isLogFull() const { return ready && atWriteLimit(); }
        /** @brief 运行中切换写满策略(切到 OverwriteOldest 可以马上接着写) */
        void setFullPolicy(FlashFullPolicy newPolicy)
        {
            policy = newPolicy;
        }

    private:
        // ---------------- 运行状态(全部可以由扫描恢复, 不需要掉电保存) ----------------
        W25Q128 &flash; ///< 由外部保证生命周期

        FlashFullPolicy policy = FlashFullPolicy::Halt; ///< 写满策略
        bool     ready       = false; ///< init() 是否成功
        uint32_t sessionId   = 0;     ///< 当前会话号
        uint32_t sessionCounter = 0;  ///< 已经分配出去的最大会话号(会话号只增不减)
        uint32_t bootCount   = 0;     ///< 上电次数(写进元数据)
        uint32_t sessionSlotLimit = FLASH_LOG_SESSION_SLOT_LIMIT; ///< 单次日志最多占用几个槽(下一次 init 生效)
        uint32_t reservedSlots    = 0; ///< 本次会话实际预留的槽数
        uint32_t sessionStartSlot = 0; ///< 本次会话起点槽(写元数据/清理时要记/用)
        uint32_t sessionStartSeq  = 0; ///< 本次会话起点槽的槽序号
        uint32_t slotUsed    = 0;     ///< 本次会话已经占用几个槽
        uint32_t curSlot     = 0;     ///< 当前写入的槽(物理序号)
        uint32_t curSlotSeq  = 0;     ///< 当前槽的槽序号
        uint32_t slotEntries = 0;     ///< 当前槽已经写了多少条
        uint32_t entryCount  = 0;     ///< 当前会话已经写了多少条(等于最后一条的 seq)

        /** 收发缓冲: 所有记录都借用它, 避免在大结构体上占用调用者栈空间 */
        uint8_t frameBuf[FLASH_FRAME_BUF_BYTES];

        // ---------------- 地址计算 ----------------
        static uint32_t metaSlotAddr(uint32_t ordinal)
        {
            return FLASH_META_OFFSET
                 + (ordinal / FLASH_META_SLOTS_PER_SECTOR) * FLASH_SECTOR_SIZE
                 + (ordinal % FLASH_META_SLOTS_PER_SECTOR) * FLASH_META_SLOT_BYTES;
        }

        static uint32_t paramSlotAddr(uint32_t ordinal)
        {
            return FLASH_PARAM_OFFSET
                 + (ordinal / FLASH_PARAM_SLOTS_PER_SECTOR) * FLASH_SECTOR_SIZE
                 + (ordinal % FLASH_PARAM_SLOTS_PER_SECTOR) * FLASH_PARAM_SLOT_BYTES;
        }

        static uint32_t logSlotAddr(uint32_t slotIndex)
        {
            return FLASH_LOG_OFFSET + slotIndex * FLASH_LOG_SLOT_SIZE;
        }

        static uint32_t logEntryAddr(uint32_t slotIndex, uint32_t index)
        {
            return logSlotAddr(slotIndex) + FLASH_LOG_ENTRY_AREA_OFFSET + index * FLASH_LOG_ENTRY_STRIDE;
        }

        // ---------------- 帧读写 ----------------
        /**
         * @brief 写一条记录: [0xAA][payload][0x00]
         * @note  先写"帧头 + 负载", 提交标记(0x00)最后单独写 1 个字节; 掉电若发生在两次写之间,
         *        这条记录就是"没写完"的, 读取时会被丢弃。同一条记录重写是安全的(Flash 只能把 1 写成 0,
         *        重复写同样的值不会出错), 所以写失败后下次会原地重写。
         */
        bool writeFrame(uint32_t addr, const void *payload, uint32_t bytes)
        {
            frameBuf[0] = FLASH_FRAME_HEAD;
            memcpy(frameBuf + 1, payload, bytes);
            frameBuf[1 + bytes] = FLASH_FRAME_TAIL;
            if (!flash.write(addr, frameBuf, bytes + 1u)) {
                return false;
            }
            const uint8_t tail = FLASH_FRAME_TAIL;
            return flash.write(addr + bytes + 1u, &tail, 1u);
        }

        /** @brief 读一条记录并校验帧头/帧尾/CRC; 返回 false = 这里没有完整记录(没写过/写一半/坏了) */
        template <typename T>
        bool readFrame(uint32_t addr, T &payload)
        {
            if (!flash.read(addr, frameBuf, sizeof(T) + 2u)) {
                return false;
            }
            if (frameBuf[0] != FLASH_FRAME_HEAD || frameBuf[sizeof(T) + 1u] != FLASH_FRAME_TAIL) {
                return false;
            }
            memcpy(&payload, frameBuf + 1, sizeof(T));
            return flashStructCrc(payload) == payload.crc;
        }

        /** @brief 读槽头(顺带校验"物理序号"与内容是否对得上) */
        bool readSlotHeader(uint32_t slotIndex, FlashLogSlotHeader &out)
        {
            if (!readFrame(logSlotAddr(slotIndex), out)) {
                return false;
            }
            return out.type == static_cast<uint8_t>(FlashRecordType::SlotHeader)
                && out.slotIndex == slotIndex;
        }

        // ---------------- 擦除 ----------------
        /**
         * @brief 从 addr 起按 64KB 块擦除 bytes 字节(要求 addr 64KB 对齐, bytes 是 64KB 整数倍)
         * @param onProgress 每擦完一个块回调一次(点灯/翻 IO/喂狗)
         */
        template <typename ProgressFn>
        bool eraseRange(uint32_t addr, uint32_t bytes, ProgressFn onProgress)
        {
            for (uint32_t offset = 0; offset < bytes; offset += W25Q128_BLOCK64K_SIZE) {
                if (!flash.eraseBlock64K(addr + offset)) {
                    return false;
                }
                onProgress();
            }
            return true;
        }

        /** @brief 擦除一个日志槽(128KB = 2 个 64KB 块) */
        bool eraseSlot(uint32_t slotIndex)
        {
            const uint32_t base = logSlotAddr(slotIndex);
            for (uint32_t offset = 0; offset < FLASH_LOG_SLOT_SIZE; offset += W25Q128_BLOCK64K_SIZE) {
                if (!flash.eraseBlock64K(base + offset)) {
                    return false;
                }
            }
            return true;
        }

        bool checkHardware(ExceptionCode &err)
        {
            const uint32_t jedec = flash.readJEDEC();
            const uint32_t expect = (static_cast<uint32_t>(W25Q128_MANUFACTURER_ID) << 16)
                                  | (static_cast<uint32_t>(W25Q128_MEMORY_TYPE) << 8)
                                  | static_cast<uint32_t>(W25Q128_CAPACITY_ID);
            if (jedec != expect) {
                err = ExceptionCode::FLASH_HARDWARE_ERROR;
                return false;
            }
            return true;
        }

        // ---------------- 元数据段 ----------------
        /**
         * @brief 读出序号最大的一条元数据记录
         * @param ordinal 该记录所在的槽序号(下一次接着写要用到)
         *
         * 优化: 同一扇区内的槽是顺序写的, 所以"最新记录所在的扇区"就是"0 号槽序号最大的扇区"。
         * 先扫每个扇区的 0 号槽(几十次小读), 再细扫那一个扇区, 不必整段扫描。
         */
        FunctionResult readNewestMeta(FlashMetaRecord &out, uint32_t &ordinal, bool &found, ExceptionCode &err)
        {
            found = false;
            bool haveSector = false;
            uint32_t bestSector = 0;
            uint32_t bestSectorSeq = 0;

            for (uint32_t sector = 0; sector < FLASH_META_SECTOR_COUNT; ++sector) {
                const uint32_t slot = sector * FLASH_META_SLOTS_PER_SECTOR;
                FlashMetaRecord rec;
                if (!readFrame(metaSlotAddr(slot), rec)) {
                    continue;
                }
                if (rec.type != static_cast<uint8_t>(FlashRecordType::Meta)) {
                    continue;
                }
                if (!haveSector || rec.seq > bestSectorSeq) {
                    haveSector = true;
                    bestSector = sector;
                    bestSectorSeq = rec.seq;
                }
            }
            if (!haveSector) {
                err = ExceptionCode::FLASH_OK;
                return FunctionResult::SUCCESS;
            }

            uint32_t bestSeq = 0;
            for (uint32_t k = 0; k < FLASH_META_SLOTS_PER_SECTOR; ++k) {
                const uint32_t slot = bestSector * FLASH_META_SLOTS_PER_SECTOR + k;
                FlashMetaRecord rec;
                if (!readFrame(metaSlotAddr(slot), rec)) {
                    continue;
                }
                if (rec.type != static_cast<uint8_t>(FlashRecordType::Meta)) {
                    continue;
                }
                if (!found || rec.seq > bestSeq) {
                    found = true;
                    bestSeq = rec.seq;
                    ordinal = slot;
                    out = rec;
                }
            }
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        /**
         * @brief 追加一条元数据记录(环形写, 写满一个扇区才擦该扇区 -> 磨损均衡)
         * @param reserveStartSlot/reserveStartSeq/reserveSlots 当前会话预留区信息(见 config 里的说明),
         *        用于下次上电判断"哪些槽已经擦好了、不用再擦"
         */
        FunctionResult appendMeta(uint32_t boots, uint32_t sessions, uint32_t reserveSlot,
                                  uint32_t reserveSeq, uint32_t reserveCount, ExceptionCode &err)
        {
            FlashMetaRecord newest;
            uint32_t newestOrdinal = 0;
            bool found = false;
            if (readNewestMeta(newest, newestOrdinal, found, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }

            uint32_t nextOrdinal = 0;
            uint32_t nextSeq = 1;
            if (found) {
                nextOrdinal = (newestOrdinal + 1u) % FLASH_META_SLOT_COUNT;
                nextSeq = newest.seq + 1u;
                // 跨到新扇区: 这个扇区里全是上一轮的旧记录, 先整扇区擦掉
                if ((nextOrdinal % FLASH_META_SLOTS_PER_SECTOR) == 0u) {
                    if (!flash.eraseSector(metaSlotAddr(nextOrdinal))) {
                        err = ExceptionCode::FLASH_ERASE_FAILED;
                        return FunctionResult::FAILED;
                    }
                }
            }

            FlashMetaRecord rec;
            rec.type = static_cast<uint8_t>(FlashRecordType::Meta);
            rec.version = FLASH_FORMAT_VERSION;
            rec.crc = 0;
            rec.seq = nextSeq;
            rec.magic = FLASH_MAGIC;
            rec.bootCount = boots;
            rec.sessionCounter = sessions;
            rec.reserveStartSlot = reserveSlot;
            rec.reserveStartSeq = reserveSeq;
            rec.reservedSlots = reserveCount;
            rec.metaSectors = FLASH_META_SECTOR_COUNT;
            rec.paramSectors = FLASH_PARAM_SECTOR_COUNT;
            rec.logSlotSectors = FLASH_LOG_SLOT_SECTOR_COUNT;
            rec.logSlots = FLASH_LOG_SLOT_COUNT;
            rec.entrySize = sizeof(LogEntry);
            rec.paramSize = sizeof(FlashParamData);
            rec.crc = flashStructCrc(rec);

            if (!writeFrame(metaSlotAddr(nextOrdinal), &rec, sizeof(rec))) {
                err = ExceptionCode::FLASH_WRITE_FAILED;
                return FunctionResult::FAILED;
            }
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        // ---------------- 会话 / 预留区 ----------------
        /**
         * @brief 开始一个新会话: 保证预留区擦干净 -> 写起点槽头 -> 记元数据
         *
         * @param startSlot    会话起点槽(物理槽号)
         * @param startSeq     起点槽的槽序号(继续单调递增)
         * @param session      会话号
         * @param erasedPrefix 从 startSlot 往后, 已经有几个槽是"擦干净且没写过"的
         *                     (上次预留区没用到的部分 + 刚刚被清掉的会话), 这部分不用再擦
         *
         * 只有真正需要回收的槽才擦: 擦除量 = 预留槽数 - erasedPrefix。
         */
        template <typename ProgressFn>
        FunctionResult startSession(uint32_t startSlot, uint32_t startSeq, uint32_t session,
                                    uint32_t erasedPrefix, ProgressFn onProgress, ExceptionCode &err)
        {
            reservedSlots = flashClampU32(sessionSlotLimit, 1u, FLASH_LOG_SLOT_COUNT);
            const uint32_t skip = flashMinU32(erasedPrefix, reservedSlots);
            for (uint32_t i = skip; i < reservedSlots; ++i) {
                const uint32_t slot = (startSlot + i) % FLASH_LOG_SLOT_COUNT;
                if (!eraseRange(logSlotAddr(slot), FLASH_LOG_SLOT_SIZE, onProgress)) {
                    err = ExceptionCode::FLASH_ERASE_FAILED;
                    return FunctionResult::FAILED;
                }
            }

            // 只给起点槽写槽头; 预留区里其余槽等到真正用到时再写,
            // 这样"最新槽"永远等于最后真正写过的那个槽, 下次上电才能算对起点。
            if (writeSlotHeader(startSlot, startSeq, session, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }

            sessionId = session;
            sessionCounter = session;
            sessionStartSlot = startSlot;
            sessionStartSeq = startSeq;
            slotUsed = 1;
            curSlot = startSlot;
            curSlotSeq = startSeq;
            slotEntries = 0;
            entryCount = 0;

            if (appendMeta(bootCount, sessionCounter, startSlot, startSeq, reservedSlots, err)
                != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }
            ready = true;
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        // ---------------- 参数段 ----------------
        /** @brief 找序号最大的一条参数槽; 找不到返回 false */
        bool findNewestParam(FlashParamSlot &out, uint32_t &ordinal)
        {
            bool found = false;
            for (uint32_t i = 0; i < FLASH_PARAM_SLOT_COUNT; ++i) {
                FlashParamSlot slot;
                if (!readFrame(paramSlotAddr(i), slot)) {
                    continue;
                }
                if (slot.type != static_cast<uint8_t>(FlashRecordType::Param)) {
                    continue;
                }
                if (!found || slot.seq > out.seq) {
                    found = true;
                    ordinal = i;
                    out = slot;
                }
            }
            return found;
        }

        // ---------------- 日志段 ----------------
        struct SlotRef
        {
            uint32_t index;
            uint32_t seq;
            uint32_t sessionId;
        };

        bool readSlotRef(uint32_t slotIndex, SlotRef &ref)
        {
            FlashLogSlotHeader h;
            if (!readSlotHeader(slotIndex, h)) {
                return false;
            }
            ref.index = slotIndex;
            ref.seq = h.slotSeq;
            ref.sessionId = h.sessionId;
            return true;
        }

        /**
         * @brief 求出日志环的"最旧槽"和"最新槽"
         *
         * 槽序号是全局单调递增的, 而槽在日志段里是环形使用的:
         *  - 最新槽 = 槽序号最大的那个;
         *  - 从最新槽往回走, 只要"槽序号正好小 1"就继续走, 走到不连续为止, 停在最旧槽;
         *    整环都用过时, 往回走 126 步正好停在最新槽的下一个(也就是马上要被覆写的那个)。
         * 全程 O(1) RAM, 不需要把会话表放进内存。
         */
        FunctionResult findRingBounds(SlotRef &oldest, SlotRef &newest, bool &empty, ExceptionCode &err)
        {
            empty = true;
            SlotRef best;
            best.index = 0;
            best.seq = 0;
            best.sessionId = 0;
            for (uint32_t i = 0; i < FLASH_LOG_SLOT_COUNT; ++i) {
                SlotRef ref;
                if (!readSlotRef(i, ref)) {
                    continue;
                }
                if (empty || ref.seq > best.seq) {
                    empty = false;
                    best = ref;
                }
            }
            if (empty) {
                err = ExceptionCode::FLASH_OK;
                return FunctionResult::SUCCESS;
            }

            newest = best;
            SlotRef cur = best;
            for (uint32_t step = 1; step < FLASH_LOG_SLOT_COUNT; ++step) {
                const uint32_t prevIndex = (cur.index + FLASH_LOG_SLOT_COUNT - 1u) % FLASH_LOG_SLOT_COUNT;
                SlotRef prev;
                if (!readSlotRef(prevIndex, prev) || prev.seq != (cur.seq - 1u)) {
                    break;
                }
                cur = prev;
            }
            oldest = cur;
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        /** @brief 写槽头(要求该槽已经擦干净) */
        FunctionResult writeSlotHeader(uint32_t slotIndex, uint32_t slotSeq, uint32_t session, ExceptionCode &err)
        {
            FlashLogSlotHeader h;
            h.type = static_cast<uint8_t>(FlashRecordType::SlotHeader);
            h.version = FLASH_FORMAT_VERSION;
            h.crc = 0;
            h.slotSeq = slotSeq;
            h.slotIndex = slotIndex;
            h.sessionId = session;
            h.crc = flashStructCrc(h);
            if (!writeFrame(logSlotAddr(slotIndex), &h, sizeof(h))) {
                err = ExceptionCode::FLASH_WRITE_FAILED;
                return FunctionResult::FAILED;
            }
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        /** @brief 当前槽写满之后还能不能再拿一个槽(OverwriteOldest 策略永远可以) */
        bool canAdvanceSlot() const
        {
            return (policy == FlashFullPolicy::OverwriteOldest) || (slotUsed < reservedSlots);
        }

        /** @brief 当前是否已经到"本次会话写不下一条"的边界 */
        bool atWriteLimit() const
        {
            return (slotEntries >= FLASH_LOG_ENTRIES_PER_SLOT) && !canAdvanceSlot();
        }

        /**
         * @brief 用下一个槽继续写同一个会话
         * @note  预留区内的槽 init() 时已经擦好了, 直接用; 超出预留区(OverwriteOldest 策略)
         *        才需要现场擦除——那会阻塞约 0.3s, 换来的是继续保留最新日志。
         *        这种"运行中擦除"属于非正常工况(飞行中停下几毫秒等擦除等于丢日志),
         *        因此静默执行, 不回调进度函数。
         */
        FunctionResult advanceSlot(ExceptionCode &err)
        {
            const uint32_t next = (curSlot + 1u) % FLASH_LOG_SLOT_COUNT;
            if (slotUsed >= reservedSlots) {
                if (!eraseSlot(next)) {
                    err = ExceptionCode::FLASH_ERASE_FAILED;
                    return FunctionResult::FAILED;
                }
            }
            if (writeSlotHeader(next, curSlotSeq + 1u, sessionId, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }
            curSlot = next;
            curSlotSeq += 1u;
            slotEntries = 0;
            slotUsed += 1u;
            return FunctionResult::SUCCESS;
        }

        /** @brief 找到某个会话的起始槽与占用的槽数(会话在环里是连续占槽的) */
        FunctionResult findSession(uint32_t target, SlotRef &first, uint32_t &slotCount, ExceptionCode &err)
        {
            SlotRef oldest;
            SlotRef newest;
            bool empty = true;
            if (findRingBounds(oldest, newest, empty, err) != FunctionResult::SUCCESS) {
                return FunctionResult::FAILED;
            }
            if (empty) {
                err = ExceptionCode::FLASH_SESSION_NOT_FOUND;
                return FunctionResult::FAILED;
            }

            bool found = false;
            slotCount = 0;
            SlotRef cur = oldest;
            for (;;) {
                if (cur.sessionId == target) {
                    if (!found) {
                        found = true;
                        first = cur;
                    }
                    slotCount += 1u;
                } else if (found) {
                    break; // 一个会话的槽是连续的, 断了就说明它结束了
                }
                if (cur.index == newest.index) {
                    break;
                }
                const uint32_t nextIndex = (cur.index + 1u) % FLASH_LOG_SLOT_COUNT;
                SlotRef next;
                if (!readSlotRef(nextIndex, next) || next.seq != (cur.seq + 1u)) {
                    break;
                }
                cur = next;
            }

            if (!found) {
                err = ExceptionCode::FLASH_SESSION_NOT_FOUND;
                return FunctionResult::FAILED;
            }
            err = ExceptionCode::FLASH_OK;
            return FunctionResult::SUCCESS;
        }

        // ---------------- 回调用的临时上下文 ----------------
        struct EntryCounter
        {
            uint32_t count;
        };

        struct EntryFinder
        {
            uint32_t target;
            LogEntry *out;
            bool found;
        };

        static bool countEntryCallback(const LogEntry &, void *ctx)
        {
            reinterpret_cast<EntryCounter *>(ctx)->count += 1u;
            return true;
        }

        static bool findEntryCallback(const LogEntry &entry, void *ctx)
        {
            EntryFinder *finder = reinterpret_cast<EntryFinder *>(ctx);
            if (entry.seq == finder->target) {
                *finder->out = entry;
                finder->found = true;
                return false;
            }
            return true;
        }
    };
} // namespace dlx

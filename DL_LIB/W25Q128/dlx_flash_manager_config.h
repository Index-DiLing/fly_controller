#pragma once
#include <stdint.h>
#include <stddef.h>
#include "dlx_w25q128_config.h"

//===================================================================================================
// dlx_flash_manager_config.h
// FlashManager(简易日志 + 持久化参数文件系统)的全部"约定":
//   1. 帧格式        每条记录都是 [0xAA][定长结构体][0x00]
//   2. 分段布局      元数据段 / 持久化参数段 / 日志段 (顺序固定, 全部 4KB 扇区对齐)
//   3. 记录结构体    元数据记录 / 参数槽 / 日志槽头 / 日志条目
//   4. 回调与策略类型
// 需要改的数值(段大小、条目结构体、单次日志上限)都在本文件里, Manager 侧不需要改。
//===================================================================================================

namespace dlx
{
    //===============================================================================================
    // 一、帧格式
    //
    //   每条记录(元数据/参数槽/槽头/日志条目)都是: [0xAA][定长结构体][0x00]
    //     - 首字节 0xAA  = 帧头, 用于快速判断"这个位置有没有写过记录";
    //     - 定长结构体    = 见下面的各个 struct;
    //     - 尾字节 0x00  = 提交标记(commit)。写入时先写"帧头 + 定长结构体",
    //                      最后再单独写这 1 个字节。掉电若发生在提交之前,
    //                      尾字节仍是擦除态 0xFF, 读取时被判为"未完成的记录",
    //                      直接当作数据末尾, 下次从这个位置重新写。
    //   擦除态(0xFF)与 0x00 区分开, 因此"未写入"和"写完了"天然可分辨。
    //
    //   各结构体里都有一个 uint16_t crc(偏移固定为 2), 覆盖除它自己以外的全部字节,
    //   用于识别位翻转等数据损坏; 帧尾只保证"完整性", CRC 保证"正确性"。
    //===============================================================================================
    constexpr uint8_t FLASH_FRAME_HEAD  = 0xAA; ///< 帧头
    constexpr uint8_t FLASH_FRAME_TAIL  = 0x00; ///< 帧尾(提交标记)
    constexpr uint8_t FLASH_ERASED_BYTE = 0xFF; ///< Flash 擦除后的字节值

    /** 记录种类: 各记录结构体的第 1 个字节, 用于区分同类记录(预留扩展) */
    enum class FlashRecordType : uint8_t
    {
        Meta       = 0x01, ///< 元数据记录   -> FlashMetaRecord
        Param      = 0x02, ///< 参数槽记录   -> FlashParamSlot
        SlotHeader = 0x03, ///< 日志槽头记录 -> FlashLogSlotHeader
        Entry      = 0x04, ///< 日志条目记录 -> LogEntry
    };

    /** 日志写满策略(init() 时指定, 运行中也可切换) */
    enum class FlashFullPolicy : uint8_t
    {
        Halt,            ///< 停机策略: 到达单次日志上限后不再写入, 返回 FLASH_LOG_FULL
        OverwriteOldest, ///< 覆写策略: 到达上限后环绕擦除最旧的日志槽继续写, 只返回提示性异常
    };

    //===============================================================================================
    // 二、CRC16 与对齐小工具
    //===============================================================================================
    /** CRC16-CCITT(多项式 0x1021, 初值 0xFFFF) */
    inline uint16_t flashCrc16(const uint8_t *data, uint32_t len, uint16_t crc = 0xFFFFu)
    {
        for (uint32_t i = 0; i < len; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (uint8_t bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                      : static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }

    /** 计算某个记录结构体的 CRC: 覆盖 [0, crc) 与 (crc, end) 两段, 跳过 crc 字段本身 */
    template <typename T>
    inline uint16_t flashStructCrc(const T &s)
    {
        const uint8_t *p = reinterpret_cast<const uint8_t *>(&s);
        constexpr uint32_t off = static_cast<uint32_t>(offsetof(T, crc));
        const uint16_t crc = flashCrc16(p, off);
        return flashCrc16(p + off + sizeof(s.crc), sizeof(T) - off - sizeof(s.crc), crc);
    }

    /** 向上取整到 4 字节 */
    constexpr uint32_t flashAlign4(uint32_t x) { return (x + 3u) & ~3u; }
    constexpr uint32_t flashMaxU32(uint32_t a, uint32_t b) { return (a > b) ? a : b; }
    constexpr uint32_t flashMinU32(uint32_t a, uint32_t b) { return (a < b) ? a : b; }
    constexpr uint32_t flashClampU32(uint32_t v, uint32_t lo, uint32_t hi)
    {
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    /**
     * @brief 空进度回调
     * 需要"我正在干活"的外部提示时, 给 init()/format() 传一个空参 lambda, 例如:
     *     fs.init(FlashFullPolicy::Halt, [&]{ led = !led; gpio = led; }, err);
     * 每擦除完一个 64KB 块调用一次, 可以拿来点灯/翻 IO/喂看门狗。
     */
    inline void flashNoProgress()
    {
    }

    //===============================================================================================
    // 三、分段布局
    //
    //   W25Q128 = 16MB = 4096 个 4KB 扇区, 三段顺序固定:
    //
    //   0x000000  ┌───────────────────────┐
    //             │ 元数据段 120KB (30扇区)│  多槽环形写, 每次 init 写一条; 扇区用完才擦除
    //   0x01E000  ├───────────────────────┤
    //             │ 参数段    8KB ( 2扇区)│  槽位轮转, 每次 saveParams 追加一个槽
    //   0x020000  ├───────────────────────┤
    //             │ 日志段 ~15.87MB (127槽)│  每槽 128KB; 一次上电(一个会话)默认预留 8 个槽顺序写条目
    //   0xFFFFFF  └───────────────────────┘
    //
    //   元数据段与参数段都比"实际需要的空间"大得多, 且内部是"多槽顺序写 + 一整扇区擦除"的
    //   环形结构, 所以擦除次数被均摊到整个段上(磨损均衡), 不必担心频繁改写。
    //===============================================================================================
    constexpr uint32_t FLASH_SECTOR_SIZE  = W25Q128_SECTOR_SIZE;                        ///< 4KB
    constexpr uint32_t FLASH_SECTOR_COUNT = W25Q128_CAPACITY / W25Q128_SECTOR_SIZE;     ///< 4096

    constexpr uint32_t FLASH_PARAM_SECTOR_COUNT    = 2;   ///< 参数段 8KB
    constexpr uint32_t FLASH_LOG_SLOT_SECTOR_COUNT = 32;  ///< 单个日志槽 128KB(必须是 64KB 块大小的整数倍)
    constexpr uint32_t FLASH_LOG_SLOT_COUNT        = 127; ///< 日志槽个数(= 最多同时保留多少次上电的日志)

    /** 元数据段 = 剩下的全部扇区(它只用来垫磨损均衡, 不占额外容量), 当前 = 30 扇区 = 120KB */
    constexpr uint32_t FLASH_META_SECTOR_COUNT = FLASH_SECTOR_COUNT
                                               - FLASH_PARAM_SECTOR_COUNT
                                               - FLASH_LOG_SLOT_SECTOR_COUNT * FLASH_LOG_SLOT_COUNT;

    constexpr uint32_t FLASH_META_OFFSET  = 0;
    constexpr uint32_t FLASH_META_SIZE    = FLASH_META_SECTOR_COUNT * FLASH_SECTOR_SIZE;
    constexpr uint32_t FLASH_PARAM_OFFSET = FLASH_META_OFFSET + FLASH_META_SIZE;
    constexpr uint32_t FLASH_PARAM_SIZE   = FLASH_PARAM_SECTOR_COUNT * FLASH_SECTOR_SIZE;
    constexpr uint32_t FLASH_LOG_OFFSET   = FLASH_PARAM_OFFSET + FLASH_PARAM_SIZE;
    constexpr uint32_t FLASH_LOG_SLOT_SIZE = FLASH_LOG_SLOT_SECTOR_COUNT * FLASH_SECTOR_SIZE;
    constexpr uint32_t FLASH_LOG_SIZE      = FLASH_LOG_SLOT_COUNT * FLASH_LOG_SLOT_SIZE;

    static_assert(FLASH_META_SECTOR_COUNT >= 2, "元数据段至少需要 2 个扇区");
    static_assert(FLASH_LOG_SLOT_SECTOR_COUNT % (W25Q128_BLOCK64K_SIZE / FLASH_SECTOR_SIZE) == 0,
                  "日志槽大小必须是 64KB 块的整数倍(槽擦除按 64KB 块进行)");
    static_assert(FLASH_PARAM_SECTOR_COUNT >= 1, "参数段至少需要 1 个扇区");
    static_assert(FLASH_LOG_SLOT_COUNT >= 2, "日志槽至少 2 个, 否则无法环形覆写");
    static_assert(FLASH_META_OFFSET + FLASH_META_SIZE + FLASH_PARAM_SIZE + FLASH_LOG_SIZE
                      == W25Q128_CAPACITY,
                  "三段之和必须正好铺满整片 Flash");

    //===============================================================================================
    // 四、元数据记录(元数据段)
    //===============================================================================================
    constexpr uint32_t FLASH_MAGIC          = 0x444C5846u; ///< 'D''L''X''F': 文件系统魔数
    constexpr uint8_t  FLASH_FORMAT_VERSION = 2;           ///< 存储格式版本(帧/记录布局不兼容时 +1)

    /**
     * @brief 元数据记录(帧: 0xAA + 本结构体 + 0x00)
     *
     * 写在元数据段内, 每次 init 追加一条(序号单调递增), 读取时取序号最大且校验通过的一条。
     * 判断"文件系统是否已经建立"就是看能不能在元数据段里找到一条合法的元数据记录。
     */
    struct FlashMetaRecord
    {
        uint8_t  type;       ///< FlashRecordType::Meta
        uint8_t  version;    ///< FLASH_FORMAT_VERSION
        uint16_t crc;        ///< 除本字段外整个结构体的 CRC16
        uint32_t seq;        ///< 写入序号(单调递增), 用于找出最新的一条
        uint32_t magic;      ///< FLASH_MAGIC
        uint32_t bootCount;  ///< 上电(init)次数, 仅作统计
        uint32_t sessionCounter; ///< 已分配出去的最大会话号(会话号严格递增, 清日志后也不会重用)
        // ---- 上一次会话的"预留区"信息 ----
        // 作用: 预留区里没被写到的槽已经擦干净了, 下次上电可以不用再擦(少等几秒)。
        // 校验方式: 用 最新槽序号 - reserveStartSeq + 1 算出上次会话用了几个槽,
        //           只有在 [1, reservedSlots] 范围内才认为"擦除状态"可信, 否则宁可多擦。
        uint32_t reserveStartSlot; ///< 上次预留区起点(日志段内的物理槽号)
        uint32_t reserveStartSeq;  ///< 上次会话起点槽的槽序号
        uint32_t reservedSlots;    ///< 上次预留了几个槽
        // ---- 布局指纹: 与本次编译的常量/结构体大小不一致时拒绝使用(防止把旧数据按新格式解析) ----
        uint32_t metaSectors;    ///< FLASH_META_SECTOR_COUNT
        uint32_t paramSectors;   ///< FLASH_PARAM_SECTOR_COUNT
        uint32_t logSlotSectors; ///< FLASH_LOG_SLOT_SECTOR_COUNT
        uint32_t logSlots;       ///< FLASH_LOG_SLOT_COUNT
        uint32_t entrySize;      ///< sizeof(LogEntry)
        uint32_t paramSize;      ///< sizeof(FlashParamData)
    };
    static_assert(offsetof(FlashMetaRecord, crc) == 2, "crc 字段必须紧跟 type/version");

    constexpr uint32_t FLASH_META_FRAME_BYTES       = sizeof(FlashMetaRecord) + 2u;
    constexpr uint32_t FLASH_META_SLOT_BYTES        = flashAlign4(FLASH_META_FRAME_BYTES);
    constexpr uint32_t FLASH_META_SLOTS_PER_SECTOR  = FLASH_SECTOR_SIZE / FLASH_META_SLOT_BYTES;
    constexpr uint32_t FLASH_META_SLOT_COUNT        = FLASH_META_SLOTS_PER_SECTOR * FLASH_META_SECTOR_COUNT;
    static_assert(FLASH_META_SLOTS_PER_SECTOR >= 1, "元数据槽太大, 一个扇区放不下");

    //===============================================================================================
    // 五、持久化参数(参数段)
    //===============================================================================================
    /** 数值参数个数(可以改成 FlightParamId::COUNT 之类, 只要求是编译期常量) */
    constexpr uint32_t FLASH_PARAM_VALUE_COUNT = 64;

    /**
     * @brief 需要持久化的参数内容
     *
     * 整个结构体作为一个槽写入 Flash, 可以按需要换成具名字段的结构体(保持定长 POD 即可)。
     * 结构体大小变了以后, 元数据里的 paramSize 会对不上, init() 会返回 FLASH_GEOMETRY_MISMATCH。
     */
    struct FlashParamData
    {
        uint32_t version;                          ///< 参数表版本(自己维护, 改动字段含义时 +1)
        float    value[FLASH_PARAM_VALUE_COUNT];   ///< 数值参数表
    };

    /**
     * @brief 参数槽(帧: 0xAA + 本结构体 + 0x00)
     *
     * 参数段内按槽顺序追加写入, 读完取"序号最大且校验通过"的槽; 槽写满一个扇区就擦除该扇区
     * 从扇区头重新开始, 因此掉电只会丢掉"当前正在写的那一个槽", 上一个有效参数始终还在。
     */
    struct FlashParamSlot
    {
        uint8_t        type;    ///< FlashRecordType::Param
        uint8_t        version; ///< 用户参数表版本(来自 FlashParamData::version 的低 8 位, 便于快速筛选)
        uint16_t       crc;     ///< 除本字段外整个结构体的 CRC16
        uint32_t       seq;     ///< 写入序号(单调递增)
        FlashParamData data;    ///< 参数内容
    };
    static_assert(offsetof(FlashParamSlot, crc) == 2, "crc 字段必须紧跟 type/version");

    constexpr uint32_t FLASH_PARAM_FRAME_BYTES      = sizeof(FlashParamSlot) + 2u;
    constexpr uint32_t FLASH_PARAM_SLOT_BYTES       = flashAlign4(FLASH_PARAM_FRAME_BYTES);
    constexpr uint32_t FLASH_PARAM_SLOTS_PER_SECTOR = FLASH_SECTOR_SIZE / FLASH_PARAM_SLOT_BYTES;
    constexpr uint32_t FLASH_PARAM_SLOT_COUNT       = FLASH_PARAM_SLOTS_PER_SECTOR * FLASH_PARAM_SECTOR_COUNT;
    static_assert(FLASH_PARAM_SLOTS_PER_SECTOR >= 1, "参数槽太大, 一个扇区放不下");

    //===============================================================================================
    // 六、日志条目 与 日志槽(日志段)
    //
    //  日志段 = FLASH_LOG_SLOT_COUNT(127) 个"槽", 每槽 FLASH_LOG_SLOT_SIZE(128KB):
    //    每次上电 = 一个日志会话, 一次可以占用多个槽(FLASH_LOG_SESSION_SLOT_LIMIT 个):
    //      init() 保证这次会话可能用到的整块预留区(槽数 × 128KB)是擦干净的, 再写起点槽的槽头,
    //      之后顺序往槽里写条目, 写满一个槽就接着用预留区里的下一个槽(不需要再擦)。
    //      预留区里没用到、没写槽头的槽不会被扫描到, 所以"最新槽"始终是真正写过的最后一个槽。
    //      预留区里"上次已经擦好、又没写过"的那部分不再重复擦(见 FlashMetaRecord 的预留区字段),
    //      所以每次上电真正要擦的, 只有需要回收的旧会话占用的那几个槽。
    //    预留区用完之后: Halt 策略 -> 返回 FLASH_LOG_FULL 停机(这就是"单次日志上限");
    //                    OverwriteOldest 策略 -> 继续占用后面的槽(现场擦掉最旧的会话),
    //                    也就是"循环覆写最旧的日志, 保留最新的日志"。
    //    槽用完一轮后回到第一个槽, 整体保留最近写过的 FLASH_LOG_SLOT_COUNT 个槽的数据。
    //    环形队列语义: 自动回收的永远是"最老"的那一段; dropSessions() 手动丢的也是最老的会话,
    //    丢完把槽并给当前会话(写指针不动), 所以新日志永远是一路往下写。
    //
    //  槽内布局:
    //    offset 0                      槽头(FlashLogSlotHeader): 会话号/槽序号
    //    offset FLASH_LOG_ENTRY_AREA_OFFSET 起, 条目按 FLASH_LOG_ENTRY_STRIDE 等间距排列
    //    (最后不足一个步长的尾巴空着不写, 保持 0xFF, 读到 0xFF 就说明这个槽写完了)
    //===============================================================================================
    /**
     * @brief 一条日志条目(帧: 0xAA + 本结构体 + 0x00)
     *
     * 前 5 个字段由 FlashManager 填写与校验, 写入时会被自动覆盖, 用户不要自己填;
     * 从 tickMs 往后的字段是业务数据, 按需要随意增删(改完重新格式化一次即可)。
     */
    struct LogEntry
    {
        // ------------------------- FlashManager 使用 -------------------------
        uint8_t  type;      ///< FlashRecordType::Entry(保留此字段用于以后扩展其它条目种类)
        uint8_t  reserved;  ///< 对齐填充, 固定写 0
        uint16_t crc;       ///< 除本字段外整个结构体的 CRC16
        uint32_t sessionId; ///< 会话号(每次上电 +1), 用于把条目归到某一次上电
        uint32_t seq;       ///< 会话内条目序号, 从 1 开始
        // ------------------------- 业务数据(示例) -------------------------
        uint32_t tickMs;    ///< 时间戳 [ms]
        float    rollRad;   ///< 姿态 roll [rad]
        float    pitchRad;  ///< 姿态 pitch [rad]
        float    yawRad;    ///< 姿态 yaw [rad]
        float    gyroX;     ///< 机体角速度 x [rad/s]
        float    gyroY;     ///< 机体角速度 y [rad/s]
        float    gyroZ;     ///< 机体角速度 z [rad/s]
        float    throttle;  ///< 归一化油门
        uint16_t motor[4];  ///< 四个电机输出
        uint16_t flags;     ///< 状态位
    };
    static_assert(offsetof(LogEntry, crc) == 2, "crc 字段必须紧跟 type/reserved");

    /**
     * @brief 日志槽头(帧: 0xAA + 本结构体 + 0x00)
     *
     * 每个槽的第一条记录。init() 擦完槽以后立刻写它, 因此"槽头合法"就表示这个槽本轮可用。
     * 扫描所有槽头即可知道: 最新写到哪(槽序号最大者)、一共有哪些会话(会话号相同 = 同一个会话)。
     */
    struct FlashLogSlotHeader
    {
        uint8_t  type;      ///< FlashRecordType::SlotHeader
        uint8_t  version;   ///< FLASH_FORMAT_VERSION
        uint16_t crc;       ///< 除本字段外整个结构体的 CRC16
        uint32_t slotSeq;   ///< 槽使用序号(全局单调递增), 最大者就是最新写入的槽
        uint32_t slotIndex; ///< 槽在日志段内的物理序号(校验地址与内容是否对应)
        uint32_t sessionId; ///< 本槽所属会话号(一个会话占多个槽时, 各槽的会话号相同)
    };
    static_assert(offsetof(FlashLogSlotHeader, crc) == 2, "crc 字段必须紧跟 type/version");

    constexpr uint32_t FLASH_SLOT_FRAME_BYTES   = sizeof(FlashLogSlotHeader) + 2u;
    constexpr uint32_t FLASH_ENTRY_FRAME_BYTES  = sizeof(LogEntry) + 2u;
    /** 日志条目步长: 帧头 + 条目 + 帧尾, 再补齐到 4 字节(填充字节保持擦除态) */
    constexpr uint32_t FLASH_LOG_ENTRY_STRIDE   = flashAlign4(FLASH_ENTRY_FRAME_BYTES);
    /** 槽内条目区起点: 槽头之后, 4 字节对齐 */
    constexpr uint32_t FLASH_LOG_ENTRY_AREA_OFFSET = flashAlign4(FLASH_SLOT_FRAME_BYTES);
    /** 每个槽能放多少条日志 */
    constexpr uint32_t FLASH_LOG_ENTRIES_PER_SLOT  =
        (FLASH_LOG_SLOT_SIZE - FLASH_LOG_ENTRY_AREA_OFFSET) / FLASH_LOG_ENTRY_STRIDE;

    /**
     * 单次日志(一个会话)最多占用几个槽 —— 也就是"单次日志上限"。
     *
     *  预留区大小 = 本值 × FLASH_LOG_SLOT_SIZE (默认 8 × 128KB = 1MB),
     *  init() 会把整块预留区一次擦干净(每擦完一个 64KB 块回调一次进度),
     *  所以上电擦除耗时 ≈ 本值 × 0.3s(典型值; 最坏 本值 × 4s)。
     *
     *  单次会话可写条目数 = 本值 × FLASH_LOG_ENTRIES_PER_SLOT
     *                      (默认 8 × 2184 = 17472 条, 按每条约 128B 折算约 2.1MB 数据)。
     *  Halt 策略写满预留区后返回 FLASH_LOG_FULL;
     *  OverwriteOldest 策略则继续占用后面的槽(现场擦最旧会话), 只报提示性异常。
     *
     *  取值范围 1 ~ FLASH_LOG_SLOT_COUNT(127); 运行时可用 setSessionSlotLimit() 改, 下一次 init 生效。
     *  注意: 预留了但没用到的槽, 下次上电会被重新擦一遍(只多花时间, 磨损增加有限)。
     */
    constexpr uint32_t FLASH_LOG_SESSION_SLOT_LIMIT = 8;

    /** dropSessions() 的 count 参数: 丢掉全部历史会话(只保留当前正在写的这次, 腾出的槽给它用) */
    constexpr uint32_t FLASH_DROP_ALL_SESSIONS = 0xFFFFFFFFu;

    static_assert(FLASH_LOG_SESSION_SLOT_LIMIT >= 1, "单次日志至少给 1 个槽");
    static_assert(FLASH_LOG_SESSION_SLOT_LIMIT <= FLASH_LOG_SLOT_COUNT,
                  "单次日志预留的槽数不能超过日志段总槽数");

    static_assert(FLASH_LOG_ENTRY_STRIDE + FLASH_LOG_ENTRY_AREA_OFFSET <= FLASH_SECTOR_SIZE,
                  "一条日志条目必须能放进一个扇区(否则会跨扇区, 槽擦除会误删)");
    static_assert(FLASH_LOG_ENTRIES_PER_SLOT >= 1, "日志条目太大, 一个槽放不下一条");

    /** Manager 内部收发一条记录所用的缓冲大小(取所有帧里最大的) */
    constexpr uint32_t FLASH_FRAME_BUF_BYTES = flashMaxU32(
        flashMaxU32(FLASH_META_FRAME_BYTES, FLASH_PARAM_FRAME_BYTES),
        flashMaxU32(FLASH_SLOT_FRAME_BYTES, FLASH_ENTRY_FRAME_BYTES));

    static_assert(FLASH_META_FRAME_BYTES <= FLASH_FRAME_BUF_BYTES, "收发缓冲装不下元数据记录");
    static_assert(FLASH_PARAM_FRAME_BYTES <= FLASH_FRAME_BUF_BYTES, "收发缓冲装不下参数槽");
    static_assert(FLASH_SLOT_FRAME_BYTES <= FLASH_FRAME_BUF_BYTES, "收发缓冲装不下槽头");
    static_assert(FLASH_ENTRY_FRAME_BYTES <= FLASH_FRAME_BUF_BYTES, "收发缓冲装不下日志条目");

    //===============================================================================================
    // 七、对外回调与信息结构体
    //===============================================================================================
    /** 一个日志会话(一次上电)的信息; 遍历时临时给出, 不需要常驻 RAM */
    struct FlashLogSessionInfo
    {
        uint32_t sessionId;   ///< 会话号(每次上电 +1)
        uint32_t firstSlot;   ///< 起始槽的物理序号
        uint32_t slotCount;   ///< 该会话占用的槽数
        uint32_t regionBytes; ///< 该会话占用的字节数(= slotCount * FLASH_LOG_SLOT_SIZE)
        bool     active;      ///< true = 当前(最新)会话, 可能还在写入
    };

    /** 遍历会话的回调; 返回 false 表示提前结束遍历 */
    typedef bool (*FlashSessionCallback)(const FlashLogSessionInfo &info, void *ctx);

    /** 遍历日志条目的回调; 返回 false 表示提前结束遍历 */
    typedef bool (*FlashEntryCallback)(const LogEntry &entry, void *ctx);

} // namespace dlx

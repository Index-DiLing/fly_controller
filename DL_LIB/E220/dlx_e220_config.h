#pragma once
//===================================================================================================
// DL_LIB/E220/dlx_e220_config.h —— E220-400T22S(LLCC68, E220-T 系列 AT 固件)的**芯片级常量**
//
// 这里放什么 / 不放什么
//   放: 手册里写死的"器件性质" —— 寄存器地址、位域、档位代码、时序、HEX/AT 指令的帧格式特征。
//   不放: **工程配置项**。本文件不 include 任何 cfg::/flight_config*, 一个引脚、一个信道、
//         一个波特率都不在这里写死; 那些值由调用方(应用层)以参数传进 dlx::E220。
//         这是本工程的规矩: DL_LIB 只接受参数、不认识配置(见 hardware_claims.hpp 文件头)。
//
// 依据(唯一来源)
//   《E220-T系列模块_UserManual_CN.pdf》V1.6(2026-05-29, 40 页, 固件 7463-0-1x 世代)。
//   本仓库的研究笔记: docs/_research/e220_400t22s.md(逐条标注了手册章节)。
//   每条常量后面都写了出处; **查不到出处的一律不写** —— 需要而手册没有的, 以"工程取值/推导"
//   明确标出并给出推导过程, 不伪装成手册数值。
//
// ⚠ 世代警告(最容易把寄存器写歪的地方, 手册 §6.2 + 研究笔记 §12)
//   有一种更老的 **E220-xxxTBL** 世代: 它的寄存器表里有 NETID, 于是 REG0..REG3 落在
//   03h..06h。**本手册这一代没有 NETID, 整张表前移一位**: REG0..REG3 = 02h..05h。
//   照抄老 TBL/E22 例程的代码会把 NETID 写进 REG0(串口/空速), 表现是"模块突然不回话"。
//   本文件只按 V1.6 的表写, 并在下面用 static_assert 把"没有 NETID"这件事钉住。
//===================================================================================================
#include <stdint.h>
namespace dlx
{
    //===============================================================================================
    // 一、寄存器地址(手册 §6.2「寄存器功能一览表」+ §6.3「E220-400/900Txxx 寄存器描述」)
    //
    //   手册的表**只列到 08h 为止**(00h ADDH · 01h ADDL · 02h REG0 · 03h REG1 · 04h REG2 ·
    //   05h REG3 · 06h CRYPT_H · 07h CRYPT_L · 08h Version)。
    //   所以这里没有 09h~0Fh 的常量: 任务书里"0x00~0x0F 那份手册表"对应的是**老 TBL 世代**
    //   的文档, 本手册查不到那一段。**宁可少写一个地址, 也不编一个查不到出处的地址** ——
    //   编出来的地址写下去不会报错, 只会静默改掉别的参数。
    //===============================================================================================
    enum class E220Reg : uint8_t
    {
        Addh    = 0x00, ///< 通信地址[15:8](默认 0); 地址 = 0xFFFF 时是广播/监听地址(不过滤)
        Addl    = 0x01, ///< 通信地址[7:0]
        Reg0    = 0x02, ///< 串口波特率[7:5] + 串口校验[4:3] + 空中速率[2:0]
        Reg1    = 0x03, ///< 分包长度[7:6] + 环境RSSI[5] + 保留[4:3] + 软切模式[2] + 发射功率[1:0]
        Reg2    = 0x04, ///< 信道 CH[7:0]; 400 频段 0~83, f = 410.125 + CH×1MHz
        Reg3    = 0x05, ///< 数据RSSI字节[7] + 传输方式[6] + 保留[5] + LBT[4] + 保留[3] + WOR周期[2:0]
        CryptH  = 0x06, ///< 密钥高字节 —— **只写, 读取恒返回 0**(§6.3 06H 行)
        CryptL  = 0x07, ///< 密钥低字节 —— 同上
        Version = 0x08, ///< 产品信息 1 字节(只读); 手册建议改用 AT+DEVTYPE=? 查询
    };

    /** REG0~REG3 = 02h~05h **连续**四个字节 ⇒ 可以一条 C0 指令整块读写(手册 §6.1 例 2 就是这个用法)。
     *  为什么强调连续: 本工程"写配置 + 回读校验"按这四字节一次做完; 分成 4 次会多 4 次超时机会,
     *  而且中途失败会留下"一半新一半旧"的参数。 */
    constexpr uint8_t E220_REG0_ADDR  = static_cast<uint8_t>(E220Reg::Reg0); // 0x02
    constexpr uint8_t E220_REG_BLOCK  = 4u;                                  // REG0..REG3
    constexpr uint8_t E220_ADDR_BLOCK = 2u;                                  // ADDH+ADDL

    // ---- "这一代没有 NETID"的编译期守卫(见文件头"世代警告") ----
    // 老 TBL 世代 NETID 在 02h、REG0 在 03h; 如果有人把老表抄进来, REG0 就会被写成 03h。
    // 这两条断言把表的结构钉住: 改地址必须同时改断言, 改不动就说明你抄的是另一份手册。
    static_assert(E220_REG0_ADDR == 0x02u, "REG0 在 02h: 本手册(V1.6)这一代没有 NETID, 整表比老 TBL 前移一位");
    static_assert(static_cast<uint8_t>(E220Reg::Reg3) == E220_REG0_ADDR + 3u,
                  "REG0..REG3 必须连续(02h..05h): 一次块读写的前提");
    static_assert(static_cast<uint8_t>(E220Reg::CryptH) == 0x06u && static_cast<uint8_t>(E220Reg::CryptL) == 0x07u,
                  "CRYPT_H/CRYPT_L 在 06h/07h: **只写**, 回读恒 0 —— 绝不能进'写入并回读校验'的块里");

    //===============================================================================================
    // 二、REG0(02h)位域 —— 手册 §6.3「02H 读/写 EG0765 UART 串口速率」那一段的三张小表
    //     bit7..bit5 = 串口波特率[2:0]; bit4..bit3 = 串口校验[1:0]; bit2..bit0 = 空中速率[2:0]
    //===============================================================================================
    constexpr uint8_t E220_REG0_BAUD_MASK   = 0xE0u; ///< [7:5]
    constexpr uint8_t E220_REG0_BAUD_SHIFT  = 5u;
    constexpr uint8_t E220_REG0_PARITY_MASK = 0x18u; ///< [4:3]
    constexpr uint8_t E220_REG0_PARITY_SHIFT = 3u;
    constexpr uint8_t E220_REG0_AIR_MASK    = 0x07u; ///< [2:0]
    constexpr uint8_t E220_REG0_AIR_SHIFT   = 0u;

    /** 串口波特率档位 = REG0[7:5] 的代码(手册 §6.3 02H 表: 000=1200 … 111=115200)。
     *  ⚠ 与 AT 指令的代码**相同**(§7.1.11 AT+UART 的 <baud> 也是 0~7), 但与
     *    cfg::E220Baud 的枚举值也恰好一致(生成器就是这么写的) —— 三处一致是巧合, 改配置时要对一眼。 */
    enum class E220BaudCode : uint8_t
    {
        B1200   = 0,
        B2400   = 1,
        B4800   = 2,
        B9600   = 3, ///< 出厂默认, 也是**配置模式强制**的档位
        B19200  = 4,
        B38400  = 5,
        B57600  = 6,
        B115200 = 7,
    };

    /** 串口校验档位。手册 §6.3(02H 表)与 §7.1.11(AT+UART 的 <parity>)给的是**同一个数值表**:
     *  0=8N1(默认) / 1=8O1 / 2=8E1。差别只在前者多给了一个"3=8N1(等同 00)"的冗余码点。
     *  为什么这里只写 3 个成员: 冗余码点 3 在 AT 指令里没有对应值, 写进来只会让人以为 AT 也能传 3。
     *  真要用寄存器码点 3, 用 E220_REG_PARITY_REDUNDANT_8N1。 */
    enum class E220ParityCode : uint8_t
    {
        N8N1 = 0, ///< 无校验(8N1)
        O8O1 = 1, ///< 奇校验(8O1)
        E8E1 = 2, ///< 偶校验(8E1)
    };
    constexpr uint8_t E220_REG_PARITY_REDUNDANT_8N1 = 3u; ///< 寄存器码点 3 也等于 8N1(手册 §6.3 原文)

    /** 空中速率 = REG0[2:0] 的代码(手册 §6.3 02H 表; §7.1.14 AT+RATE 用的是同一组代码)。
     *  ⚠ 代码 0/1/2 **都是 2.4k**(手册原表就这么写, 不是笔误): 默认值 010(=2) 是 2.4k。
     *  ⚠ 与 cfg::AirRate **不是同一套编号**: cfg::AirRate 是 0=2.4k/1=4.8k/…/5=62.5k 的"连续档位",
     *     寄存器是 3=4.8k/…/7=62.5k。这两者混用的表现是"空速比预期低一档", 而且链路还能通, 极难发现。 */
    enum class E220AirRateCode : uint8_t
    {
        K2_4_A = 0, ///< 2.4 kbps
        K2_4_B = 1, ///< 2.4 kbps
        K2_4_C = 2, ///< 2.4 kbps(出厂默认)
        K4_8   = 3, ///< 4.8 kbps
        K9_6   = 4, ///< 9.6 kbps
        K19_2  = 5, ///< 19.2 kbps
        K38_4  = 6, ///< 38.4 kbps
        K62_5  = 7, ///< 62.5 kbps
    };

    /** 波特率档位 → bps。为什么要有它: 手册只给了代码表, 而"串口速率 ≤ 1.25×空中速率"这条
     *  工程约束(研究笔记 §4 按吞吐推算, **不是手册原文**)必须拿 bps 才能算。 */
    constexpr uint32_t e220BaudBps(E220BaudCode code)
    {
        return (code == E220BaudCode::B1200)     ? 1200u
               : (code == E220BaudCode::B2400)   ? 2400u
               : (code == E220BaudCode::B4800)   ? 4800u
               : (code == E220BaudCode::B9600)   ? 9600u
               : (code == E220BaudCode::B19200)  ? 19200u
               : (code == E220BaudCode::B38400)  ? 38400u
               : (code == E220BaudCode::B57600)  ? 57600u
               : (code == E220BaudCode::B115200) ? 115200u
                                                 : 0u;
    }

    /** 空中速率档位 → bps(手册 §6.3 02H 表) */
    constexpr uint32_t e220AirRateBps(E220AirRateCode code)
    {
        return (code == E220AirRateCode::K2_4_A || code == E220AirRateCode::K2_4_B ||
                code == E220AirRateCode::K2_4_C)
                   ? 2400u
               : (code == E220AirRateCode::K4_8)  ? 4800u
               : (code == E220AirRateCode::K9_6)  ? 9600u
               : (code == E220AirRateCode::K19_2) ? 19200u
               : (code == E220AirRateCode::K38_4) ? 38400u
               : (code == E220AirRateCode::K62_5) ? 62500u
                                                  : 0u;
    }

    //===============================================================================================
    // 三、REG1(03h)位域 —— 手册 §6.3「03H 读/写 REG1」段
    //     bit7..6 = 分包长度[1:0]; bit5 = 环境噪声RSSI使能; bit4/bit3 = 保留; bit2 = 软切模式; bit1..0 = 发射功率
    //===============================================================================================
    constexpr uint8_t E220_REG1_PACKET_MASK  = 0xC0u; ///< [7:6]
    constexpr uint8_t E220_REG1_PACKET_SHIFT = 6u;
    constexpr uint8_t E220_REG1_RSSI_EN      = 0x20u; ///< [5] 环境噪声 RSSI 使能
    constexpr uint8_t E220_REG1_RESERVED     = 0x18u; ///< [4:3] **保留**: 上位机会主动清 0(§6.3 原文)
    constexpr uint8_t E220_REG1_SOFT_SWITCH  = 0x04u; ///< [2] 软件切模式使能(需要重启生效)
    constexpr uint8_t E220_REG1_POWER_MASK   = 0x03u; ///< [1:0]
    constexpr uint8_t E220_REG1_POWER_SHIFT  = 0u;

    /** 分包长度 = REG1[7:6](手册 §6.3 03H 表; §7.1.12 AT+PACKET 代码相同) */
    enum class E220PacketCode : uint8_t
    {
        P200 = 0, ///< 200 字节(出厂默认)
        P128 = 1,
        P64  = 2,
        P32  = 3,
    };

    /** 发射功率 = REG1[1:0]。手册 §6.3 03H 表按型号给表:
     *  22dBm 型号(本板 E220-400T22S): 00=22dBm(默认) / 01=17 / 10=13 / 11=10。
     *  ⚠ 手册自身不一致(研究笔记 §13): §7.1.21 AT+POWER 章把第 3 档写成 **14dBm**, 寄存器表写 **13dBm**。
     *    我们只把**寄存器码点**写成常量, 不把 dBm 写进枚举名, 免得把这份不一致抄进代码。 */
    enum class E220TxPowerCode : uint8_t
    {
        P22dBm = 0,
        P17dBm = 1,
        P13or14dBm = 2, ///< 手册 §6.3 写 13dBm、§7.1.21 写 14dBm —— 出处冲突, 见上
        P10dBm = 3,
    };

    //===============================================================================================
    // 四、REG2(04h)/REG3(05h)位域 + 信道范围
    //===============================================================================================
    /** 信道: REG2 整个字节(手册 §6.3 04H 行) */
    constexpr uint8_t  E220_CHANNEL_MASK = 0xFFu;
    /** 400 频段信道数: 0~83 共 84 个(手册 §6.3 04H 行「0-83分别代表总共84个信道（适用400频段）」) */
    constexpr uint8_t  E220_CHANNEL_MAX_400 = 83u;
    /** 400 频段载波频率 = 410.125MHz + CH×1MHz(手册 §6.3 04H 行原文) */
    constexpr uint32_t E220_FREQ_BASE_KHZ_400 = 410125u;
    constexpr uint32_t E220_FREQ_STEP_KHZ     = 1000u;

    constexpr uint8_t E220_REG3_DRSSI_BYTE   = 0x80u; ///< [7] 收到的每个包后追加 1 字节 RSSI
    constexpr uint8_t E220_REG3_TRANS_MODE   = 0x40u; ///< [6] 0=透明传输(默认) 1=定点传输
    constexpr uint8_t E220_REG3_RESERVED     = 0x28u; ///< [5]/[3] 保留(§6.3 05H 行: 「5预留」「3预留」)
    constexpr uint8_t E220_REG3_LBT_EN       = 0x10u; ///< [4] 先听后发; 最长停留 2s 后强制发射
    constexpr uint8_t E220_REG3_WOR_MASK     = 0x07u; ///< [2:0] 空中唤醒周期(仅模式 1 有效, 收发双方必须一致)
    constexpr uint8_t E220_REG3_WOR_SHIFT    = 0u;

    /** WOR 周期: T = (1+WOR)×500ms(手册 §6.3 05H 行「周期T=（1+WOR）*500ms」) */
    constexpr uint16_t e220WorPeriodMs(uint8_t worCode) { return (uint16_t)((1u + (worCode & 0x07u)) * 500u); }

    //===============================================================================================
    // 五、工作模式(M0/M1)—— 手册 §5.1 表「模式（0-3） M1M0 模式介绍 备注」(PAGE 13)
    //
    //   原文表(列顺序是 **M1、M0**):
    //     模式 0 传输模式     M1M0 = 0 0  串口打开, 无线打开, 透明传输
    //     模式 1 WOR发送模式  M1M0 = 0 1  WOR 发送数据, WOR 接收数据(支持空中唤醒)
    //     模式 2 WOR接收模式  M1M0 = 1 0  WOR 发送关闭, WOR 接收数据
    //     模式 3 休眠模式     M1M0 = 1 1  可配置参数(§7 章首: 配置模式串口强制固定 9600 8N1)
    //   §5.1 开头一句: 「本章节中描述的 M1M0 引脚 0 代表低电平, 1 代表高电平」。
    //
    //   ⚠ 写成"M0/M1"(引脚号升序)时, 上表要左右对调才等价 —— 任务书里的
    //     「1/0=WOR 发送, 0/1=WOR 接收」就是 M0/M1 次序, 与手册表**同一件事**:
    //       WOR 发送 = M0 高 M1 低(手册 M1M0=01) ; WOR 接收 = M0 低 M1 高(手册 M1M0=10)。
    //     两种写法混用会把 WOR 收/发对调, 而它们在透传链路里表现都是"收不到", 无法区分。
    //     本驱动内部只认"模式号 0..3", 引脚电平由 modePinLevels() 唯一推导, 不给人对调的机会。
    //===============================================================================================
    constexpr uint8_t E220_MODE_TRANSPARENT = 0u; ///< (M1,M0)=(0,0) 透传。§3.1 定案: 本工程常驻此模式
    constexpr uint8_t E220_MODE_WOR_TX      = 1u; ///< (M1,M0)=(0,1) WOR 发送
    constexpr uint8_t E220_MODE_WOR_RX      = 2u; ///< (M1,M0)=(1,0) WOR 接收
    constexpr uint8_t E220_MODE_CONFIG      = 3u; ///< (M1,M0)=(1,1) 休眠/配置。AT 与 HEX 指令只在此模式被接受
    constexpr uint8_t E220_MODE_MAX         = 3u;

    /** 模式号 → (M0 电平, M1 电平)。**唯一**的引脚映射推导点(见上面 §5.1 的表)。
     *  为什么不是 constexpr: C++11 的 constexpr 函数不能返回 void(要常量求值的话得改成返回
     *  打包后的两位, 那反而让调用点更难读)。这里是纯计算, inline 足够。 */
    inline void e220ModePinLevels(uint8_t mode, bool &m0High, bool &m1High)
    {
        m0High = (mode & 0x01u) != 0u; ///< M0 = 模式号的 bit0(手册 M1M0 表的**右**列)
        m1High = (mode & 0x02u) != 0u; ///< M1 = 模式号的 bit1(表头写 M1M0, 列序 M1 在前)
    }

    //===============================================================================================
    // 六、AUX 语义与时序 —— 手册 §2.1 引脚表(第 24 脚) + §5.2 AUX 时序
    //
    //   极性依据(两处独立表述, 都对得上):
    //     ① §2.1 引脚表第 24 脚 AUX: 「输出 1.指示模块工作状态（可以悬空）」——只说"指示状态";
    //     ② §5.2.4 注意事项表第 1 条原文: 「当 AUX 输出低电平时, 表示模块繁忙, 此时不会进行工作模式检测。」
    //        ⇒ **低 = 忙, 高 = 空闲**。这就是本驱动 auxReady() 的判据。
    //   ⚠ 注意 §7.1.7: 固件 7463-0-15 起 AT+UAUX 默认 0, AUX **不再指示无线发送过程**,
    //     只跟踪串口缓存。所以 "AUX 高" 只证明"串口缓存空闲", 不证明"空口发完了" ——
    //     时分调度若要靠 AUX 节流, 必须先 AT+UAUX=1(本工程 link.e220.kUaux 就是为此设成 true)。
    //===============================================================================================
    constexpr bool E220_AUX_LEVEL_IDLE = true;  ///< 高 = 空闲(§5.2.4 注意事项 1)
    constexpr bool E220_AUX_LEVEL_BUSY = false; ///< 低 = 忙

    constexpr uint32_t E220_POWERUP_READY_MS = 16u;  ///< §5.2.1 表 T1: 上电到"用户可操作"典型值 16ms
    constexpr uint32_t E220_POWERUP_AUX_FIRST_LOW_MS = 5u; ///< §5.2.1 表 T2: 上电后 AUX 首次拉低的延迟
    constexpr uint32_t E220_MODE_SWITCH_BUSY_MS = 11u; ///< §5.2.4 表: 全部 12 种模式切换 AUX 忙 9~11ms(取上界)
    constexpr uint32_t E220_TX_INDICATE_MS = 1u;       ///< §5.2.3 表 T1: 串口首字节被识别到 AUX 拉低的延迟
    constexpr uint32_t E220_DEFAULT_URXT_BYTE_TIMES = 3u; ///< §7.1.13: 出厂断帧条件 = 3 字节时间

    /** 配置模式下的串口参数: **强制 9600 8N1**(§7 章首「配置模式串口强制固定 9600 8N1」;
     *  同一句在 §7.1.11 AT+UART 特性说明里重复了一次)。所以进配置模式前调用方必须先把 MCU 串口
     *  换成 9600 —— 驱动只切引脚和等时序, 不动串口(见 dlx_e220.hpp 类注释里的调用顺序)。 */
    constexpr uint32_t E220_CFG_BAUD_BPS       = 9600u;
    constexpr E220BaudCode E220_CFG_BAUD_CODE  = E220BaudCode::B9600;
    constexpr E220ParityCode E220_CFG_PARITY_CODE = E220ParityCode::N8N1;

    //===============================================================================================
    // 七、HEX 指令格式(手册 §6.1「HEX 指令格式」, 配置模式 9600 8N1 下有效)
    //   设置寄存器     : C0 + 起始地址 + 长度 + 参数      → 响应 C1 + 起始地址 + 长度 + 参数
    //   读取寄存器     : C1 + 起始地址 + 长度             → 响应 C1 + 起始地址 + 长度 + 参数
    //   设置临时寄存器 : C2 + 起始地址 + 长度 + 参数      → 响应 C1 + 起始地址 + 长度 + 参数(**掉电不保存**)
    //   格式错误       : 响应 FF FF FF
    //   原文例: 配置信道 0x09 → 发 C0 04 01 09, 返回 C1 04 01 09
    //===============================================================================================
    constexpr uint8_t E220_HEX_SET_SAVE = 0xC0u; ///< 设置并保存(写 flash, 掉电保留)
    constexpr uint8_t E220_HEX_READ     = 0xC1u; ///< 读; 也是**所有**响应的首字节
    constexpr uint8_t E220_HEX_SET_TEMP = 0xC2u; ///< 临时设置(掉电丢失)
    constexpr uint8_t E220_HEX_ERROR    = 0xFFu; ///< 格式错误响应 = FF FF FF(§6.1 序号 4)

    /** HEX 指令头长度: 操作码 + 起始地址 + 长度 */
    constexpr uint8_t E220_HEX_HEAD_BYTES = 3u;
    /** 一次块读写的最大字节数: 手册只给了"起始地址 + 长度"一个字节的长度域 ⇒ 255 是格式上限,
     *  但本驱动只服务 ADDH..REG3(00h~05h = 6 字节)。取 8 留一点余量, 并把接收缓冲钉在这个尺寸上
     *  (9 字节应答 @9600 ≈ 9.4ms, 是 E220_AT_TIMEOUT_MS 的取数依据)。 */
    constexpr uint8_t  E220_REG_XFER_MAX   = 8u;
    constexpr uint16_t E220_HEX_REPLY_MAX  = (uint16_t)(E220_HEX_HEAD_BYTES + E220_REG_XFER_MAX); // 11

    //===============================================================================================
    // 八、AT 指令格式(手册 §7「AT 指令」)
    //   §7 章首: ① 参数配置/查询必须在配置模式; ② 固件 ≤7463-0-14 末尾不需要回车换行;
    //            ③ 固件 ≥7463-0-15 **自动识别**末尾回车换行, 不加也识别;
    //            ④ 识别或执行异常应答 "=ERR"; ⑤ 参数超范围会被"限制"(钳位)而不是报错。
    //   应答格式(逐条): 设置类 = "=OK"(§7.1.1/7.1.5/7.1.7/…), 查询类 = "AT+XXX=<值>"(§7.1.3/7.1.4/…)。
    //   ⚠ 手册自身不一致(研究笔记 §13): §7.1.6 AT+MODE 与 §7.1.21 AT+POWER 的**设置**示例应答写的是
    //     "AT+XXX=<v>" 而不是 "=OK"。所以判定"设置成功"必须**同时接受**这两种形态(见 dlx_e220.hpp
    //     的 replyIsSetOk())。
    //   ⚠ 手册**没有**规定应答的行结束符, 也没规定应答之间的间隔; 因此驱动按"字节间空隙"判一条应答
    //     结束(见 E220_AT_REPLY_GAP_MS), 而不是按 CR/LF —— 有 CR/LF 时它一样会被空隙判据覆盖。
    //===============================================================================================
    constexpr char E220_AT_OK[]  = "=OK";  ///< 设置类成功应答(§7.1.1 等)
    constexpr char E220_AT_ERR[] = "=ERR"; ///< 识别/执行异常应答(§7 章首第 4 条)
    /** 查询类指令的结尾: "AT+XXX=?"(§7.1.3/7.1.4/7.1.11…) */
    constexpr char E220_AT_QUERY_SUFFIX[] = "=?";

    /** AT 命令名(手册 §7.1 的指令列表)。只列本工程用得到的; 其余(FPOWER/XCAP/…)是产测指令, 拒绝使用。
     *  ⚠ 没有 "AT+VER"/"AT+REG0" 这两条指令 —— 查版本是 AT+FWCODE=?, 版本字符串形如 "FWCODE=7463-0-17";
     *    寄存器只能走 §6.1 的 HEX 指令。任务书里的 AT+VER/AT+REGn 属于**别的固件世代**的写法。 */
    constexpr char E220_AT_FWCODE[]   = "AT+FWCODE";   ///< §7.1.4 查询软件版本
    constexpr char E220_AT_DEVTYPE[]  = "AT+DEVTYPE";  ///< §7.1.3 查询产品型号
    constexpr char E220_AT_UART[]     = "AT+UART";     ///< §7.1.11 串口波特率与校验(仅 2 个参数: 波特率, 校验)
    constexpr char E220_AT_RATE[]     = "AT+RATE";     ///< §7.1.14 空中速率
    constexpr char E220_AT_CHANNEL[]  = "AT+CHANNEL";  ///< §7.1.9 信道(400 频段 0~83)
    constexpr char E220_AT_ADDR[]     = "AT+ADDR";     ///< §7.1.8 通信地址(0~65535; 65535=广播/监听)
    constexpr char E220_AT_KEY[]      = "AT+KEY";      ///< §7.1.10 密钥(**不能查询**; 不一致时静默丢包)
    constexpr char E220_AT_PACKET[]   = "AT+PACKET";   ///< §7.1.12 分包长度
    constexpr char E220_AT_URXT[]     = "AT+URXT";     ///< §7.1.13 串口断帧条件(字节时间, 1~255)
    constexpr char E220_AT_TRANS[]    = "AT+TRANS";    ///< §7.1.15 0=透明 1=定点
    constexpr char E220_AT_LBT[]      = "AT+LBT";      ///< §7.1.16 先听后发
    constexpr char E220_AT_DRSSI[]    = "AT+DRSSI";    ///< §7.1.20 每包追加 RSSI 字节(会破坏定长帧!)
    constexpr char E220_AT_UAUX[]     = "AT+UAUX";     ///< §7.1.7 AUX 是否指示无线发送过程(本工程必须 1)
    constexpr char E220_AT_SWITCH[]   = "AT+SWITCH";   ///< §7.1.5 软件切模式使能(需要重启)
    constexpr char E220_AT_MODE[]     = "AT+MODE";     ///< §7.1.6 软件切模式(需先 AT+SWITCH=1)
    /** ⚠ §5.3「串口升级固件」: AT+IAP 进固件升级模式, 波特率自动变 115200, **要上电保持 60 秒**才退得出来,
     *  否则重启后无限进升级模式。本驱动**不提供**这条指令(连一个包装函数都不给), 免得哪天有人顺手调用。 */
    constexpr char E220_AT_IAP_FORBIDDEN[] = "AT+IAP";

    //===============================================================================================
    // 九、超时/缓冲区(工程取值, 手册没有这些数; 每个都写清推导过程)
    //===============================================================================================
    /** 一条 AT 应答的最大长度(含可能的 CR/LF)。最长的一条是 "DEVTYPE=E220-400T30S"(18 字符)、
     *  "FWCODE=7463-0-17"(15 字符); 48 = 2.5 倍余量, 留给别的固件版本的啰嗦应答。 */
    constexpr uint16_t E220_AT_REPLY_MAX = 48u;
    /** AT 命令读取缓冲(含结尾 0 的位置)。**为什么不是 E220_AT_REPLY_MAX + 1**: 解析函数要当字符串用,
     *  多留一个字节放 '\0', 这样调用方不用自己补。 */
    constexpr uint16_t E220_AT_TEXT_MAX = (uint16_t)(E220_AT_REPLY_MAX + 1u);

    /** 读一条应答的总超时: @9600 8N1 一个字符 10 位 = 1.042ms; 18 字符 ≈ 18.8ms。
     *  取 100ms ≈ 5 倍余量 —— 覆盖"模块刚切完模式、内部还在重启串口"的抖动。 */
    constexpr uint32_t E220_AT_TIMEOUT_MS = 100u;

    /** 一条应答的**字节间空隙**判据: 连续这么多毫秒没有新字节 ⇒ 这条应答结束了。
     *  取 5ms 的依据: ① @9600 一字节 ≈ 1.04ms, 5ms ≈ 5 个字节时间, 不会把同一帧切断;
     *  ② 手册 §7.1.13 说模块自己的断帧判据是 3 字节时间 —— 我们取 ≥ 它, 保证模块认为"还没断"的时候
     *     我们也不会误判结束。 */
    constexpr uint32_t E220_AT_REPLY_GAP_MS = 5u;

    /** 模式切换后等 AUX 变高的默认超时。
     *
     *  手册 §5.2.4 给的是"模式切换 T1"表 —— 本工程原先按"全部 12 种切换 9~11ms"取了 50ms(4.5 倍)。
     *  ⚠ 2026-10-06 上板复盘: 官方手册的 T1 表里 **休眠→透传那一档写的是 39~40ms**
     *  ([ebyte 模式切换表](https://www.ebyte.com/downpdf/3080.html)), 也就是说 9~11ms 并不是这张表的
     *  上界; 而 §5.2.4 注 3 的"退出配置模式时模块用**新参数**重载"**根本没给时长**。
     *  于是 50ms 会把"模块只是慢"误判成"模块不理我" —— 实测日志正是这个形态:
     *  `配置=FAIL(试了 3 轮)` + `AUX超时=3 应答异常=0 寄存器不一致=0`,
     *  即 AT/HEX 全程正常(证明模块活着、确实在配置模式), 只有 AUX 等不到高。
     *  取 150ms(≈ 手册上界 40ms 的 3.75 倍)。这一段在**开机路径**上, 多等 100ms 换"不误判"值得。
     *  (与 cfg::link::tdma::kMasterFrameLatencyUs 无关: 那是空口+串口的**每帧**时延, 不是这里。) */
    constexpr uint32_t E220_AUX_READY_TIMEOUT_MS = 150u;

    /** **退出配置模式**(参数重载)后等 AUX 变高的超时。
     *  依据: §5.2.4 注 3 只说"期间 AUX 低", 没说多久 —— 而这一刻模块正在把新参数(波特率/空速/
     *  信道/功率)重新加载进射频前端, 比普通模式切换重。取 300ms: 开机路径上的一次性成本,
     *  换掉"因为等 50ms 就判死整条链路"这种代价完全不对等的失败。 */
    constexpr uint32_t E220_AUX_RELOAD_TIMEOUT_MS = 300u;

    /** AUX 无法确认"重载完成"时的**保守固定等待**(应用层用它, 见 app_flight_stack 的 configureLink)。
     *
     *  为什么需要它: AUX 线本身有问题时(没接/被拉死/没有上拉), 我们既等不到高, 也没有别的证据;
     *  但**不能因此判死整条链路** —— ① AT/HEX 全程通过说明模块活着且在配置模式并按我们的值回了读;
     *  ② 本固件运行期根本不读 AUX(config 里的 kAuxGate 至今没有接线, 见 app_link_thread);
     *  ③ 真正的验收是"能不能收到上行帧"(解锁门槛要求连续 kMinValidUplink 帧)。
     *  取 200ms ≈ 手册上界 40ms 的 5 倍, 盖住重载窗口后照常把 MCU 串口切到工作速率。 */
    constexpr uint32_t E220_AUX_RELOAD_FALLBACK_MS = 200u;


    /** 写完 M0/M1 之后, 等"AUX 开始变忙"的探测窗口。
     *  手册没给"引脚改变 → AUX 拉低"的起始延迟(§5.2.4 只给了忙的**时长** 9~11ms),
     *  所以这是一个推导值: 取 §5.2.3 的 T1=1ms(串口首字节→AUX 拉低)再留 3 倍 = 3ms。
     *  它的用途只有一个: **区分"模块检测到了切换"和"模块根本没理我们"** —— 见 setMode() 注释。 */
    constexpr uint32_t E220_AUX_BUSY_APPEAR_MS = 3u;

    /** 上电后等 AUX 变高的默认超时: §5.2.1 T1=16ms, 取 40ms(2.5 倍余量)。
     *  超时的含义很重: AUX 一直低 = 模块没上电 / 在 IAP 升级模式 / AUX 被外部拉死。 */
    constexpr uint32_t E220_POWERUP_AUX_TIMEOUT_MS = 40u;

    /** M0/M1 电平写完后的稳定等待。手册没给数值, 这是"给引脚电平和模块输入采样留一个远大于
     *  上升沿、又远小于 9~11ms 切换时间"的窗口(100µs = 168MHz 下 16800 个周期)。 */
    constexpr uint32_t E220_MODE_PIN_SETTLE_US = 100u;

    /** 透传接收的默认"首字节"等待与"字节间空隙"判据(工程取值: 链路槽 10ms 量级)。
     *  首字节超时只是"这一轮没有数据"的返回, 不是错误; 空隙判据用 AT 应答的同一套推理(≥3 字节时间)。 */
    constexpr uint32_t E220_RX_FIRST_BYTE_TIMEOUT_MS = 20u;
    constexpr uint32_t E220_RX_GAP_TIMEOUT_MS        = 5u;

    /** 超时参数上限: 所有等待函数都会把调用方给的超时钳到这里。
     *  依据: 轮询迭代兜底的上界 = timeoutMs × E220_POLL_ITERS_PER_MS, 要留在 uint32 内
     *  (60000 × 20000 = 1.2e9 < 4.29e9)。顺带挡住"传了 0xFFFFFFFF 当超时"这种调用。 */
    constexpr uint32_t E220_TIMEOUT_MAX_MS = 60000u;

    /** 轮询兜底: 每毫秒最多允许的轮询次数。
     *  推导: 168MHz 下一次轮询 ≈ 时基函数(DWT 读 + 32 位除法 ≈ 40 周期) + USART_GetFlagStatus
     *  (标准外设库函数调用, 读 SR ≈ 30 周期) + 分支 ≈ 100 周期 ≈ 0.6µs ⇒ 约 1700 次/ms。
     *  取 20000/ms = **12 倍余量**, 既不会在正常硬件上先于时间判据触发, 又能在时基不走
     *  (DWT 没使能 / CYCCNT 恒 0)时把死循环变成"有界地失败"。 */
    constexpr uint32_t E220_POLL_ITERS_PER_MS = 20000u;
    /** 兜底的固定余量: 超时为 0 时也要允许至少一次探测, 否则"超时 0"会连一个字节都读不到。 */
    constexpr uint32_t E220_POLL_ITER_SLACK = 2000u;

    //===============================================================================================
    // 十、编译期守卫: 位域不能重叠(重叠 = 写 A 会顺手清掉 B, 而上板表现只是"某个参数偶尔不对")
    //===============================================================================================
    static_assert((E220_REG0_BAUD_MASK & E220_REG0_PARITY_MASK) == 0u &&
                      (E220_REG0_BAUD_MASK & E220_REG0_AIR_MASK) == 0u &&
                      (E220_REG0_PARITY_MASK & E220_REG0_AIR_MASK) == 0u,
                  "REG0 三个位域必须互不重叠(手册 §6.3 02H 表: [7:5]/[4:3]/[2:0])");
    static_assert((E220_REG0_BAUD_MASK | E220_REG0_PARITY_MASK | E220_REG0_AIR_MASK) == 0xFFu,
                  "REG0 三个位域必须铺满整个字节(§6.3 02H 表里没有空位)");
    static_assert((E220_REG1_PACKET_MASK & E220_REG1_RSSI_EN) == 0u && (E220_REG1_RSSI_EN & E220_REG1_SOFT_SWITCH) == 0u &&
                      (E220_REG1_SOFT_SWITCH & E220_REG1_POWER_MASK) == 0u &&
                      (E220_REG1_RESERVED & (E220_REG1_PACKET_MASK | E220_REG1_RSSI_EN | E220_REG1_SOFT_SWITCH |
                                             E220_REG1_POWER_MASK)) == 0u,
                  "REG1 位域重叠(§6.3 03H 表: [7:6]分包 [5]环境RSSI [4:3]保留 [2]软切 [1:0]功率)");
    static_assert((E220_REG3_DRSSI_BYTE & E220_REG3_TRANS_MODE) == 0u && (E220_REG3_TRANS_MODE & E220_REG3_LBT_EN) == 0u &&
                      (E220_REG3_LBT_EN & E220_REG3_WOR_MASK) == 0u &&
                      (E220_REG3_RESERVED & (E220_REG3_DRSSI_BYTE | E220_REG3_TRANS_MODE | E220_REG3_LBT_EN |
                                             E220_REG3_WOR_MASK)) == 0u,
                  "REG3 位域重叠(§6.3 05H 表: [7]包RSSI [6]传输方式 [5]保留 [4]LBT [3]保留 [2:0]WOR)");
    static_assert(E220_REG_XFER_MAX >= E220_REG_BLOCK, "块读写上限必须装得下 REG0..REG3 四个字节");
    static_assert(E220_AT_REPLY_MAX >= 24u, "AT 应答缓冲太小: 'DEVTYPE=E220-400T30S' 就已经 18 字符");
    static_assert(E220_TIMEOUT_MAX_MS * E220_POLL_ITERS_PER_MS < 0xFFFFFFFFu,
                  "超时钳位 × 轮询兜底 会溢出 uint32: 调大这两个数之前先改轮询的类型");
    static_assert(E220_AUX_READY_TIMEOUT_MS > E220_MODE_SWITCH_BUSY_MS,
                  "等 AUX 变高的超时必须大于手册 §5.2.4 的切换忙上界, 否则必然误判失败");
    static_assert(E220_AUX_RELOAD_TIMEOUT_MS >= E220_AUX_READY_TIMEOUT_MS,
                  "退出配置(参数重载)的超时不该比普通模式切换更短: 重载比切换重");
    static_assert(E220_AUX_RELOAD_FALLBACK_MS > E220_MODE_SWITCH_BUSY_MS,
                  "AUX 无法确认时的保守等待必须盖过切换忙窗口");
    static_assert(E220_POWERUP_AUX_TIMEOUT_MS > E220_POWERUP_READY_MS,
                  "上电等 AUX 的超时必须大于手册 §5.2.1 的 T1(16ms)");
    static_assert(e220BaudBps(E220_CFG_BAUD_CODE) == E220_CFG_BAUD_BPS,
                  "配置模式波特率常量与档位代码对不上(手册 §7: 配置模式强制 9600 8N1)");
} // namespace dlx

#pragma once
//===================================================================================================
// DL_LIB/E220/dlx_e220.hpp —— E220-400T22S(UART 透传 LoRa 模块)的配置与字节通道驱动
//
// 这个类**只做三件事**:
//   1. M0/M1 引脚时序: 一般(透传)模式 ↔ 配置模式 的切换, 以及 AUX 忙/闲判定;
//   2. 配置通道: AT 指令(§7)与 HEX 寄存器指令(§6.1), 每一笔都能失败、能回读校验;
//   3. 字节通道: 把字节发出去(sendRaw/sendWhenIdle)、把字节收进来(receive)。
//
// 它**不做**什么(边界写清楚, 免得下一个人把调度塞进来):
//   - 不做时分调度、不做 TDMA 相位、不做 ACK/重传/丢包统计 —— 那是链路线程的事;
//   - 不认识 cfg::: 引脚、波特率、信道、按键全部由调用方以参数传入(DL_LIB 只接受参数, 见
//     hardware_claims.hpp 文件头); 本文件一个工程配置项都不 include;
//   - 不重配串口: 进配置模式时模块**强制 9600 8N1**(手册 §7 章首), 但"把 MCU 的 USART 换成 9600"
//     这件事由调用方做 —— 驱动只管引脚与时序。调用顺序见下面「配置模式的调用顺序」。
//   - 不做帧解析: 线上帧是 DL_LIB/link/dlx_link_frame.hpp(9 种定长帧 + CRC8), 本类只搬字节。
//
//---------------------------------------------------------------------------------------------------
// 一、M0/M1 模式表(手册 §5.1 表, PAGE 13; §5.1 开头: "0 代表低电平, 1 代表高电平")
//
//   模式号  名称        手册表(M1 M0)   等价写法(M0/M1)   说明
//     0    传输模式        0  0            0 / 0         串口打开 + 无线打开 + 透明传输 ← 本工程常驻
//     1    WOR 发送        0  1            1 / 0         支持空中唤醒
//     2    WOR 接收        1  0            0 / 1         WOR 发送关闭
//     3    休眠/配置       1  1            1 / 1         **只有这个模式接受 AT/HEX 配置指令**
//
//   为什么驱动内部只认"模式号 0..3"、引脚电平由 modePinLevels() 唯一推导:
//     手册表头写的是 "M1 M0"(列序 M1 在前), 而大家口头习惯说 "M0/M1"(引脚号升序), 两者左右相反。
//     把它写成两处 if 判断, 迟早有人把 WOR 收/发对调 —— 而这两种模式在链路上都表现为"收不到",
//     根本区分不出来。所以: 唯一的映射点 + 一个单测直接断言四种组合。
//   为什么 WOR 在本工程不用: 本工程是遥控器主端/飞控从端的**时分透传**(§3.1 定案), WOR 的
//     收发双方周期必须一致(§6.3 05H 行), 与时分调度是两套机制, 混用会把时隙搅乱。
//
//---------------------------------------------------------------------------------------------------
// 二、AUX(高=空闲, 低=忙)—— 依据与**它的局限**
//
//   极性依据(手册两处独立表述, 互相印证):
//     ① §2.1 引脚表第 24 脚: AUX「输出 1.指示模块工作状态(可以悬空)」;
//     ② §5.2.4「注意事项」第 1 条原文: 「当 AUX 输出低电平时, 表示模块繁忙, 此时不会进行工作模式检测。」
//        ⇒ 低 = 忙, 高 = 空闲(auxReady() 的判据); 顺带给出一个硬约束: **忙的时候发模式切换会被忽略**。
//   时序依据(§5.2):
//     §5.2.1 上电: T1=16ms(到"用户可操作"), T2=5ms(上电后 AUX 首次拉低之前的窗口);
//     §5.2.3 无线发射: 串口首字节被识别 → AUX 拉低 1ms; 缓存空闲拉高 = 3 字节断帧时间 或 空口总耗时;
//     §5.2.4 模式切换: **全部 12 种切换** AUX 忙 9~11ms。
//
//   ⚠ 三条必须知道的局限(都会让"等 AUX 高"给出的保证比想象中弱):
//     a) §7.1.7: 固件 ≥7463-0-15 起 AT+UAUX 默认 0, AUX **不再指示无线发送过程**, 只跟踪串口缓存。
//        所以"AUX 高"只证明串口缓存空, 不证明空口发完了。本工程 link.e220.kUaux=true 就是为了打开它;
//        即便如此, 驱动这一层也不拿 AUX 当"发完了"的证据(那是调度层结合 ToA 预算判的事)。
//     b) §5.2.4 注 3: 从模式 3 退出或复位过程中, 模块会**重新加载用户参数**, 期间 AUX 低。
//        结论: 退出配置模式后必须等 AUX 高再动串口(否则那段时间的字节没人收)。
//     c) §2.1/§5.2.1: AUX 在模块上电初期是"内部上拉输入", 被外部持续拉低 >1s 会强制进 IAP 升级模式
//        (要上电保持 60 秒才退得出来)。⇒ **绝不能把 AUX 当普通输入去驱动/下拉**; 本类只读它。
//
//---------------------------------------------------------------------------------------------------
// 三、配置模式的调用顺序(**必须照这个顺序**, 串口重配不在本类里做)
//
//   // ① 把 MCU 串口切成配置模式的固定参数: 9600 8N1(手册 §7 章首: "配置模式串口强制固定 9600 8N1")
//   port.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 9600);
//   // ② 引脚切到模式 3 + 等 AUX 忙脉冲结束 + **用一条只读 AT 指令证明"真的在配置模式"**
//   if (!e220.enterConfigMode()) { /* 失败: 整段重试(cfg::hw::link::kCfgRetry), 别只重试一条命令 */ }
//   // ③ 在配置模式里读写: AT 指令 / HEX 寄存器(建议"写 + 回读比对", 见 applyRegs)
//   // ④ 引脚切回模式 0 + 等 AUX 高(退出时模块会用**新参数**重载, 期间 AUX 低)
//   if (!e220.enterTransparentMode()) { /* 同上 */ }
//   // ⑤ 现在模块已经在新波特率上: 把 MCU 串口切到传输波特率
//   //    顺序不能反 —— 先切 MCU 再退出配置模式的话, 那 9~11ms 的重载窗口里两边波特率不一致
//   port.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 57600);
//
//   为什么 enterConfigMode() 里要多花一条 AT 指令做"主动证明":
//     引脚电平本身不是证据。① 模块可能压根没检测到(忙的时候引脚变化会被忽略, §5.2.4 注 1);
//     ② 引脚本来就在 1,1(上一次失败后的重试)时, 模块没有"变化"可检测, 也就不会有忙脉冲;
//     ③ AT 只在配置模式被接受(§7 章首)。所以能收到应答 = 最强证据, 而且这条指令是只读的
//     (AT+FWCODE=? 查版本, §7.1.4), 不会改任何参数。
//
//---------------------------------------------------------------------------------------------------
// 四、为什么这里**不用** USART::receive()
//
//   DL_LIB/dlx_usart.hpp 的 `uint8_t USART::receive()` 与 `receive(len)` 里的等待方向是**反的**:
//       while (USART_GetFlagStatus(..., USART_FLAG_RXNE));   // RXNE=1(有数据)时立刻返回, 没数据时死等
//   于是它对"没数据"的反应是**死等**, 对"有数据"的反应是**立刻返回一个可能过期的字节** —— 两件事
//   都正好和调用方想要的相反。而且没有超时, 模块没接就是永久卡死(飞控里等于挂机)。
//   所以本类自己按字节轮询: USART_GetFlagStatus(port.peripheral(), USART_FLAG_RXNE) + USART_ReceiveData,
//   每一次等待都同时受 **时间** 与 **轮询次数** 两个上界约束(见下面五)。
//
//---------------------------------------------------------------------------------------------------
// 五、"绝不 while(1)"是怎么保证的
//
//   每个等待循环都有一个 Wait 预算: 超时值(毫秒, 钳到 E220_TIMEOUT_MAX_MS)+ 迭代兜底
//   (timeoutMs × E220_POLL_ITERS_PER_MS + 固定余量)。**两条判据任意一条成立就退出**:
//     - 时间判据: (nowMs - startMs) >= timeout, 无符号减法, 时基回绕也正确;
//     - 迭代判据: 即使时基**完全不走**(DWT 没使能 / CYCCNT 恒 0 / 有人把时钟函数写错), 循环也会
//       在有限次之后退出并返回失败。这条兜底是"绝不卡死"的**结构性**保证, 不是"应该不会"。
//   单测里用两个假时钟分别验证这两条路径(见 test/e220_check/check.cpp 第 4 组)。
//
//---------------------------------------------------------------------------------------------------
// 六、为什么每个函数都要能失败(NRF24L01 那次的教训, 见 release.cpp 起的这套规矩)
//
//   旧 NRF24L01 驱动的 `init()` **只写寄存器就返回 true**, 从来没验证过芯片在不在: 板子上没插模块
//   也照样"初始化成功", 于是所有链路问题都被拖到飞行中才暴露(release.cpp 里那段
//   "NRF24L01: init=OK(注意: 驱动只写寄存器, 恒返回 true)" 的日志就是为这个坑打的补丁)。
//   本类不接受"写进去就当生效":
//     - 每一条等待都有超时, 超时返回 false;
//     - 写寄存器用 writeRegsVerified(): 先写、再读、再逐字节比对, 不一致就是 false(RegMismatch);
//     - AT 设置类指令只证明"模块听懂了" —— 参数是否真的变了必须靠回读(AT+XXX=? 或 HEX 读寄存器),
//       这一点写在这里, 因为它决定了应用层的写法, 而不是本类内部能兜住的。
//   唯一"不返回 bool"的是 sendRaw(): 这一层没有任何可判定的失败条件(UART 的 TXE 会一直置位),
//   编一个恒 true 的返回值就是 NRF 那个坑的翻版 —— 所以它的名字就叫 sendRaw(纯字节泵),
//   要"发之前先确认模块空闲"请用 sendWhenIdle()(那个有 AUX 超时, 会返回 false)。
//
//---------------------------------------------------------------------------------------------------
// 七、最小用法示例(配置 → 回读校验 → 回透传)
//
//   // 引脚来自配置(这里写字面值只为让示例能读; 真实代码用 cfg::hw::link::kM0/kM1/kAux,
//   // 引脚绑定的编译期断言在应用层 hardware_claims.hpp, 不在本库)
//   USART port = USART::USART3_TD8_RD9();
//   E220  e220(port, GPIOProfile::D3, GPIOProfile::D4, GPIOProfile::A2);
//
//   port.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 9600);   // ① 配置模式固定 9600
//   if (!e220.begin() || !e220.enterConfigMode()) { /* 失败: 重试整段 */ }
//
//   // 用芯片级构造函数拼 REG0..REG3(不知道任何工程配置):
//   E220Regs regs;
//   regs.reg0 = E220::buildReg0((uint8_t)E220BaudCode::B57600, (uint8_t)E220ParityCode::N8N1,
//                               (uint8_t)E220AirRateCode::K62_5);
//   regs.reg1 = E220::buildReg1((uint8_t)E220PacketCode::P32, (uint8_t)E220TxPowerCode::P22dBm, false);
//   regs.reg2 = 17;                                    // 信道 17 = 433.125MHz
//   regs.reg3 = E220::buildReg3(false, false, false, 0); // 透明传输 / 不开 LBT / 不加 RSSI 字节
//   if (!e220.applyRegs(regs)) { /* 写+回读不一致: 重试整段 */ }
//
//   uint8_t rb[E220_REG_BLOCK];                        // 再独立回读一次给日志/自检用
//   if (!e220.readRegs(E220_REG0_ADDR, rb, E220_REG_BLOCK)) { /* 读失败 */ }
//
//   if (!e220.enterTransparentMode()) { /* 重试整段 */ }
//   port.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 57600);   // ⑤ 模块已在新波特率上
//
//   // 透传收发(时分调度在链路线程里, 不在这里):
//   e220.sendRaw(frame, len);                          // 一个射频包 = 一次连续写完(§7.1.13 断帧)
//   uint16_t n = e220.receive(buf, sizeof(buf));       // 0 = 这一轮没有数据(不是错误)
//
//   C++11 / 无异常 / 无 RTTI / 无动态分配 / header-only。
//===================================================================================================
#include <stdint.h>
#include <string.h> // strlen/memcmp/memcpy: 标准库, 不引入 printf/动态分配

#include "stm32f4xx.h" // USART_GetFlagStatus / USART_ReceiveData / DWT / CoreDebug / SystemCoreClock
#include "dlx_gpio_profile.h"
#include "dlx_gpio.hpp"  // GPIO(推挽输出 / 输入)
#include "dlx_usart.hpp" // USART(只用 send() 与 peripheral())
#include "dlx_delay.hpp" // delay_us: 引脚电平建立窗口(见 setMode)
#include "dlx_timebase.hpp" // dwtMillis(): 把 CYCCNT 展开成**单调**毫秒(25.57s 回绕不能直接除)
#include "dlx_e220_config.h"

namespace dlx
{
    /**
     * AUX(PA2)的引脚模式: **输入 + 内部上拉**, 永不驱动。
     *
     * 三条理由(与 begin() 的注释同源, 这里只列结论):
     *   ① 本板 AUX 直接接 GPIO, 没有外部上拉 ⇒ NOPULL 时是悬空输入, 读数随机;
     *   ② 内部上拉只有 ~40kΩ, 对"被短路/被器件拉低"仍然读得到低 —— 不会掩盖故障;
     *   ③ 模块上电窗口里 AUX 被持续拉低 >1s 会进 IAP 升级模式(文件头二.c), 上拉输入是
     *      "绝不把它拉低"的电路实现。
     */
    constexpr GPIOModeProfile E220_AUX_PIN_MODE = GPIOModeProfile::IN_UP;

    /** REG0..REG3 四个字节(手册 §6.3 02H~05H)。
     *  为什么做成 POD 结构体而不是四个散参数: 它天然是一个事务 —— 手册 §6.1 例 2 就是
     *  "C0 00 03 12 34 62" 一条指令连着写多字节; 拆成四次写会多三次超时机会, 而且中途失败会
     *  留下"一半新一半旧"的参数(比如波特率变了、空速没变), 那种状态在链路上极难定位。 */
    struct E220Regs
    {
        uint8_t reg0; ///< 02h: 串口波特率 + 串口校验 + 空中速率
        uint8_t reg1; ///< 03h: 分包长度 + 环境RSSI + 软切模式 + 发射功率
        uint8_t reg2; ///< 04h: 信道
        uint8_t reg3; ///< 05h: 包RSSI字节 + 传输方式 + LBT + WOR周期
    };

    class E220
    {
    public:
        /** 毫秒时基函数(单调、允许回绕)。默认实现见 dwtMillis(); 单测注入假时钟来验证有界性。 */
        typedef uint32_t (*MillisFn)();

        /** 一帧 DMA 发送缓冲上限: 链路单帧 ≤21B, 给 32B(与 E220 分包一致)。 */
        static constexpr uint16_t kTxDmaCapacity = 32u;

        /** 工作模式(手册 §5.1; 数值就是手册的"模式(0-3)", 引脚电平由 modePinLevels() 推导) */
        enum class Mode : uint8_t
        {
            Transparent = E220_MODE_TRANSPARENT, ///< 0: (M1,M0)=(0,0) 透传 —— 本工程常驻
            WorTx       = E220_MODE_WOR_TX,      ///< 1: (M1,M0)=(0,1) WOR 发送
            WorRx       = E220_MODE_WOR_RX,      ///< 2: (M1,M0)=(1,0) WOR 接收
            Config      = E220_MODE_CONFIG,      ///< 3: (M1,M0)=(1,1) 休眠/配置
        };

        /** 失败原因(给日志/自检用; 每次都只记**最近一次**)。为什么要它: 全部返回 bool 的话,
         *  上板看到"进配置模式失败"时分不清是"AUX 没动"还是"模块不回话", 而这两者的排查方向完全不同。 */
        enum class Fail : uint8_t
        {
            None = 0,
            BadParam,    ///< 调用方参数不合法(空指针/长度越界): 属于写错了, 不是模块的问题
            AuxTimeout,  ///< 等 AUX 电平超时(模块没上电/忙太久/AUX 线没接)
            RxTimeout,   ///< 等串口字节超时(模块没接/波特率不对/模块在透传模式下不理 AT)
            BadReply,    ///< 收到的不是期望的应答(含 HEX 的 FF FF FF 错误响应)
            RegMismatch, ///< 写寄存器后回读不一致(模块没接上/写保护/参数被钳位)
            TxBusy,      ///< sendDma: 上一帧 DMA 还没发完(调用方应放弃本轮)
            TxTimeout,   ///< sendDma 后等不到 TC/DMA 完成(线被拉死/模块异常)
        };

        /**
         * @param port  E220 所在串口(**不重配它**: 见文件头「配置模式的调用顺序」)
         * @param m0    M0 脚(推挽输出)  §2.1 第 20 脚 —— 不可悬空
         * @param m1    M1 脚(推挽输出)  §2.1 第 21 脚 —— 不可悬空
         * @param aux   AUX 脚(输入)     §2.1 第 24 脚 —— 只读, 绝不驱动(§5.2.1 IAP 陷阱)
         * @param nowMs 毫秒时基。默认 DWT 周期计数器; 单测传假时钟。
         */
        E220(USART &port, GPIOProfile m0, GPIOProfile m1, GPIOProfile aux, MillisFn nowMs = &E220::dwtMillis)
            : _port(port), _m0(m0), _m1(m1), _aux(aux), _nowMs(nowMs), _mode(Mode::Transparent), _m0Level(false),
              _m1Level(false), _pinsDriven(false), _modeConfirmed(false), _lastFail(Fail::None), _pollIters(0u),
              _auxTimeouts(0u), _rxTimeouts(0u), _badReplies(0u), _regMismatches(0u), _auxBusyMisses(0u),
              _auxWaitMs(0u), _auxLevelAtBoot(false), _replyLen(0u), _versionLen(0u),
              _txView(_txRaw, kTxDmaCapacity), _txDma(_txView, port.getDefaultDMATxProfile())
        {
            _reply[0]   = '\0';
            _version[0] = '\0';
        }

        //===========================================================================================
        // 生命周期
        //===========================================================================================
        /**
         * @brief 配引脚方向 → 先钉在透传模式(M0=M1=0) → 等 AUX 高(模块上电就绪)
         *
         * 为什么一开始就钉透传: §5.1.1 本工程常驻透传; 而且 MCU 复位期间 GPIO 是高阻, 模块内部对
         *   M0/M1 只有"极弱上拉"(§2.1 第 20/21 脚, 原文"不可悬空"), 只能靠外部下拉保证落在透传 ——
         *   软件这边一上电就再钉一次, 是第二道保险。
         * 为什么 AUX 用 **IN_UP**(2026-10-06 改; 原为 IN_NOPULL): 三条理由, 都比"靠板上那颗 10k"
         *   更硬 ——
         *   ① 这块板的 AUX 是**直接接到 GPIO** 的(用户确认): 没有外部上拉时, NOPULL 就是"悬空输入",
         *      读数随机; IN_UP 让"悬空"变成一个**确定的高电平 = 空闲**, 而不是碰运气。
         *   ② 原注释担心"内部上拉会掩盖 AUX 被外部短路拉低" —— 这条不成立: 内部上拉只有 ~40kΩ,
         *      对地被短路/被器件主动拉低时引脚仍然是低(0V), 该报的照样报得到。
         *   ③ 它正好命中 IAP 陷阱的安全侧(文件头二.c): 模块**上电初期**把 AUX 当"内部上拉输入",
         *      被外部持续拉低 >1s 就进固件升级模式(要上电保持 60s 才退得出来)。把 MCU 侧做成上拉输入,
         *      就是"绝不把它拉低"这句话的电路实现。**注意**: 若板上另有下拉电阻, 固件救不了它 ——
         *      拔掉模块量 PA2 应当是 ~3.3V; 量到 0V 就是下拉电阻要去掉(见 M4/M5 记录 §7)。
         * @return true = AUX 在超时内变高(模块确实活了); false = 模块没就绪/没接/或 AUX 线有问题。
         *         ⚠ **false 不等于"链路不能配"**: 判据在应用层(configureLink) —— 先看 AT 能不能通,
         *         AT 通了就说明模块活着, AUX 只是"辅助证据"。
         */
        bool begin(uint32_t auxTimeoutMs = E220_POWERUP_AUX_TIMEOUT_MS)
        {
            // 为什么是 2MHz 而不是 50MHz: 这两个脚只用来选模式(切换频率以秒计), 边沿速率越低,
            // 对紧挨着的 433MHz 射频前端的干扰越小 —— 手册 §8「硬件设计」明确要求高频数字走线远离模块。
            _m0.init(GPIOModeProfile::OUT_PP_NOPULL_2MHz);
            _m1.init(GPIOModeProfile::OUT_PP_NOPULL_2MHz);
            _aux.init(E220_AUX_PIN_MODE);

            // 先读**原始**电平 —— 必须在写任何模式引脚**之前**: 写 M0/M1 会让模块忙 9~11ms(§5.2.4),
            // 之后再读就分不清"上电时就是低(接线/下拉的锅)"与"是我们刚把它写忙了"。
            // 日志里那行 AUX 诊断靠这个数定向(见 app_flight_stack 的 AUX 专项诊断)。
            _auxLevelAtBoot = _aux.read();

            _m0 = false;
            _m1 = false;
            _m0Level     = false;
            _m1Level     = false;
            _pinsDriven  = true;
            _mode        = Mode::Transparent;
            _modeConfirmed = false;

            if (!waitAux(auxTimeoutMs)) {
                return false; // AuxTimeout 已记在 lastFail(); _modeConfirmed 保持 false
            }
            _modeConfirmed = true;
            return true;
        }

        /**
         * @brief **上电最早期**就把 AUX 引脚配成"上拉输入"(静态, 不需要 E220 实例)。
         *
         * 为什么单独给一个静态入口、并且要在 main() 的第一条语句调用:
         *   模块与 MCU 同时上电, 而模块在自己的上电窗口里会检查 AUX —— 被持续拉低 >1s 就进 IAP
         *   升级模式(文件头二.c)。MCU 复位后 PA2 是高阻输入(不驱动, 安全), 但**只要有别的代码在
         *   这条线上先配成了输出/下拉, 这个安全就不成立了**。所以在任何外设初始化之前先把它钉成
         *   "上拉输入": 既给线路一个确定电平, 又从结构上杜绝"我们把它拉低"。
         * 为什么不用 begin() 代劳: begin() 在链路自检里(IMU/flash/USART 之后), 那时模块早已上电;
         *   而且它还会切模式引脚 —— 早期只需要"这一根脚有确定电平、且永不驱动"。
         * 绝不驱动: 本类只读 AUX(见文件头二.c 与 auxReady())。
         */
        static void auxPinSafeNow(GPIOProfile aux)
        {
            GPIO g(aux);
            g.init(E220_AUX_PIN_MODE);
        }

        /** 走线自检的结果 —— 全是**量出来的事实**, 不是推断(启动日志按行打印, 见 app_flight_stack) */
        struct LineHealth
        {
            bool auxHigh;   ///< AUX 现在的电平(true = 空闲)
            bool auxDriven; ///< AUX 抗内部下拉读到高 ⇒ **外部有驱动**(模块已上电且 AUX 线通)
            bool rxHigh;    ///< 模块 TXD → MCU RX 这根线现在的电平(UART 空闲应为高)
            bool rxDriven;  ///< 同上"抗下拉读高" ⇒ 模块 TXD 通了。"收字节=0"时**先看这一条**
        };

        /**
         * @brief 上电自检: AUX / 模块 TXD 两根**输入线**上到底有没有外部驱动
         *
         * 为什么需要它: 飞控这一侧此前只有"AT 通不通"一个硬件判据, 而 AT 能证明的只有
         *   ①模块上电 ②TX/RX 两根线通(有来有回) —— 它证明不了"模块 TXD 这根线**现在**还是活的",
         *   也说不清 AUX 到底是"模块说空闲"还是"我们自己的上拉"。本函数把这两件事量出来:
         *   让引脚先"内部下拉"再读 —— 外部有驱动时读得到高(40kΩ 内部下拉压不住它),
         *   线是死的(悬空/断线/模块没上电)时只会读回低。
         *
         * ⚠ 只碰这两根的上下拉与方向, 不碰 M0/M1、不切模式; 结束时把工作模式还回去:
         *   AUX → E220_AUX_PIN_MODE(上拉输入, IAP 陷阱的安全条件), RX → USART3 的 AF7。
         *   M0/M1 的焊盘证据不在这里 —— 它在 setMode() 里顺手做(那里本来就要写这两根引脚)。
         */
        static LineHealth probeLines(GPIOProfile aux, GPIOProfile rx)
        {
            LineHealth h{};
            {
                GPIO ga(aux);
                ga.init(GPIOModeProfile::IN_NOPULL);
                h.auxHigh = ga.read();
                ga.init(GPIOModeProfile::IN_DOWN); // 内部下拉压不住外部驱动 ⇒ 读到高 = 有人在驱动
                h.auxDriven = ga.read();
                ga.init(E220_AUX_PIN_MODE); // 恢复本工程要的"上拉输入"
            }
            {
                GPIO gr(rx);
                gr.init(GPIOModeProfile::IN_NOPULL);
                h.rxHigh = gr.read();
                gr.init(GPIOModeProfile::IN_DOWN);
                h.rxDriven = gr.read();
                // RX 是 USART3 的 AF 脚: 探测完必须交还 AF7, 否则后面 USART3 一个字节都收不到
                gr.init(GPIOModeProfile::AF7_PP_NOPULL_50MHz);
            }
            return h;
        }

        /**
         * @brief 只配三根引脚的方向: M0/M1 = 推挽输出, AUX = 输入。**不写电平, 不碰串口**。
         *
         * 与 begin() 的关系: begin() = initPins() + 钉透传(M0=M1=0) + 等 AUX 高。
         *   分成两个入口是为了让调用方能先只配方向(比如开机初始化阶段按顺序拉引脚),
         *   再决定什么时候等模块上电就绪。
         * 为什么电平留到 setMode()/enterConfigMode() 去写(这里不写):
         *   ① M0/M1 复位后 ODR=0 ⇒ 本来就在 (M1,M0)=(0,0)=透传模式(手册 §5.1 模式 0),
         *      这里再写一次 0 没有任何电气变化;
         *   ② 模块**只在引脚组合发生变化时才产生 AUX 忙脉冲**(手册 §5.2.4 注 1: 忙的时候不做模式检测),
         *      而 setMode() 正是靠"忙脉冲"当"模块确实看到切换了"的证据(见 setMode 注释)。
         *      在这里先写一次 0 只会把那次真正的切换边沿提前用掉, 让证据变弱。
         * 为什么 AUX 用 E220_AUX_PIN_MODE(输入 + 上拉): 见该常量与 begin() 的注释 —— 本板 AUX
         *   直接接 GPIO(无外部上拉), 上拉输入既给线路确定电平, 又是"绝不把它拉低"的电路实现;
         *   内部 40kΩ 不足以掩盖"被短路/被器件拉低", 该报的故障照样报得到。
         * 为什么 M0/M1 用 2MHz 而不是 50MHz: 这两个脚只用来选模式(切换频率以秒计), 边沿速率越低,
         *   对紧挨着的 433MHz 射频前端的干扰越小(手册 §8「硬件设计」要求高频数字走线远离模块)。
         */
        void initPins()
        {
            _m0.init(GPIOModeProfile::OUT_PP_NOPULL_2MHz);
            _m1.init(GPIOModeProfile::OUT_PP_NOPULL_2MHz);
            _aux.init(E220_AUX_PIN_MODE);

            // 记账与"构造函数刚结束"一致: 软件还没驱动过电平, 也没确认过任何模式。
            _pinsDriven    = false;
            _mode          = Mode::Transparent; // 复位后的引脚组合就是模式 0
            _modeConfirmed = false;
            _m0Level       = false;              // 复位后 ODR=0: 这是**实际**电平, 不是"我们写下去的"
            _m1Level       = false;
        }

        /**
         * @brief 默认毫秒时基: DWT 周期计数器 → **单调**毫秒(见 dlx_timebase.hpp)
         *
         * 为什么不用 rt_tick_get_millisecond(): ① DL_LIB 不认识 RTOS(库的规矩, 换个工程就不该改库);
         *   ② 开机早期调度器还没起来, 那时也要能算超时(配置模块正是开机路径)。
         * 为什么不用 dlx_delay.hpp 那套读 SysTick->VAL: 那是"延时"不是"单调时基", 而且 SysTick 归
         *   RT-Thread 管, 它的 LOAD/VAL 在调度器跑起来之后不再是稳定时间源。
         * ⚠ 回绕(2026-10-06 更正): 旧注释写"除法后回绕周期 ≈ 49.7 天" —— **错的**。CYCCNT 是 32 位
         *   周期计数器, 168MHz 下每 2^32/168e6 = **25.57 秒**就绕一圈; `/(SystemCoreClock/1000)`
         *   只是把回绕点换算成毫秒数, **搬不走回绕本身**(仍然是 25.57 秒一次)。
         *   所以这里走 dlx_timebase 的无符号差分累加, 把时基展开成真正单调的微秒(回绕 71.6 分钟);
         *   而所有等待仍然用无符号减法比较 ⇒ 单次超时 < 25s 就正确(E220_TIMEOUT_MAX_MS=60s 是钳位,
         *   实际用的默认值都是几十毫秒)。真正兜底的是轮询迭代上界, 不依赖这里正确。
         */
        static uint32_t dwtMillis()
        {
            if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) {
                CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 打开跟踪单元
                DWT->CYCCNT = 0u;
                DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 打开周期计数
            }
            const uint32_t cyclesPerUs = (uint32_t)(SystemCoreClock / 1000000u);
            if (cyclesPerUs == 0u) {
                return 0u; // 时钟没配好: 恒 0(时基判据失效), 由轮询迭代兜底保证不死等
            }
            // 函数内静态: POD 零初始化 ⇒ 常量初始化, 不需要运行期守卫(AT 等待都在单线程的开机路径)
            static UsExpander s_exp{};
            (void)usExpanderStep(s_exp, DWT->CYCCNT, cyclesPerUs); // 推进到"现在"
            return usExpanderMillis(s_exp);
        }

        //===========================================================================================
        // 模式切换(只做引脚 + 时序; 串口重配由调用方做)
        //===========================================================================================
        /** 模式号 → (M0 高?, M1 高?)。**唯一**的引脚映射点, 依据见文件头「M0/M1 模式表」 */
        static void modePinLevels(Mode m, bool &m0High, bool &m1High)
        {
            e220ModePinLevels(static_cast<uint8_t>(m), m0High, m1High);
        }

        /**
         * @brief 把 M0/M1 设到目标模式, 并**尽力证明**模块真的切过去了
         *
         * 判定过程(每一步的依据都在括号里):
         *   0. 幂等: 引脚本来就等于目标组合、且上次确认过 ⇒ 立刻返回(不白等忙脉冲);
         *   1. **等 AUX 变高(空闲)** —— §5.2.4 注 1: 模块忙的时候**不做模式检测**,
         *      此时写引脚等于白写(2026-10-06 补: 这条漏掉过, 见函数体内 ① 的长注释);
         *   2. 写 M0/M1 两根引脚;
         *   3. delay_us(100): 给电平建立 + 模块输入采样留窗口(手册没给这个数, 这是推导值, 见 config 头);
         *   4. 等 AUX **变低**(忙) —— §5.2.4: 全部 12 种模式切换 AUX 都忙 9~11ms。
         *      为什么要等"变低"而不是直接等"变高": AUX 在空闲时本来就是高的, 直接等"高"会**立刻成功**,
         *      于是"模块根本没理我们"和"模块切好了"得到同一个结果 —— 那是假成功。
         *      ⚠ 这条只有在第 1 步真的等到过空闲时才成立; 否则"AUX 已经低"与"我们的忙脉冲"
         *      分不开(旧代码就是这样把"flash 还在写"读成"切换被看到"的)。
         *   5. 等 AUX 变高(忙结束) —— 超时返回 false。
         *
         * 返回值语义(重要): false = "没能证明切换成功", **不表示引脚没动** —— 引脚一定已经设成目标组合了
         *   (这是本函数的语义: 设引脚 + 等确认)。所以调用方拿到 false 应当**整段重试**(cfg::hw::link::kCfgRetry),
         *   而不是"假设模块还在原模式继续用"。
         *   为什么不做内部重试: 重试策略(次数/间隔/要不要重配串口)是应用层的决定, 驱动里藏一个重试
         *   只会让上层的重试次数变成乘法关系。
         */
        bool setMode(Mode m, uint32_t auxTimeoutMs = E220_AUX_READY_TIMEOUT_MS)
        {
            if (static_cast<uint8_t>(m) > E220_MODE_MAX) {
                fail(Fail::BadParam);
                return false;
            }
            bool m0High = false;
            bool m1High = false;
            modePinLevels(m, m0High, m1High);

            const bool samePins = _pinsDriven && (m0High == _m0Level) && (m1High == _m1Level);
            _mode       = m;

            if (samePins && _modeConfirmed) {
                return true; // 幂等: 引脚没变化, 模块不会有忙脉冲可等, 上次又已经确认过
            }

            /**
             * ① **先等模块空闲, 再动引脚** —— 2026-10-06 上板复盘补上的一步。
             *
             * 它是"配置=OK、寄存器逐字节一致、密钥已写, 却一个字节都收不到"的**元凶**:
             *   手册 §5.2.4 注 1(本文件头二.b 也抄过): **模块忙的时候不做模式检测** ——
             *   此时把 M0/M1 写过去, 这次切换会被**丢掉**, 模块仍然停在原模式里。
             *   而本工程的启动路径必然踩到它: applyLinkConfig() 的最后两条 AT+KEY / AT+UAUX
             *   都要写 flash, 写完 AUX 还是低的, 紧接着 enterTransparentMode() 就把引脚写成 (0,0)
             *   ⇒ 切换被丢 ⇒ **模块留在配置模式**: 空口既不发也不收, 而"配置=OK"是真的
             *   (它确实在配置模式里陪着我们做完了全套 AT)。
             *
             * 为什么旧顺序里那条"忙脉冲证据"没能报出来: waitAuxLow() 有快路径 ——
             *   AUX 本来就低(flash 还在写)就直接算"看到忙脉冲了"。于是"我们自己的切换"与
             *   "模块正在忙别的事"混成同一件事 ⇒ **假成功**。
             * 新顺序下忙脉冲才是真证据: 只有**先确认空闲**(wasIdle),
             *   之后那一次"高→低→高"才说明模块确实看到了这次切换。
             *
             * 代价(可接受): AUX 线坏掉(恒低)时这里会先白等一个超时。但那种情况下忙脉冲本来就
             *   不可信, 驱动照实记 auxBusyMisses 并返回 false("没能证明", 不是"没切换"),
             *   由应用层决定保守兜底 —— 见 configureLink 的 ⑤。
             */
            const bool wasIdle = waitAux(auxTimeoutMs);

            _m0         = m0High;
            _m1         = m1High;
            _m0Level    = m0High;
            _m1Level    = m1High;
            _pinsDriven = true;

            delay_us(E220_MODE_PIN_SETTLE_US);

            /**
             * ② 写完引脚**回读自己的焊盘** —— 推挽输出下 IDR 就是真实电平, 读到与目标不符
             *    就说明这根线被外部拽住了(短路/重负载/焊接), 模块**永远看不到**这次切换。
             *    这条是从遥控器侧移植的: 那边 GPIO46 写 1 回读 0, 由此抓出一根被拽低的 M0。
             *    为什么放在这里而不是单独做一次自检: 这里本来就要写这两根引脚, 证据是**免费**的,
             *    而单独探测还得把引脚来回翻一遍(那本身又会引入一次可能被丢掉的切换)。
             */
            _m0PadHigh = _m0.read();
            _m1PadHigh = _m1.read();
            if (_m0PadHigh != m0High || _m1PadHigh != m1High) {
                ++_padMismatches;
            }

            const bool sawBusy = waitAuxLow(E220_AUX_BUSY_APPEAR_MS);
            // 证据可信(切换前空闲 + 看到忙脉冲) ⇒ 按正常超时等忙窗口结束;
            // 不可信 ⇒ 只做一个保守固定等待, 让"新参数重载"那段时间过去(AUX 不再是判据)。
            const bool idleAfter = waitAux((wasIdle && sawBusy) ? auxTimeoutMs : E220_AUX_RELOAD_FALLBACK_MS);

            _modeConfirmed = wasIdle && sawBusy && idleAfter;
            if (!_modeConfirmed) {
                // 记 AuxTimeout 而不是别的: 上层(configureLink 的 ⑤)按"AUX 无法确认"这一条决定
                // 是否用保守等待兜底继续 —— 这与旧行为一致, 免得把"没能证明"升级成"链路配不上"。
                ++_auxBusyMisses; // "没能证明这次切换被模块看到": 忙的时候切、或根本没有忙脉冲
                fail(Fail::AuxTimeout);
            }
            return _modeConfirmed;
        }

        /**
         * @brief 进配置模式: setMode(Config) + **用一条只读 AT 指令证明"真的在配置模式"**
         *
         * 为什么必须做这个主动证明(不能只看引脚/AUX):
         *   ① §5.2.4 注 1: 忙的时候引脚变化会被忽略;
         *   ② 引脚本来就在 1,1(上一次失败后的重试)时模块没有"变化"可检测, 不会有忙脉冲;
         *   ③ §7 章首: AT 指令只在配置模式被接受 ⇒ 能收到应答就是最强证据。
         * 为什么用 AT+FWCODE=?(§7.1.4)而不是随便一条: 它是**只读**的, 失败也不会改坏任何参数,
         *   而且顺手把版本号缓存下来(queryVersion 会用到)。
         * @param auxTimeoutMs   模式切换的 AUX 超时
         * @param probeTimeoutMs 主动证明那一条 AT 的超时
         * @return true 仅当"引脚已设 + 模块在配置模式回话了"。**返回 true 之后才能发 AT/HEX 指令。**
         * @warning 调用前请先把 MCU 串口切成 9600 8N1(§7 章首强制) —— 本函数不碰串口配置。
         */
        bool enterConfigMode(uint32_t auxTimeoutMs = E220_AUX_READY_TIMEOUT_MS,
                             uint32_t probeTimeoutMs = E220_AT_TIMEOUT_MS)
        {
            if (_mode == Mode::Config && _modeConfirmed) {
                return true; // 已经在配置模式且证明过: 不再发指令(避免在错误状态下往空口灌 AT 字符串)
            }
            const bool pulseOk = setMode(Mode::Config, auxTimeoutMs); // 失败也继续: 靠下面的主动证明定论
            if (!probeConfigMode(probeTimeoutMs)) {
                _modeConfirmed = false;
                return false;
            }
            _modeConfirmed = true;
            if (!pulseOk) {
                ++_auxBusyMisses; // 记一笔: "AT 能通但没看到忙脉冲" = 引脚本来就在 1,1 或 AUX 线有问题
            }
            return true;
        }

        /**
         * @brief 回透传模式(M0=M1=0)并等 AUX 高
         *
         * 为什么这里**没有** §"主动证明": 透传模式下模块对 AT 指令没有应答通道(字节会被当成数据发到空中),
         *   所以"回到透传"事后只能靠"发一帧、对端/统计确认"来验证 —— 那是链路层的事。驱动这一层能给
         *   出的最强证据就是 §5.2.4 的忙→闲时序。
         * @warning 手册 §5.2.4 注 3: 退出模式 3 时模块会用**新参数**重载, 期间 AUX 低 ⇒ 本函数返回 true
         *          (AUX 已高)之后调用方才可以动串口/发字节。
         */
        bool enterTransparentMode(uint32_t auxTimeoutMs = E220_AUX_RELOAD_TIMEOUT_MS)
        {
            return setMode(Mode::Transparent, auxTimeoutMs);
        }

        /**
         * @brief 退出配置模式 = M0=M1=0(透传模式, 手册 §5.1 模式 0)并等 AUX 高。
         *
         * 为什么只是 enterTransparentMode() 的别名: 两者是同一件事, 这里是**接线契约**用的名字
         *   (上层按"进配置 / 退配置"成对调用, 读起来不容易只进不出)。
         * ⚠ 手册 §5.2.4 注 3: 从模式 3 退出时模块会用**新参数**重载, 期间 AUX 低 —— 所以本函数返回
         *   true(AUX 已高)之后, 调用方才能把 MCU 串口切到传输波特率、才能开始发字节; 顺序不能反
         *   (先切 MCU 再退配置模式的话, 那 9~11ms 的重载窗口里两边波特率不一致)。
         */
        bool exitConfigMode(uint32_t timeoutMs)
        {
            return enterTransparentMode(timeoutMs);
        }

        Mode mode() const { return _mode; }
        bool inConfigMode() const { return _mode == Mode::Config; }
        /** 最近一次模式切换是否被"证明"过(忙脉冲 + 配置模式的 AT 应答)。给自检/日志用 */
        bool modeConfirmed() const { return _modeConfirmed; }

        //===========================================================================================
        // AUX
        //===========================================================================================
        /** AUX 是否空闲(高)。**非阻塞**, 只读一次引脚。依据: 手册 §5.2.4 注意事项第 1 条 */
        bool auxReady() { return _aux.read() == E220_AUX_LEVEL_IDLE; }

        /** 等 AUX 变高(空闲)。超时返回 false(记 AuxTimeout)。
         *  有界性: 时间判据 + 轮询迭代兜底(见文件头五) —— AUX 恒低时它一定会返回。
         *  快路径: 已经空闲就直接返回, **连时钟都不读**(空闲是常态: 每 10ms 的链路槽都会问一次,
         *  而时基要走一次 DWT 读 + 32 位除法)。 */
        bool waitAux(uint32_t timeoutMs)
        {
            if (_aux.read() == E220_AUX_LEVEL_IDLE) {
                _pollIters = 0u;
                _auxWaitMs = 0u; // 快路径: 本来就在高
                return true;
            }
            Wait w;
            waitBegin(w, timeoutMs);
            for (;;) {
                if (_aux.read() == E220_AUX_LEVEL_IDLE) {
                    _pollIters = w.iters;
                    _auxWaitMs = (uint32_t)(_nowMs() - w.startMs); // 实测等了多久: 上板日志靠它判断"模块慢"还是"线有问题"
                    return true;
                }
                if (waitExpired(w)) {
                    _pollIters = w.iters;
                    _auxWaitMs = (uint32_t)(_nowMs() - w.startMs);
                    fail(Fail::AuxTimeout);
                    return false;
                }
            }
        }

        /** 等 AUX 变低(忙)。**探测**性质: 失败不记 lastFail(它常被用来判断"忙脉冲有没有出现",
         *  "没出现"是合法结果, 不是故障)。同样有快路径。 */
        bool waitAuxLow(uint32_t timeoutMs)
        {
            if (_aux.read() == E220_AUX_LEVEL_BUSY) {
                _pollIters = 0u;
                return true;
            }
            Wait w;
            waitBegin(w, timeoutMs);
            for (;;) {
                if (_aux.read() == E220_AUX_LEVEL_BUSY) {
                    _pollIters = w.iters;
                    return true;
                }
                if (waitExpired(w)) {
                    _pollIters = w.iters;
                    return false;
                }
            }
        }

        /** 等 AUX 到**指定**电平(接线契约重载): wantHigh=true 等空闲(高), false 等忙(低)。
         *  为什么要有它: "等忙脉冲"和"等忙结束"是两件事, 用一个 bool 参数表达比记两个函数名可靠。
         *  语义与上面两个完全一致(含 has 快路径): 到达目标电平 → true; 超时 → false(AuxTimeout 只在
         *  wantHigh=true 时记, 因为"没等到忙脉冲"是合法探测结果, 见 waitAuxLow 注释)。 */
        bool waitAux(uint32_t timeoutMs, bool wantHigh)
        {
            return wantHigh ? waitAux(timeoutMs) : waitAuxLow(timeoutMs);
        }

        /** const 版本: 只读一次 AUX 引脚(判据与上面非 const 的 auxReady() 逐字相同)。
         *  为什么需要: 日志/状态上报常常拿到的是 `const E220&`(比如某个只读的健康检查),
         *  为此把整条调用链的 const 去掉不值得。这里用 const_cast 是**安全**的:
         *  AUX 只读(文件头二.c: 绝不驱动), 读一次引脚不修改任何可见状态。 */
        bool auxReady() const { return const_cast<E220 *>(this)->_aux.read() == E220_AUX_LEVEL_IDLE; }

        //===========================================================================================
        // AT 指令(手册 §7)
        //===========================================================================================
        /**
         * @brief 发一条 AT 指令并读回**原始应答文本**
         *
         * 为什么先 discardRx(): 如果刚从透传模式过来, RXNE 里可能还压着对面发来的数据字节, 不丢掉就会
         *   把应答读串(表现为"应答里混了垃圾"——而 NRF 那次教过: 不能假设总线是干净的)。
         * 为什么发送时**不加 CR/LF**: §7 章首说固件 ≥7463-0-15 会自动识别末尾回车换行、不加也识别;
         *   而 ≤7463-0-14 不识别 —— 不加 CR/LF 是两种固件都能工作的写法。
         * @param cmd       形如 "AT+UART=6,0"(不带 CR/LF; 以 '\0' 结尾)
         * @param reply     出参缓冲(会补 '\0')
         * @param replyCap  缓冲容量(**含** '\0' 位)
         * @param replyLen  出参: 实际收到的应答字节数
         * @param timeoutMs 等**第一个**字节的超时; 之后的字节间空隙用 E220_AT_REPLY_GAP_MS 判结束
         * @return true = 收到至少一个字节(这条应答结束了)。**内容对不对由调用方判断** ——
         *         模块回 "=ERR" 时本函数同样返回 true(那也是一条合法应答)。
         */
        bool atCommand(const char *cmd, char *reply, uint16_t replyCap, uint16_t &replyLen,
                       uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            replyLen = 0u;
            uint16_t len = 0u;
            if (!transact(cmd, timeoutMs, len)) {
                return false;
            }
            replyLen = copyOut(reply, replyCap, len);
            return true;
        }

        /**
         * @brief 4 参重载(接线契约用): **自动补 CR/LF**, 读一条应答, 返回"这一步成功了吗"
         *
         * 与上面 5 参版本的差别只有三点, 每条都有依据:
         *   ① **自动补 "\r\n"**: 手册 §7 章首第 3 条 —— 固件 ≥7463-0-15 **自动识别**末尾回车换行,
         *      补了不会错。⚠ 同一句还说 ≤7463-0-14 那一代是"末尾不需要回车换行", 即**不保证**容忍
         *      多出来的 CR/LF; 本工程这块是 7463-0-1x 世代(见 config 头 / 研究笔记 §12), 所以照契约补。
         *      5 参版本故意不补, 为的是两种固件都稳 —— 两件事都留着, 由调用方选。
         *   ② 返回值 = "应答里出现 OK 且没有 ERR"(设置类指令成功的形态, 手册 §7.1.1 的 "=OK"),
         *      **不是** "收到回话了"。查询类应答形如 "AT+XXX=<值>"(§7.1.3/§7.1.4)不含 OK ⇒ 这条重载
         *      返回 false —— 要查询请用 queryVersion()/queryUartCode() 或 5 参版本。
         *   ③ out 拿到的是**原始**应答文本(可能带前导 CR/LF), 判失败时也能拿去打日志。
         * @param outBytes out 的容量(**含** '\0' 位); 传 0 或空指针 ⇒ 直接 false(不崩、不发)
         * @return true 仅当: 发出去 ∧ 收到一条应答 ∧ 应答判 OK
         */
        bool atCommand(const char *cmd, char *out, uint16_t outBytes, uint32_t timeoutMs)
        {
            if (out != 0 && outBytes > 0u) {
                out[0] = '\0'; // 先给个空串: 失败时调用方打日志不会读到脏数据
            }
            if (cmd == 0 || out == 0 || outBytes == 0u) {
                fail(Fail::BadParam);
                return false;
            }
            const uint16_t n = (uint16_t)strlen(cmd);
            char           line[E220_AT_TEXT_MAX + 2u]; // 命令 + CR + LF
            if (n == 0u || (uint32_t)n + 2u >= (uint32_t)sizeof(line)) {
                fail(Fail::BadParam); // 空命令/太长: 宁可拒绝, 也不要把命令截断成**另一条**指令发出去
                return false;
            }
            memcpy(line, cmd, n);
            line[n]      = '\r';
            line[n + 1u] = '\n';
            line[n + 2u] = '\0';

            uint16_t len = 0u;
            if (!transact(line, timeoutMs, len)) {
                return false; // 一个字都没回来: RxTimeout 已记在 lastFail()
            }
            (void)copyOut(out, outBytes, len);
            if (contains(_reply, len, E220_AT_ERR)) {
                fail(Fail::BadReply);
                return false;
            }
            if (!contains(_reply, len, "OK")) {
                fail(Fail::BadReply);
                return false;
            }
            return true;
        }

        /**
         * @brief 发设置类 AT 指令, 判"成功"= 应答含 "=OK" 或 "AT+XXX=<值>"
         *
         * 为什么要接受两种形态: 手册 §7.1.1/7.1.5/7.1.7… 的设置应答是 "=OK", 但 §7.1.6(AT+MODE)与
         *   §7.1.21(AT+POWER)的示例应答写的是 "AT+XXX=<v>" —— 手册自身不一致(研究笔记 §13 记录在案)。
         *   只认一种的话, 换一个固件版本就会把成功判成失败。
         * 为什么还要判 "=ERR": §7 章首第 4 条 —— 识别/执行异常应答 "=ERR", 那是明确失败。
         * @warning true 只证明"模块听懂了这条指令", **不证明参数真的变成了你要的值**
         *          (手册还说参数超范围会被"限制"而不是报错)。要证明值变了必须回读:
         *          AT+XXX=? 或 HEX 读寄存器, 见 queryUartCode()/writeRegsVerified()。
         */
        bool atSet(const char *cmd, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            uint16_t len = 0u;
            if (!transact(cmd, timeoutMs, len)) {
                return false;
            }
            char name[16];
            atName(cmd, name, sizeof(name));
            if (replyIsSetOk(_reply, len, name)) {
                return true;
            }
            fail(Fail::BadReply);
            return false;
        }

        /** 查询类指令: 发 "<name>=?"(§7.1.3/7.1.4/7.1.11 的查询格式), 回原始应答文本 */
        bool atQuery(const char *name, char *reply, uint16_t replyCap, uint16_t &replyLen,
                     uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            replyLen = 0u;
            char cmd[24];
            if (!buildQuery(name, cmd, sizeof(cmd))) {
                fail(Fail::BadParam);
                return false;
            }
            return atCommand(cmd, reply, replyCap, replyLen, timeoutMs);
        }

        /** 查软件版本(§7.1.4): 发 AT+FWCODE=?, 期望应答 "FWCODE=7463-0-17"。
         *  为什么校验前缀: 少了 "FWCODE=" 的应答多半是上一轮的残留或噪声, 不能当版本号用。 */
        bool queryVersion(char *out, uint16_t cap, uint16_t &len, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            len = 0u;
            if (!atQuery(E220_AT_FWCODE, out, cap, len, timeoutMs)) {
                return false;
            }
            if (!contains(out, len, "FWCODE=")) {
                fail(Fail::BadReply);
                return false;
            }
            cacheVersion(out, len);
            return true;
        }

        /** 查产品型号(§7.1.3): 发 AT+DEVTYPE=?, 期望应答 "DEVTYPE=E220-400T22S" 一类 */
        bool queryDeviceType(char *out, uint16_t cap, uint16_t &len, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            len = 0u;
            if (!atQuery(E220_AT_DEVTYPE, out, cap, len, timeoutMs)) {
                return false;
            }
            if (!contains(out, len, "DEVTYPE=")) {
                fail(Fail::BadReply);
                return false;
            }
            return true;
        }

        /** 设置串口参数(§7.1.11): AT+UART=<baud 0..7>,<parity 0..2>。
         *  ⚠ 这条只改**模块**; MCU 侧要自己 init() 到同一个波特率。手册: "切换工作模式后生效"。 */
        bool setUartCode(uint8_t baudCode, uint8_t parityCode, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            if ((baudCode & 0xF8u) != 0u || (parityCode & 0xFCu) != 0u) {
                fail(Fail::BadParam);
                return false;
            }
            char cmd[24];
            if (!buildSet2(E220_AT_UART, baudCode, parityCode, cmd, sizeof(cmd))) {
                fail(Fail::BadParam);
                return false;
            }
            return atSet(cmd, timeoutMs);
        }

        /** 回读串口参数(§7.1.11): 发 AT+UART=?, 期望应答 "AT+UART=3,0" ⇒ 解析出 (波特率档, 校验档)。
         *  这是"设置到底生效没有"的**唯一**证据(见 atSet 的警告)。 */
        bool queryUartCode(uint8_t &baudCode, uint8_t &parityCode, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            char    txt[E220_AT_TEXT_MAX];
            uint16_t len = 0u;
            if (!atQuery(E220_AT_UART, txt, sizeof(txt), len, timeoutMs)) {
                return false;
            }
            if (!parseUartReply(txt, len, baudCode, parityCode)) {
                fail(Fail::BadReply);
                return false;
            }
            return true;
        }

        /** 设置空中速率(§7.1.14): AT+RATE=<0..7>。⚠ 收发**双方必须相同**(§6.3 02H 表) */
        bool setAirRate(uint8_t rateCode, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return atSet1(E220_AT_RATE, rateCode, 0x07u, timeoutMs);
        }

        /** 设置信道(§7.1.9): AT+CHANNEL=<0..83>(400 频段)。f = 410.125 + CH×1MHz */
        bool setChannel(uint8_t channel, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return atSet1(E220_AT_CHANNEL, channel, 0xFFu, timeoutMs);
        }

        /** 设置通信地址(§7.1.8): AT+ADDR=<0..65535>。65535 = 广播/监听(不过滤) */
        bool setAddr(uint16_t addr, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            char cmd[24];
            if (!buildSetU16(E220_AT_ADDR, addr, cmd, sizeof(cmd))) {
                fail(Fail::BadParam);
                return false;
            }
            return atSet(cmd, timeoutMs);
        }

        /** 设置密钥(§7.1.10): AT+KEY=<0..65535>。
         *  ⚠ 手册原文: "不能查询(无法读取密码)" ⇒ **只能设置, 无法回读校验**。
         *    密钥不一致的表现是"解不出正确数据包，通不过校验并丢弃" —— 和"距离不够"一模一样。
         *    所以本工程两端都用 0(link.e220.kKey), 并依赖"链路能通"来间接证明。 */
        bool setKey(uint16_t key, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            char cmd[24];
            if (!buildSetU16(E220_AT_KEY, key, cmd, sizeof(cmd))) {
                fail(Fail::BadParam);
                return false;
            }
            return atSet(cmd, timeoutMs);
        }

        /** 设置分包长度(§7.1.12): AT+PACKET=<0..3>。本工程用 3(=32B)保证"一帧 = 一个射频包" */
        bool setPacketCode(uint8_t packetCode, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return atSet1(E220_AT_PACKET, packetCode, 0x03u, timeoutMs);
        }

        /** 设置断帧条件(§7.1.13): AT+URXT=<1..255> 字节时间。
         *  ⚠ 主机暂停超过它, 模块就把一帧拆成两个射频包 ⇒ 每帧必须一次连续写完(链路层约束) */
        bool setURxt(uint8_t byteTimes, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            if (byteTimes == 0u) {
                fail(Fail::BadParam);
                return false;
            }
            return atSet1(E220_AT_URXT, byteTimes, 0xFFu, timeoutMs);
        }

        /** 传输方式(§7.1.15): true=透明(默认, 本工程必须), false=定点 */
        bool setTransparentMode(bool on, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return atSet1(E220_AT_TRANS, on ? 0u : 1u, 0x01u, timeoutMs);
        }

        /** 先听后发(§7.1.16): 手册明说最长可等 2 秒才强发 ⇒ 控制链路绝不能开(本工程 false) */
        bool setLbt(bool on, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return atSet1(E220_AT_LBT, on ? 1u : 0u, 0x01u, timeoutMs);
        }

        /** 每包追加 RSSI 字节(§7.1.20): 开了之后**每个收到的包长度都变**, 定长帧解析直接崩(本工程 false) */
        bool setDrssi(bool on, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return atSet1(E220_AT_DRSSI, on ? 1u : 0u, 0x01u, timeoutMs);
        }

        /** AUX 是否指示无线发送过程(§7.1.7): 固件 ≥7463-0-15 默认 0(AUX 只跟踪串口缓存)。
         *  本工程必须设 1(link.e220.kUaux), 否则"等 AUX 高"不代表空口发完了。 */
        bool setUAux(bool on, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return atSet1(E220_AT_UAUX, on ? 1u : 0u, 0x01u, timeoutMs);
        }

        //-------------------------------------------------------------------------------------------
        // 故意**不提供**的两条指令(需要时按下面的说明自己发; 不给"看起来能用"的封装是有理由的)
        //
        // ★ AT+RESET(§7.1.1) / AT+DEFAULT(§7.1.2):
        //   手册原文: 固件 ≤7463-0-15 **不应答**(连 "=OK" 都没有), ≥7463-0-16 才回 "=OK"。
        //   ⇒ 同一个函数的"成功判据"在两种固件上不一样, 封成 bool 只能得到一个"在某半个世界上恒假"
        //     的函数(或者更糟: 把超时也当成功, 于是模块不在时也说复位成功)。
        //   ⇒ 真要用请这么写: atSet("AT+RESET")(收到 "=OK" 或超时都不奇怪 —— 它只表示命令发出去了)
        //     → delay ≥30ms(手册: "应答后(=OK)才开始执行真正的复位过程, 用户请等待约 30ms 再执行下一步")
        //     → waitAux() 等 AUX 高(复位期间 AUX 走一遍忙/闲, §5.2.4 注 3), **这才是复位成功的证据**。
        //   为什么这件事在本工程重要: E220 的 RESET 脚**没引到 MCU**(config 的 hw.link 里没有 kReset),
        //   所以软复位是上电配置流程唯一的软恢复手段 —— 失败重试策略(cfg::hw::link::kCfgRetry)会用到它。
        //
        // ★ AT+IAP(§5.3「串口升级固件」):
        //   进固件升级模式后波特率自动变 115200, 且**必须上电保持 60 秒**才退得出来, 否则重启也会
        //   无限进升级模式 ⇒ 等于把模块变砖。这里连包装函数都不给, 免得哪天有人顺手调用。
        //-------------------------------------------------------------------------------------------

        /** 最近一条应答的文本(内部缓冲, 以 '\0' 结尾; 失败或没跑过指令时是空串) */
        const char *lastReply() const { return _reply; }
        /** 最近一次成功 readVersion 的应答(形如 "FWCODE=7463-0-17"); 没成功过则空串 */
        const char *cachedVersion() const { return _version; }
        /** cachedVersion() 的有效长度 */
        uint16_t cachedVersionLength() const { return _versionLen; }

        //===========================================================================================
        // HEX 寄存器通道(手册 §6.1)
        //
        //   为什么寄存器走 HEX 而不是 AT: **手册这一代没有 AT+REGn 这条指令**(研究笔记 §12)。
        //   寄存器只有 §6.1 的三条 HEX 指令(C0/C1/C2)。任务书里提到的 "AT+REG0" 属于别的固件世代,
        //   照那个写会得到 "=ERR"(或者更糟: 被当成未知指令什么也不做)。
        //===========================================================================================
        /**
         * @brief 读寄存器: 发 "C1 起始地址 长度", 收 "C1 起始地址 长度 参数…" 并逐项校验
         * @return true 仅当: 首字节 C1 ∧ 地址/长度与请求**一致** ∧ 数据长度足够。
         *         收到 FF FF FF(§6.1 序号 4 的格式错误响应)判 BadReply。
         */
        bool readRegs(uint8_t startAddr, uint8_t *out, uint8_t len, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            if (out == 0 || len == 0u || len > E220_REG_XFER_MAX) {
                fail(Fail::BadParam);
                return false;
            }
            const uint8_t req[E220_HEX_HEAD_BYTES] = {E220_HEX_READ, startAddr, len};
            uint8_t       rsp[E220_HEX_REPLY_MAX];
            if (!hexTransact(req, E220_HEX_HEAD_BYTES, rsp, len, timeoutMs)) {
                return false;
            }
            if (!parseHexReply(rsp, (uint16_t)(E220_HEX_HEAD_BYTES + len), startAddr, out, len)) {
                fail(Fail::BadReply);
                return false;
            }
            return true;
        }

        /**
         * @brief 写寄存器: 发 "C0/C2 起始地址 长度 参数…", 并把**应答里回显的参数**与写入值逐字节比对
         *
         * 手册 §6.1 的响应格式就是 "C1 + 起始地址 + 长度 + 参数"(与写入内容相同), 所以"写"这一步本身
         * 就能抓出"没写进去/写歪了"。比对失败判 RegMismatch。
         * @param save true = C0(写入并保存, 掉电保留); false = C2(临时, 掉电丢失)
         * @warning ⚠ **不要把 CRYPT_H/CRYPT_L(06h/07h)放进任何块里**: 手册 §6.3 明说密钥"只写,
         *          读取返回 0" ⇒ 带密钥的块**永远**回读不一致, 会把好模块判成坏的。密钥只能靠 AT+KEY。
         */
        bool writeRegs(uint8_t startAddr, const uint8_t *data, uint8_t len, bool save,
                       uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            if (data == 0 || len == 0u || len > E220_REG_XFER_MAX) {
                fail(Fail::BadParam);
                return false;
            }
            uint8_t req[E220_HEX_HEAD_BYTES + E220_REG_XFER_MAX];
            req[0] = save ? E220_HEX_SET_SAVE : E220_HEX_SET_TEMP;
            req[1] = startAddr;
            req[2] = len;
            memcpy(&req[E220_HEX_HEAD_BYTES], data, len);

            // 两个缓冲必须分开: rsp 是"整条应答"(3+len), echo 只放解出来的参数(len)。
            // 合成一个会变成 memcpy 源/目的重叠(UB) —— 4 字节时"看着能跑", 换长度就炸。
            uint8_t rsp[E220_HEX_REPLY_MAX];
            uint8_t echo[E220_REG_XFER_MAX];
            if (!hexTransact(req, (uint8_t)(E220_HEX_HEAD_BYTES + len), rsp, len, timeoutMs)) {
                return false;
            }
            if (!parseHexReply(rsp, (uint16_t)(E220_HEX_HEAD_BYTES + len), startAddr, echo, len)) {
                fail(Fail::BadReply);
                return false;
            }
            if (memcmp(echo, data, len) != 0) {
                fail(Fail::RegMismatch);
                return false;
            }
            return true;
        }

        /**
         * @brief 写 + 回读 + 比对。**这是本驱动推荐的写寄存器方式**(也是 NRF 那个坑的正面解法)
         *
         * 为什么"写"的回显比对还不够, 还要再读一次: 回显只证明模块**收对了字节**, 不证明它**存进去了**;
         * 参数被钳位(§7 章首第 5 条: 超范围会被"限制")时回显仍然是你写的值, 只有再读一次才看得见。
         */
        bool writeRegsVerified(uint8_t startAddr, const uint8_t *data, uint8_t len, bool save,
                               uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            if (data == 0 || len == 0u || len > E220_REG_XFER_MAX) {
                fail(Fail::BadParam);
                return false;
            }
            if (!writeRegs(startAddr, data, len, save, timeoutMs)) {
                return false;
            }
            uint8_t back[E220_REG_XFER_MAX];
            if (!readRegs(startAddr, back, len, timeoutMs)) {
                return false;
            }
            if (memcmp(back, data, len) != 0) {
                fail(Fail::RegMismatch);
                return false;
            }
            return true;
        }

        /** 读一个寄存器 */
        bool readReg(uint8_t addr, uint8_t &value, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return readRegs(addr, &value, 1u, timeoutMs);
        }

        /** 写一个寄存器(带回显比对) */
        bool writeReg(uint8_t addr, uint8_t value, bool save, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return writeRegs(addr, &value, 1u, save, timeoutMs);
        }

        /** 写一个寄存器 + 回读校验 */
        bool writeRegVerified(uint8_t addr, uint8_t value, bool save, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return writeRegsVerified(addr, &value, 1u, save, timeoutMs);
        }

        /** 一块写 REG0..REG3(02h~05h, 连续四字节)+ 回读校验。
         *  为什么是"整块": 见 E220Regs 的注释(避免半新半旧的参数); 手册 §6.1 例 2 就是这个用法。 */
        bool applyRegs(const E220Regs &r, uint32_t timeoutMs = E220_AT_TIMEOUT_MS)
        {
            return writeRegsVerified(E220_REG0_ADDR, &r.reg0, E220_REG_BLOCK, true, timeoutMs);
        }

        //===========================================================================================
        // 纯函数(不碰硬件; 单测直接打, 应用层也可以拿来拼参数)
        //===========================================================================================
        /** REG0 = 波特率[7:5] | 校验[4:3] | 空中速率[2:0](手册 §6.3 02H 表)。
         *  越界位用掩码切掉而不是移位污染邻域: 传进来 0xFF 时宁可"高位被丢"也不要串到别的位域上。 */
        static uint8_t buildReg0(uint8_t baudCode, uint8_t parityCode, uint8_t airRateCode)
        {
            return (uint8_t)(((baudCode & 0x07u) << E220_REG0_BAUD_SHIFT) |
                             ((parityCode & 0x03u) << E220_REG0_PARITY_SHIFT) |
                             ((airRateCode & 0x07u) << E220_REG0_AIR_SHIFT));
        }

        /** REG1 = 分包长度[7:6] | 环境RSSI[5] | 软切[2]=0 | 发射功率[1:0](手册 §6.3 03H 表)。
         *  保留位 bit4/bit3 **恒写 0**: 手册说上位机会主动清它们, 写进去只会得到不一致的行为。 */
        static uint8_t buildReg1(uint8_t packetCode, uint8_t txPowerCode, bool ambientRssi)
        {
            return (uint8_t)(((packetCode & 0x03u) << E220_REG1_PACKET_SHIFT) | (ambientRssi ? E220_REG1_RSSI_EN : 0u) |
                             (txPowerCode & E220_REG1_POWER_MASK));
        }

        /** REG3 = 包RSSI[7] | 传输方式[6] | LBT[4] | WOR[2:0](手册 §6.3 05H 表); 保留位恒 0。
         *  fixedPoint=true 会吃掉每帧前 3 字节(地址高/低 + 信道)⇒ 定长帧解析必崩, 本工程恒 false。 */
        static uint8_t buildReg3(bool drssiByte, bool fixedPoint, bool lbt, uint8_t worCycle)
        {
            return (uint8_t)((drssiByte ? E220_REG3_DRSSI_BYTE : 0u) | (fixedPoint ? E220_REG3_TRANS_MODE : 0u) |
                             (lbt ? E220_REG3_LBT_EN : 0u) | ((worCycle & E220_REG3_WOR_MASK) << E220_REG3_WOR_SHIFT));
        }

        /** 应答是不是"格式错误"(§6.1 序号 4: FF FF FF) */
        static bool replyIsHexError(const uint8_t *reply, uint16_t len)
        {
            return reply != 0 && len >= E220_HEX_HEAD_BYTES && reply[0] == E220_HEX_ERROR &&
                   reply[1] == E220_HEX_ERROR && reply[2] == E220_HEX_ERROR;
        }

        /**
         * @brief 解析 HEX 应答 "C1 起始地址 长度 参数…", 校验地址/长度与请求一致后拷贝数据
         *
         * 为什么必须校验地址与长度(而不是"从第 4 个字节开始拷贝"): 模块应答里的地址/长度就是我们请求的,
         *   它们对不上说明这一帧不是这条指令的应答(残留/串帧/错误响应) —— 照抄数据会把上一轮的参数
         *   当成这一轮的, 而"读回来的值看着挺正常"正是最难查的那类错。
         */
        static bool parseHexReply(const uint8_t *reply, uint16_t len, uint8_t startAddr, uint8_t *out, uint8_t outLen)
        {
            if (reply == 0 || out == 0 || outLen == 0u) {
                return false;
            }
            if (len < (uint16_t)(E220_HEX_HEAD_BYTES + outLen)) {
                return false;
            }
            if (reply[0] != E220_HEX_READ || reply[1] != startAddr || reply[2] != outLen) {
                return false;
            }
            memcpy(out, &reply[E220_HEX_HEAD_BYTES], outLen);
            return true;
        }

        /** 从 "…<key>=<数值>…" 里解析一个数; 数值接受十进制或 0x/0X 前缀的十六进制。
         *  为什么两种都收: AT 应答里的值都是十进制, 而上位机/日志里常见 "0x02" 写法, 同一个解析器
         *  省得调用方各写一份(也就不会有一份把 0x0F 当成 0 的错)。 */
        static bool parseKeyedNumber(const char *text, uint16_t len, const char *key, uint32_t &value)
        {
            value = 0u;
            const int32_t at = indexOf(text, len, key);
            if (at < 0) {
                return false;
            }
            uint16_t pos = (uint16_t)((uint16_t)at + (uint16_t)strlen(key));
            while (pos < len && (text[pos] == ' ' || text[pos] == '\t')) {
                ++pos;
            }
            if (pos >= len || text[pos] != '=') {
                return false;
            }
            return parseNumber(text, len, (uint16_t)(pos + 1u), value);
        }

        /**
         * @brief 解析"寄存器值的文本形式": "+REG0=2" / "REG0=0x0F" / "0x0F" / "15"
         *
         * ⚠ **手册里没有 AT+REGn 这条指令**(研究笔记 §12)。这个函数只是一个**兼容**解析器:
         *   有些上位机/固件会在日志或非官方指令里给这种文本。主路径仍然是 §6.1 的 HEX 应答
         *   (parseHexReply)。写在这里是为了"任务书要求的 +REG0=2 形态能解析", 不是因为有这条指令。
         * @return true = 解析出一个 ≤255 的数; false = 文本里没有可用的数或超出 u8
         */
        static bool parseRegText(const char *text, uint16_t len, uint8_t &value)
        {
            value = 0u;
            if (text == 0) {
                return false;
            }
            uint16_t pos = 0u;
            while (pos < len && (text[pos] == ' ' || text[pos] == '\t' || text[pos] == '\r' || text[pos] == '\n' ||
                                 text[pos] == '+')) {
                ++pos;
            }
            // "REG<数字>=" 前缀: 有就跳过去, 没有就当整串是一个数(纯数字/0x 形式)
            if (pos + 4u <= len && upperOf(text[pos]) == 'R' && upperOf(text[pos + 1u]) == 'E' &&
                upperOf(text[pos + 2u]) == 'G' && text[pos + 3u] >= '0' && text[pos + 3u] <= '3') {
                pos = (uint16_t)(pos + 4u);
                while (pos < len && text[pos] != '=') {
                    ++pos;
                }
                if (pos >= len) {
                    return false;
                }
                ++pos; // 跳过 '='
            }
            uint32_t raw = 0u;
            if (!parseNumber(text, len, pos, raw) || raw > 0xFFu) {
                return false;
            }
            value = (uint8_t)raw;
            return true;
        }

        /** 应答判成功: 不含 "=ERR" ∧ (含 "OK" ∨ 含 "<指令名>=")。
         *
         *  三条依据:
         *   ① 手册 §7 章首第 4 条: 识别/执行异常应答 "=ERR" —— 它是**最高优先级**的否定判据;
         *   ② 手册 §7.1.1/7.1.5/7.1.7… 的设置应答是 "=OK"; 而裸 "OK"(不带 '=')在串口助手里手工敲
         *      AT 时也是常见形态(有些固件版本/上位机日志就这么显示) —— 多认一个**肯定**标记,
         *      不会把 "=ERR" 判成成功(上面第 ① 条先拦);
         *   ③ 手册 §7.1.6(AT+MODE)与 §7.1.21(AT+POWER)的设置示例应答写的是 "AT+XXX=<v>",
         *      与 ② 的写法自相矛盾(研究笔记 §13 记录在案)。只认一种的话, 换个固件版本就会把成功
         *      判成失败 —— 所以两种都认。
         *  ⚠ 本函数判的是"模块听懂了", **不是"参数真的变了"**: 那必须回读(见 atSet 的警告)。 */
        static bool replyIsSetOk(const char *reply, uint16_t len, const char *cmdName)
        {
            if (reply == 0 || contains(reply, len, E220_AT_ERR)) {
                return false;
            }
            if (contains(reply, len, E220_AT_OK)) { // "=OK"
                return true;
            }
            if (contains(reply, len, "OK")) { // 裸 "OK"(见上面 ②)
                return true;
            }
            if (cmdName != 0 && cmdName[0] != '\0' && contains(reply, len, cmdName) &&
                contains(reply, len, "=")) {
                return true;
            }
            return false;
        }

        //===========================================================================================
        // 透传字节通道
        //===========================================================================================
        /**
         * @brief 把字节原样写进模块串口(**同步阻塞到 TXE**, 无 AUX 等待、无流控)
         *
         * 为什么返回 void: 这一层没有可判定的失败条件 —— STM32 的 TXE 在发送寄存器空时一直置位,
         *   写进去不会失败。编一个"恒 true 的 bool"就是 NRF24L01 那个"init() 恒 true"的坑的翻版,
         *   所以这里的名字就叫 sendRaw(纯字节泵), 由链路层用 AUX/空口预算/对端回执去判"发出去没有"。
         * ⚠ 一帧必须**一次连续写完**: 手册 §7.1.13 的断帧条件(默认 3 字节时间 ≈ 3ms @9600)一旦被中间
         *   停顿超过, 模块就把这一帧拆成两个射频包 —— 接收端的定长帧解析会把它当成噪声。
         *   所以本函数把整段交给 USART::send 一口气写完, 中间不做任何等待/AUX 查询。
         */
        void sendRaw(const uint8_t *data, uint16_t len)
        {
            if (data == 0 || len == 0u) {
                return;
            }
            _port.send(data, len);
        }

        /**
         * @brief 等 AUX 空闲后再发(链路层节流用)。**不发**在 AUX 忙的时候硬灌数据
         * @return false = AUX 在超时内一直忙(记 AuxTimeout), 一个字节都没发
         */
        bool sendWhenIdle(const uint8_t *data, uint16_t len,
                          uint32_t auxTimeoutMs = E220_AUX_READY_TIMEOUT_MS)
        {
            if (data == 0 || len == 0u) {
                fail(Fail::BadParam);
                return false;
            }
            if (!waitAux(auxTimeoutMs)) {
                return false;
            }
            sendRaw(data, len);
            return true;
        }

        //===========================================================================================
        // 透传发送(DMA) —— v2 链路运行期只走这条路径(见 sendDma 的注释)
        //===========================================================================================
        /**
         * @brief 用 DMA 把一帧**连续**写进模块串口(不等发送完成)
         *
         * 为什么必须 DMA(而不是 sendRaw 的 TXE 轮询): 断帧条件是"主机停顿 > URXT(3 字节时间,
         *   @57600 ≈ 521us)"。轮询发送靠线程自己跑, 一旦被更高优先级线程/中断抢占超过 0.5ms,
         *   模块就把这一帧劈成两个射频包 —— 接收端按定长帧解析会把它当噪声, 表现成"负载重时
         *   偶发丢帧"。DMA 由硬件连续供数, 与线程调度无关。
         *
         * 本函数只把帧交给 DMA; "什么时候发完"用 txBusy()/waitTxDone() 查。
         * @return false = 上一帧还没发完 / 参数非法 / 初始化失败(原因见 lastFail(); 这里不等待)
         */
        bool sendDma(const uint8_t *data, uint16_t len)
        {
            if (data == 0u || len == 0u || len > kTxDmaCapacity) {
                fail(Fail::BadParam);
                ++_txFailures;
                return false;
            }
            if (txBusy()) {
                fail(Fail::TxBusy); // 上一帧还在线上: 调用方应放弃本轮, 绝不排队硬灌
                ++_txFailures;
                return false;
            }
            if (!initTxDma()) {
                return false;
            }
            memcpy(_txRaw, data, len);
            _txView.reset();
            _txView.cur = (uint8_t *)(_txView.src + len); // 本次有效长度(视图与 DMA 传输长度一致)
            _txDma.setBuffer(_txView, len);
            _txDma.start();
            _txBusy    = true;
            _txStartMs = _nowMs();
            ++_txQueued;
            return true;
        }

        /** 上一帧是否还在发(DMA 搬字节 或 USART 移位寄存器未空)。完成/超时自动收尾。 */
        bool txBusy()
        {
            if (!_txBusy) {
                return false;
            }
            const bool dmaDone  = _txDma.isComplete();
            const bool lineIdle = (USART_GetFlagStatus(_port.peripheral(), USART_FLAG_TC) != RESET);
            if (dmaDone && lineIdle) {
                _txBusy = false;
                ++_txCompleted;
                return false;
            }
            if ((uint32_t)(_nowMs() - _txStartMs) > kTxTimeoutMs) {
                _txBusy = false; // 必须收尾: 否则调用方永远等不到"空闲"
                fail(Fail::TxTimeout);
                ++_txFailures;
                return false;
            }
            return true;
        }

        /** 有界等待上一帧发完; false = 超时(调用方按"本轮已发/已放弃"处理, 不重试) */
        bool waitTxDone(uint32_t timeoutMs = 10u)
        {
            const uint32_t t0 = _nowMs();
            while (txBusy()) {
                if ((uint32_t)(_nowMs() - t0) > timeoutMs) {
                    return false;
                }
            }
            return true;
        }

        uint32_t txQueuedCount() const { return _txQueued; }
        uint32_t txCompletedCount() const { return _txCompleted; }
        uint32_t txFailureCount() const { return _txFailures; }

        /**
         * @brief 透传模式下的"**真的起射频了吗**"自检: 发一小段字节, 用 AUX 忙窗口当证据
         *
         * 为什么必须有它: configureLink() 的第 ⑤ 步("回透传")**没有任何 AT 可以作证** ——
         *   透传模式下模块对 AT 没有应答通道, 而"配置=OK"只证明**配置模式**里一切正常。
         *   于是"模块其实还留在配置模式"这件事, 在日志上与"链路通了但对端没发"长得一模一样。
         *   本函数把它分开: UAUX=1 时 AUX 跟的是**无线发送过程** ⇒ 在透传模式下发一串字节,
         *   AUX 会低十几毫秒(实测 8B ≈ 16.5ms); 而在配置模式里这串字节会被当 AT 命令吃掉,
         *   不会有这个长度的忙窗口。busyMs 一起返回, 让判读有数据可依(不是"有/没有"两态)。
         *
         * ⚠ 它会**真的占用一小段空口**(8~14 字节), 所以只该在启动自检/台架排查里用, 不要在
         *   运行期当心跳发 —— 从端的时隙纪律不允许。
         *
         * @param busyMs 实测 AUX 低电平时长 [ms]
         * @return true 仅当"发之前空闲 + 发之后看到忙窗口 + 忙完又回到空闲"
         */
        bool txProbe(const uint8_t *frame, uint16_t len, uint32_t &busyMs,
                     uint32_t auxTimeoutMs = E220_AUX_READY_TIMEOUT_MS)
        {
            busyMs = 0u;
            if (frame == 0 || len == 0u) {
                fail(Fail::BadParam);
                return false;
            }
            if (!waitAux(auxTimeoutMs)) {
                return false; // 发之前就不空闲: 拿不到干净的证据(也记过 AuxTimeout 了)
            }
            sendRaw(frame, len); // 一次写完: 中间插空隙会被模块切成两个空口包
            const uint32_t t0 = _nowMs();
            if (!waitAuxLow(E220_AUX_BUSY_APPEAR_MS)) {
                ++_auxBusyMisses; // 一个字节都没让它忙起来 ⇒ 不在透传模式, 或 AUX 线不通
                return false;
            }
            if (!waitAux(auxTimeoutMs)) {
                return false;
            }
            busyMs = (uint32_t)(_nowMs() - t0);
            return true;
        }

        /**
         * @brief 收一段字节: 先等第一个字节(超时返回 0), 再按"字节间空隙"把这一批收完
         *
         * 为什么按空隙断批而不是按长度: 透传模式下模块**不给你帧边界**(手册 §8: "所发即所得"),
         *   它能给的只有"这一段连续到达的字节"。真帧边界由 dlx_link_frame.hpp 的 tryDecode() 定。
         * @param firstByteTimeoutMs 等第一个字节的超时; 超时返回 0, 且**不算失败**(链路空闲是常态,
         *        每 10ms 记一次错会把诊断日志淹掉), lastFail 保持原样
         * @param gapTimeoutMs 字节间空隙; 超过它认为这一批结束
         * @return 收到的字节数(0 = 这一轮没有数据); 缓冲满也返回(cap)
         */
        uint16_t receive(uint8_t *out, uint16_t cap, uint32_t firstByteTimeoutMs = E220_RX_FIRST_BYTE_TIMEOUT_MS,
                         uint32_t gapTimeoutMs = E220_RX_GAP_TIMEOUT_MS)
        {
            if (out == 0 || cap == 0u) {
                fail(Fail::BadParam);
                return 0u;
            }
            uint8_t b = 0u;
            if (!readByte(b, firstByteTimeoutMs)) {
                return 0u; // 没有数据: 正常
            }
            out[0]        = b;
            uint16_t n    = 1u;
            while (n < cap) {
                if (!readByte(b, gapTimeoutMs)) {
                    break; // 空隙到了: 这一批结束
                }
                out[n++] = b;
            }
            return n;
        }

        /** 丢掉串口里已经压着的字节(RXNE 还置位就一直读), 最多 maxBytes 个。
         *  为什么需要: 从透传切到配置模式时, 之前飞进来的数据可能还在 RXNE/DR 里, 不丢掉就会把
         *  紧随其后的 AT 应答读串。返回丢掉的字节数(给日志看"透传期有没有漏收")。 */
        uint16_t discardRx(uint16_t maxBytes = 64u)
        {
            uint16_t n = 0u;
            while (n < maxBytes) {
                if (USART_GetFlagStatus(_port.peripheral(), USART_FLAG_RXNE) == RESET) {
                    break;
                }
                (void)USART_ReceiveData(_port.peripheral());
                ++n;
            }
            return n;
        }

        //===========================================================================================
        // 诊断(给自检/日志; 都不改变行为)
        //===========================================================================================
        Fail     lastFail() const { return _lastFail; }        ///< 最近一次失败原因
        uint32_t pollIterations() const { return _pollIters; } ///< 最近一次等待实际轮询了几次(单测断言有界性)
        uint32_t auxTimeouts() const { return _auxTimeouts; }  ///< 等 AUX 超时累计
        uint32_t rxTimeouts() const { return _rxTimeouts; }    ///< 等字节超时累计
        uint32_t badReplies() const { return _badReplies; }    ///< 应答不符/格式错误累计
        uint32_t regMismatches() const { return _regMismatches; } ///< 寄存器写入回读不一致累计
        uint32_t auxBusyMisses() const { return _auxBusyMisses; } ///< 模式切换没看到 AUX 忙脉冲的次数
        /** 最近一次写完 M0/M1 之后**回读焊盘**得到的电平。与目标不符 ⇒ 这根线被外部拽住:
         *  模块看不到我们的切换(它会停在别的模式上, 而 AT 那条判据完全正常)。 */
        bool m0PadHigh() const { return _m0PadHigh; }
        bool m1PadHigh() const { return _m1PadHigh; }
        uint32_t padMismatches() const { return _padMismatches; } ///< 焊盘回读与目标不符的累计次数
        /** AUX 的**原始**电平(true = 高 = 空闲)。上板排查"AUX 线到底怎么样"时唯一可信的量。 */
        bool auxLevel() { return _aux.read(); }
        /** begin() 里"等之前"记下的原始电平: 用来区分"上电就低"(⇒ 可能已经进了 IAP / 被下拉)
         *  与"本来高、切模式后才低"(⇒ 模块真的在忙)。 */
        bool auxLevelAtBoot() const { return _auxLevelAtBoot; }
        /** 最近一次 waitAux() 实测等了多久 [ms](0 = 快路径直接就是高)。
         *  它把"模块慢"与"AUX 线有问题"分开: 前者是一个有限的实数(比如 40ms、120ms),
         *  后者会正好等于调用方给的超时值。 */
        uint32_t auxWaitMsLast() const { return _auxWaitMs; }
        uint16_t lastReplyLength() const { return _replyLen; }    ///< lastReply() 的有效长度
        /** 最近一次 HEX 事务模块原样回过来的字节(失败时用来定位: FF FF FF = 格式错 / 空 = 没回话) */
        const uint8_t *lastHexReply() const { return _hexReply; }
        uint8_t        lastHexReplyLength() const { return _hexReplyLen; }

    private:
        //-------------------------------------------------------------------------------------------
        // 等待预算: 时间判据 + 迭代兜底(见文件头五)。**所有**等待都必须走这两个函数。
        //-------------------------------------------------------------------------------------------
        struct Wait
        {
            uint32_t startMs;
            uint32_t timeoutMs;
            uint32_t iterLimit;
            uint32_t iters;
        };

        void waitBegin(Wait &w, uint32_t timeoutMs) const
        {
            if (timeoutMs > E220_TIMEOUT_MAX_MS) {
                timeoutMs = E220_TIMEOUT_MAX_MS; // 钳位: 见 config 头里对迭代上界溢出的推导
            }
            w.timeoutMs = timeoutMs;
            w.iters     = 0u;
            w.iterLimit = timeoutMs * E220_POLL_ITERS_PER_MS + E220_POLL_ITER_SLACK;
            w.startMs   = _nowMs();
        }

        /** @return true = 该放弃了(超时 或 迭代到顶)。每次调用算一次迭代。 */
        bool waitExpired(Wait &w) const
        {
            ++w.iters;
            if (w.iters > w.iterLimit) {
                return true; // 时基不走时的结构性兜底: 循环一定结束
            }
            return (uint32_t)(_nowMs() - w.startMs) >= w.timeoutMs;
        }

        //-------------------------------------------------------------------------------------------
        // 底层收发
        //-------------------------------------------------------------------------------------------
        /** 读一个字节(超时返回 false)。**不记 lastFail**: 它是"这一轮有没有字节"的纯探测,
         *  上层(读应答/读一批数据)才知道"没字节"算不算失败。 */
        bool readByte(uint8_t &b, uint32_t timeoutMs)
        {
            Wait w;
            waitBegin(w, timeoutMs);
            for (;;) {
                if (USART_GetFlagStatus(_port.peripheral(), USART_FLAG_RXNE) != RESET) {
                    b          = (uint8_t)(USART_ReceiveData(_port.peripheral()) & 0x00FFu);
                    _pollIters = w.iters;
                    return true;
                }
                if (waitExpired(w)) {
                    _pollIters = w.iters;
                    return false;
                }
            }
        }

        /** 读满 n 个字节(整体一个超时, 不是每字节一个)。失败记 RxTimeout。 */
        bool readExact(uint8_t *out, uint16_t n, uint32_t timeoutMs)
        {
            Wait     w   = Wait();
            uint16_t got = 0u;
            waitBegin(w, timeoutMs);
            for (;;) {
                if (USART_GetFlagStatus(_port.peripheral(), USART_FLAG_RXNE) != RESET) {
                    out[got] = (uint8_t)(USART_ReceiveData(_port.peripheral()) & 0x00FFu);
                    ++got;
                    if (got >= n) {
                        _pollIters = w.iters;
                        return true;
                    }
                    continue; // 有数据就接着读, 不给它喘息时间(块读只有几个字节)
                }
                if (waitExpired(w)) {
                    _pollIters = w.iters;
                    fail(Fail::RxTimeout);
                    return false;
                }
            }
        }

        //-------------------------------------------------------------------------------------------
        // 应答读取
        //-------------------------------------------------------------------------------------------
        /** 发命令 + 读一条文本应答到内部 _reply。返回"是否收到了至少 1 个字节"。 */
        bool transact(const char *cmd, uint32_t timeoutMs, uint16_t &len)
        {
            len = 0u;
            if (cmd == 0 || cmd[0] == '\0') {
                fail(Fail::BadParam);
                return false;
            }
            (void)discardRx(); // 先清残留: 见 atCommand 注释
            sendRaw(reinterpret_cast<const uint8_t *>(cmd), (uint16_t)strlen(cmd));
            return readReply(timeoutMs, len);
        }

        /** 读一条应答: 等第一个字节(超时 ⇒ RxTimeout), 之后每字节用"空隙"判据收尾。
         *  结束条件是**字节间空隙**(手册没规定行结束符)或缓冲满 —— 两者都是有界的。 */
        bool readReply(uint32_t timeoutMs, uint16_t &len)
        {
            len       = 0u;
            _reply[0] = '\0';
            uint8_t b = 0u;
            if (!readByte(b, timeoutMs)) {
                fail(Fail::RxTimeout);
                _replyLen = 0u;
                return false;
            }
            _reply[len++] = (char)b;
            while (len < E220_AT_REPLY_MAX) {
                if (!readByte(b, E220_AT_REPLY_GAP_MS)) {
                    break; // 空隙到了: 这条应答结束
                }
                _reply[len++] = (char)b;
            }
            _reply[len] = '\0';
            _replyLen   = len;
            return true;
        }

        /** HEX 事务: 发请求 → 先读 3 字节头(能立刻认出 FF FF FF) → 再读 len 个参数。
         *  为什么要分两段读: 收到 FF FF FF 时只有 3 个字节, 若按 3+len 一次等, 会白等到超时,
         *  错误也会被报成 RxTimeout(而不是 BadReply) —— 排查方向就错了。 */
        bool hexTransact(const uint8_t *req, uint8_t reqLen, uint8_t *rsp, uint8_t dataLen, uint32_t timeoutMs)
        {
            _hexReplyLen = 0u;
            (void)discardRx();
            sendRaw(req, reqLen);
            if (!readExact(rsp, E220_HEX_HEAD_BYTES, timeoutMs)) {
                return false; // 一个字都没回来: 自检报告里会看到 "HEX 回读=(空)"
            }
            recordHexReply(rsp, E220_HEX_HEAD_BYTES);
            if (replyIsHexError(rsp, E220_HEX_HEAD_BYTES)) {
                fail(Fail::BadReply);
                return false;
            }
            if (!readExact(&rsp[E220_HEX_HEAD_BYTES], dataLen, timeoutMs)) {
                return false; // 头回来了、数据没回来(长度短)
            }
            recordHexReply(&rsp[E220_HEX_HEAD_BYTES], dataLen);
            return true;
        }

        /** 配置模式的主动证明: 发只读的 AT+FWCODE=? 并**要求回话**
         *  成功判据(为什么这么松又这么严):
         *    - 松: 任何应答都算("=ERR"也算) —— 手册 §7 章首说识别异常就是回 "=ERR", 能回话就说明
         *      AT 通道活着、也就是在配置模式(透传模式下字节会被发到空中, 没人回话);
         *    - 严: 应答不能与命令**逐字节相同** —— TX/RX 短接或自环时会收到自己的回声, 那不是证据。 */
        bool probeConfigMode(uint32_t timeoutMs)
        {
            char     txt[E220_AT_TEXT_MAX];
            uint16_t len = 0u;
            if (!atQuery(E220_AT_FWCODE, txt, sizeof(txt), len, timeoutMs)) {
                return false; // 一个字都没回来: RxTimeout 已在 readReply 里记下
            }
            if (len == 0u) {
                fail(Fail::BadReply);
                return false;
            }
            const char *echo = "AT+FWCODE=?";
            if (len == (uint16_t)strlen(echo) && memcmp(txt, echo, len) == 0) {
                fail(Fail::BadReply); // 自己听自己: 不是模块在回话
                return false;
            }
            cacheVersion(txt, len);
            return true;
        }

        void cacheVersion(const char *txt, uint16_t len)
        {
            uint16_t n = (len < (uint16_t)(sizeof(_version) - 1u)) ? len : (uint16_t)(sizeof(_version) - 1u);
            memcpy(_version, txt, n);
            _version[n]  = '\0';
            _versionLen  = n;
        }

        void fail(Fail f)
        {
            _lastFail = f;
            switch (f) {
            case Fail::AuxTimeout:  ++_auxTimeouts; break;
            case Fail::RxTimeout:   ++_rxTimeouts; break;
            case Fail::BadReply:    ++_badReplies; break;
            case Fail::RegMismatch: ++_regMismatches; break;
            default: break; // None/BadParam 没有计数器
            }
        }

        //-------------------------------------------------------------------------------------------
        // 小工具(不依赖 printf/stdio; 全部有界)
        //-------------------------------------------------------------------------------------------
        static char upperOf(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c; }

        /** 子串查找: 返回下标, 找不到返回 -1(不用 strstr, 因为应答不是以 '\0' 结尾的原始缓冲) */
        static int32_t indexOf(const char *hay, uint16_t hayLen, const char *needle)
        {
            if (hay == 0 || needle == 0) {
                return -1;
            }
            const uint16_t n = (uint16_t)strlen(needle);
            if (n == 0u || hayLen < n) {
                return -1;
            }
            for (uint16_t i = 0u; i + n <= hayLen; ++i) {
                if (memcmp(&hay[i], needle, n) == 0) {
                    return (int32_t)i;
                }
            }
            return -1;
        }

        static bool contains(const char *hay, uint16_t hayLen, const char *needle)
        {
            return indexOf(hay, hayLen, needle) >= 0;
        }

        /** 解析十进制或 0x/0X 前缀的十六进制数(至少一位数字才算成功); 越界返回 false */
        static bool parseNumber(const char *text, uint16_t len, uint16_t pos, uint32_t &value)
        {
            value = 0u;
            if (text == 0 || pos >= len) {
                return false;
            }
            uint32_t base = 10u;
            if (pos + 1u < len && text[pos] == '0' && (text[pos + 1u] == 'x' || text[pos + 1u] == 'X')) {
                base = 16u;
                pos  = (uint16_t)(pos + 2u);
            }
            uint32_t acc  = 0u;
            uint16_t used = 0u;
            while (pos < len) {
                const char c   = text[pos];
                uint32_t   dig = 0u;
                if (c >= '0' && c <= '9') {
                    dig = (uint32_t)(c - '0');
                } else if (base == 16u && upperOf(c) >= 'A' && upperOf(c) <= 'F') {
                    dig = (uint32_t)(upperOf(c) - 'A' + 10);
                } else {
                    break;
                }
                if (dig >= base) {
                    break;
                }
                acc = acc * base + dig;
                if (acc > 0xFFFFFFu) {
                    return false; // 明显越界: 不再累加(避免 uint32 回绕出"看着合理"的数)
                }
                ++used;
                ++pos;
            }
            if (used == 0u) {
                return false;
            }
            value = acc;
            return true;
        }

        /** 从 "AT+UART=6,0" 里取出指令名 "AT+UART"(给 replyIsSetOk 用) */
        static void atName(const char *cmd, char *out, uint16_t cap)
        {
            uint16_t n = 0u;
            if (out == 0 || cap == 0u) {
                return;
            }
            while (cmd != 0 && cmd[n] != '\0' && cmd[n] != '=' && n + 1u < cap) {
                out[n] = cmd[n];
                ++n;
            }
            out[n] = '\0';
        }

        /** "<name>=?"(手册 §7.1.3 等查询格式)。长度不足返回 false, 不截断(截断会变成另一条指令)。 */
        static bool buildQuery(const char *name, char *out, uint16_t cap)
        {
            const uint16_t n = (uint16_t)strlen(name);
            const uint16_t s = (uint16_t)strlen(E220_AT_QUERY_SUFFIX);
            if (out == 0 || (uint16_t)(n + s + 1u) > cap) {
                return false;
            }
            memcpy(out, name, n);
            memcpy(&out[n], E220_AT_QUERY_SUFFIX, s);
            out[n + s] = '\0';
            return true;
        }

        static bool buildSet1(const char *name, uint8_t v, char *out, uint16_t cap)
        {
            char num[6];
            (void)u16ToDec(v, num);
            const uint16_t n = (uint16_t)strlen(name);
            const uint16_t m = (uint16_t)strlen(num);
            if (out == 0 || (uint16_t)(n + 1u + m + 1u) > cap) {
                return false;
            }
            memcpy(out, name, n);
            out[n] = '=';
            memcpy(&out[n + 1u], num, m);
            out[n + 1u + m] = '\0';
            return true;
        }

        static bool buildSet2(const char *name, uint8_t a, uint8_t b, char *out, uint16_t cap)
        {
            char na[6];
            char nb[6];
            (void)u16ToDec(a, na);
            (void)u16ToDec(b, nb);
            const uint16_t n = (uint16_t)strlen(name);
            const uint16_t m = (uint16_t)strlen(na);
            const uint16_t k = (uint16_t)strlen(nb);
            if (out == 0 || (uint16_t)(n + 1u + m + 1u + k + 1u) > cap) {
                return false;
            }
            uint16_t pos = 0u;
            memcpy(out, name, n);
            pos        = n;
            out[pos++] = '=';
            memcpy(&out[pos], na, m);
            pos += m;
            out[pos++] = ',';
            memcpy(&out[pos], nb, k);
            pos += k;
            out[pos] = '\0';
            return true;
        }

        static bool buildSetU16(const char *name, uint16_t v, char *out, uint16_t cap)
        {
            char           num[6];
            const uint16_t m = u16ToDec(v, num);
            const uint16_t n = (uint16_t)strlen(name);
            if (out == 0 || (uint16_t)(n + 1u + m + 1u) > cap) {
                return false;
            }
            memcpy(out, name, n);
            out[n] = '=';
            memcpy(&out[n + 1u], num, m);
            out[n + 1u + m] = '\0';
            return true;
        }

        /** 无符号 → 十进制(不引入 printf); 返回位数。上限 5 位(≤65535), 够本文件所有参数用。 */
        static uint8_t u16ToDec(uint16_t v, char *out)
        {
            char    tmp[5];
            uint8_t n = 0u;
            do {
                tmp[n] = (char)('0' + (char)(v % 10u));
                v      = (uint16_t)(v / 10u);
                ++n;
            } while (v != 0u && n < 5u);
            for (uint8_t i = 0u; i < n; ++i) {
                out[i] = tmp[n - 1u - i];
            }
            out[n] = '\0';
            return n;
        }

        /** "AT+UART=6,0" → baud=6, parity=0。格式不对返回 false(不猜) */
        static bool parseUartReply(const char *txt, uint16_t len, uint8_t &baud, uint8_t &parity)
        {
            baud   = 0u;
            parity = 0u;
            uint32_t a = 0u;
            uint32_t b = 0u;
            // 键用 "UART"(不是 "UART="): parseKeyedNumber 自己会跳过键后面的 '='
            if (!parseKeyedNumber(txt, len, "UART", a) || a > 7u) {
                return false;
            }
            const int32_t at = indexOf(txt, len, "UART=");
            if (at < 0) {
                return false;
            }
            uint16_t pos = (uint16_t)((uint16_t)at + 5u); // 跳过 "UART="
            while (pos < len && txt[pos] != ',') {
                ++pos;
            }
            if (pos >= len) {
                return false; // 没有第二个参数: 格式不符合 §7.1.11
            }
            if (!parseNumber(txt, len, (uint16_t)(pos + 1u), b) || b > 2u) {
                return false;
            }
            baud   = (uint8_t)a;
            parity = (uint8_t)b;
            return true;
        }

        uint16_t copyOut(char *dst, uint16_t cap, uint16_t len)
        {
            if (dst == 0 || cap == 0u) {
                return 0u;
            }
            uint16_t n = (len < (uint16_t)(cap - 1u)) ? len : (uint16_t)(cap - 1u);
            memcpy(dst, _reply, n);
            dst[n] = '\0';
            return n;
        }

        bool atSet1(const char *name, uint8_t v, uint8_t mask, uint32_t timeoutMs)
        {
            if ((v & (uint8_t)~mask) != 0u) {
                fail(Fail::BadParam);
                return false;
            }
            char cmd[24];
            if (!buildSet1(name, v, cmd, sizeof(cmd))) {
                fail(Fail::BadParam);
                return false;
            }
            return atSet(cmd, timeoutMs);
        }

        //-------------------------------------------------------------------------------------------
        // 成员(声明顺序 = 构造初始化顺序, 避免 -Wreorder)
        //-------------------------------------------------------------------------------------------
        USART   &_port;
        GPIO     _m0;
        GPIO     _m1;
        GPIO     _aux;
        MillisFn _nowMs;

        Mode _mode;          ///< 目标/当前模式(引脚已经设成的组合)
        bool _m0Level;       ///< 我们写下去的 M0 电平(不读回引脚: 输出模式下读 IDR 会被负载影响)
        bool _m1Level;
        bool _pinsDriven;    ///< begin() 之前没驱动过引脚, 那时"引脚组合"没有意义
        bool _modeConfirmed; ///< 最近一次切换是否被证明过(忙脉冲/AT 应答)

        /** 写完 M0/M1 之后**回读焊盘**得到的电平 + 与目标不符的累计次数。
         *  为什么要单独记: 线被外部拽住(短路/重负载)时驱动端与模块端是两个世界 ——
         *  我们以为"已经切到透传", 模块却还停在配置模式, 而 AT 那条判据完全看不出来。 */
        bool     _m0PadHigh = false;
        bool     _m1PadHigh = false;
        uint32_t _padMismatches = 0u;

        Fail     _lastFail;
        uint32_t _pollIters;
        uint32_t _auxTimeouts;
        uint32_t _rxTimeouts;
        uint32_t _badReplies;
        uint32_t _regMismatches;
        uint32_t _auxBusyMisses;

        /** 最近一次 waitAux() 实测等待时长 [ms] 与 begin() 之前看到的原始 AUX 电平。
         *  这两个数只服务"上板第一眼": 把"模块慢"与"AUX 线有问题"分开(见 auxWaitMsLast())。 */
        uint32_t _auxWaitMs;
        bool     _auxLevelAtBoot;

        char     _reply[E220_AT_TEXT_MAX];   ///< 最近一条 AT 应答文本
        uint16_t _replyLen;
        char     _version[E220_AT_TEXT_MAX]; ///< 最近一次成功的 FWCODE 应答
        uint16_t _versionLen;
        /**
         * 最近一次 HEX 事务里模块**原样回过来的字节**(成功失败都记)。
         *
         * 为什么值得占这十几个字节: 首次上板时 `applyRegs` 失败只会得到一个 "配置=FAIL" ——
         * 而这三种情况的下一步动作毫无交集:
         *   回 `FF FF FF`                      ⇒ 模块说"这条命令格式不对"(固件世代/命令形式问题);
         *   回 `C1 xx xx ...` 但参数与写入不一致 ⇒ 值不被接受(被钳位), 该改的是值不是接线;
         *   一个字都没回                        ⇒ 根本没进配置模式 / 波特率不对 / 接线。
         * 没有原始字节就只能猜, 而猜错的代价是拆焊。
         */
        uint8_t _hexReply[E220_HEX_REPLY_MAX + 4u];
        uint8_t _hexReplyLen;

        /** 把这次读到的字节追加进 _hexReply(只为自检报告, 不参与任何判据) */
        void recordHexReply(const uint8_t *b, uint8_t n)
        {
            for (uint8_t k = 0u; k < n && _hexReplyLen < sizeof(_hexReply); ++k) {
                _hexReply[_hexReplyLen++] = b[k];
            }
        }

        /**
         * @brief 首次 sendDma 时建立 TX DMA(懒初始化)
         *
         * 为什么不在构造函数里做: 构造函数可能在 RCC/DMA 时钟就绪之前运行; 而 setDMASend() 会
         * 写 DMA 与 USART 的寄存器(它内部负责使能 DMA1/2 时钟)。懒初始化把"必须发生在时钟之后"
         * 这件事交给第一次真正发送的时机(那时肯定已经初始化完)。
         */
        bool initTxDma()
        {
            if (_txInited) {
                return true;
            }
            // setDMASend 负责: 使能 USART TX DMA 请求 + 建 DMA 对象 + init(长度=缓冲容量)
            _txDma    = _port.setDMASend(_txView);
            _txInited = true;
            return true;
        }

        // ---- TX DMA(声明顺序 = 构造初始化顺序; _txView 必须在 _txDma 之前) ----
        uint8_t    _txRaw[kTxDmaCapacity];
        ByteBuffer _txView;
        DMA        _txDma;
        bool       _txInited    = false;
        bool       _txBusy      = false;
        uint32_t   _txStartMs   = 0u;
        uint32_t   _txQueued    = 0u;
        uint32_t   _txCompleted = 0u;
        uint32_t   _txFailures  = 0u;
        static constexpr uint32_t kTxTimeoutMs = 50u; ///< 单帧 21B 只需 ~3.7ms; 50ms 是卡死判据
    };

    // 块写 REG0..REG3 靠的是"四个 uint8_t 成员连续、无填充"(见 applyRegs 里的 &r.reg0)。
    // 这条断言把这个前提钉住: 结构体一旦被改成含 padding 的形态, 编译先报错而不是悄悄写歪。
    static_assert(sizeof(E220Regs) == E220_REG_BLOCK, "E220Regs 必须是 4 个连续字节, 否则块写会把填充字节也发出去");

} // namespace dlx

#pragma once
#include <stdint.h>
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_iic_profile.hpp"
#include "dlx_bytebuffer.hpp"
#include "dlx_delay.hpp" // 总线恢复要手工控时序
namespace dlx
{
    /**
     * @brief 一条 IIC 总线: 管理 I2C 外设的初始化与通用时序原语(起始/停止/等待事件/字节收发)
     * @note 同步&阻塞设计, 不做中断/DMA, 不处理错误与超时(与旧 dl_iic 行为一致)
     */
    class IICBus
    {
    private:
        IICBusProfile profile;
        GPIOProfile   sclPin{GPIOProfile::B6}; ///< 保存引脚, 供总线恢复时切回普通开漏输出用
        GPIOProfile   sdaPin{GPIOProfile::B7};

        /**
         * 单次事件等待的轮询上限(约几百微秒~几毫秒, 远大于 100kHz 下一个字节的 90us)。
         * 超了就认定总线异常, 本次传输作废 —— 宁可让这一次读失败, 也不能把整机卡在开机里。
         */
        static constexpr uint32_t kEventWaitLimit = 200000u;
        bool busError = false; ///< 本次传输是否已出错(超时)
        uint32_t _lastFailedEvent = 0; ///< 最近一次超时的事件掩码(见 waitEvent 的说明)
        uint32_t _eventTimeouts   = 0; ///< 累计事件超时次数

        /**
         * 会让"继续等事件"永远等不到的错误标志: AF(无应答) / BERR(总线错) / ARLO(仲裁丢失) / OVR(溢出)。
         *
         * 2026-10-18 新增(台架实测逼出来的): 这几个标志**一旦置位就会粘住**, 而原来的 `waitEvent` 只轮询
         * 期待的事件, 从不看它们 —— 于是一次 NACK 之后每次事务都空转到上限(台架实测单次 ≈64ms),
         * 并且因为没人清标志, 后续事务**永远**失败。实测证据见 docs/气压计0x404定位_台架.md §8.2:
         *   · 对幽灵地址 0x33 写一次(必然 NACK) ⇒ SR1 从 0x0000 变 0x0400(AF);
         *   · 之后连正常地址 0x76 都读不通, 10 次尝试一次没成功(attempts_to_ok = -1, 20/20);
         *   · `recover()` 走完 SR1 仍是 0x0501(AF+BERR) ⇒ **PE 关/开与 9 个 SCL 脉冲都清不掉**;
         *   · 只有重跑 `I2C_Init` 才立刻恢复(同一次实验里 `speed` 命令验证过)。
         *
         * 为什么用 `I2C_SR1_*`(设备头里的位定义)而不是 SPL 的 `I2C_FLAG_*`: 两者数值完全相同(都是 SR1 的位),
         * 但 `I2C_SR1_*` 在真机设备头和主机侧寄存器级替身里都有, 而 `I2C_FLAG_*` 只有 SPL 提供 —— 用前者
         * 这份驱动才能在"真机"和"主机单测"两种编译环境里都成立(见 test/iic_stub/)。
         */
        static constexpr uint32_t kErrorFlags = I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR;
        uint32_t _errorLatches = 0; ///< 因 SR1 错误标志而提前放弃事务的次数(观测用, 见 errorLatches())

        /**
         * 保存 init() 的参数, 供 recover() 重跑 I2C_Init 用。
         * 为什么必须存: SPL 的 I2C_Init 要吃一个 I2C_InitTypeDef; recover() 里没有别的来源能重建它,
         * 而"重跑 I2C_Init"正是把被标志位粘住的外设拉回已知状态的**唯一**手段(见 kErrorFlags 的说明)。
         */
        I2C_InitTypeDef savedInit{};
        bool initSaved = false; ///< init() 是否调用过(没调用过就不做 I2C_Init, 保持旧行为)

        /** 清 SR1 上的错误标志。SPL 的 I2C_ClearFlag 是"写 0 清除"(I2Cx->SR1 = ~flagpos), 正是这几个位的语义。 */
        void clearErrorFlags()
        {
            I2C_ClearFlag(getI2C_TypeDef(), kErrorFlags);
        }

        inline I2C_TypeDef *getI2C_TypeDef()
        {
            switch (profile) {
                case IICBusProfile::I2C1_Profile:
                    return I2C1;
                case IICBusProfile::I2C2_Profile:
                    return I2C2;
                case IICBusProfile::I2C3_Profile:
                    return I2C3;
            }
            return I2C1;
        }

        inline uint32_t getRCC_APB1Periph()
        {
            switch (profile) {
                case IICBusProfile::I2C1_Profile:
                    return RCC_APB1Periph_I2C1;
                case IICBusProfile::I2C2_Profile:
                    return RCC_APB1Periph_I2C2;
                case IICBusProfile::I2C3_Profile:
                    return RCC_APB1Periph_I2C3;
                default:
                    return 0;
            }
        }

    public:
        IICBus(IICBusProfile profile) : profile(profile)
        {
#warning [Experimental]
        }

        // 本总线的事件中断号, 与 USART/SPI 的 profile 风格一致, 同步模式下暂不使用
        inline uint8_t getIRQn()
        {
            return static_cast<uint8_t>(profile);
        }

        // SCL/SDA 均为 I2C 复用(AF4)开漏模式, 上拉由板级外部电阻提供
        inline GPIOModeProfile getGPIOAFProfile()
        {
            return GPIOModeProfile::AF4_OD_NOPULL_50MHz;
        }

        // ------------------------------------------------------------------
        // 工厂函数: 配置 SCL/SDA 引脚(AF4 开漏), 返回总线对象
        // ------------------------------------------------------------------
        static inline IICBus make(GPIOProfile sclPin, GPIOProfile sdaPin, IICBusProfile profile)
        {
            IICBus bus(profile);
            bus.sclPin = sclPin;
            bus.sdaPin = sdaPin;
            GPIO scl(sclPin);
            scl.init(GPIOModeProfile::AF4_OD_NOPULL_50MHz);
            GPIO sda(sdaPin);
            sda.init(GPIOModeProfile::AF4_OD_NOPULL_50MHz);
            return bus;
        }

        // I2C1: SCL=B6, SDA=B7 (对应旧 dIIC1_cA6_dA7)
        static inline IICBus IIC1_SB6_DB7()
        { return make(GPIOProfile::B6, GPIOProfile::B7, IICBusProfile::I2C1_Profile); }
        // I2C1: SCL=B8, SDA=B9 (对应旧 dIIC1_cA8_dA9)
        static inline IICBus IIC1_SB8_DB9()
        { return make(GPIOProfile::B8, GPIOProfile::B9, IICBusProfile::I2C1_Profile); }
        // I2C2: SCL=B10, SDA=B11 (对应旧 dIIC2_cB10_dB11)
        static inline IICBus IIC2_SB10_DB11()
        { return make(GPIOProfile::BA, GPIOProfile::BB, IICBusProfile::I2C2_Profile); }
        /**
         * I2C2 的第二组引脚: SCL=PF1, SDA=PF0。
         *
         * 为什么需要它: VL53L1X 排在 PF0/PF1(`hw.range_.kScl = F1` / `kSda = F0`)，而气压计占着
         * I2C1 的 PB6/PB7。F4 的 I2C 外设**没有重映射寄存器**(不像 USART 有 USARTx_REMAP)，
         * 换引脚就是换一组 AF4 的管脚，所以这里只需要多一个工厂，init()/读写逻辑一行都不用改。
         *
         * 注意 SCL 在 F1、SDA 在 F0 —— 名字里的顺序就是 `SF1` 然后 `DF0`，别按数字大小顺过来。
         */
        static inline IICBus IIC2_SF1_DF0()
        { return make(GPIOProfile::F1, GPIOProfile::F0, IICBusProfile::I2C2_Profile); }
        // I2C3: SCL=A8, SDA=C9
        static inline IICBus IIC3_SA8_DC9()
        { return make(GPIOProfile::A8, GPIOProfile::C9, IICBusProfile::I2C3_Profile); }

        /**
         * @brief 初始化 I2C 外设并开启使能
         * @param mode IICBusModeProfile, 按位拼装的模式配置, 见 dlx_iic_profile.hpp
         * @param speed I2C 时钟频率, 一般 100000(标准) 或 400000(快速)
         * @param selfAddress 本机地址(7位), 主机模式下无实际意义, 给个非0值即可
         * @note SCL/SDA 引脚由工厂方法配置(见 getGPIOAFProfile)
         */
        void init(IICBusModeProfile mode, uint32_t speed, uint16_t selfAddress)
        {
            RCC_APB1PeriphClockCmd(getRCC_APB1Periph(), ENABLE);

            uint16_t modeVal = static_cast<uint16_t>(mode);

            I2C_InitTypeDef i2cInit;
            i2cInit.I2C_ClockSpeed          = speed;
            i2cInit.I2C_Mode                = I2C_Mode_I2C; // 固定 IIC, 不使用 SMBUS
            i2cInit.I2C_DutyCycle           = (modeVal & 0x4000) ? I2C_DutyCycle_16_9 : I2C_DutyCycle_2;
            i2cInit.I2C_OwnAddress1         = selfAddress;
            i2cInit.I2C_Ack                 = modeVal & 0x0400; // 0x0400=Enable, 0x0000=Disable
            i2cInit.I2C_AcknowledgedAddress = (modeVal & 0x8000) ? I2C_AcknowledgedAddress_10bit
                                                                 : I2C_AcknowledgedAddress_7bit;
            savedInit = i2cInit; // 存一份给 recover() 重跑 I2C_Init(见 savedInit 的说明)
            initSaved = true;
            I2C_Init(getI2C_TypeDef(), &i2cInit);
            I2C_Cmd(getI2C_TypeDef(), ENABLE);
        }

        /**
         * @brief 阻塞等待 I2C 事件标志
         * @return true = 事件到了; false = 超时
         *
         * @note 这里以前是**无超时死等**(`while (I2C_CheckEvent(...) != SUCCESS);`),
         *       只要总线异常(从机不响应 / 上拉不足 / 速率跑不到 / SDA 被拉低)就会永久卡死。
         *       气压计是在**开机初始化**里读的, 卡在这里的后果是整机起不来(实测踩过: 飞行模式
         *       开机卡在 BMI088/BME280 初始化之间, 心跳灯定住、NRF 不出信号)。
         *       现在改成有限次轮询: 失败后本次传输作废, 上层照 FLASH_NOK/baroOk=false 降级继续跑。
         */
        bool waitEvent(uint32_t event)
        {
            for (uint32_t i = 0; i < kEventWaitLimit; ++i) {
                /**
                 * 先看错误标志(2026-10-18 新增): AF/BERR/ARLO/OVR 置位之后, 期待的事件**永远不会再来**,
                 * 继续轮询只是白烧 CPU —— 台架实测单次空转 ≈64ms, 而 sns_baro(prio 22) 空转时会把
                 * 优先级更低的 link_tel(24)/logger(26) 全部饿死, 这正是"0x404 与 0x204 同窗"的机制。
                 * 为什么每 512 轮才查一次: 每次查要多读一遍 SR1; 512 轮的代价 ≈0.2ms, 而正常路径
                 * (事件马上就到)几乎不付。i==0 必查 ⇒ "标志早就粘住"的情形会在几微秒内退出。
                 *
                 * 这里**故意不清标志**: 失败瞬间的 SR1 是台架/诊断最需要的证据(哪个位? AF 还是 BERR?)。
                 * 清标志与把外设拉回已知状态统一由下一次事务开头的 recover() 做(见它的说明)。
                 */
                if (((i & 0x1FFu) == 0u) && ((getI2C_TypeDef()->SR1 & kErrorFlags) != 0u)) {
                    ++_errorLatches;
                    busError = true;
                    _lastFailedEvent = event;
                    ++_eventTimeouts;
                    return false;
                }
                if (I2C_CheckEvent(getI2C_TypeDef(), event) == SUCCESS) {
                    return true;
                }
            }
            busError = true; // 本次传输作废, 后续步骤不再空等
            /**
             * 记下**超时的是哪个事件** —— 这是"总线异常"这四个字里唯一有信息量的部分。
             *
             * 2026-10-03: VL53L1X 的 91 字节 ULD 写报失败, 但随后的回读显示那 91 字节
             * **其实写进去了**(6 个采样点里 4 个与期望一致, 另 2 个是器件自己的活状态寄存器)。
             * 只报"失败"没法区分下面几种完全不同的情况, 而它们的修法毫无交集:
             *   卡在 `MASTER_TRANSMITTER_MODE_SELECTED`(ADDR) ⇒ 从机没应答地址(器件没起来/地址错);
             *   卡在 `MASTER_BYTE_TRANSMITTED`(TXE|BTF)    ⇒ 传到中途被 NACK 或 SCL 被拉住;
             *   卡在最后的 STOP 之后                        ⇒ 数据其实发完了, 是收尾判据太严。
             * 记 32 位掩码而不是只记次数: `I2C_CheckEvent` 的 event 就是 SR1|SR2 的位组合,
             * 打出来能直接对上 RM0090 的事件表。
             */
            _lastFailedEvent = event;
            ++_eventTimeouts;
            return false;
        }

        /**
         * @brief 发送起始信号(首次为 START, 总线已占用时为重复 START), 并等待主机模式选定
         */
        void start()
        {
            if (busError) {
                return;
            }
            I2C_GenerateSTART(getI2C_TypeDef(), ENABLE);
            waitEvent(I2C_EVENT_MASTER_MODE_SELECT);
        }

        /**
         * @brief 发送停止信号
         */
        void stop()
        {
            I2C_GenerateSTOP(getI2C_TypeDef(), ENABLE);
        }

        /**
         * @brief 发送 7 位从机地址 + 读写位, 并等待对应模式选定
         * @param slaveAddress7 7位地址, 不含读写位
         * @param direction I2C_Direction_Transmitter / I2C_Direction_Receiver
         */
        void sendAddress(uint8_t slaveAddress7, uint8_t direction)
        {
            if (busError) {
                return;
            }
            I2C_Send7bitAddress(getI2C_TypeDef(), slaveAddress7 << 1, direction);
            if (direction == I2C_Direction_Transmitter) {
                waitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
            } else {
                waitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
            }
        }

        /**
         * @brief 阻塞发送一个字节, 等待发送完成(BTF)
         */
        void sendByte(uint8_t data)
        {
            if (busError) {
                return;
            }
            I2C_SendData(getI2C_TypeDef(), data);
            waitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        }

        /**
         * @brief 将 buffer 中已写入的数据(从 src 开始的 used() 字节)依次发出, 不修改 buffer 指针
         */
        void sendBuffer(ByteBuffer &buffer)
        {
            uint16_t length = buffer.used();
            for (uint16_t i = 0; i < length; ++i) {
                sendByte(buffer.src[i]);
            }
        }

        /**
         * @brief 接收一个字节
         * @param ack true 回 ACK(非最后一个字节), false 回 NACK(最后一个字节)
         * @return 接收到的字节
         */
        uint8_t receiveByte(bool ack)
        {
            if (busError) {
                return 0u;
            }
            I2C_TypeDef *i2cx = getI2C_TypeDef();
            I2C_AcknowledgeConfig(i2cx, ack ? ENABLE : DISABLE);
            waitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);
            return static_cast<uint8_t>(I2C_ReceiveData(i2cx));
        }

        /**
         * @brief 设置 ACK 开关(默认应答状态), 读操作结束恢复用
         */
        void setAck(bool enable)
        {
            I2C_AcknowledgeConfig(getI2C_TypeDef(), enable ? ENABLE : DISABLE);
        }

        /** 本次传输是否出过错(超时); 每次传输开始前由 IICDevice 清零 */
        bool hasError() const { return busError; }
        void clearError() { busError = false; }

        /** 最近一次 `waitEvent` 超时的事件掩码(= I2C_CheckEvent 的入参, 能直接对 RM0090 事件表) */
        uint32_t lastFailedEvent() const { return _lastFailedEvent; }
        /** 累计事件超时次数(与"传输失败次数"不是一回事: 一次传输里可能超时多次) */
        uint32_t eventTimeouts() const { return _eventTimeouts; }

        /**
         * 因 SR1 错误标志(AF/BERR/ARLO/OVR)而提前放弃事务的次数。
         * 与 eventTimeouts 分开计: 这个数 > 0 说明**总线上真的出过错标志**(NACK/总线错/溢出),
         * 而不只是"事件迟迟不来"。日志里同时给出这两个数, 就能区分两类完全不同的总线故障。
         */
        uint32_t errorLatches() const { return _errorLatches; }

        /**
         * @brief 总线卡死恢复: 松开外设 -> 手工发 9 个 SCL 脉冲 + STOP -> 重新使能外设
         *
         * 从机在上电抖动/掉电瞬间可能把 SDA 拉住不放(实测: flash 初始化那段的干扰会让
         * BME280 把总线拉死), 这时不管重试多少次都是超时 —— 必须让时钟多走几个脉冲,
         * 把从机"卡在半途"的那一个字节顶完, 它才肯放开 SDA。
         *
         * 时序用 5us 半周期(约 100kHz), 对齐标准模式, 不挑从机。
         */
        void recover()
        {
            I2C_TypeDef *i2cx = getI2C_TypeDef();
            I2C_Cmd(i2cx, DISABLE); // 让外设松开总线, 之后手工控制电平

            GPIO scl(sclPin);
            GPIO sda(sdaPin);
            scl.init(GPIOModeProfile::OUT_OD_NOPULL_50MHz);
            sda.init(GPIOModeProfile::OUT_OD_NOPULL_50MHz);

            sda = 1;
            for (uint32_t i = 0; i < 9u; ++i) { // 9 个脉冲: 足够让从机走完一个字节
                scl = 0;
                delay_us(5);
                scl = 1;
                delay_us(5);
            }
            // 手工 STOP: SCL 高电平期间让 SDA 由低变高
            sda = 0;
            delay_us(5);
            scl = 1;
            delay_us(5);
            sda = 1;
            delay_us(5);

            scl.init(getGPIOAFProfile()); // 回到 I2C 复用
            sda.init(getGPIOAFProfile());

            /**
             * 2026-10-18: 原实现到这里就 `I2C_Cmd(i2cx, ENABLE)` 收工, 注释写着"配置寄存器没被复位"。
             * 台架实测证明这不够(docs/气压计0x404定位_台架.md §8.2): 一次 NACK 让 SR1.AF 置位之后,
             * 走完这个函数 SR1 仍从 0x0500 变成 0x0501(AF+BERR 都在), 下一次事务照样失败,
             * 20/20 次注入在 10 次尝试内**一次都没恢复**; 而随后重跑一次 I2C_Init 就立刻好。
             * ⇒ 恢复必须包含"重跑 I2C_Init" + 显式清错误标志(SR1 是写 0 清除), 只切 PE 救不回来。
             * I2C_Init 按 SPL 要求应在 PE=0 时调用 —— 本函数开头就 DISABLE 过, 此刻正好还是关的。
             */
            if (initSaved) {
                I2C_Init(i2cx, &savedInit);
            }
            clearErrorFlags();
            I2C_Cmd(i2cx, ENABLE);
            busError = false;
        }
    };

    /**
     * @brief IIC 设备: 保存总线引用与从机地址的薄封装, 提供 write/read 方便外部使用
     */
    class IICDevice
    {
    private:
        IICBus &bus;
        uint16_t address; // 7位从机地址, 不含读写位

    public:
        IICDevice(IICBus &bus, uint16_t address) : bus(bus), address(address)
        {
        }

        /**
         * @brief 写操作: 发送起始+地址, 将 buffer 中已写入的数据依次发出, 最后停止
         * @note 外部需要写寄存器时自行拼接: 把 寄存器地址+数据 一起放进 buffer
         * @note 不保证不溢出, 数据长度由调用方保证
         * @return true = 本次传输全程正常; false = 中途事件超时(总线异常)
         */
        bool write(ByteBuffer &buffer)
        {
            if (bus.hasError()) {
                bus.recover(); // 上次卡死过: 先把总线救回来再发
            }
            bus.clearError(); // 本次传输重新计数
            bus.start();
            bus.sendAddress(address, I2C_Direction_Transmitter);
            bus.sendBuffer(buffer);
            bus.stop();
            return !bus.hasError();
        }

        /**
         * @brief 读操作: 先发送 writeBuffer 中的内容(如寄存器地址), 再重复起始并读取 length 字节到 readBuffer
         * @note 符合大多数 IIC 设备的读时序: 发寄存器地址 -> 重复起始 -> 连续读
         * @note readBuffer 是追加写入的(从当前 cur 开始, 用其自带 write 接口),
         *       外部使用前需自行 reset(), 否则数据会接在已有内容之后
         * @note 不保证不溢出, readBuffer 需从 cur 起预留至少 length 字节
         * @return true = 本次传输全程正常; false = 中途事件超时(读数不可信)
         */
        bool read(ByteBuffer &readBuffer, ByteBuffer &writeBuffer, uint16_t length)
        {
            if (bus.hasError()) {
                bus.recover(); // 上次卡死过: 先把总线救回来再读
            }
            bus.clearError(); // 本次传输重新计数
            bus.start();
            bus.sendAddress(address, I2C_Direction_Transmitter);
            bus.sendBuffer(writeBuffer);

            // 重复起始, 切换为接收方向
            bus.start();
            bus.sendAddress(address, I2C_Direction_Receiver);

            for (uint16_t i = 0; i < length; ++i) {
                bool isLast = (i == length - 1);
                readBuffer.write(bus.receiveByte(!isLast));
            }

            bus.stop();
            bus.setAck(true); // 恢复默认应答, 便于下次多字节读
            return !bus.hasError();
        }
    };
} // namespace dlx

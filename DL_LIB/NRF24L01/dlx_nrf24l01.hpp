#pragma once
#include <stdint.h>
#include "dlx_spi.hpp"
#include "dlx_gpio.hpp"
#include "dlx_delay.hpp"
#include "dlx_bytebuffer.hpp"
#include "dlx_exit.hpp"
#include "dlx_nvic_it.h"
#include "dlx_nrf24l01_config.h"

namespace dlx
{
    /**
     * @brief NRF24L01 2.4G 无线收发驱动(SPI + 中断/轮询, 基于 dlx 库)
     *
     * 硬件要点(数据手册):
     *  - SPI 从机, 模式 0(CPOL=0/CPHA=0), MSB 先发, 时钟最高 8MHz;
     *  - CSN 由传入 SPI 的 NSS 引脚承担, 片选低有效;
     *  - CE 高电平触发发送/进入接收, 发送脉冲需 >= 10us(Thce);
     *  - IRQ 低有效, 写 STATUS 对应位(写 1)清除并释放 IRQ;
     *  - RX_DR 处理流程(手册 8.5 注 b): 读载荷 -> 清 RX_DR -> 查 FIFO_STATUS
     *    是否还有数据 -> 有则重复, 本驱动按此顺序循环取空 RX FIFO(最多 3 包).
     *
     * 三种运行模式由 dlx_nrf24l01_config.h 中的宏三选一:
     *  - FULL_IRQ : IRQ 中断里直接做 SPI 读写并填 rxBuf, read() 只等缓冲区;
     *  - HALF_IRQ : IRQ 中断只置标志, read()/write() 在主程序里处理 SPI;
     *  - POLLING  : 不用外部中断, read()/write() 直接轮询 STATUS.
     *
     * read()/write() 都带超时, 不会永久阻塞.
     */
    class NRF24L01
    {
    private:
        SPI &spi;          ///< 已初始化的 SPI(其 NSS 作为 CSN 片选)
        GPIO ce;           ///< 使能引脚 CE
        EXTI_Line exti;    ///< 中断引脚 IRQ(仅中断模式使用)
        RingByteBuffer rxBuf; ///< 接收缓冲区, 包装外部传入的 ByteBuffer

        volatile bool irqFlag; ///< HALF 模式: ISR 只置位, 主程序消费
        volatile bool txDone;  ///< FULL 模式: ISR 置位表示发送完成/失败
        volatile bool txFail;  ///< FULL 模式: 发送失败(MAX_RT)

        /* ==================== SPI 命令 ==================== */
        void sendCmd(NRF24L01_Cmd cmd)
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(cmd));
            spi.deselect();
        }

        /** 读一包 RX 载荷(固定 NRF24L01_RX_PACKET_SIZE 字节)并写入 rxBuf */
        void readPayload()
        {
            uint8_t tmp[NRF24L01_RX_PACKET_SIZE];
            spi.select();
            spi.swap(static_cast<uint8_t>(NRF24L01_Cmd::R_RX_PAYLOAD));
            for (uint8_t i = 0; i < NRF24L01_RX_PACKET_SIZE; ++i) {
                tmp[i] = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            }
            spi.deselect();
            rxBuf.write(tmp, NRF24L01_RX_PACKET_SIZE); // 满则丢弃(默认不覆盖)
        }

        /** 写一包 TX 载荷(1~32 字节) */
        void writePayload(const uint8_t *data, uint8_t len)
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(NRF24L01_Cmd::W_TX_PAYLOAD));
            for (uint8_t i = 0; i < len; ++i) {
                spi.swap(data[i]);
            }
            spi.deselect();
        }

        /**
         * @brief 按手册流程取空 RX FIFO(读载荷 -> 清 RX_DR -> 查 FIFO_STATUS)
         * FULL 模式在 ISR 里调用, HALF/POLLING 在主程序里调用.
         */
        void drainRx()
        {
            do {
                readPayload();
                writeReg(NRF24L01_Reg::Status, NRF24L01_STATUS_RX_DR);
            } while ((readReg(NRF24L01_Reg::FifoStatus) & NRF24L01_FIFO_RX_EMPTY) == 0);
        }

        /** 回到接收模式: PRIM_RX=1 + CE=1, 等 130us(Tstby2a)稳定 */
        void enterRx()
        {
            ce = 0;
            writeReg(NRF24L01_Reg::Config, NRF24L01_CONFIG_RX);
            ce = 1;
            delay_us(150);
        }

        /** 进入发送模式: PRIM_RX=0, CE 保持低 */
        void enterTx()
        {
            ce = 0;
            writeReg(NRF24L01_Reg::Config, NRF24L01_CONFIG_TX);
        }

        /** 等待发送完成(TX_DS 成功 / MAX_RT 失败), 带超时 */
        bool waitTxDone(uint32_t timeoutMs)
        {
#if defined(NRF24L01_MODE_FULL_IRQ)
            txDone = txFail = false;
            uint32_t elapsed = 0;
            while (!txDone) {
                if (elapsed >= timeoutMs) {
                    txDone = txFail = false;
                    return false;
                }
                delay_us(1000);
                ++elapsed;
            }
            const bool ok = !txFail;
            txDone = txFail = false;
            return ok;
#elif defined(NRF24L01_MODE_HALF_IRQ)
            txDone = txFail = false;
            uint32_t elapsed = 0;
            while (!txDone) {
                if (irqFlag) {
                    irqFlag = false;
                    handleIrqAll();
                }
                if (elapsed >= timeoutMs) {
                    return false;
                }
                delay_us(1000);
                ++elapsed;
            }
            const bool ok = !txFail;
            txDone = txFail = false;
            return ok;
#else // NRF24L01_MODE_POLLING
            uint32_t elapsed = 0;
            while (true) {
                const uint8_t status = readReg(NRF24L01_Reg::Status);
                if (status & NRF24L01_STATUS_MAX_RT) {
                    writeReg(NRF24L01_Reg::Status, NRF24L01_STATUS_MAX_RT);
                    return false;
                }
                if (status & NRF24L01_STATUS_TX_DS) {
                    writeReg(NRF24L01_Reg::Status, NRF24L01_STATUS_TX_DS);
                    return true;
                }
                if (elapsed >= timeoutMs) {
                    return false;
                }
                delay_us(1000);
                ++elapsed;
            }
#endif
        }

#if defined(NRF24L01_MODE_FULL_IRQ)
        /** ISR 入口: 完整处理 STATUS(中断里做 SPI, 保持 ISR 自包含) */
        static void irqTrampoline(void *ctx)
        {
            static_cast<NRF24L01 *>(ctx)->handleIrqFull();
        }

        void handleIrqFull()
        {
            const uint8_t status = readReg(NRF24L01_Reg::Status);
            if (status & NRF24L01_STATUS_MAX_RT) {
                writeReg(NRF24L01_Reg::Status, NRF24L01_STATUS_MAX_RT);
                txFail = true;
                txDone = true;
            } else if (status & NRF24L01_STATUS_TX_DS) {
                writeReg(NRF24L01_Reg::Status, NRF24L01_STATUS_TX_DS);
                txDone = true;
            }
            if (status & NRF24L01_STATUS_RX_DR) {
                drainRx();
            }
        }
#elif defined(NRF24L01_MODE_HALF_IRQ)
        /** ISR 入口: 只置标志, SPI 留给主程序(ISR 最简) */
        static void irqTrampoline(void *ctx)
        {
            static_cast<NRF24L01 *>(ctx)->irqFlag = true;
        }

        /** 主程序上下文: 根据 STATUS 处理发送结果并取空 RX FIFO */
        void handleIrqAll()
        {
            const uint8_t status = readReg(NRF24L01_Reg::Status);
            if (status & NRF24L01_STATUS_MAX_RT) {
                writeReg(NRF24L01_Reg::Status, NRF24L01_STATUS_MAX_RT);
                txFail = true;
                txDone = true;
            } else if (status & NRF24L01_STATUS_TX_DS) {
                writeReg(NRF24L01_Reg::Status, NRF24L01_STATUS_TX_DS);
                txDone = true;
            }
            if (status & NRF24L01_STATUS_RX_DR) {
                drainRx();
            }
        }
#endif

    public:
        /**
         * @param interface  已 init 的 SPI(其 NSS 引脚即 NRF24L01 的 CSN)
         * @param ceProfile  CE 使能引脚
         * @param irqProfile IRQ 中断引脚(仅中断模式使用, 轮询模式可随便给)
         * @param buffer     接收 I/O 缓冲区(内部包成环形缓冲)
         * @note buffer 至少应能容纳 NRF24L01_RX_PACKET_SIZE 字节(一包), 否则满时新包会丢
         */
        NRF24L01(SPI &interface, GPIOProfile ceProfile, GPIOProfile irqProfile, ByteBuffer &buffer)
            : spi(interface), ce(ceProfile), exti(irqProfile), rxBuf(buffer),
              irqFlag(false), txDone(false), txFail(false)
        {
#warning [Experimental]
        }

        ~NRF24L01()
        {
        }

        /**
         * @brief 上电并应用 dlx_nrf24l01_config.h 中的全部配置
         *
         * 流程: 上电(等 1.5ms 晶振) -> 写寄存器配置 -> 清空 FIFO 与 IRQ 标志
         * -> (中断模式)挂 EXTI 中断 -> 进入接收模式.
         */
        bool init()
        {
            ce.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
            ce = 0;

            // PWR_UP=1 进入 Standby-I, 等晶振稳定(数据手册 Tpd2stby = 1.5ms)
            writeReg(NRF24L01_Reg::Config, NRF24L01_CONFIG_RX);
            delay_ms(2);

            // 配置寄存器(W_REGISTER 仅限 Power down / Standby 模式, 当前 CE=0 满足)
            writeReg(NRF24L01_Reg::EnAa, NRF24L01_EN_AA);
            writeReg(NRF24L01_Reg::EnRxAddr, NRF24L01_EN_RXADDR);
            writeReg(NRF24L01_Reg::SetupAw, NRF24L01_SETUP_AW);
            writeReg(NRF24L01_Reg::SetupRetr, NRF24L01_SETUP_RETR);
            writeReg(NRF24L01_Reg::RfCh, NRF24L01_RF_CH);
            writeReg(NRF24L01_Reg::RfSetup, NRF24L01_RF_SETUP);
            writeReg(NRF24L01_Reg::TxAddr, NRF24L01_TX_ADDR, 5);
            writeReg(NRF24L01_Reg::RxAddrP0, NRF24L01_RX_ADDR, 5);
            writeReg(NRF24L01_Reg::RxPwP0, NRF24L01_RX_PACKET_SIZE);
            sendCmd(NRF24L01_Cmd::FLUSH_TX);
            sendCmd(NRF24L01_Cmd::FLUSH_RX);

            // 清掉可能残留的 IRQ 标志, 再挂中断, 避免上电瞬间误触发
            writeReg(NRF24L01_Reg::Status,
                     NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);
#if !defined(NRF24L01_MODE_POLLING)
            exti.init(EXTIModeProfile::Falling, NVICPriorityProfile::P1_S1,
                      &NRF24L01::irqTrampoline, this);
#endif

            rxBuf.reset();
            enterRx();
            return true;
        }

        /**
         * @brief 发送一包数据(阻塞等待发送完成, 带超时), 结束后回到接收模式
         *
         * 流程: 进 TX -> 清 TX FIFO -> 写载荷 -> CE 高 >= 10us 脉冲触发
         * -> 等 TX_DS(成功)或 MAX_RT(失败) -> 回 RX.
         *
         * @return true = 发送成功(TX_DS), false = 失败(MAX_RT)或超时
         */
        bool write(const uint8_t *data, uint16_t len = NRF24L01_TX_PACKET_SIZE,
                   uint32_t timeoutMs = NRF24L01_WAIT_TIMEOUT_MS)
        {
            if (data == 0 || len == 0 || len > NRF24L01_TX_PACKET_SIZE) {
                return false;
            }
            enterTx();
            sendCmd(NRF24L01_Cmd::FLUSH_TX); // 清掉上次 MAX_RT 可能残留的载荷
            writePayload(data, static_cast<uint8_t>(len));
            ce = 1;
            delay_us(12); // Thce >= 10us
            ce = 0;

            const bool ok = waitTxDone(timeoutMs);
            enterRx();
            return ok;
        }

        /**
         * @brief 接收一包数据(阻塞等待, 带超时)
         *
         * 数据先进入 rxBuf(ISR 或轮询填入), 本函数等到 len 字节后取出.
         * 建议 len 与 NRF24L01_RX_PACKET_SIZE 一致(整包读取).
         *
         * @return true = 取到 len 字节, false = 超时
         */
        bool read(uint8_t *data, uint16_t len = NRF24L01_RX_PACKET_SIZE,
                  uint32_t timeoutMs = NRF24L01_WAIT_TIMEOUT_MS)
        {
            if (data == 0 || len == 0) {
                return false;
            }
            uint32_t elapsed = 0;
            while (rxBuf.available() < len) {
#if defined(NRF24L01_MODE_HALF_IRQ)
                if (irqFlag) {
                    irqFlag = false;
                    handleIrqAll();
                }
#elif defined(NRF24L01_MODE_POLLING)
                if (readReg(NRF24L01_Reg::Status) & NRF24L01_STATUS_RX_DR) {
                    drainRx();
                }
#endif
                // FULL_IRQ: rxBuf 由 ISR 直接填充, 这里只等待
                if (elapsed >= timeoutMs) {
                    return false;
                }
                delay_us(1000);
                ++elapsed;
            }
            return rxBuf.read(data, len);
        }

        /** @brief rxBuf 中当前可读字节数(不阻塞) */
        uint16_t available() const
        {
            return rxBuf.available();
        }

        /** @brief 清空 RX FIFO 与接收缓冲区 */
        void flushRx()
        {
            sendCmd(NRF24L01_Cmd::FLUSH_RX);
            rxBuf.reset();
        }

        /** @brief 清空 TX FIFO */
        void flushTx()
        {
            sendCmd(NRF24L01_Cmd::FLUSH_TX);
        }

        /* ==================== 寄存器访问(与旧驱动同名能力) ==================== */
        void writeReg(NRF24L01_Reg reg, uint8_t value)
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(static_cast<uint8_t>(NRF24L01_Cmd::W_REGISTER) |
                                          static_cast<uint8_t>(reg)));
            spi.swap(value);
            spi.deselect();
        }

        void writeReg(NRF24L01_Reg reg, const uint8_t *data, uint8_t len)
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(static_cast<uint8_t>(NRF24L01_Cmd::W_REGISTER) |
                                          static_cast<uint8_t>(reg)));
            for (uint8_t i = 0; i < len; ++i) {
                spi.swap(data[i]);
            }
            spi.deselect();
        }

        uint8_t readReg(NRF24L01_Reg reg)
        {
            uint8_t v;
            readRegs(reg, &v, 1);
            return v;
        }

        void readRegs(NRF24L01_Reg reg, uint8_t *out, uint8_t len)
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(static_cast<uint8_t>(NRF24L01_Cmd::R_REGISTER) |
                                          static_cast<uint8_t>(reg)));
            for (uint8_t i = 0; i < len; ++i) {
                out[i] = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            }
            spi.deselect();
        }
    };
} // namespace dlx

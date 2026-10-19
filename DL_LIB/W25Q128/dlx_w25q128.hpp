#pragma once
#include <stdint.h>
#include "dlx_spi.hpp"
#include "dlx_delay.hpp"
#include "dlx_w25q128_config.h"
namespace dlx
{
    /**
     * @brief W25Q128JV SPI NOR Flash 驱动(标准 SPI, 基于 dlx 库)
     *
     * 硬件要点(数据手册):
     *  - 只依赖标准 SPI 四根线: /CS(由传入 SPI 的 NSS 承担)、CLK、DI、DO;
     *  - 本工程 W25Q128JVSIQ 是 "-IQ" 后缀: 出厂 QE(Quad 使能)固定为 1,
     *    /HOLD 与 /WP 引脚功能默认关闭, 因此只接三根线(SCK/MOSI/MISO)+/CS
     *    即可正常工作, WP/HOLD 悬空或接高都不影响;
     *  - SPI 支持 Mode 0(0,0)与 Mode 3(1,1), 建议 Mode 0;
     *    标准读(03h)时钟最高 50MHz, 其它指令最高 133MHz, 本驱动不限速;
     *  - 写/擦除/写状态指令必须以字节边界结束(/CS 高), 且 BUSY 期间
     *    除读状态外其它指令都会被忽略, 所以本驱动每次操作后都等 BUSY 清零.
     *
     * 容量 16MB, 24bit 地址; 页 256B(编程不可跨页, 超页会回绕覆盖),
     * 扇区 4KB / 块 32KB / 块 64KB / 整片擦除.
     */
    class W25Q128
    {
    private:
        SPI &spi; ///< 已 init 的 SPI, 其 NSS 即 /CS

        /** 最近一次 init() 的识别结果(给日志用: "识别失败"要能带上实际读到的 ID) */
        uint32_t lastJedec{0u};
        uint8_t  lastAttempts{0u};

        /** 只发一条指令(如 Write Enable) */
        void cmdOnly(W25Q128_Cmd cmd)
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(cmd));
            spi.deselect();
        }

        /** 读一个状态寄存器(05h/35h/15h), /CS 拉低后第 1 拍就出数据 */
        uint8_t readStatus(W25Q128_Cmd cmd)
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(cmd));
            const uint8_t v = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            spi.deselect();
            return v;
        }

        /** 写一个状态寄存器(01h/31h/11h), 调用前必须已 Write Enable */
        void writeStatusRaw(W25Q128_Cmd cmd, uint8_t value)
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(cmd));
            spi.swap(value);
            spi.deselect();
        }

        /** 发 24bit 地址(高位在前) */
        void sendAddress(uint32_t addr)
        {
            spi.swap(static_cast<uint8_t>((addr >> 16) & 0xFF));
            spi.swap(static_cast<uint8_t>((addr >> 8) & 0xFF));
            spi.swap(static_cast<uint8_t>(addr & 0xFF));
        }

        /**
         * @brief 擦除指令公共实现: Write Enable -> 指令 + 地址 -> 等 BUSY
         */
        bool erase(W25Q128_Cmd cmd, uint32_t addr, uint32_t timeoutMs)
        {
            addr &= W25Q128_ADDR_MASK;
            writeEnable();
            spi.select();
            spi.swap(static_cast<uint8_t>(cmd));
            sendAddress(addr);
            spi.deselect();
            return waitBusy(timeoutMs);
        }

    public:
        W25Q128(SPI &interface)
            : spi(interface)
        {
#warning [Experimental]
        }

        ~W25Q128()
        {
        }

        /**
         * @brief 初始化: 退出 Power-down -> 等就绪 -> 校验 JEDEC ID -> 清除块保护
         *
         * 出厂默认 BP/TB/SEC/SRP 均为 0(无保护), 这里仍显式清一次,
         * 防止旧芯片或上次使用改过保护位导致写不进去.
         *
         * **为什么带重试(2026-10-18 现场问题)**:
         *   本板 /CS 没有外部上拉, MCU 复位期间它是浮空的(实测默认低), 芯片可能在上电/复位
         *   过程中被 SCK 上的噪声灌进杂散指令而暂时"不理人"(JEDEC 读回 0x171717, SR1=0x17),
         *   一段时间后自己恢复. 这种瞬时失败不该被当成"芯片坏了": 每次失败先发 66h/99h 软复位
         *   (手册 7.4.15/7.4.16: 把芯片从任何未知状态拉回上电初始状态) + Release Power-Down,
         *   隔 W25Q128_ID_RETRY_GAP_MS 再试, 最多 W25Q128_ID_ATTEMPTS 次.
         *
         * 失败时调用方可以读 lastJedecId()/lastInitAttempts() 把**实际 ID 值**打进日志 ——
         * 只报"识别失败"没有价值, "JEDEC=0x171717, 试了 9 轮"才能一眼定位是芯片没应答.
         *
         * @return true = 识别为 W25Q128JV(EF 40 18) 且状态寄存器已清干净
         */
        bool init()
        {
            lastJedec      = 0u;
            lastAttempts   = 0u;
            for (uint8_t attempt = 0; attempt < W25Q128_ID_ATTEMPTS; ++attempt) {
                ++lastAttempts;
                if (attempt > 0u) {
                    // 上一轮读到的是坏 ID: 先按手册把芯片拉回上电初始状态, 再等残留擦写结束
                    reset();
                    delay_us(W25Q128_ID_RESET_GAP_US);
                }
                releasePowerDown(); // 若上次掉电前执行过 Power-down, 必须先唤醒
                delay_us(W25Q128_POWER_DOWN_RELEASE_US);
                delay_ms(W25Q128_POWER_UP_MS); // tPUW: 刚上电/刚复位后芯片尚未准备好
                if (!waitBusy(W25Q128_BUSY_WAIT_MS)) {
                    continue; // 芯片还在忙(可能是上一次杂散指令触发的擦写), 等下一轮
                }

                lastJedec = readJEDEC();
                if (lastJedec != ((uint32_t)W25Q128_MANUFACTURER_ID << 16 |
                                  (uint32_t)W25Q128_MEMORY_TYPE << 8 |
                                  (uint32_t)W25Q128_CAPACITY_ID)) {
                    delay_ms(W25Q128_ID_RETRY_GAP_MS);
                    continue;
                }

                // 清块保护(BP/TB/SEC/SRP 全 0); -IQ 的 QE=1 在状态寄存器-2, 不受影响
                writeEnable();
                writeStatusRaw(W25Q128_Cmd::WriteStatus1, 0x00);
                // 写状态是"非易失"写入: tW 最大 15ms, 期间芯片只理读状态.
                // 这里**不能**只靠 waitBusy 的第一次读判断(它可能在 BUSY 还没拉高时就返回),
                // 必须回读确认状态寄存器真的变成了 0x00.
                delay_ms(1); // 先让 BUSY 有时间拉高, 再进轮询
                if (!waitBusy(W25Q128_STATUS_WRITE_TIMEOUT_MS)) {
                    continue;
                }
                if (readStatus1() != 0x00u) {
                    continue; // 写保护没清掉(或刚才那一下读到了浮空值): 重来
                }
                return true;
            }
            return false;
        }

        /** @brief 最近一次 init() 读到的 JEDEC ID(失败时用它把实际值打进日志) */
        uint32_t lastJedecId() const { return lastJedec; }
        /** @brief 最近一次 init() 总共试了几轮(W25Q128_ID_ATTEMPTS 封顶) */
        uint8_t lastInitAttempts() const { return lastAttempts; }


        /* ==================== 状态与忙等待 ==================== */

        bool isBusy()
        {
            return (readStatus1() & W25Q128_SR1_BUSY) != 0;
        }

        /** @brief 轮询 BUSY 直到清零, 超时返回 false */
        bool waitBusy(uint32_t timeoutMs)
        {
            uint32_t elapsed = 0;
            while (isBusy()) {
                if (elapsed >= timeoutMs) {
                    return false;
                }
                delay_us(1000);
                ++elapsed;
            }
            return true;
        }

        void writeEnable()
        {
            cmdOnly(W25Q128_Cmd::WriteEnable);
        }

        void writeDisable()
        {
            cmdOnly(W25Q128_Cmd::WriteDisable);
        }

        uint8_t readStatus1() { return readStatus(W25Q128_Cmd::ReadStatus1); }
        uint8_t readStatus2() { return readStatus(W25Q128_Cmd::ReadStatus2); }
        uint8_t readStatus3() { return readStatus(W25Q128_Cmd::ReadStatus3); }

        /** @brief 写状态寄存器-1(自动 Write Enable 并等 BUSY) */
        void writeStatus1(uint8_t value)
        {
            writeEnable();
            writeStatusRaw(W25Q128_Cmd::WriteStatus1, value);
            waitBusy(W25Q128_STATUS_WRITE_TIMEOUT_MS);
        }

        void writeStatus2(uint8_t value)
        {
            writeEnable();
            writeStatusRaw(W25Q128_Cmd::WriteStatus2, value);
            waitBusy(W25Q128_STATUS_WRITE_TIMEOUT_MS);
        }

        void writeStatus3(uint8_t value)
        {
            writeEnable();
            writeStatusRaw(W25Q128_Cmd::WriteStatus3, value);
            waitBusy(W25Q128_STATUS_WRITE_TIMEOUT_MS);
        }

        /* ==================== 读取 ==================== */

        /**
         * @brief 读任意长度数据(Read Data 03h, 跨页自动连续读)
         * @param addr 24bit 起始地址
         */
        bool read(uint32_t addr, uint8_t *data, uint32_t len)
        {
            if (data == 0 || len == 0) {
                return false;
            }
            addr &= W25Q128_ADDR_MASK;
            spi.select();
            spi.swap(static_cast<uint8_t>(W25Q128_Cmd::ReadData));
            sendAddress(addr);
            for (uint32_t i = 0; i < len; ++i) {
                data[i] = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            }
            spi.deselect();
            return true;
        }

        /** @brief JEDEC ID: 返回 (MF << 16) | (MT << 8) | CAP, W25Q128JV = 0xEF4018 */
        uint32_t readJEDEC()
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(W25Q128_Cmd::JEDECID));
            const uint8_t mf = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            const uint8_t mt = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            const uint8_t cp = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            spi.deselect();
            return ((uint32_t)mf << 16) | ((uint32_t)mt << 8) | cp;
        }

        /** @brief 设备 ID(90h): 返回 (MF << 8) | ID, W25Q128JV-IQ = 0xEF17 */
        uint16_t readDeviceID()
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(W25Q128_Cmd::ManufacturerDeviceID));
            spi.swap(static_cast<uint8_t>(0x00)); // dummy
            spi.swap(static_cast<uint8_t>(0x00)); // dummy
            spi.swap(static_cast<uint8_t>(0x00)); // 00h
            const uint8_t mf = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            const uint8_t id = static_cast<uint8_t>(spi.swap(static_cast<uint8_t>(0x00)));
            spi.deselect();
            return (uint16_t)((uint16_t)mf << 8 | id);
        }

        /* ==================== 写入 ==================== */

        /**
         * @brief 单页编程(Page Program 02h)
         * @param addr 页内地址, 要求 addr%256 + len <= 256(不可跨页)
         * @param len  1~256
         * @return true = 编程完成(含 BUSY 等待); false = 参数非法或超时
         */
        bool pageProgram(uint32_t addr, const uint8_t *data, uint32_t len)
        {
            if (data == 0 || len == 0 || len > W25Q128_PAGE_SIZE) {
                return false;
            }
            addr &= W25Q128_ADDR_MASK;
            if ((addr & (W25Q128_PAGE_SIZE - 1)) + len > W25Q128_PAGE_SIZE) {
                return false; // 跨页: 会回绕覆盖页首, 直接拒绝
            }
            writeEnable();
            spi.select();
            spi.swap(static_cast<uint8_t>(W25Q128_Cmd::PageProgram));
            sendAddress(addr);
            for (uint32_t i = 0; i < len; ++i) {
                spi.swap(data[i]);
            }
            spi.deselect();
            return waitBusy(W25Q128_PAGE_PROGRAM_TIMEOUT_MS);
        }

        /**
         * @brief 写任意长度数据, 自动按 256B 页边界切分并逐页编程
         * @note Flash 只能把 1 写成 0, 写前必须先擦除, 否则数据不是预期值
         */
        bool write(uint32_t addr, const uint8_t *data, uint32_t len)
        {
            if (data == 0 || len == 0) {
                return false;
            }
            while (len > 0) {
                const uint32_t pageLeft = W25Q128_PAGE_SIZE - (addr & (W25Q128_PAGE_SIZE - 1));
                const uint32_t chunk = (len < pageLeft) ? len : pageLeft;
                if (!pageProgram(addr, data, chunk)) {
                    return false;
                }
                addr += chunk;
                data += chunk;
                len -= chunk;
            }
            return true;
        }

        /* ==================== 擦除 ==================== */

        /** @brief 4KB 扇区擦除(addr 按扇区对齐) */
        bool eraseSector(uint32_t addr)
        {
            return erase(W25Q128_Cmd::SectorErase, addr, W25Q128_SECTOR_ERASE_TIMEOUT_MS);
        }

        /** @brief 32KB 块擦除(addr 按 32KB 对齐) */
        bool eraseBlock32K(uint32_t addr)
        {
            return erase(W25Q128_Cmd::BlockErase32K, addr, W25Q128_BLOCK32K_ERASE_TIMEOUT_MS);
        }

        /** @brief 64KB 块擦除(addr 按 64KB 对齐) */
        bool eraseBlock64K(uint32_t addr)
        {
            return erase(W25Q128_Cmd::BlockErase64K, addr, W25Q128_BLOCK64K_ERASE_TIMEOUT_MS);
        }

        /** @brief 整片擦除(最长 200s, 超时可传 W25Q128_CHIP_ERASE_TIMEOUT_MS) */
        bool eraseChip(uint32_t timeoutMs = W25Q128_CHIP_ERASE_TIMEOUT_MS)
        {
            writeEnable();
            cmdOnly(W25Q128_Cmd::ChipErase);
            return waitBusy(timeoutMs);
        }

        /* ==================== 电源与复位 ==================== */

        /** @brief 进入 Power-down(约 1uA; 之后只接受 Release Power-down 指令) */
        void powerDown()
        {
            cmdOnly(W25Q128_Cmd::PowerDown);
        }

        /** @brief 退出 Power-down(ABh + 3 dummy, 同时可读设备 ID) */
        void releasePowerDown()
        {
            spi.select();
            spi.swap(static_cast<uint8_t>(W25Q128_Cmd::ReleasePowerDown));
            spi.swap(static_cast<uint8_t>(0x00));
            spi.swap(static_cast<uint8_t>(0x00));
            spi.swap(static_cast<uint8_t>(0x00));
            spi.deselect();
        }

        /** @brief 软件复位(66h + 99h, 恢复上电初始状态), 等 tRST 30us */
        void reset()
        {
            cmdOnly(W25Q128_Cmd::EnableReset);
            cmdOnly(W25Q128_Cmd::ResetDevice);
            delay_us(30);
            waitBusy(100);
        }
    };
} // namespace dlx

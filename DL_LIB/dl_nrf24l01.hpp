#include "dl_spi.hpp"
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_exti.hpp"
#include "global.h"
namespace dl
{
    enum class NRF24L01_REGISTERS : uint8_t {
        CONFIG      = 0x00,
        EN_AA       = 0x01,
        EN_RXADDR   = 0x02,
        SETUP_AW    = 0x03,
        SETUP_RETR  = 0x04,
        RF_CH       = 0x05,
        RF_SETUP    = 0x06,
        STATUS      = 0x07,
        OBSERVE_TX  = 0x08,
        CD          = 0x09,
        RX_ADDR_P0  = 0x0A,
        RX_ADDR_P1  = 0x0B,
        RX_ADDR_P2  = 0x0C,
        RX_ADDR_P3  = 0x0D,
        RX_ADDR_P4  = 0x0E,
        RX_ADDR_P5  = 0x0F,
        TX_ADDR     = 0x10,
        RX_PW_P0    = 0x11,
        RX_PW_P1    = 0x12,
        RX_PW_P2    = 0x13,
        RX_PW_P3    = 0x14,
        FIFO_STATUS = 0x17
    };
    enum class NRF24L01_MODE : bool {
        RX = false,
        TX = true
    };

    enum class NRF24L01_INSTRUCTIONS : uint8_t {
        READ_REGISTER    = 0b00000000,
        WRITE_REGISTER   = 0b00100000,
        READ_RX_PAYLOAD  = 0b01100001,
        WRITE_TX_PAYLOAD = 0b10100000,
        FLUSH_TX         = 0b11100001,
        FLUSH_RX         = 0b11100010,
    };

    class nrf24l01
    {
    private:
        NRF24L01_MODE mode;

        uint8_t RX_PACKET_SIZE;
        uint8_t TX_PACKET_SIZE;

        dl::SPI spix;
        uint8_t rx_addr[5];
        uint8_t tx_addr[5];

        GPIO_Pin_Type ce;
        // GPIO_Pin_Type irq;

    public:
        nrf24l01(dl::SPI &xspi, GPIO_Pin_Config ce, /* GPIO_Pin_Config irq,*/ uint8_t rx_addr[5], uint8_t tx_addr[5], uint8_t RPZ, uint8_t TPZ);

        void writeRegister(NRF24L01_REGISTERS reg, uint8_t value);

        void writeRegister(NRF24L01_REGISTERS reg, uint8_t *value, uint8_t len);

        uint8_t readRegister(NRF24L01_REGISTERS reg);
        uint8_t readRegister(NRF24L01_REGISTERS reg, uint8_t *data, uint16_t len);

        void send(uint8_t *data);

        void receive(uint8_t *data, uint16_t timeout);

        void init();

        ~nrf24l01();
    };
    nrf24l01::nrf24l01(dl::SPI &xspi, GPIO_Pin_Config ce, /* GPIO_Pin_Config irq,*/ uint8_t rx_addr[5], uint8_t tx_addr[5], uint8_t RPZ = 16, uint8_t TPZ = 16)
        : spix(xspi), ce(ce.pin), /* irq(irq.pin),*/ RX_PACKET_SIZE(RPZ), TX_PACKET_SIZE(TPZ)
    {
        GPIO_Pin_Init(ce, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP);
        // GPIO_Pin_Init(irq, GPIO_Mode_IN, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP);
        memcpy(this->rx_addr, rx_addr, 5); // 设置RX地址
        memcpy(this->tx_addr, tx_addr, 5); // 设置TX地址
        // 因为IRQ未启用，空着
        /*
        dl::EXTI_Init(irq, EXTI_Trigger_Rising, 1, 1, [this]() {
            if (mode == NRF24L01_MODE::TX) {
                // 两种情况,一是发送失败,二是FIFO清空
                uint8_t status = readRegister(NRF24L01_REGISTERS::STATUS);
                if (status & 1 << 5) {
                    // 成功,拉低CE
                    this->ce = 0;
                    txCount  = 0;
                } else if (status & 1 << 4) {
                    // 失败,但理论上不允许失败,报个错
                    // TODO:报个错
                }
                // 恢复到RX模式
                writeRegister(NRF24L01_REGISTERS::CONFIG, 0x0C);
                this->ce = 1;
            } else {
                // 可以读取,开始读取
                uint8_t status = readRegister(NRF24L01_REGISTERS::STATUS);
                if (!(status & 1 << 4)) {
                    // 出事
                }
                // 读取数据
                while (status & 1 << 4) {

                    uint8_t buffer[RX_PACKET_SIZE];
                    spix.select().write(static_cast<uint8_t>(NRF24L01_INSTRUCTIONS::READ_RX_PAYLOAD)).read(buffer, RX_PACKET_SIZE).deselect();

                    memcpy(rx_buf + rxHead, buffer, RX_PACKET_SIZE);
                    rxCount += RX_PACKET_SIZE;
                    rxCount = rxCount > rxBufferLength? rxBufferLength : rxCount;
                    rxHead += RX_PACKET_SIZE;
                    rxHead %= rxBufferLength;
                    //会覆盖数据,能存多少全凭本事
                    status    = readRegister(NRF24L01_REGISTERS::STATUS);
                }
            }
        });
        */
    }

    void nrf24l01::writeRegister(NRF24L01_REGISTERS reg, uint8_t value)
    {
        spix.select()
            .write(static_cast<uint8_t>(NRF24L01_INSTRUCTIONS::WRITE_REGISTER) | static_cast<uint8_t>(reg))
            .write(value)
            .deselect();
    }

    void nrf24l01::writeRegister(NRF24L01_REGISTERS reg, uint8_t *value, uint8_t len)
    {
        spix.select()
            .write(static_cast<uint8_t>(NRF24L01_INSTRUCTIONS::WRITE_REGISTER) | static_cast<uint8_t>(reg))
            .write(value, len)
            .deselect();
    }
    uint8_t nrf24l01::readRegister(NRF24L01_REGISTERS reg)
    {
        uint8_t value = 0;
        spix.select()
            .write(static_cast<uint8_t>(NRF24L01_INSTRUCTIONS::READ_REGISTER) | static_cast<uint8_t>(reg))
            .read(&value)
            .deselect();
        return value;
    }
    uint8_t nrf24l01::readRegister(NRF24L01_REGISTERS reg, uint8_t *data, uint16_t len)
    {
        spix.select()
            .write(static_cast<uint8_t>(NRF24L01_INSTRUCTIONS::READ_REGISTER) | static_cast<uint8_t>(reg))
            .read(data, len)
            .deselect();
        return len;
    }

    void nrf24l01::init()
    {
        ce = 0;
        writeRegister(NRF24L01_REGISTERS::CONFIG, 0x0B);
        // 全部默认即可,设置一下自己的TX,RX地址

        writeRegister(NRF24L01_REGISTERS::TX_ADDR, tx_addr, 5); // 设置TX地址

        writeRegister(NRF24L01_REGISTERS::RX_ADDR_P0, tx_addr, 5); // 设置RX地址

        writeRegister(NRF24L01_REGISTERS::RX_PW_P0, RX_PACKET_SIZE); // 设置RX数据宽度

        writeRegister(NRF24L01_REGISTERS::EN_RXADDR, 0x03);

        writeRegister(NRF24L01_REGISTERS::RF_SETUP, 0x0F);
        ce = 1;
        dl::delay_ms(1);
    }

    // 永远立即发送
    void nrf24l01::send(uint8_t *data)
    {
        ce = 0;
        writeRegister(NRF24L01_REGISTERS::CONFIG, 0x0A); // 进入发送模式
        uint8_t buffer[TX_PACKET_SIZE + 1];
        buffer[0] = static_cast<uint8_t>(NRF24L01_INSTRUCTIONS::WRITE_TX_PAYLOAD);
        memcpy(buffer + 1, data, TX_PACKET_SIZE);
        spix.select().write(buffer, TX_PACKET_SIZE + 1).deselect();
        ce = 1;
        while (!readRegister(NRF24L01_REGISTERS::STATUS) & 0x20) {
            // 此处可能超时
        }
        writeRegister(NRF24L01_REGISTERS::CONFIG, 0x0B); // 进入接收模式
    }
    void nrf24l01::receive(uint8_t *data, uint16_t timeout = 50)
    {

        while (!(readRegister(NRF24L01_REGISTERS::STATUS) & 0x40)) {
            // 等待数据来
        }
        writeRegister(NRF24L01_REGISTERS::STATUS, 0x40);
        spix.select().write(static_cast<uint8_t>(NRF24L01_INSTRUCTIONS::READ_RX_PAYLOAD)).read(data, RX_PACKET_SIZE).deselect();

        return;

        // 否则等待数据
    }
    nrf24l01::~nrf24l01()
    {
    }

} // namespace dl

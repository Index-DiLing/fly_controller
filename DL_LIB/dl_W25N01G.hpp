#pragma once

#include "dl_spi.hpp"

namespace dl
{
    enum class W25N01ZEIG_STATUS_REGISTER : uint8_t {
        PROTECTION    = 0xA0,
        CONFIGURATION = 0xB0, // ... MODE ... ...
        STATUS        = 0xC0 // ... ERASE_FAIL WSL BUSY
    };

    class W25N01ZEIG
    {
    private:
        SPI &spix;
        const uint8_t dummy = 0x00;

    public:
        W25N01ZEIG(SPI &xspi);
        // 写一个字节多没意思啊.jpg

        /**
         * @note读取寄存器
         */
        void writeRegister(W25N01ZEIG_STATUS_REGISTER reg, uint8_t data);

        uint8_t readStatusRegister(W25N01ZEIG_STATUS_REGISTER reg);

        /**
         * @note 加载指定页到缓冲区 2K
         */
        void pageDataRead(uint16_t pageAddress);

        /**
         * @note 读取缓冲区中指定列,这个列实际就是一个Byte
         */
        void readData(uint16_t colunmAddress,uint8_t *data, uint16_t len);


        /**
         * 写入缓冲区,主机向缓冲区写入
         */

        void loadProgramData(uint16_t pageAddress, uint8_t *data, uint16_t len);

        /**
         * 开启写入
         */
        void writeEnable();

        /**
         * 关闭全阵列写保护,这个在初始化时执行即可
         */
        void disableWriteProtect();

        /**
         *执行写入操作,将缓冲区数据写入指定页 
         */
        void programExecute(uint16_t pageAddress);

        /**
         * 擦除
         */
        void erase(uint16_t pageAddress);


        /**
         * 检查闪存是否处于忙碌
         */
        bool isBusy();

        ~W25N01ZEIG();
    };

    void W25N01ZEIG::pageDataRead(uint16_t pageAddress)
    {
        uint8_t cmd[4] = {0x13, dummy,static_cast<uint8_t>(pageAddress >> 8), static_cast<uint8_t>(pageAddress & 0xFF)};
        spix.send(cmd, 4);
    }

    void W25N01ZEIG::writeRegister(W25N01ZEIG_STATUS_REGISTER reg, uint8_t data)
    {
        uint8_t cmd[3] = {0x01, static_cast<uint8_t>(reg), data};
        spix.select().write(cmd, 3).deselect();
    }

    //警告:只能隔一段时间调用一次
    uint8_t W25N01ZEIG::readStatusRegister(W25N01ZEIG_STATUS_REGISTER reg)
    {
        uint8_t cmd[2] = {0x05, static_cast<uint8_t>(reg)};
        uint8_t data[2];
        spix.select();
        spix.write(cmd, 2);
        spix.read(data, 2);
        spix.deselect();
        return data[1];
    }

    void W25N01ZEIG::readData(uint16_t colunmAddress,uint8_t *data, uint16_t len)
    {
        uint8_t cmd[5] = {0x03,static_cast<uint8_t>(colunmAddress >> 8), static_cast<uint8_t>(colunmAddress & 0xFF),dummy,dummy};
        spix.select().write(cmd, 5);
        spix.clearRXBuffer();
        spix.read(data, len).deselect();
    }
    void  W25N01ZEIG::loadProgramData(uint16_t colunmnAddress, uint8_t *data, uint16_t len){
        // len<2048
        if (len > 2048 || len == 0) return;
        uint8_t cmd[4] = {0x02, static_cast<uint8_t>(colunmnAddress >> 8), static_cast<uint8_t>(colunmnAddress & 0xFF), dummy};
        spix.select().write(cmd, 4).write(data, len).deselect();
    }

    void W25N01ZEIG::writeEnable(){
        spix.send(0x06);
    }

    void W25N01ZEIG::disableWriteProtect(){
        uint8_t cmd[3] = {0x01, static_cast<uint8_t>(W25N01ZEIG_STATUS_REGISTER::PROTECTION), 0x00};
        //关闭全阵列保护
        spix.send(cmd, 3);
    }

    void W25N01ZEIG::programExecute(uint16_t pageAddress){
        uint8_t cmd[4] = {0x10,dummy,static_cast<uint8_t>( pageAddress >> 8),static_cast<uint8_t>( pageAddress & 0xFF)};
        spix.send(cmd, 4);
    }

    void W25N01ZEIG::erase(uint16_t pageAddress)
    {
        uint8_t cmd[4] = {0xD8,dummy,static_cast<uint8_t>( pageAddress >> 8),static_cast<uint8_t>( pageAddress & 0xFF)};
        spix.send(cmd, 4);
    }

    bool W25N01ZEIG::isBusy()
    {
        return (readStatusRegister(W25N01ZEIG_STATUS_REGISTER::STATUS) & 0x01) != 0;
    }


     W25N01ZEIG::W25N01ZEIG(SPI &xspi) : spix(xspi)
    {
        spix.deselect();
    }

    W25N01ZEIG::~W25N01ZEIG()
    {
    }

} // namespace dl

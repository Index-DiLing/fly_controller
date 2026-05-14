#include "stm32f4xx.h"
#include "dl_spi.hpp"
#include "dl_gpio.hpp"
#include "dl_delay.hpp"
namespace dl
{
    class LCD_ST7735S
    {
    private:
        /* data */
        dl::SPI &spi_trans;
        dl::GPIO_Pin_Type dc;
        dl::GPIO_Pin_Type rst;
        void writeCommand(uint8_t cmd)
        {
            dc=0;
            spi_trans.send(cmd);
            
        }
        void writeData(uint8_t data)
        {
            dc=1;
            spi_trans.send(data);
        }

    public:
        LCD_ST7735S(dl::SPI &spi_,dl::GPIO_Pin_Config dc_,dl::GPIO_Pin_Config rst_) : spi_trans(spi_)
        {

         dc = dc_.pin;
         rst = rst_.pin;
         GPIO_Pin_Init(rst_,GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
         GPIO_Pin_Init(dc_,GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
        }
        void init()
        { 
            rst = 1;
            delay_ms(1);
            //------------------------------------ST7735R Reset Sequence-----------------------------------------//
            rst = 0;
            delay_ms(1); // Delay 1ms
            rst=1;
            delay_ms(500); // Delay 120ms
            //--------------------------------End ST7735R Reset Sequence --------------------------------------//

            //--------------------------------End ST7735S Reset Sequence --------------------------------------//
            writeCommand(0x11); // Sleep out
            delay_ms(120);      // Delay 120ms
                                //------------------------------------ST7735S Frame Rate-----------------------------------------//

            //------------------------------------ST7735S Frame Rate-----------------------------------------//
            writeCommand(0xB1);
            writeData(0x05);
            writeData(0x3C);
            writeData(0x3C);
            writeCommand(0xB2);
            writeData(0x05);
            writeData(0x3C);
            writeData(0x3C);
            writeCommand(0xB3);
            writeData(0x05);
            writeData(0x3C);
            writeData(0x3C);
            writeData(0x05);
            writeData(0x3C);
            writeData(0x3C);
            //------------------------------------End ST7735S Frame Rate---------------------------------//
            writeCommand(0xB4); // Dot inversion
            writeData(0x03);
            //------------------------------------ST7735S Power Sequence---------------------------------//
            writeCommand(0xC0);
            writeData(0x28);
            writeData(0x08);
            writeData(0x04);
            writeCommand(0xC1);
            writeData(0XC0);
            writeCommand(0xC2);
            writeData(0x0D);
            writeData(0x00);
            writeCommand(0xC3);
            writeData(0x8D);
            writeData(0x2A);
            writeCommand(0xC4);
            writeData(0x8D);
            writeData(0xEE);
            //---------------------------------End ST7735S Power Sequence-------------------------------------//
            writeCommand(0xC5); // VCOM
            writeData(0x1A);
            writeCommand(0x36); // MX, MY, RGB mode
            writeData(0xC0);
            //------------------------------------ST7735S Gamma Sequence---------------------------------//
            writeCommand(0xE0);
            writeData(0x04);
            writeData(0x22);
            writeData(0x07);
            writeData(0x0A);
            writeData(0x2E);
            writeData(0x30);
            writeData(0x25);
            writeData(0x2A);
            writeData(0x28);
            writeData(0x26);
            writeData(0x2E);
            writeData(0x3A);
            writeData(0x00);
            writeData(0x01);
            writeData(0x03);
            writeData(0x13);
            writeCommand(0xE1);
            writeData(0x04);
            writeData(0x16);
            writeData(0x06);
            writeData(0x0D);
            writeData(0x2D);
            writeData(0x26);
            writeData(0x23);
            writeData(0x27);
            writeData(0x27);
            writeData(0x25);
            writeData(0x2D);
            writeData(0x3B);
            writeData(0x00);
            writeData(0x01);
            writeData(0x04);
            writeData(0x13);
            //------------------------------------End ST7735S Gamma Sequence-----------------------------//
            writeCommand(0x3A); // 65k mode
            writeData(0x05);
            writeCommand(0x29); // Display on
        }
        void setLocation(uint16_t x, uint16_t y)
        {
            writeCommand(0x2A); // Column Address Set
            writeData(x >> 8);
            writeData(x & 0x7F);
            writeCommand(0x2B); // Row Address Set
            writeData(y >> 8);
            writeData(y & 0x9F);
        }

        void writePixel(uint16_t pix)
        {
            writeData(pix >> 8);
            writeData(pix & 0xFF);
        }
        void startWrite()
        {
            writeCommand(0x2C);
        }
        void writePixel(uint16_t *pix, uint16_t size)
        { 
            for (uint16_t i = 0; i < size; i++) {
                writeData(pix[i] >> 8);
                writeData(pix[i] & 0xFF);
            }
        }
        ~LCD_ST7735S() = default;
    };

} // namespace dl


#include <stm32f4xx.h>
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_delay.hpp"
#include "dl_esp.hpp"
#include "dl_socket.hpp"
#include "dl_usart1.hpp"
#include "dl_lcd_st7735s.hpp"
#include "dl_spi1.hpp"
#include "dl_delay.hpp"
#include "dl_esp.hpp"
#include "dl_dma.hpp"
#include "dl_decompressor.hpp"
inline void init()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    dl::GPIO rst(GPIOA, GPIO_Pin_3, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    debugInit;
    GPIO_SetBits(debugGPIO);
}
// TX:PA9

dl::LCD_ST7735S lcd(dl::dSPI1, GPIOA, GPIO_Pin_0, GPIOA, GPIO_Pin_1);

// uint8_t pre = 0;

uint8_t lcd_buf[128 * 160 * 2]; //总缓冲区,与画面内容同步

uint8_t temp_buf[128*160*2+200];//足量缓冲,包含压缩的头信息;

int main()
{
    init();
    // ESP RST PA3
    //  DEBUG A11
    
    dl::Socket socket(dl::usart1);
    dl::esp::init(socket);

    // DC GPIOA P0
    // RST GPIOA P1
    lcd.init();
    lcd.setLocation(0, 0);
    lcd.startWrite();
    for (uint16_t i = 0; i < 128 * 160; i++) {
        lcd.writePixel(0xE8E4);
    }

    // 启用DMA
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    dl::DMA dma(
        DMA2_Stream2,
        DMA_FLAG_TCIF2,
        DMA_Channel_4,
        (uint32_t)(USART1_BASE + 0x04),
        (uint32_t)temp_buf,
        DMA_DIR_PeripheralToMemory,
        128 * 160 * 2+4,
        DMA_PeripheralInc_Disable,
        DMA_MemoryInc_Enable,
        DMA_PeripheralDataSize_Byte,
        DMA_MemoryDataSize_Byte,
        DMA_Mode_Normal,
        DMA_Priority_High,
        DMA_FIFOMode_Disable,
        DMA_FIFOThreshold_HalfFull,
        DMA_MemoryBurst_Single,
        DMA_PeripheralBurst_Single);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

    dl::DMA dma_spi(
        DMA2_Stream3,
        DMA_FLAG_TCIF3,
        DMA_Channel_3,
        (uint32_t)(SPI1_BASE + 0x0C),
        (uint32_t)lcd_buf,
        DMA_DIR_MemoryToPeripheral, // 内存向外设写入
        128 * 160 * 2,
        DMA_PeripheralInc_Disable, // 禁用外设递增
        DMA_MemoryInc_Enable,      // 启用内存递增
        DMA_PeripheralDataSize_Byte,
        DMA_MemoryDataSize_Byte,
        DMA_Mode_Normal,
        DMA_Priority_High,
        DMA_FIFOMode_Disable,
        DMA_FIFOThreshold_HalfFull,
        DMA_MemoryBurst_Single,
        DMA_PeripheralBurst_Single);
    SPI_DMACmd(SPI1, SPI_DMAReq_Tx, ENABLE);

    dma.start();
    socket.sendCommand(101);
    dma.wait();

    //这里读取到的信息前两个字节是有效长度信息,对于第一帧,再两个字节表示总帧数量
    uint16_t length = (temp_buf[0] << 8) + temp_buf[1];//这里写入的是下一帧的总长度信息
    
    uint16_t total_frames = (temp_buf[2] << 8) + temp_buf[3];
    //这里第一2次需要手动搬运数据
    for (uint16_t i = 4; i < 128*160*2+4; i++)
    {
        lcd_buf[i-4] = temp_buf[i];
    }
    dma_spi.start();
    dma_spi.wait();

    

    dma.reset(length+2, temp_buf);//下一帧传输准备
    dma.start();
    socket.sendCommand(101);
    dma.wait();

   
    while (total_frames--)
    {
        //前两个字节是长度,这里不需要忽略即可
        
        dl::decompress(temp_buf+2,lcd_buf,length,2);

        //更新长度,准备接收下一帧,这个长度是不包含自身的,这个长度是字节长度

        length = (temp_buf[0] << 8) + temp_buf[1];

        //解压完毕,开始两个DMA的传输
        dma_spi.reset(128 * 160 * 2, lcd_buf);
        dma_spi.start();

        dma.reset(length+2, temp_buf);
        dma.start();
        socket.sendCommand(101);
        if (DMA2_Stream2->NDTR != 0)
        {
            GPIO_ResetBits(debugGPIO);
        }
        dma.wait();
        dma_spi.wait();
    }
    /*
    DMA开启,读取压缩帧数据->解压缩,写入缓冲区->DMA重启,读取下一帧 
                                            DMA开启,写入屏幕
    */
/*
    while (true) {

        dma.reset(128 * 160 * 2, lcd_buf2);
        dma.start();
        socket.sendCommand(100);

        dma_spi.start();
        //启用传输
        dma_spi.wait();
        dma_spi.reset(128 * 160 * 2, lcd_buf2);

        dma.wait();
        dma.reset(128 * 160 * 2, lcd_buf1);
        dma.start();
        socket.sendCommand(100);

        dma_spi.start();
        //启用传输
        dma_spi.wait();
        dma_spi.reset(128 * 160 * 2, lcd_buf1);
        
        dma.wait();
    }
    dl::set_it_callback(USART1_IRQn, [](){
        if (USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
        {
            if (!flag)
            {
                pre = USART_ReceiveData(USART1);

            }else{
                lcd.writePixel((pre<<8)+ USART_ReceiveData(USART1));
            }
            flag = !flag;
        }
    });
    */

    while (1) {
        /* code */
    }

    return 0;
}


#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_delay.hpp"
#include "dl_esp.hpp"
#include "dl_lcd_st7735s.hpp"
#include "dl_spi1.hpp"
#include "dl_delay.hpp"
#include "dl_esp.hpp"
#include "dl_socket.hpp"
#include "dl_usart.hpp"
#include "dl_timer.hpp"
#include "dl_dma.hpp"
#include "dl_dshot.hpp"
#include "dl_iic.hpp"
#include "dl_bme280.hpp"

#include <string>
#include <format>
#define MAX_16_BITS 65536

#define CODE_END    while (1);

inline void init()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    debugInit;
    GPIO_ResetBits(debugGPIO);
}
// TX:PA9

static char str[300];
int main()
{

    init();
    /*
    ESP RST PA3
    DEBUG A11
    */
    dl::USART usart1X(1520000, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None, 1, 1, USART1);
    dl::Socket socket(usart1X);
    // dl::esp::init(socket);

    socket.sendString("Hello");

    dl::IIC bus(I2C1, 100000, I2C_Mode_I2C, I2C_DutyCycle_2, 0x51, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);

    dl::IICDevice bme280(bus, 0x76);
    // 先读取 0x88~0xA1（26字节）

    bme280.read(0x88, 26);
    // 解析前 12 个校准参数（dig_T1~dig_P9）
    memcpy(&dl::BME280::calib, &bme280.buffer, 26);
    // 再读取 0xE1~0xE7（7字节）
    bme280.read(0xE1, 7);

    // 写入剩下七个字节
    dl::BME280::calib.dig_H2 = (bme280.buffer[1] << 8) | bme280.buffer[0];
    dl::BME280::calib.dig_H3 = bme280.buffer[2];
    dl::BME280::calib.dig_H4 = (bme280.buffer[3] << 4) | (bme280.buffer[4] & 0x0F);
    dl::BME280::calib.dig_H5 = (bme280.buffer[5] << 4) | ((bme280.buffer[4] >> 4) & 0x0F);
    dl::BME280::calib.dig_H6 = bme280.buffer[6];
    // 神秘仪式这一块.

    bme280.write(0xF2, 0x01); // 湿度 ×1
    bme280.write(0xF4, 0x57); // 温度 ×2，压力 ×16，正常模式
    bme280.write(0xF5, 0x10); // 待机 0.5ms，滤波器 16

    dl::BME280::bmeData data;

    bme280.read(0xF7, 8);
    data.pressure    = ((uint32_t)bme280.buffer[0] << 12) | ((uint32_t)bme280.buffer[1] << 4) | ((uint32_t)bme280.buffer[2] >> 4);
    data.temperature = ((uint32_t)bme280.buffer[3] << 12) | ((uint32_t)bme280.buffer[4] << 4) | ((uint32_t)bme280.buffer[5] >> 4);
    data.humidity    = (((uint16_t)bme280.buffer[6]) << 8) | ((uint16_t)bme280.buffer[7]);
    uint32_t temp    = dl::BME280::BME280_compensate_T_int32(data.temperature);
    uint32_t pres    = dl::BME280::BME280_compensate_P_int64(data.pressure);
    uint32_t humi    = dl::BME280::bme280_compensate_H_int32(data.humidity);
    bool x =true;
    float p0 = 0;

    while (1) {

        bme280.read(0xF7, 8);
        // 好了现在拼接
        data.pressure    = ((uint32_t)bme280.buffer[0] << 12) | ((uint32_t)bme280.buffer[1] << 4) | ((uint32_t)bme280.buffer[2] >> 4);
        data.temperature = ((uint32_t)bme280.buffer[3] << 12) | ((uint32_t)bme280.buffer[4] << 4) | ((uint32_t)bme280.buffer[5] >> 4);
        data.humidity    = (((uint16_t)bme280.buffer[6]) << 8) | ((uint16_t)bme280.buffer[7]);
        temp             = dl::BME280::BME280_compensate_T_int32(data.temperature);
        pres             = dl::BME280::BME280_compensate_P_int64(data.pressure);
        humi             = dl::BME280::bme280_compensate_H_int32(data.humidity);

        float p  = (pres) / 256.f;
        float t  = temp / (100.f);
        double h = (humi) / 1024.f;

        if (x)
        {
          p0 = p;
          x=!x;
        }
        
        float deltahight = (((pow((double)p0 / p, 1 / 5.257) - 1) * (t + 273.15)) / 0.0065);
        

        sprintf(str, "pressure: %f\ntempreture: %f\nhumidity: %f\n", p, t, h);
        socket.sendString(str);
        dl::delay_ms(50);
    }

    while (1);
}

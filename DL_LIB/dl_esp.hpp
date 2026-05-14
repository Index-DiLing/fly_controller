#pragma once
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_delay.hpp"
#include "dl_socket.hpp"
namespace dl
{
    namespace esp
    {

      /**
       * 需要硬编码 WIFI名称和密码,服务器端口和IP
       */
      void init(dl::Socket s){
        //s.sendString("AT+CWJAP=\"SSID\",\"PASSWORD\"\r\n");
        delay_ms(3000);
        //AT+CIPSTART="TCP","192.168.137.1",8051
        s.sendString("AT+CIPSTART=\"TCP\",\"192.168.137.1\",8051\r\n");
        delay_ms(200);
        s.sendString("AT+CIPMODE=1\r\n");
        delay_ms(200);
        s.sendString("AT+CIPSEND\r\n");
        delay_ms(200);
      }

    }
} // namespace dl


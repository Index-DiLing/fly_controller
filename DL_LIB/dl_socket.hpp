#pragma once
#include "stm32f4xx.h"
#include "dl_nvic_it.h"
#include "dl_bytebuffer.hpp"
#include "dl_usart.hpp"
// 1Byte命令码
// 发出命令后需要传数据则先传出长度码,一字节短数据两字节长数据,数据接收到位也有回应(0x00);

namespace dl
{
    class Socket
    {
    private:
        dl::USART &usart;

    public:
        Socket(dl::USART &usartx) : usart(usartx)
        {
        }
        // 这两个函数是没有封装的,慎用
        void sendString(const char *str)
        {
            while (*str != '\0') {
                usart.send((uint8_t)*str++);
            }
        }
        // 阻塞式等待，用于指令传输,$占位符,可以处理些数据


        bool waitForString(const char *str)
        {

            uint16_t len = strlen(str);
            uint16_t i   = 0;
            while (len--) {
                uint8_t c = usart.read();
                if (str[i] == '$') {
                    continue;
                }
                if (str[i] != c) {
                    return false;
                }
                i++;
            }
            return true;
        }


        bool waitForString(const char *str, char *data)
        {

            uint16_t len = strlen(str);
            uint16_t i   = 0;
            while (len--) {
                uint8_t c = usart.read();
                if (str[i] == '$') {
                    *data++ = c;
                    continue;
                }
                if (str[i] != c) {
                    return false;
                }
                i++;
            }
            *data = '\0';
            return true;
        }

        bool waitForCommandT(const char *str, char *data)
        {
            sendString("Waiting for command: ");
            sendString(str);
            sendString("\r\n");
            bool res = waitForString(str, data);
            if (res) {
                sendString("Received command: ");
            } else {
                sendString("Command receive failed: ");
            }
            sendString(str);
            sendString("\r\n");
            return res;
        }
        bool waitForCommandT(const char *str)
        {
            sendString("Waiting for command: ");
            sendString(str);
            sendString("\r\n");
            bool res = waitForString(str);
            if (res) {
                sendString("Received command: ");
            } else {
                sendString("Command receive failed: ");
            }
            sendString(str);
            sendString("\r\n");
            return res;
        }

        void sendCommand(uint8_t cmd)
        {
            usart.send(cmd);
        }
        void sendData(uint8_t *data, uint16_t len)
        {
            usart.send(data, len);
        }
        void sendData(ByteBuffer& buf)
        {
            usart.send(buf.src,buf.length());
            buf.reset();
        }
        uint8_t read()
        {
            return usart.read();
        }

        uint16_t read(uint8_t *data, uint16_t size)
        {
            usart.read(data, size);
            return size;
        }

        void ASyncRead(uint8_t *data, uint16_t size){
            usart.ASyncRead(data, size);
        }
        

        bool ASyncWait(){
            usart.ASyncReadWait();
            return true;
        }


        void request(uint8_t* rq, uint16_t len,uint8_t* rsp, uint16_t rlen){
            
        }

    };

} // namusartace dl

#pragma once
#include <stdint.h>
#include <memory>
#include "functional"
#include "Fatfs/ff.h"
#include "global.h"
#include "dl_bme280.hpp"
#include "dl_error.hpp"
#include "dl_bytebuffer.hpp"
namespace dl
{
    /*<MESSAGE_STRUCT_CODE_START>*/
struct MessageManager{
     ByteBuffer& buffer;
     MessageManager(ByteBuffer& buffer):
     buffer(buffer)
     {
     }
    bool b280Msg(uint32_t temperature,uint32_t pressure,uint16_t humidity,uint8_t* data = NULL, uint16_t length = 0)
    {
       if(10+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)4);
       buffer.write(temperature);
       buffer.write(pressure);
       buffer.write(humidity);
       return true;
    }
    bool errorMsg(uint32_t error,uint8_t* data = NULL, uint16_t length = 0)
    {
       if(4+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)37);
       buffer.write(error);
       return true;
    }
    bool initMsg(uint8_t* data = NULL, uint16_t length = 0)
    {
       if(0+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)19);
       return true;
    }
    bool logMsg(uint8_t* data = NULL, uint16_t length = 0)
    {
       if(2+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)1);
       buffer.write((uint16_t)length);
       buffer.write(data,length);
       return true;
    }
    bool motorMsg(uint16_t v0,uint16_t v1,uint16_t v2,uint16_t v3,uint8_t* data = NULL, uint16_t length = 0)
    {
       if(8+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)5);
       buffer.write(v0);
       buffer.write(v1);
       buffer.write(v2);
       buffer.write(v3);
       return true;
    }
    bool posMsg(float q0,float q1,float q2,float q3,float gyro0,float gyro1,float gyro2,float acceleration0,float acceleration1,float acceleration2,float magnetometer0,float magnetometer1,float magnetometer2,uint8_t* data = NULL, uint16_t length = 0)
    {
       if(52+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)2);
       buffer.write(q0);
       buffer.write(q1);
       buffer.write(q2);
       buffer.write(q3);
       buffer.write(gyro0);
       buffer.write(gyro1);
       buffer.write(gyro2);
       buffer.write(acceleration0);
       buffer.write(acceleration1);
       buffer.write(acceleration2);
       buffer.write(magnetometer0);
       buffer.write(magnetometer1);
       buffer.write(magnetometer2);
       return true;
    }
    bool requestBooleanParamMsg(uint8_t* data = NULL, uint16_t length = 0)
    {
       if(2+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)10);
       buffer.write((uint16_t)length);
       buffer.write(data,length);
       return true;
    }
    bool requestFloatParamMsg(uint8_t* data = NULL, uint16_t length = 0)
    {
       if(2+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)9);
       buffer.write((uint16_t)length);
       buffer.write(data,length);
       return true;
    }
    bool requestIntParamMsg(uint8_t* data = NULL, uint16_t length = 0)
    {
       if(2+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)11);
       buffer.write((uint16_t)length);
       buffer.write(data,length);
       return true;
    }
    bool syncMsg(uint8_t* data = NULL, uint16_t length = 0)
    {
       if(0+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)6);
       return true;
    }
    bool _bug(uint8_t* data = NULL, uint16_t length = 0)
    {
       if(0+length>buffer.remaining()){return false;}
       buffer.write((uint8_t)0);
       return true;
    }
};
/*<MESSAGE_STRUCT_CODE_END>*/

    namespace message
    {
        void fetchFile(dl::Socket &soc, const char *filename)
        {
            FIL file;
            FILINFO info;
            FRESULT fres = f_stat(filename, &info);

            logger << "START FETCH,File Result: " << fres << LCMD::NFLUSH;

            char f[64] = {'/'};
            strcat(f, filename);

            fres = f_open(&file, f, FA_CREATE_ALWAYS | FA_WRITE);

            logger << "Open File Result: " << fres << LCMD::NFLUSH;

            soc.sendString("<FETCH-INIT> ");
            soc.sendString(filename);
            soc.sendString("\n");

            uint64_t fileSize = 0;

            soc.read((uint8_t *)&fileSize, 8);
            logger << "File Size: " << (uint32_t)(fileSize) << LCMD::NFLUSH;
            uint32_t blockNum = fileSize / 2048;

            uint8_t *recvBuf = new uint8_t[2048];

            logger << "Block Num: " << blockNum << LCMD::NFLUSH;

            uint32_t l = 0;

            while (blockNum--) {
                soc.sendString("<FETCH-BLOCK>\n");
                soc.read(recvBuf, 2048);
                f_write(&file, recvBuf, 2048, &l);
            }
            soc.sendString("<FETCH-BLOCK>\n");
            soc.read(recvBuf, fileSize % 2048);
            f_write(&file, recvBuf, fileSize % 2048, &l);
            f_close(&file);

            delete[] recvBuf;
            logger << "FETCH END" << LCMD::NFLUSH;
        }
#if 0
        void mainTransferBin(dl::Socket soc)
        {
            soc.sendData((uint8_t *)&globalErrorCode, 4);

            soc.sendData((uint8_t *)&dt, 4);

            soc.sendData((uint8_t *)&q0, 4);

            soc.sendData((uint8_t *)&q1, 4);

            soc.sendData((uint8_t *)&q2, 4);

            soc.sendData((uint8_t *)&q3, 4);

            soc.sendData((uint8_t *)&bmeData.pressure, 4);

            soc.sendData((uint8_t *)&bmeData.temperature, 4);

            soc.sendData((uint8_t *)&bmeData.humidity, 2);

            soc.sendData((uint8_t *)&throttleValue[0], 2);

            soc.sendData((uint8_t *)&throttleValue[1], 2);

            soc.sendData((uint8_t *)&throttleValue[2], 2);

            soc.sendData((uint8_t *)&throttleValue[3], 2);
        }
#endif
    } // namespace message

} // namespace dl

#pragma once
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
namespace dl
{
    //还挺危险的这函数
    void decompress(
        uint8_t* encoded_p,
        uint8_t* decoded_p,

        uint32_t encodeed_len,
        
        uint8_t wordsLen
    ){
        //wordsLen代表单元长度(单位 字节)
        while (encodeed_len>0)
        {
            uint8_t meta = *encoded_p;
            encoded_p++;
            if (meta&128)
            {
                //需要替换的字节数
                uint8_t length = wordsLen*(meta & 127);
                encodeed_len-=(length+1);
                while (length--)
                {
                    *decoded_p++ =  *encoded_p++;//交换字节,数量为Length
                }
            }else{
                //不需要交换字节,移动decoded_p即可
                decoded_p+= meta*wordsLen;
                encodeed_len--;
            }
        }
        //覆写完毕
    }
} // namespace dl

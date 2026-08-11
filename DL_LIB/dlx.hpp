

struct DLX_ProtocolBuffer {
    uint8_t crc4_itu(uint8_t *data, uint16_t length){
        uint8_t i;
        uint8_t crc = 0;                    // Initial value
        while(length--)
        {
            crc ^= *data++;                 // crc ^= *data; data++;
            for (i = 0; i < 8; ++i)
            {
                if (crc & 1)
                    crc = (crc >> 1) ^ 0x0C;// 0x0C = (reverse 0x03)>>(8-4)
                else
                    crc = (crc >> 1);
            }
        }
        return crc;
    }
    ByteBuffer& buffer;
    DLX_ProtocolBuffer(ByteBuffer& buffer):buffer(buffer){ }
    bool PersonW(int16_t name,float *age,uint8_t *log,uint16_t log_len){
         uint8_t *p = buffer.cur;
         if(buffer.remaining()<46 + (log_len) ){return false;}
         buffer.write<uint16_t>(0);//DUMMY
         buffer.write<int16_t>(name);
         buffer.write(reinterpret_cast<uint8_t*>( age ),40);
         buffer.write<uint16_t>(log_len); buffer.write(reinterpret_cast<uint8_t*>( log ),log_len);
         uint16_t dlx_header = static_cast<uint16_t>(((1<<15) | (0<<5) | (crc4_itu(p+2,46 | (log_len))<<1)));
         *p     = dlx_header & 0xff;
         *(p+1) = dlx_header & 0xff;
         return true;
    }
}
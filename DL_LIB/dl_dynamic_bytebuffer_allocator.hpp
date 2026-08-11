#pragma once
#include "dl_bytebuffer.hpp"
#include "dl_error.hpp"

namespace dl
{
    
    class DynamicByteBufferAllocator
    {
    private:
        uint16_t SIZE  = 0;
        uint16_t allocated = 0;
        dl::ExceptionManager& exceptionManager;
    public:
        DynamicByteBufferAllocator(dl::ExceptionManager& manager,uint16_t max):
        exceptionManager(manager)
        {
            SIZE = max;
        }
        ~DynamicByteBufferAllocator(){
            
        }
        ByteBuffer allocate(uint16_t size){
            if(size + allocated > SIZE){
                exceptionManager.error(EXCEPTIONS::EXCEPTION_MANAGER_OVERFLOW);
            }
            allocated+=size;
            return ByteBuffer(new uint8_t[size],size);
        }
    };
} // namespace dl

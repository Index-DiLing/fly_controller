#pragma once
#include <stdint.h>
namespace dlx
{

    //对于Error的定义是 会引发不可处理的运行时异常
    enum class ErrorCode
    {
        /* data */
    };

    //Exception则可处理
    enum class ExceptionCode
    :uint16_t{

    };

    enum class FunctionResult:bool{
        SUCCESS = true,
        FAILED = false
    };
    

    class Exception
    {
    private:
        /* data */
    public:

        ExceptionCode code;

        Exception(ExceptionCode code){

        }
        ~Exception(){

        }
    };
    
} // namespace dlx

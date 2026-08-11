
#pragma once
#include "stdint.h"
#include "dl_bytebuffer.hpp"
namespace dl
{
    /**
     * @brief 使用时间戳来生成此代码
     *
     * @param code
     */
    /**
     * Code由约定组成,其中
     * 最高位1是必须捕获处理的异常
     * 最高位0是可以忽略的异常
     */
    enum class EXCEPTIONS : uint32_t {
        EXCEPTION_MANAGER_OVERFLOW,
        DYNAMIC_BYTEBUFFER_MANAGER_OVERFLOW
    };
    class ExceptionManager
    {
    private:
        uint32_t *list;
        uint16_t size;
        uint16_t cur = 0;

    public:
        ExceptionManager(const ByteBuffer& buffer);
        ~ExceptionManager();
        void error(EXCEPTIONS exp);
        bool catchError(EXCEPTIONS exp);
    };
    ExceptionManager::ExceptionManager(const ByteBuffer& buffer)
    {
        list = buffer.toArray<uint32_t>(&size);
    }
    ExceptionManager::~ExceptionManager()
    {

    }
    void ExceptionManager::error(EXCEPTIONS exp)
    {
        if (size == cur) {
            return;
        }
        if (size - cur == 1 && exp != EXCEPTIONS::EXCEPTION_MANAGER_OVERFLOW) {
            error(EXCEPTIONS::EXCEPTION_MANAGER_OVERFLOW);
        }
        list[cur++] = static_cast<uint32_t>(exp);
    }
    bool ExceptionManager::catchError(EXCEPTIONS exp)
    {
        uint16_t j = 0;
        bool find = false;
        for (uint16_t i = 0; i < cur; i++) {
            if (list[i] != static_cast<uint32_t>(exp) || find) {
                list[j++] = list[i];
            }else{
                find = true;
            }
        }
        cur = j;
        return find;
    }
} // namespace dl

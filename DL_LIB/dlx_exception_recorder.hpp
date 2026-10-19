#pragma once
#include <stdint.h>
#include "dlx_exception.h"

/**
 * @brief 这是Exception-Recorder-Handler的一部分,来自dlx的错误处理模式,类似原生的Exception-throw-catch.
 * 
 * 被调用方声明自身可能抛出异常-外部传入Recorder-外部处理
 * 
 * 被一路抛出到顶层则由全局HANDLER处理,只是这个过程不是编译器自动的而是手动传递的——程序员必须保证自身传递异常,否则异常会丢失.
 *
 */

namespace dlx
{
    template <uint8_t SIZE>
    class ExceptionRecorderTemplate
    {
    private:
        ExceptionCode _records[SIZE]{};
        uint8_t _size{0u};

    public:
        // 如果没有溢出，那就记录,溢出了则无所谓溢出几个,知道了溢出的数量并不能解决问题
        void throws(ExceptionCode code)
        {
            if (_size < SIZE - 1) {
                _records[_size++] = code;
            } else {
                _records[SIZE - 1] = ExceptionCode::EXCEPTION_OVERFLOW;
            }
        }
        void throws(const ExceptionRecorderTemplate &other)
        {
            for (uint8_t i = 0; i < other.size(); ++i) {
                throws(other.at(i));
            }
        }
        uint8_t size() const
        {
            return _size;
        }
        bool empty() const
        { return _size == 0u; }
        ExceptionCode at(uint8_t index) const
        {
            return (index < _size) ? _records[index] : ExceptionCode::EXCEPTION_NONE;
        }

        uint8_t count(ExceptionCode code) const
        {
            uint8_t n = 0;
            for (uint8_t i = 0; i < _size; ++i) {
                if (_records[i] == code) {
                    ++n;
                }
            }
            return n;
        }

        bool has(ExceptionCode code) const
        {
            return count(code) != 0u;
        }

        bool any() const
        {
            return _size != 0u;
        }

        // check是消耗的
        uint8_t check(ExceptionCode code) 
        {
            uint8_t p = 0;
            for (uint8_t i = 0; i < _size; i++) {
                if (_records[i] != code) {
                    _records[p++] = _records[i];
                }
            }
            uint8_t r = _size - p;
            _size     = p;
            return r;
        }

        bool checkFirst(ExceptionCode code)
        {
            for (uint8_t i = 0; i < _size; ++i) {
                if (_records[i] == code) {
                    for (uint8_t j = static_cast<uint8_t>(i + 1u); j < _size; ++j) {
                        _records[j - 1u] = _records[j];
                    }
                    --_size;
                    return true;
                }
            }
            return false;
        }
        ExceptionCode pop()
        {
            if (_size == 0u) {
                return ExceptionCode::EXCEPTION_NONE;
            }
            const ExceptionCode first = _records[0];
            for (uint8_t i = 1u; i < _size; ++i) {
                _records[i - 1u] = _records[i];
            }
            --_size;
            return first;
        }
        uint8_t transferTo(ExceptionRecorderTemplate &dst)
        {
            const uint8_t n = _size;
            for (uint8_t i = 0; i < _size; ++i) {
                dst.throws(_records[i]);
            }
            clear();
            return n;
        }

        void clear()
        {
            _size = 0u;
        }
    };
    using ExceptionRecorder = ExceptionRecorderTemplate<16 + 1>;
    template <uint8_t SIZE>
    inline bool recordCall(ExceptionRecorderTemplate<SIZE> &rec, FunctionResult result, ExceptionCode err)
    {
        const bool ok = (result == FunctionResult::SUCCESS);
        if (!ok) {
            rec.throws(err);
        }
        return ok;
    }

} // namespace dlx

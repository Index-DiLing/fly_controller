#pragma once
// 日志渠道包含文本与异常两条信箱；生产者只投递，拥有者统一成帧和发送。
#include <stdint.h>

#include "dlx_exception.h"
#include "dlx_text_mailbox.hpp"
#include "dlx_value_mailbox.hpp"

namespace dlx
{

namespace logchannel
{
/** 异常请求槽数。异常是"一条一条的现场", 16 条足够扛住一次连发 */
constexpr uint16_t EXCEPTION_SLOTS = 16;
} // namespace logchannel

/** 异常渠道: 定长值信箱(中断安全) */
using ExceptionMailbox = ValueMailbox<ExceptionCode, logchannel::EXCEPTION_SLOTS>;

/**
 * @brief 当前生效的日志渠道
 *
 * 由拥有者填好并挂到上下文里(见 ReleaseContext::channel)。两个成员都允许为 nullptr:
 *   nullptr = 还没有人接 —— 此时生产方**什么都不做**(丢弃), 这比"自己去发"安全得多。
 */
struct LogChannel
{
    TextMailbox      *textMailbox;
    ExceptionMailbox *exceptionMailbox;
};

} // namespace dlx

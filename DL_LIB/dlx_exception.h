#pragma once
#include <stdint.h>
namespace dlx
{
    /**
     * @brief 全局错误码(16 位足够整个系统使用)
     *
     * 约定:
     *  - 数值按模块划分区段, 区段内顺序定义即可, 不做"必须连续"的要求;
     *  - 0x0000 不使用(保留给"未初始化/未知"), 所以每个区段都不从 0 开始;
     *  - 函数返回 FunctionResult 表示"这次调用是否正常完成",
     *    具体结果/异常通过 ExceptionCode& 送出(可能是错误, 也可能只是提示性异常)。
     *
     * 已分配区段:
     *  0x0100 ~ 0x01FF  Flash 文件系统(FlashManager)
     */
    enum class ExceptionCode
    :uint16_t{
        // ============================ Flash 文件系统 ============================
        FLASH_OK                = 0x0100, ///< 无异常(正常完成)
        FLASH_SYSTEM_CREATED    = 0x0101, ///< 未检测到文件系统, 已整片擦除并新建(调用仍算成功, 仅提示异常)
        FLASH_GEOMETRY_MISMATCH = 0x0102, ///< 已有文件系统与本次编译的段布局/结构体大小不一致(不自动重建, 防止误删数据)
        FLASH_HARDWARE_ERROR    = 0x0103, ///< 读不到正确的 Flash ID(SPI 未初始化或硬件异常)
        FLASH_ERASE_FAILED      = 0x0104, ///< 擦除超时/失败
        FLASH_WRITE_FAILED      = 0x0105, ///< 写入超时/失败
        FLASH_READ_FAILED       = 0x0106, ///< 读取失败
        FLASH_NOT_READY         = 0x0107, ///< 未成功 init() 就调用其它接口
        FLASH_LOG_FULL          = 0x0108, ///< 单次日志已达上限且策略为 Halt: 本次写入被拒绝(日志数据仍然完整)
        FLASH_LOG_OVERWRITTEN   = 0x0109, ///< 单次日志已达上限, 已环绕覆写最旧的日志槽(提示性异常, 写入本身成功)
        FLASH_SESSION_NOT_FOUND = 0x010A, ///< 找不到指定的日志会话
        FLASH_PARAM_NOT_FOUND   = 0x010B, ///< 参数段还没有任何可用参数(上层用默认值即可)
        FLASH_INVALID_ARGUMENT  = 0x010C, ///< 入参非法(空指针等)
        FLASH_ENTRY_NOT_FOUND   = 0x010D, ///< 指定会话里没有这个序号的日志条目
    };

    enum class FunctionResult:bool{
        SUCCESS = true,
        FAILED = false
    };
    
} // namespace dlx

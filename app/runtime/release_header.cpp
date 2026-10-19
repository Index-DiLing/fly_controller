// 公共运行句柄的唯一全局定义。
#include "app/runtime/release_header.h"

namespace dlx
{

/** 文本日志出口(飞行模式置 nullptr = 完全静音) */
//USART *debugPort = nullptr;
// removed at 2026/10/2

/** 文本日志要用的串口/协议上下文(main 里绑定一次; 没绑定就不打印) */
ReleaseContext *logContext = nullptr;

/** flash 擦除进度灯的当前相位(低电平点亮); 结束时统一熄灭, 见 onFlashIdle */
bool flashLedPhase = false;

} // namespace dlx

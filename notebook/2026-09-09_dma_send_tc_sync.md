# 串口 DMA 发送 reset/start/wait 顺序与 TC 同步

## 现象
- `main_att_ekf.cpp` 里自作主张把发送顺序调成 `serialDMA.reset(); serialDMA.start(); serialDMA.wait();`，
  想让发送"异步、不阻塞"。结果程序第一次进 `wait()` 就卡死，串口无输出。

## 根因
- `DMA::wait()` 实现是：`while (DMA_GetFlagStatus(TC) == RESET);` —— 一直**空等"传输完成(TC)标志"被置位**。
- `DMA::reset(size)` 会先 `Disable + 清 TC 标志 + 重载 NDTR/M0AR`；`init()` 时也清过 TC。
- 所以在**第一次调用 `wait()` 时**，还没有任何一次真正启动过的传输，TC 标志是 0（已被清零），
  `wait()` 会死等一个"根本不会发生的完成"。不是等"上一个传输"，而是等一个不存在的传输。

## 正确用法（Normal 模式、单缓冲：下一次发送前必须等上一次发完）
```
protocol.QuatW(...) / EKFW(...);            // 1. 把新帧写进发送 buffer
serialDMA.wait();                            // 2. 等上一次传输完成
serialDMA.reset(protocol.buffer.used());     // 3. 清 TC、重载 NDTR/M0AR
serialDMA.start();                           // 4. 启动本次发送
```

## 关键点
- `wait()` 是等 TC 置位；TC 在 `reset()` / `init()` 被清，传输完成后由硬件置位。
- 单缓冲/普通模式：发送缓冲在整个 DMA 读它的期间必须有效且不被改写。
- 想"异步不阻塞"不能只靠交换顺序实现：要么环形/双缓冲，要么用"首次启动标志"跳过第一次 `wait()`，
  否则第一次必然卡死。

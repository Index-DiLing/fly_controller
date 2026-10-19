#pragma once
#include "rtthread.h"
#include "flight_config.hpp"
#include "app/runtime/release_header.h"
#include "app/runtime/app_exception_handler.h"
#include "app/boot/app_boot_report.hpp"
#include "dlx_spi.hpp"
#include "BMI088/dlx_bmi088.hpp"
// 启动期设备初始化与异常诊断。

namespace dlx
{

// 二、IMU（BMI088）—— 必需件: 姿态环没有它就完全没有意义
/**
 * @brief 初始化 IMU：SPI 总线 + 器件 + 自检, 并把**失败类型**分清楚
 *
 * 为什么要回读一次 CHIP_ID: `BMI088::init()` 只给一个 bool, 而"读不到 ID"(接线/片选/供电)
 *   与"ID 对但自检没过"(器件或量程配置)的排查动作完全不同。这里把两者拆成不同的异常码,
 *   顺带还能指出是加速度计那一根还是陀螺那一根。
 *
 */
inline bool bringUpImu(SPI &spi, BMI088 &dev, ExceptionRecorder &rec)
{
    spi.init(cfg::hw::imu::kSpi);
    const bool ok = (dev.init() != 0);

    // 回读 CHIP_ID：既用于**分类**失败原因, 也用于详细日志(片选脚由 formatPin 格式化, 不硬编码)
    uint8_t    accelId = 0u;
    uint8_t    gyroId  = 0u;
    dev.readChipIds(accelId, gyroId);
    const bool accelOk = (accelId == 0x1E);
    const bool gyroOk  = (gyroId == 0x0F);

    if (!ok) {
        if (!accelOk) {
            rec.throws(ExceptionCode::IMU_CHIP_ID_ACCEL);
        }
        if (!gyroOk) {
            rec.throws(ExceptionCode::IMU_CHIP_ID_GYRO);
        }
        if (accelOk && gyroOk) {
            rec.throws(ExceptionCode::IMU_SELFTEST_FAILED); // ID 都对, 那就是自检/量程配置
        }
    }

    // 诊断: 无条件调用, 打不打由 logLine 自己决定(见 app_log.hpp)
    char csAccel[8];
    char csGyro[8];
    bootLogLine("[Dev ] BMI088 : 自检=%s (ID 校验 + 加速度计/陀螺自检) CS: 加速度计=%s 陀螺=%s\r\n",
            ok ? "OK" : "FAIL", formatPin(cfg::hw::imu::kAccelCs, csAccel, sizeof(csAccel)),
            formatPin(cfg::hw::imu::kGyroCs, csGyro, sizeof(csGyro)));
    bootLogLine("[Dev ] BMI088 : 回读 CHIP_ID 加速度计=0x%02X(期望0x1E) 陀螺=0x%02X(期望0x0F) -> %s\r\n",
            static_cast<unsigned>(accelId), static_cast<unsigned>(gyroId),
            (accelOk && gyroOk) ? "一致" : "不一致(接线/片选/供电?)");
    logSpiStatus("SPI1 IMU", SPI1, cfg::hw::imu::kSpi, apb2ClockHz());
    return ok;
}

} // namespace dlx

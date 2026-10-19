//===================================================================================================
// test/bmp388_error_provenance_test.cpp —— 证明 0x404(BARO_READ_FAILED) 不止"寄存器读失败"一个来源
//
// 背景: DL_LIB/BMP388/dlx_bmp388.hpp 里 BARO_READ_FAILED(0x404) 至少有三个来源:
//   (A) readBurst()  :331       —— 突发读 I2C 事务失败(超时), failStreak++ / staleStreak++
//   (B) read()       :513-515   —— 突发读**成功**但补偿结果非有限或压力 <= 0, failStreak **不变**
//   (C) tryInit()    :467/:472  —— 只在初始化回读里出现
// (另有 app/sensors/app_sensors.hpp:546 的"高度非有限"也会报同一个码, 与 I2C 完全无关。)
//
// 本测试用 test/bmp388_stub 的假 IIC(只按寄存器地址存取)驱动**真实的** dlx_bmp388.hpp, 断言:
//   tx_src  : 总线事务失败 -> Failed + err==0x404 + failCount()==1 + staleCount()==1
//   comp_src: 事务成功(读回 raw=0xFFFFFF)、但补偿出的压力 <= 0
//             -> Failed + err==0x404 + failCount()==0 + **staleCount()==0**
//   两条都成立 = "0x404 => 寄存器读取失败" 这个推断不成立。
//
// 编译(仓库根):
//   g++ -std=c++11 -Wall -Wextra -I test/bmp388_stub -I test/stub -I DL_LIB -I . `
//       test/bmp388_error_provenance_test.cpp -o $env:TEMP\bmp388_error_provenance_test.exe
//===================================================================================================
#include <math.h>
#include <stdio.h>
#include "BMP388/dlx_bmp388.hpp"

static unsigned checks   = 0u;
static unsigned failures = 0u;

#define CHECK(condition)                                        \
    do {                                                        \
        ++checks;                                               \
        if (!(condition)) {                                     \
            ++failures;                                         \
            printf("FAIL line %d: %s\n", __LINE__, #condition); \
        }                                                       \
    } while (false)

/**
 * 标定系数(0x31~0x45, 21 字节, 小端; 见 dlx_bmp388.hpp:194-215 的地址表)。
 * 取值刻意让补偿公式容易把压力算成负数, 同时又不是全 0/全 FF(readCalib 的有效性检查要能过):
 *   T1=1(0x0001) -> par_t1 = 256
 *   T2=1(0x0001) -> par_t2 = 2^-30
 *   P1=0x8000(-32768) -> par_p1 = (-32768-16384)/2^20 = -0.046875  <- 让 u*par_p1 变成大负数
 *   其余系数为 0
 */
static const uint8_t kCalib[21] = {
    0x01u, 0x00u, // 0x31 T1 = 1
    0x01u, 0x00u, // 0x33 T2 = 1
    0x00u,        // 0x35 T3 = 0
    0x00u, 0x80u, // 0x36 P1 = 0x8000 (-32768)
    0x00u, 0x40u, // 0x38 P2 = 0x4000 (-> par_p2 = 0)
    0x00u,        // 0x3A P3 = 0
    0x00u,        // 0x3B P4 = 0
    0x00u, 0x00u, // 0x3C P5 = 0
    0x00u, 0x00u, // 0x3E P6 = 0
    0x00u,        // 0x40 P7 = 0
    0x00u,        // 0x41 P8 = 0
    0x00u, 0x00u, // 0x42 P9 = 0
    0x00u,        // 0x44 P10 = 0
    0x00u,        // 0x45 P11 = 0
};

/** 让 tryInit() 能成功: CHIP_ID=0x50 + 有效标定系数(配置寄存器由驱动自己写、自己能回读) */
static void prepare(dlx::IICBus &bus)
{
    bus.registers[0x00] = dlx::BMP388_CHIP_ID_VALUE;
    for (unsigned index = 0u; index < sizeof(kCalib); ++index) {
        bus.registers[0x31u + index] = kCalib[index];
    }
}

int main()
{
    // =============================================================================================
    // 来源 A(tx 类): I2C 突发读失败 -> 0x404 且 failCount()/staleCount() 都 +1
    // =============================================================================================
    dlx::IICBus busA;
    prepare(busA);
    dlx::BMP388 baroA(busA);
    dlx::ExceptionCode errA = dlx::ExceptionCode::GNSS_NO_FIX;
    dlx::FunctionResult initA = baroA.tryInit(errA);
    CHECK(initA == dlx::FunctionResult::SUCCESS);
    CHECK(baroA.isReady());
    CHECK(baroA.failCount() == 0u);

    busA.registers[0x03] = dlx::BMP388_STATUS_DRDY_PRESS;
    busA.failRead        = true; // 突发读事务失败
    float pressureA      = 11.0f;
    float temperatureA   = 22.0f;
    const dlx::BMP388::ReadResult resultA = baroA.read(pressureA, temperatureA, errA);
    const bool txSourceOk = (resultA == dlx::BMP388::ReadResult::Failed) &&
                            (errA == dlx::ExceptionCode::BARO_READ_FAILED) &&
                            (baroA.failCount() == 1u) && (baroA.staleCount() == 1u);
    CHECK(resultA == dlx::BMP388::ReadResult::Failed);
    CHECK(errA == dlx::ExceptionCode::BARO_READ_FAILED);
    CHECK(baroA.failCount() == 1u);
    CHECK(baroA.staleCount() == 1u);
    printf("[tx_src]   failRead=true  -> result=Failed err=0x%04X failCount=%u staleCount=%u\n",
           (unsigned)errA,
           (unsigned)baroA.failCount(),
           (unsigned)baroA.staleCount());

    // =============================================================================================
    // 来源 B(comp 类): I2C 突发读**成功**, 但补偿结果压力 <= 0 -> 同一个 0x404, 且 failCount()==0
    // =============================================================================================
    dlx::IICBus busB;
    prepare(busB);
    dlx::BMP388 baroB(busB);
    dlx::ExceptionCode errB = dlx::ExceptionCode::GNSS_NO_FIX;
    CHECK(baroB.tryInit(errB) == dlx::FunctionResult::SUCCESS);
    CHECK(baroB.failCount() == 0u);

    busB.registers[0x03] = dlx::BMP388_STATUS_DRDY_PRESS; // DRDY: 有新样本
    for (uint8_t reg = 0x04u; reg <= 0x09u; ++reg) {
        busB.registers[reg] = 0xFFu; // pressure=0xFFFFFF, temperature=0xFFFFFF
    }

    // 先证明"这次事务是真的成功了": readRaw 走的是同一个 readBurst(), 返回 NewData 且拿到原始值
    uint32_t rawPressure    = 0u;
    uint32_t rawTemperature = 0u;
    dlx::ExceptionCode rawErr = dlx::ExceptionCode::GNSS_NO_FIX;
    const dlx::BMP388::ReadResult rawResult = baroB.readRaw(rawPressure, rawTemperature, rawErr);
    CHECK(rawResult == dlx::BMP388::ReadResult::NewData);
    CHECK(rawPressure == 0xFFFFFFu);
    CHECK(rawTemperature == 0xFFFFFFu);
    CHECK(baroB.failCount() == 0u);

    float pressureB    = 11.0f;
    float temperatureB = 22.0f;
    const dlx::BMP388::ReadResult resultB = baroB.read(pressureB, temperatureB, errB);
    const bool compSourceOk = (rawResult == dlx::BMP388::ReadResult::NewData) &&
                              (resultB == dlx::BMP388::ReadResult::Failed) &&
                              (errB == dlx::ExceptionCode::BARO_READ_FAILED) &&
                              (baroB.failCount() == 0u) && (baroB.staleCount() == 0u);
    CHECK(resultB == dlx::BMP388::ReadResult::Failed);
    CHECK(errB == dlx::ExceptionCode::BARO_READ_FAILED);
    CHECK(baroB.failCount() == 0u);  // <- 关键判别式: I2C 事务成功, failStreak 没有动
    CHECK(baroB.staleCount() == 0u);
    CHECK(pressureB == 11.0f && temperatureB == 22.0f); // 失败时不得写出不可信样本
    printf("[comp_src] readRaw=NewData(raw=0x%06X) 但补偿后 pressure<=0"
           " -> result=Failed err=0x%04X failCount=%u staleCount=%u\n",
           (unsigned)rawPressure,
           (unsigned)errB,
           (unsigned)baroB.failCount(),
           (unsigned)baroB.staleCount());

    // =============================================================================================
    printf("bmp388_error_provenance_test: tx_src=%s comp_src=%s\n",
           txSourceOk ? "ok" : "FAIL",
           compSourceOk ? "ok" : "FAIL");
    printf("  -> 0x404 至少有两个互斥的判别式: failCount()==1 = 总线事务失败; "
           "failCount()==0 = 事务成功但补偿结果非法(与寄存器读取无关)。\n");
    printf("  (%u checks, %u failures)\n", checks, failures);
    return failures == 0u ? 0 : 1;
}

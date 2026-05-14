// For test SD

#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_delay.hpp"
#include "dl_socket.hpp"
#include "dl_usart.hpp"
#include "global.h"
#include "dl_iic.hpp"
#include "dl_imu.hpp"
#include "DL_AHRS/MadgwickAHRS.h"
#include "dl_bme280.hpp"
#include "dl_bytebuffer.hpp"

inline void GPIOClockInit()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
}

inline void init()
{
    GPIOClockInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Config(SystemCoreClock / 1000);
    dl::delay_ms(100);
    dl::GPIO_Pin_Init(dGPIOFPin3, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    dl::GPIO_Pin_Init(dGPIOFPin4, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    dl::GPIO_Pin_Init(dGPIOFPin5, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    q0 = 0;
    q1 = -0.77;
    q2 = -0.63;
    q3 = 0;
}
// TX:PA9

static uint8_t rx_buf[256];

#include "dl_log.h"
#include "dl_message.hpp"
#include "SDIO/stm324x7i_eval_sdio_sd.h"
#include "Fatfs/ff.h"
#include "dl_config.hpp"
#include "dl_error.hpp"
#include "dl_pid.hpp"
#include "dl_dshot.hpp"
dl::IMUGData mpu9250_data;

dl::BME280::bmeData bmeData;

volatile float roll, pitch, yaw;

uint8_t *XmmsgBuf = new uint8_t[512];
uint8_t *XrmsgBuf = new uint8_t[512];

dl::ByteBuffer msgBuf(XmmsgBuf, 512);
dl::ByteBuffer rmsgBuf(XrmsgBuf, 512);

void lamp(uint8_t s)
{
    while (true) {
        dPF3 = 1;
        dl::delay_ms(s);
        dPF3 = 0;
        dPF4 = 1;
        dl::delay_ms(s);
        dPF4 = 0;
    }
}

int main()
{
    uint8_t *uniBuf = new uint8_t[128];
    dl::MessageManager mm(msgBuf);

    bool testSensor  = false;
    bool rq          = false;
    bool delayStart  = false;
    bool enableDshot = false;
    init();
    dl::USART usart1X(dUSART1_tA9_rA10, rx_buf, sizeof(rx_buf), 1, 1, 1520000);
    dl::Socket socket(usart1X);

    log_func = [&socket, &mm](const char *msg) {
        mm.logMsg((uint8_t *)msg, strlen(msg));
        socket.sendData(msgBuf);
    };

    dl::IIC iicBus2(dIIC2_cB10_dB11);
    // dl::IIC iicBus1(dIIC1_cA6_dA7);

    dl::MPU9250 mpu9250(MPU9250_ADDRESS_AD0_LOW, &mpu9250_data);

    dPF3 = 1;
    mpu9250.init(512, &iicBus2);
    dPF3 = 0;

    socket.ASyncRead(rmsgBuf.src, 4); // 绕过wrapper

    mm.initMsg();

    mm.requestBooleanParamMsg(strWithLen("isUnlockRequired"));

    mm.requestBooleanParamMsg(strWithLen("isTestSensor"));

    mm.requestBooleanParamMsg(strWithLen("isDelayStart"));

    mm.requestBooleanParamMsg(strWithLen("isEnableDshot"));

    socket.sendData(msgBuf);

    socket.ASyncWait();

    rq          = rmsgBuf.read<uint8_t>() > 0;
    testSensor  = rmsgBuf.read<uint8_t>() > 0;
    delayStart  = rmsgBuf.read<uint8_t>() > 0;
    enableDshot = rmsgBuf.read<uint8_t>() > 0;

    if (delayStart) {
        dl::delay_ms(3000);
    }

    logger << "isUnlockRequired: " << rq << LCMD::NFLUSH;
    if (rq) {
        dPF4 = 1;
        socket.ASyncRead(rmsgBuf.src, 4);
        mm.syncMsg();
        socket.sendData(msgBuf);
        socket.ASyncWait();
        if (rmsgBuf.read<uint32_t>() != 721) {
            // error
        }
        dPF4 = 0;
    }
    rmsgBuf.reset();

    socket.ASyncRead(rmsgBuf.src, 16);
    mm.requestIntParamMsg((uint8_t *)"DshotStartMotorValue", strlen("DshotStartMotorValue"));
    mm.requestIntParamMsg(strWithLen("lampTime"));
    mm.requestIntParamMsg((uint8_t *)"DshotStartDelay", strlen("DshotStartDelay"));
    mm.requestIntParamMsg((uint8_t *)"DshotRunningDelay", strlen("DshotRunningDelay"));
    mm.requestIntParamMsg(strWithLen("targetPitch"));
    socket.sendData(msgBuf);
    socket.ASyncWait();
    uint16_t dvalue[4];
    logger << "DshotStartMotorValue: " << rmsgBuf.read<int>();
    uint8_t lt= (uint8_t) rmsgBuf.read<int>();
    logger << "lampTime: " << lt << LCMD::NFLUSH;
    dl::DShot ds(dl::DSHOT_TIM::DSHOT_TIM1, dl::DSHOT_RATE::DSHOT_300,(uint8_t) rmsgBuf.read<int>(),(uint8_t) rmsgBuf.read<int>());

    if (enableDshot) {
        ds.start();
    }
    logger << "initMotorValue: " << dvalue[0] << LCMD::NFLUSH;

    ds.encodePreLoadDshotData(dvalue);
    
    // dl::BME280::bme280_init(&iicBus1);
    uint32_t t = SystemClockMilliseconds;
    dPF3       = 1;
    while (SystemClockMilliseconds - t < 10000 || testSensor) {
        uint32_t tt = SystemClockMilliseconds;
        mpu9250.read();
        MadgwickAHRSupdate(mpu9250_data.gyro[0] / 57.29f, mpu9250_data.gyro[1] / 57.29f, mpu9250_data.gyro[2] / 57.29f, mpu9250_data.accel[0], mpu9250_data.accel[1], mpu9250_data.accel[2], mpu9250_data.mag[0], mpu9250_data.mag[1], mpu9250_data.mag[2]);
        sampleFreq = (1000.0f / (float)(SystemClockMilliseconds - tt));
        if (SystemClockMilliseconds - tt < 10) {
            dl::delay_ms(10 - SystemClockMilliseconds + tt);
        }
        convertQuantToEuler();
        logger << "YPR: " << yaw << " " << pitch << " " << roll << " ";
        logger << "Initing AHRS " << SystemClockMilliseconds - t << LCMD::NFLUSH;
    }
    dPF3         = 0;
    target_pitch = rmsgBuf.read<int>();
#if 0
#endif
    while (1) {
        uint32_t tt = SystemClockMilliseconds;
        mpu9250.read();
        MadgwickAHRSupdate(mpu9250_data.gyro[0] / 57.29f, mpu9250_data.gyro[1] / 57.29f, mpu9250_data.gyro[2] / 57.29f, mpu9250_data.accel[0], mpu9250_data.accel[1], mpu9250_data.accel[2], mpu9250_data.mag[0], mpu9250_data.mag[1], mpu9250_data.mag[2]);
        //dl::BME280::bme280_read(&iicBus1, &bmeData);

        convertQuantToEuler();
        PIDRateControl();
        PIDAngleControl(&ds);
        
        // logger << "YPR: " << yaw << " " << pitch << " " << roll << " " << LCMD::NFLUSH;
        // logger << "updatePID throttles: " << (uint32_t)throttleValueF[0] << " " << (uint32_t)throttleValueF[1] << " " << (uint32_t)throttleValueF[2] << " " << (uint32_t)throttleValueF[3] << LCMD::NFLUSH;
        mm.motorMsg(throttleValue[0], throttleValue[1], throttleValue[2], throttleValue[3]);
        // mm.b280Msg(bmeData.temperature, bmeData.humidity, bmeData.pressure);
        mm.posMsg(q0, q1, q2, q3, mpu9250_data.gyro[0], mpu9250_data.gyro[1], mpu9250_data.gyro[2], mpu9250_data.accel[0], mpu9250_data.accel[1], mpu9250_data.accel[2], mpu9250_data.mag[0], mpu9250_data.mag[1], mpu9250_data.mag[2]);
        socket.sendData(msgBuf);

        // 频率修正
        sampleFreq = (1000.0f / (float)(SystemClockMilliseconds - tt));
        if (SystemClockMilliseconds - tt < 10) {
            dl::delay_ms(10 - SystemClockMilliseconds + tt);
        }
    }

    lamp(lt);
}
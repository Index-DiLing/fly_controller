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

    dl::MessageManager mm(msgBuf);

    init();
    dl::USART usart1X(dUSART1_tA9_rA10, rx_buf, sizeof(rx_buf), 1, 1, 1520000);
    dl::Socket socket(usart1X);

    log_func = [&socket, &mm](const char *msg) {
        mm.logMsg((uint8_t *)msg, strlen(msg));
        socket.sendData(msgBuf);
    };

    dl::IIC iicBus2(dIIC2_cB10_dB11);

    dl::MPU9250 mpu9250(MPU9250_ADDRESS_AD0_LOW, &mpu9250_data);

    dPF3 = 1;
    mpu9250.init(512, &iicBus2);
    dPF3 = 0;

    socket.ASyncRead(rmsgBuf.src, 20);

    mm.initMsg();

    mm.requestIntParamMsg((uint8_t *)"DshotStartMotorValue", strlen("DshotStartMotorValue"));


    mm.requestIntParamMsg(strWithLen("lampTime"));

    mm.requestIntParamMsg((uint8_t *)"DshotStartDelay", strlen("DshotStartDelay"));

    mm.requestIntParamMsg((uint8_t *)"DshotRunningDelay", strlen("DshotRunningDelay"));
    
    mm.requestIntParamMsg(strWithLen("DshotInitTime"));

    socket.sendData(msgBuf);

    socket.ASyncWait();

    uint16_t dvalue[4];

    uint16_t dv = rmsgBuf.read<int>();

    uint8_t lt = rmsgBuf.read<int>();

    uint8_t iDelay = rmsgBuf.read<int>();

    uint8_t rDelay = rmsgBuf.read<int>();

    uint16_t initTime = rmsgBuf.read<int>();

    dvalue[0] = dv;
    dvalue[1] = dv;
    dvalue[2] = dv;
    dvalue[3] = dv;

    logger << "lampTime: " << lt << LCMD::NFLUSH;

    logger << "DshotInitTime: " << initTime << LCMD::NFLUSH;

    logger << "initMotorValue: " << dvalue[2] << LCMD::NFLUSH;

    dl::DShot ds(dl::DSHOT_TIM::DSHOT_TIM1, dl::DSHOT_RATE::DSHOT_300, iDelay, rDelay);

    // 启动dshot;
    ds.start(initTime);

    ds.encodePreLoadDshotData(dvalue);

    uint32_t t = SystemClockMilliseconds;

    while (true) {

        dPF3        = 1;
        uint32_t tt = SystemClockMilliseconds;
        mpu9250.read();

        MadgwickAHRSupdate(mpu9250_data.gyro[0] / 57.29f, mpu9250_data.gyro[1] / 57.29f, mpu9250_data.gyro[2] / 57.29f, mpu9250_data.accel[0], mpu9250_data.accel[1], mpu9250_data.accel[2], mpu9250_data.mag[0], mpu9250_data.mag[1], mpu9250_data.mag[2]);
        sampleFreq = (1000.0f / (float)(SystemClockMilliseconds - tt));
        convertQuantToEuler();
        mm.posMsg(q0, q1, q2, q3, mpu9250_data.gyro[0], mpu9250_data.gyro[1], mpu9250_data.gyro[2], mpu9250_data.accel[0], mpu9250_data.accel[1], mpu9250_data.accel[2], mpu9250_data.mag[0], mpu9250_data.mag[1], mpu9250_data.mag[2]);
        if (SystemClockMilliseconds - tt < 10) {
            dl::delay_ms(10 - SystemClockMilliseconds + tt);
        }
        socket.sendData(msgBuf);

        dPF3 = 0;
    }
    lamp(lt);
}
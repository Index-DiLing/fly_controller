
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "stdio.h"
#include "dlx.hpp"
#include "rtthread.h"
#include "BMI088/dlx_bmi088.hpp"
#include "DL_AHRS/MadgwickAHRS.hpp"
#include "NRF24L01/dlx_nrf24l01.hpp"
using namespace dlx;

char g_fixed_str_buffer[128];
#define gfsb (int8_t *)g_fixed_str_buffer, (uint16_t)strlen(g_fixed_str_buffer)
int fixed_sprintf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int ret = vsnprintf(g_fixed_str_buffer, 128, format, args);
    va_end(args);
    return ret;
}

int main()
{

    DLX_NVIC_AutoConfig();

    GPIO pf3(GPIOProfile::F3);

    pf3.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);

    pf3 = 0;

    uint8_t rxBuffer[256];

    ByteBuffer rxBuf(rxBuffer, 256);

    RingByteBuffer buf(rxBuf);

    auto usart1 = USART::USART1_TA9_RAA(); // 自动配置 A9/A10(AA) 为 USART1 的 AF 引脚

    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, buf);

    uint8_t dmaB[128];

    ByteBuffer dmaBuffer(dmaB, 128);

    uint8_t fr[128];
    ByteBuffer frame(fr, 128);
    DLX_ProtocolBuffer protocol(dmaBuffer, &buf, &frame);

    bool atom = true;

    protocol.setUnBlockCallbackFunction(+[](UnBlock *, void *at) {
        bool* p = static_cast<bool*>(at);
        *p = false; }, &atom);
    while (atom) {
        if (protocol.check()) {
            pf3 = 1;
        }
    }

    auto spi = SPI::SPI1_SA5_MIA6_MOA7(GPIOProfile::A3);

    spi.init(SPIModeProfile::FD_M_8B_CH_2E_NS_BR256_MSB);

    auto bmi = BMI088(spi, GPIOProfile::BC, GPIOProfile::BD);

    uint8_t self = bmi.init();

    auto mutex = rt_sem_create("sensor", 0, RT_IPC_FLAG_PRIO);

    auto timer = rt_timer_create("imut", +[](void *mut) { rt_sem_release(static_cast<rt_sem_t>(mut)); }, mutex, 50, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);

    uint8_t acbb[256];
    uint8_t gybb[256];
    ByteBuffer acb(acbb, 256);
    ByteBuffer gyb(gybb, 256);

    auto ac = Queue<AccelerometerRaw>(acb);
    auto gy = Queue<GyroscopeRaw>(gyb);

    fixed_sprintf("Hello,DLX\n");

    protocol.SimpleLogW(gfsb);

    auto serialDMA = usart1.setDMASend(dmaBuffer);

    MadgwickAHRS ahrs(200,0.078);

    // ------------------------------------------------------------------
    // NRF24L01 链路(新增): SPI3 挂 PC10(SCK)/PC11(MISO)/PC12(MOSI),
    // CSN = PA4(软件片选), CE = PC7, IRQ = PC8.
    // 本端参数已与遥控器端对齐(见 dlx_nrf24l01_config.h):
    //   信道 40 / 2Mbps+0dBm+LNA / 通道0 自动应答 / 500us+5 次重发 / 32B 载荷 / 4ms 超时.
    // SPI 使用模式 0(CPOL=0, CPHA=0), MSB 在前.
    // ------------------------------------------------------------------
    auto spi3 = SPI::SPI3_SCA_MICB_MOCC(GPIOProfile::A4);
    spi3.init(SPIModeProfile::FD_M_8B_CL_1E_NS_BR8_MSB);

    uint8_t nrfRxRaw[128];                       // NRF 接收环形缓冲(可容约 4 包)
    ByteBuffer nrfRxByteBuf(nrfRxRaw, sizeof(nrfRxRaw));
    NRF24L01 nrf(spi3, GPIOProfile::C7, GPIOProfile::C8, nrfRxByteBuf);
    nrf.init();

    // 上行(发)缓冲: FlightCoreStatus 帧 = 2B 头 + 30B 载荷 = 32B
    uint8_t nrfTxRaw[32];
    ByteBuffer nrfTxByteBuf(nrfTxRaw, sizeof(nrfTxRaw));
    DLX_ProtocolBuffer nrfTxProtocol(nrfTxByteBuf);

    // 下行(收)进行 DLX 解析: 收到 ControlToFlight -> 触发回调
    uint8_t nrfDummyRaw[64];                     // CPU 不复用, 仅占位 buffer 成员
    ByteBuffer nrfDummyByteBuf(nrfDummyRaw, sizeof(nrfDummyRaw));
    uint8_t nrfFrRaw[32];
    ByteBuffer nrfFrame(nrfFrRaw, sizeof(nrfFrRaw));
    uint8_t nrfRingRaw[256];
    ByteBuffer nrfRingBuf(nrfRingRaw, sizeof(nrfRingRaw));
    RingByteBuffer nrfRing(nrfRingBuf);
    DLX_ProtocolBuffer nrfRxProtocol(nrfDummyByteBuf, &nrfRing, &nrfFrame);

    // 遥控器控制数据暂不参与控制, 仅用 PF5 灯指示"收到一帧控制数据".
    GPIO ledPF5(GPIOProfile::F5);
    ledPF5.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    ledPF5 = 0;
    // PF4 灯指示"已向遥控器回传一帧姿态".
    GPIO ledPF4(GPIOProfile::F4);
    ledPF4.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    ledPF4 = 0;
    nrfRxProtocol.setControlToFlightCallbackFunction(
        +[](ControlToFlight *ctl, void *ctx) {
            // 需要解算时可用 ctl->targetPitch()/targetRoll()/throttle() 等字段
            static bool on = false;
            on = !on;
            *static_cast<GPIO *>(ctx) = on;
        },
        &ledPF5);

    rt_timer_start(timer);
    while (true) {
        rt_sem_take(mutex, RT_WAITING_FOREVER);

        bmi.fifoRead(ac, gy);
        dmaBuffer.reset();
        auto l = fixed_sprintf("size:%u,%u", ac.size(), gy.size());
        
        while (ac.size() > 0 && gy.size()>0)
        {
            AccelerometerRaw acr;
            ac.pop(acr);

            // protocol.AccelRawW(acr.data);

            GyroscopeRaw gyr;
            gy.pop(gyr);

            // protocol.GyroRawW(gyr.data);
            
            ahrs.MadgwickAHRSupdateIMU(StandardIMUGRads{
                .accel = BMI088::getAccelerationG(acr),
                .gyro = BMI088::getAngularVelocityRads(gyr)
            });
        }
        ac.clear();
        gy.clear();

        // ---- 通过 NRF 回传姿态(FlightCoreStatus, 类型 10) ----
        Quaternion quat = ahrs.getQuaternion();
        nrfTxByteBuf.reset();
        if (nrfTxProtocol.FlightCoreStatusW(
                quat.data[0], quat.data[1], quat.data[2], quat.data[3],
                0, 0, 0, 0,          // motor0..3(本测试未驱动电机)
                0.0f,                // height(本测试未估计高度)
                0)) {                // error
            // 阻塞等待发送完成(自动应答), 之后再回到接收模式
            if (nrf.write(nrfTxByteBuf.src, nrfTxByteBuf.used(),
                          NRF24L01_WAIT_TIMEOUT_MS)) {
                static bool txOn = false;
                txOn = !txOn;
                ledPF4 = txOn;
            }
        }

        // ---- 通过 NRF 接收遥控器控制帧(ControlToFlight, 类型 9) ----
        // 先处理一次接收(内部会 drain 当前 FIFO), 再把整包喂给 DLX 解析器.
        uint8_t nrfPkt[NRF24L01_RX_PACKET_SIZE];
        if (nrf.read(nrfPkt, NRF24L01_RX_PACKET_SIZE, NRF24L01_WAIT_TIMEOUT_MS)) {
            nrfRing.write(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            while (nrf.available() >= NRF24L01_RX_PACKET_SIZE) {
                if (!nrf.read(nrfPkt, NRF24L01_RX_PACKET_SIZE, 0)) {
                    break;
                }
                nrfRing.write(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            }
        }
        while (nrfRing.available() >= NRF24L01_RX_PACKET_SIZE) {
            const uint16_t before = nrfRing.available();
            bool consumed = nrfRxProtocol.check(); // 命中后调用 ControlToFlight 回调
            if (!consumed && nrfRing.available() == before) {
                // 无法解析(未知帧类型)且未被 check() 消费: 整包丢弃, 保持 32B 对齐
                nrfRing.read(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            }
        }
        
        protocol.SimpleLogW(gfsb);
        protocol.QuatW(ahrs.getQuaternion().data);

        serialDMA.reset(protocol.buffer.used());

        serialDMA.start();

        serialDMA.wait();
    }


    while (true) {
        serialDMA.start();
        serialDMA.wait();
        serialDMA.reset();
    }
}

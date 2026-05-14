#pragma once

#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_dma.hpp"
namespace dl
{

    struct __USART_Type {
        USART_TypeDef *usart;

        uint8_t usartIRQn;
        uint32_t usartAPBPeriph;

        GPIO_Pin_Type txPin;
        GPIO_Pin_Type rxPin;
        uint16_t GPIO_PinSource_TX;
        uint16_t GPIO_PinSource_RX;
        uint8_t GPIO_AF;

        uint32_t rtxGPIOAHBPeriph;
        const dl::DMAConfig dmaConfig;
    };

#define dUSART1_tA9_rA10 \
    dl::__USART_Type     \
    { USART1, USART1_IRQn, RCC_APB2Periph_USART1, dPA9, dPA10, GPIO_PinSource9, GPIO_PinSource10, GPIO_AF_USART1, RCC_AHB1Periph_GPIOA, dl::dDMA_USART1_RX }
#define dUSART1_tB6_rB7 \
    dl::__USART_Type    \
    { USART1, USART1_IRQn, RCC_APB2Periph_USART1, dPB6, dPB7, GPIO_PinSource6, GPIO_PinSource7, GPIO_AF_USART1, RCC_AHB1Periph_GPIOB, dl::dDMA_USART1_RX }

#define dUSART2_tA2_rA3 \
    dl::__USART_Type    \
    { USART2, USART2_IRQn, RCC_APB1Periph_USART2, dPA2, dPA3, GPIO_PinSource2, GPIO_PinSource3, GPIO_AF_USART2, RCC_AHB1Periph_GPIOA }
#define dUSART2_tD5_rD6 \
    dl::__USART_Type    \
    { USART2, USART2_IRQn, RCC_APB1Periph_USART2, dPD5, dPD6, GPIO_PinSource5, GPIO_PinSource6, GPIO_AF_USART2, RCC_AHB1Periph_GPIOD }

#define dUSART3_tB10_rB11 \
    dl::__USART_Type      \
    { USART3, USART3_IRQn, RCC_APB1Periph_USART3, dPB10, dPB11, GPIO_PinSource10, GPIO_PinSource11, GPIO_AF_USART3, RCC_AHB1Periph_GPIOB }
#define dUSART3_tC10_rC11 \
    dl::__USART_Type      \
    { USART3, USART3_IRQn, RCC_APB1Periph_USART3, dPC10, dPC11, GPIO_PinSource10, GPIO_PinSource11 GPIO_AF_USART3, RCC_AHB1Periph_GPIOC }

#define dUSART6_tC6_rC7 \
    dl::__USART_Type    \
    { USART6, USART6_IRQn, RCC_APB2Periph_USART6, dPC6, dPC7, GPIO_PinSource6, GPIO_PinSource7, GPIO_AF_USART6, RCC_AHB1Periph_GPIOC }
#define dUSART6_tG14_rG9 \
    dl::__USART_Type     \
    { USART6, USART6_IRQn, RCC_APB2Periph_USART6, dPG14, dPG9, GPIO_PinSource14, GPIO_PinSource9, GPIO_AF_USART6, RCC_AHB1Periph_GPIOG }

    class USART
    {
    private:
        USART_TypeDef *usart;
        uint8_t *rx_buf;
        uint16_t rx_p      = 0;
        uint16_t rx_size   = 0;
        bool isDMATransfer = false;

        dl::DMA RxDMA;

        /**
         * @brief 这是串口
         *
         */
    public:
        USART(__USART_Type usartType,
              uint8_t *rx_buf_ptr,                                          // 接收缓冲区指针
              uint16_t rx_buf_size,                                         // 接收缓冲区大小
              uint8_t NVIC_IRQChannelPreemptionPriority,                    // 中断抢占优先级
              uint8_t NVIC_IRQChannelSubPriority,                           // 中断子优先级
              uint32_t baudRate,                                            // 波特率
              uint16_t wordLength          = USART_WordLength_8b,           // 字长
              uint16_t stopBits            = USART_StopBits_1,              // 停止位
              uint16_t parity              = USART_Parity_No,               // 校验
              uint16_t mode                = USART_Mode_Rx | USART_Mode_Tx, // 模式
              uint16_t hardwareFlowControl = USART_HardwareFlowControl_None // 硬件流控
              ) : RxDMA(DMA(usartType.dmaConfig, 0, nullptr))
        {
            usart   = usartType.usart;
            rx_size = rx_buf_size;
            rx_buf = rx_buf_ptr;

            RCC_AHB1PeriphClockCmd(usartType.rtxGPIOAHBPeriph, ENABLE);
            RCC_APB2PeriphClockCmd(usartType.usartAPBPeriph, ENABLE);

            GPIO_InitTypeDef gpio_init{
                .GPIO_Pin   = usartType.txPin.pin,
                .GPIO_Mode  = GPIO_Mode_AF,
                .GPIO_Speed = GPIO_Speed_50MHz,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_PuPd  = GPIO_PuPd_UP};
            GPIO_Init(usartType.txPin.gpiox, &gpio_init);

            gpio_init.GPIO_Pin  = usartType.rxPin.pin;
            gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
            GPIO_Init(usartType.rxPin.gpiox, &gpio_init);

            GPIO_PinAFConfig(usartType.txPin.gpiox, usartType.GPIO_PinSource_TX, usartType.GPIO_AF);
            GPIO_PinAFConfig(usartType.rxPin.gpiox, usartType.GPIO_PinSource_RX, usartType.GPIO_AF);
            USART_InitTypeDef USART_InitStructure;
            USART_InitTypeDef usart_init = USART_InitTypeDef{
                .USART_BaudRate            = baudRate,
                .USART_WordLength          = wordLength,
                .USART_StopBits            = stopBits,
                .USART_Parity              = parity,
                .USART_Mode                = mode,
                .USART_HardwareFlowControl = hardwareFlowControl};
            USART_Init(usartType.usart, &usart_init);
            USART_ITConfig(usartType.usart, USART_IT_RXNE, ENABLE);
            dl::NVIC_IT_init_plus(usartType.usartIRQn, NVIC_IRQChannelPreemptionPriority, NVIC_IRQChannelSubPriority, [this]() {
                if (USART_GetITStatus(usart, USART_IT_RXNE) && rx_p < rx_size) {
                    rx_buf[rx_p++] = USART_ReceiveData(usart);
                }
            });

            USART_Cmd(usartType.usart, ENABLE);
        }
        void send(uint8_t data)
        {
            USART_SendData(usart, data);
            while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET);
        }
        void send(uint8_t *data, uint16_t size)
        {
            for (uint16_t i = 0; i < size; i++) {
                USART_SendData(usart, data[i]);
                while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET);
            }
        }

        inline bool isRxDataReady()
        {
            return rx_p == rx_size;
        }
        inline void loadRxBuf(void *buf, uint16_t size)
        {
            if (!isRxDataReady()) {
                // error
            }
            rx_buf = (uint8_t *)buf;
            rx_p    = 0;
            rx_size = size;
        }

        void forceReloadRxBuf()
        {
            rx_p    = 0;
            rx_size = 0;
        }

        // 读取一个字节
        uint16_t read()
        {
            uint8_t data = 0;
            loadRxBuf(&data, 1);
            while (!isRxDataReady());
            return data;
        }
        uint16_t read(uint8_t *data, uint16_t size)
        {
            if (isDMATransfer) {
                USART_DMACmd(usart, USART_DMAReq_Rx, ENABLE);
                RxDMA.reset(size, (uint32_t *)data);
                RxDMA.start();
                RxDMA.wait();
                USART_DMACmd(usart, USART_DMAReq_Rx, DISABLE);
            } else {
                loadRxBuf(data, size);
                while (!isRxDataReady()){
                    
                }
            }
            return 0;
        }
        void ASyncRead(uint8_t *data, uint16_t size)
        {
            if (isDMATransfer) {
                USART_DMACmd(usart, USART_DMAReq_Rx, ENABLE);
                RxDMA.reset(size, (uint32_t *)data);
                RxDMA.start();
            } else {
                loadRxBuf(data, size);
            }
        }

        void ASyncReadWait()
        {
            if (isDMATransfer) {
                RxDMA.wait();
                USART_DMACmd(usart, USART_DMAReq_Rx, DISABLE);
            } else {
            while (!isRxDataReady());
            }
        }
        void enableDMA()
        {
            isDMATransfer = true;
            USART_ITConfig(usart, USART_IT_RXNE, DISABLE);
        }
        void disableDMA()
        {
            isDMATransfer = false;
            USART_ITConfig(usart, USART_IT_RXNE, ENABLE);
            RxDMA.stop();
        }
    };
} // namespace dl

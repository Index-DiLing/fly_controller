#pragma once
#include "stm32f4xx.h"
#include "dl_nvic_it.h"
#include <functional>

namespace dl
{
    struct DMAConfig {
        DMA_Stream_TypeDef *DMAx_Stream;
        uint32_t DMA_FLAG;
        uint32_t _DMA_Channel;
        uint32_t _DMA_PeripheralBaseAddr;
        uint32_t _DMA_DIR;
        uint32_t _DMA_PeripheralInc;
        uint32_t _DMA_MemoryInc;
        uint32_t _DMA_PeripheralDataSize;
        uint32_t _DMA_MemoryDataSize;
        uint32_t _DMA_Mode;
        uint32_t _DMA_Priority;
        uint32_t _DMA_FIFOMode;
        uint32_t _DMA_FIFOThreshold;
        uint32_t _DMA_MemoryBurst;
        uint32_t _DMA_PeripheralBurst;
        IRQn DMA_IT_IRQn;
        uint32_t DMA_IT;
        uint32_t DMA_ITIFx;
    };

    //DMA2_Stream5已经被TIM1优先占用
    //DMA2_Stream6被SDIO优先占用
    const DMAConfig dDMA_USART1_RX{
        .DMAx_Stream = DMA2_Stream2,
        .DMA_FLAG =DMA_FLAG_TCIF2,
        ._DMA_Channel = DMA_Channel_4,
        ._DMA_PeripheralBaseAddr = (uint32_t)(USART1->DR),
        ._DMA_DIR = DMA_DIR_PeripheralToMemory,
        ._DMA_PeripheralInc = DMA_PeripheralInc_Disable,
        ._DMA_MemoryInc = DMA_MemoryInc_Enable,
        ._DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
        ._DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
        ._DMA_Mode = DMA_Mode_Normal,
        ._DMA_Priority = DMA_Priority_High,
        ._DMA_FIFOMode = DMA_FIFOMode_Disable,
        ._DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull,
        ._DMA_MemoryBurst = DMA_MemoryBurst_Single,
        ._DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
        .DMA_IT_IRQn = DMA2_Stream3_IRQn,
        .DMA_IT = DMA_IT_TC,
        .DMA_ITIFx = DMA_IT_TCIF0,
    };

    class DMA
    {
    private:
        /* data */
        DMA_Stream_TypeDef *dmax_streamx;
        uint32_t dma_flagx;

    public:
        DMA(
            DMA_Stream_TypeDef *DMAx_Stream,
            uint32_t DMA_FLAG,
            uint32_t _DMA_Channel,
            uint32_t _DMA_PeripheralBaseAddr,
            uint32_t _DMA_Memory0BaseAddr,
            uint32_t _DMA_DIR,
            uint32_t _DMA_BufferSize,
            uint32_t _DMA_PeripheralInc,
            uint32_t _DMA_MemoryInc,
            uint32_t _DMA_PeripheralDataSize,
            uint32_t _DMA_MemoryDataSize,
            uint32_t _DMA_Mode,
            uint32_t _DMA_Priority,
            uint32_t _DMA_FIFOMode,
            uint32_t _DMA_FIFOThreshold,
            uint32_t _DMA_MemoryBurst,
            uint32_t _DMA_PeripheralBurst);
        DMA(const DMAConfig &config, uint32_t datasize, void *p);
        DMA(const DMAConfig &config, uint32_t datasize, void *p,std::function<void(DMA_Stream_TypeDef *)> callback);
        ~DMA();
        void start();
        void wait();
        void reset(uint32_t datasize, uint32_t *p);
        void setInterrupt(std::function<void(DMA_Stream_TypeDef *)> callback, IRQn DMA_IT_FLAGn, uint32_t DMA_IT, uint32_t DMA_ITIFx);
        void stop();
        void setDoubleBuffer(uint8_t* secondBuf);
        bool isTransferingFirstBuffer();
    };

    DMA::DMA(
        DMA_Stream_TypeDef *DMAx_Stream,  // 流选择
        uint32_t DMA_FLAG,                // 标志选择
        uint32_t _DMA_Channel,            // 通道选择
        uint32_t _DMA_PeripheralBaseAddr, // 外设地址
        uint32_t _DMA_Memory0BaseAddr,    // 内存地址
        uint32_t _DMA_DIR,                // DMA方向
        uint32_t _DMA_BufferSize,         // 传输数据量
        uint32_t _DMA_PeripheralInc,      // 外设地址递增
        uint32_t _DMA_MemoryInc,          // 内存地址递增
        uint32_t _DMA_PeripheralDataSize, // 外设数据宽度
        uint32_t _DMA_MemoryDataSize,     // 内存数据宽度
        uint32_t _DMA_Mode,               // 工作模式
        uint32_t _DMA_Priority,           // 优先级
        uint32_t _DMA_FIFOMode,
        uint32_t _DMA_FIFOThreshold,
        uint32_t _DMA_MemoryBurst,
        uint32_t _DMA_PeripheralBurst)
    {

        if ((uint32_t)DMAx_Stream > DMA2_BASE) // 这是DMA2
        {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
        } else {
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        }

        dmax_streamx = DMAx_Stream;
        dma_flagx    = DMA_FLAG;
        DMA_InitTypeDef init{
            .DMA_Channel            = _DMA_Channel,
            .DMA_PeripheralBaseAddr = _DMA_PeripheralBaseAddr,
            .DMA_Memory0BaseAddr    = _DMA_Memory0BaseAddr,
            .DMA_DIR                = _DMA_DIR,
            .DMA_BufferSize         = _DMA_BufferSize,
            .DMA_PeripheralInc      = _DMA_PeripheralInc,
            .DMA_MemoryInc          = _DMA_MemoryInc,
            .DMA_PeripheralDataSize = _DMA_PeripheralDataSize,
            .DMA_MemoryDataSize     = _DMA_MemoryDataSize,
            .DMA_Mode               = _DMA_Mode,
            .DMA_Priority           = _DMA_Priority,
            .DMA_FIFOMode           = _DMA_FIFOMode,
            .DMA_FIFOThreshold      = _DMA_FIFOThreshold,
            .DMA_MemoryBurst        = _DMA_MemoryBurst,
            .DMA_PeripheralBurst    = _DMA_PeripheralBurst};
        DMA_Init(DMAx_Stream, &init);
        DMA_ClearFlag(DMAx_Stream, DMA_FLAG); // 清除标记
    }

    DMA::DMA(const DMAConfig &config, uint32_t datasize, void *p)
    :DMA(config.DMAx_Stream,config.DMA_FLAG,config._DMA_Channel,config._DMA_PeripheralBaseAddr, (uint32_t)p,config._DMA_DIR,datasize,config._DMA_PeripheralInc,config._DMA_MemoryInc,config._DMA_PeripheralDataSize,config._DMA_MemoryDataSize,config._DMA_Mode,config._DMA_Priority,config._DMA_FIFOMode,config._DMA_FIFOThreshold,config._DMA_MemoryBurst,config._DMA_PeripheralBurst)
    {


    }

    DMA::DMA(const DMAConfig &config, uint32_t datasize, void *p, std::function<void(DMA_Stream_TypeDef *)> callback):DMA(config,datasize,p)
    {
        setInterrupt(callback, config.DMA_IT_IRQn, config.DMA_IT, config.DMA_ITIFx);
    }

    void DMA::start()
    {
        DMA_Cmd(dmax_streamx, ENABLE);
    }
    void DMA::wait()
    {
        while (!DMA_GetFlagStatus(dmax_streamx, dma_flagx));
    }

    /**
     * @brief 设置DMA中断
     */
    void DMA::setInterrupt(std::function<void(DMA_Stream_TypeDef *)> callback, IRQn DMA_IT_IRQn, uint32_t DMA_IT, uint32_t DMA_ITIFx)
    {
        DMA_ITConfig(dmax_streamx, DMA_IT, ENABLE);
        auto streamx = dmax_streamx;
        dl::NVIC_IT_init_plus(DMA_IT_IRQn, 1, 1, [callback, streamx, DMA_ITIFx]() {
            if (DMA_GetITStatus(streamx, DMA_ITIFx)) {
                callback(streamx);
            }
        });
    }
    void DMA::stop()
    {
        DMA_Cmd(dmax_streamx, DISABLE);
    }
    DMA::~DMA()
    {
    }
    void DMA::reset(uint32_t datasize, uint32_t *p)
    {
        DMA_Cmd(dmax_streamx, DISABLE);
        while (dmax_streamx->CR & DMA_SxCR_EN);
        DMA_ClearFlag(dmax_streamx, dma_flagx);
        DMA_SetCurrDataCounter(dmax_streamx, datasize);
        dmax_streamx->M0AR = (uint32_t)p;
    }

    /**
     * @date 2026-5-13
     * 
     * 
     */
    void DMA::setDoubleBuffer(uint8_t* secondBuf){
        DMA_DoubleBufferModeConfig(dmax_streamx,(uint32_t) secondBuf,DMA_Memory_0);
    }
    bool DMA::isTransferingFirstBuffer(){
        return (dmax_streamx->CR & DMA_SxCR_CT) == 0;
    }

} // namespace dl

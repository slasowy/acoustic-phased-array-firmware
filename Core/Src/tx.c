void System_Init(void) {
    // Enable clocks
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB4ENR  |= RCC_AHB4ENR_GPIODEN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    __DSB();

    // PD12 as fast output
    GPIOD->MODER   &= ~(3UL << 24);
    GPIOD->MODER   |=  (1UL << 24); // Output
    GPIOD->OSPEEDR |=  (3UL << 24); // Very high speed
}

static void dma_stream_init(DMA_Stream_TypeDef *stream,
                            DMAMUX_Channel_TypeDef *mux_ch,
                            volatile uint32_t *src,
                            volatile uint32_t *dst,
                            uint32_t count)
{
    stream->CR &= ~DMA_SxCR_EN;
    while (stream->CR & DMA_SxCR_EN);

    mux_ch->CCR = DMAMUX1_TIM3_UP_ID;

    stream->PAR  = (uint32_t)dst;
    stream->M0AR = (uint32_t)src;
    stream->NDTR = count;
    stream->CR   =
        DMA_SxCR_PL_1    |  // High priority
        DMA_SxCR_MSIZE_1 |  // 32-bit memory
        DMA_SxCR_PSIZE_1 |  // 32-bit peripheral
        DMA_SxCR_MINC    |  // Memory increment
        DMA_SxCR_CIRC    |  // Circular
        DMA_SxCR_DIR_0   |  // Mem → periph
        DMA_SxCR_EN;
}

void DMA_Waveform_Init(void)
{
    // DMA2 clock may not be enabled yet — ensure it is
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
    __DSB();

    dma_stream_init(DMA1_Stream2, DMAMUX1_Channel2,
                    PORTD_waveform_buffer, &GPIOD->BSRR, 250);

    dma_stream_init(DMA1_Stream3, DMAMUX1_Channel3,
                    PORTF_waveform_buffer, &GPIOF->BSRR, 250);

    dma_stream_init(DMA2_Stream2, DMAMUX1_Channel10,
                    PORTA_waveform_buffer, &GPIOA->BSRR, 250);

    dma_stream_init(DMA2_Stream3, DMAMUX1_Channel11,
                    PORTB_waveform_buffer, &GPIOB->BSRR, 250);
}

void TIM3_Waveform_Init(void)
{
    TIM3->CR1  &= ~TIM_CR1_CEN;   // Stop first in case HAL left it running
    TIM3->PSC   = 0;
    TIM3->ARR   = 19;              // 10 MHz — adjust if your APB1 timer clk differs
    TIM3->EGR   = TIM_EGR_UG;     // Force update to load PSC/ARR
    TIM3->SR   &= ~TIM_SR_UIF;    // Clear update flag set by EGR
    TIM3->DIER |= TIM_DIER_UDE;   // DMA request on update
    TIM3->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;
}
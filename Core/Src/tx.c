#include "tx.h"
#include "lwip.h"

__attribute__((section(".adcarray")))
__attribute__((aligned(32)))
uint32_t PORTD_waveform_buffer[250];

__attribute__((section(".adcarray")))
__attribute__((aligned(32)))
uint32_t PORTF_waveform_buffer[250];

__attribute__((section(".adcarray")))
__attribute__((aligned(32)))
uint32_t PORTA_waveform_buffer[250];

__attribute__((section(".adcarray")))
__attribute__((aligned(32)))
uint32_t PORTB_waveform_buffer[250];

volatile uint32_t pulse_counter = 0;

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

    // dma_stream_init(DMA1_Stream2, DMAMUX1_Channel2,
    //                 PORTD_waveform_buffer, &GPIOD->BSRR, 250);

    dma_stream_init(DMA1_Stream3, DMAMUX1_Channel3,
                    PORTF_waveform_buffer, &GPIOF->BSRR, 250);

    dma_stream_init(DMA2_Stream2, DMAMUX1_Channel10,
                    PORTA_waveform_buffer, &GPIOA->BSRR, 250);

    dma_stream_init(DMA2_Stream3, DMAMUX1_Channel11,
                    PORTB_waveform_buffer, &GPIOB->BSRR, 250);
}

void DMA1_Stream2_Init(void) // separate for DMA1 Stream 3 to enable interrupt
{
    dma_stream_init(DMA1_Stream2, DMAMUX1_Channel2,
                    PORTD_waveform_buffer, &GPIOD->BSRR, 250);

    // Enable transfer complete interrupt
    DMA1_Stream2->CR |= DMA_SxCR_TCIE;

    // Register and enable the IRQ
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
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

void fill_waveform_buffers(float delay_seconds)
{
    const float time_step = 1.0f / 10000000.0f;
    const float two_pi_f  = 2.0f * 3.14159265f * 40000.0f;

    for (int i = 0; i < 250; i++) {
        float t = time_step * i;

        // Precompute all 16 channel polarities
        // Channel index matches the ordered pin list:
        // 0=F6, 1=A15, 2=F2, 3=A0, 4=F1, 5=F0,
        // 6=D13, 7=D12, 8=F13, 9=F12, 10=D15, 11=D14,
        // 12=A3, 13=B3, 14=B11, 15=B10
        int high[16];
        for (int ch = 0; ch < 16; ch++) {
            high[ch] = cosf(two_pi_f * (t + ch * delay_seconds)) > 0;
        }

        // ---- PORTF: channels 0(F6), 2(F2), 4(F1), 5(F0), 8(F13), 9(F12) ----
        uint32_t pf = 0;
        pf |= high[0] ? (1UL << 6)  : (1UL << 22);  // F6
        pf |= high[2] ? (1UL << 2)  : (1UL << 18);  // F2
        pf |= high[4] ? (1UL << 1)  : (1UL << 17);  // F1
        pf |= high[5] ? (1UL << 0)  : (1UL << 16);  // F0
        pf |= high[8] ? (1UL << 13) : (1UL << 29);  // F13
        pf |= high[9] ? (1UL << 12) : (1UL << 28);  // F12
        PORTF_waveform_buffer[i] = pf;

        // ---- PORTA: channels 1(A15), 3(A0), 12(A3) ----
        uint32_t pa = 0;
        pa |= high[1]  ? (1UL << 15) : (1UL << 31);  // A15
        pa |= high[3]  ? (1UL << 0)  : (1UL << 16);  // A0
        pa |= high[12] ? (1UL << 3)  : (1UL << 19);  // A3
        PORTA_waveform_buffer[i] = pa;

        // ---- PORTD: channels 6(D13), 7(D12), 10(D15), 11(D14) ----
        uint32_t pd = 0;
        pd |= high[6]  ? (1UL << 13) : (1UL << 29);  // D13
        pd |= high[7]  ? (1UL << 12) : (1UL << 28);  // D12
        pd |= high[10] ? (1UL << 15) : (1UL << 31);  // D15
        pd |= high[11] ? (1UL << 14) : (1UL << 30);  // D14
        PORTD_waveform_buffer[i] = pd;

        // ---- PORTB: channels 13(B3), 14(B11), 15(B10) ----
        uint32_t pb = 0;
        pb |= high[13] ? (1UL << 3)  : (1UL << 19);  // B3
        pb |= high[14] ? (1UL << 11) : (1UL << 27);  // B11
        pb |= high[15] ? (1UL << 10) : (1UL << 26);  // B10
        PORTB_waveform_buffer[i] = pb;
    }
}

void Waveform_Stop(void)
{
    // ---- Stop TIM3 first to prevent any further DMA requests ----
    TIM3->CR1  &= ~TIM_CR1_CEN;
    TIM3->DIER &= ~TIM_DIER_UDE;

    // ---- Disable all four DMA streams ----
    DMA1_Stream2->CR &= ~DMA_SxCR_EN;
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;

    // Wait for all streams to actually stop
    while (DMA1_Stream2->CR & DMA_SxCR_EN);
    while (DMA1_Stream3->CR & DMA_SxCR_EN);
    while (DMA2_Stream2->CR & DMA_SxCR_EN);
    while (DMA2_Stream3->CR & DMA_SxCR_EN);

    // ---- Drive all waveform pins low via BRR (bit reset register) ----
    // PORTA: A0, A3, A15
    GPIOA->BSRR = PA0_CLR | PA3_CLR | PA15_CLR;

    // PORTB: B3, B10, B11
    GPIOB->BSRR = PB3_CLR | PB10_CLR | PB11_CLR;

    // PORTD: D12, D13, D14, D15
    GPIOD->BSRR = PD12_CLR | PD13_CLR | PD14_CLR | PD15_CLR;

    // PORTF: F0, F1, F2, F6, F12, F13
    GPIOF->BSRR = PF0_CLR | PF1_CLR | PF2_CLR | PF6_CLR | PF12_CLR | PF13_CLR;
}

void DMA1_Stream2_IRQHandler(void)
{
    // Check transfer complete flag for Stream2 (TCIF2 is bit 5 of LISR)
    if (DMA1->LISR & DMA_LISR_TCIF2) {
        // Clear the flag
        DMA1->LIFCR = DMA_LIFCR_CTCIF2;

        pulse_counter++;        
    }
}

void tx_on(){
  HAL_GPIO_WritePin(RX_EN_SIG_GPIO_Port, RX_EN_SIG_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RX_EN_REF_GPIO_Port, RX_EN_REF_Pin, GPIO_PIN_RESET);

  HAL_Delay(100);

  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);
}

void transmit(uint32_t pulses, float *delays){ //delay must be 16 elements long
    pulse_counter = 0;

    fill_waveform_buffers(delays[0]);
    DMA_Waveform_Init();
    DMA1_Stream2_Init();
    tx_on();
    TIM3_Waveform_Init();

     while(pulse_counter < pulses){
         //MX_LWIP_Process();

     }
    Waveform_Stop();
    return;

}
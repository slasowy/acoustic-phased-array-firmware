/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include "ethernetif.h"
#include "lwip.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
//#include "tcpClientRAW.h"
#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_hal_uart.h"
#include "udpServerRAW.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_tim.h"
#include "timeouts.h"

extern struct netif gnetif;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim23;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM23_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_CONVERTED_DATA_BUFFER_SIZE 640


extern struct netif gnetif;

/* USER CODE END PD */
/* USER CODE BEGIN PV */
//uint16_t uhADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE] __attribute__((at(0x24000000))); // Place buffer in non-cacheable SRAM
//uint16_t uhADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; // Place buffer in non-cacheable SRAM
__attribute__((section(".adcarray"))) uint32_t uhADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
__attribute__((section(".adcarray"))) uint32_t adc1_rx_data[ADC_CONVERTED_DATA_BUFFER_SIZE];
__attribute__((section(".adcarray"))) uint32_t adc2_rx_data[ADC_CONVERTED_DATA_BUFFER_SIZE];

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

/* USER CODE END PV */
/* USER CODE BEGIN PFP */

volatile uint8_t adc1_first_half_ready = 0;
volatile uint8_t adc1_second_half_ready = 0;

volatile uint8_t adc2_first_half_ready = 0;
volatile uint8_t adc2_second_half_ready = 0;

volatile int16_t converted_data[ADC_CONVERTED_DATA_BUFFER_SIZE];
volatile int16_t converted_data2[ADC_CONVERTED_DATA_BUFFER_SIZE];

volatile uint16_t converted_data_index = 0;

volatile uint8_t adc1_configured = 0;

volatile uint8_t packet_ID = 0;

uint8_t a1_first = 0;
int16_t adc_data[ADC_CONVERTED_DATA_BUFFER_SIZE + 1]; // +1 for packet ID at the start

volatile uint8_t PWM_buffer[1000];
volatile uint16_t PWM_index = 0;

volatile uint8_t time_int;

uint8_t pin_state[256];

#define DMAMUX1_TIM3_UP_ID  27


/* =========================================================
 * BSRR BIT DEFINES
 * SET   = bit N      (lower 16 bits)
 * CLEAR = bit N+16   (upper 16 bits)
 * ========================================================= */

/* PORTA */
#define PA0_SET   (1UL << 0)
#define PA0_CLR   (1UL << 16)
#define PA3_SET   (1UL << 3)
#define PA3_CLR   (1UL << 19)
#define PA15_SET  (1UL << 15)
#define PA15_CLR  (1UL << 31)

/* PORTB */
#define PB3_SET   (1UL << 3)
#define PB3_CLR   (1UL << 19)
#define PB10_SET  (1UL << 10)
#define PB10_CLR  (1UL << 26)
#define PB11_SET  (1UL << 11)
#define PB11_CLR  (1UL << 27)

/* PORTD */
#define PD12_SET  (1UL << 12)
#define PD12_CLR  (1UL << 28)
#define PD13_SET  (1UL << 13)
#define PD13_CLR  (1UL << 29)
#define PD14_SET  (1UL << 14)
#define PD14_CLR  (1UL << 30)
#define PD15_SET  (1UL << 15)
#define PD15_CLR  (1UL << 31)

/* PORTF */
#define PF0_SET   (1UL << 0)
#define PF0_CLR   (1UL << 16)
#define PF1_SET   (1UL << 1)
#define PF1_CLR   (1UL << 17)
#define PF2_SET   (1UL << 2)
#define PF2_CLR   (1UL << 18)
#define PF6_SET   (1UL << 6)
#define PF6_CLR   (1UL << 22)
#define PF12_SET  (1UL << 12)
#define PF12_CLR  (1UL << 28)
#define PF13_SET  (1UL << 13)
#define PF13_CLR  (1UL << 29)


void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi){
  if(hspi->Instance == SPI1) {
    packet_ID++;
    // if(adc1_first_half_ready ==1){
    //   HAL_UART_Transmit(&huart3, (uint8_t*)"Error: ADC1 first half ready again before second half was processed\n", 75, HAL_MAX_DELAY);
    // }
    if(!adc1_configured){
      adc1_configured = 1;
      // uint32_t channel_select[8] = { 0x4000 << 4, 0x4000 << 4, 0x8000 << 4, 0x8000 << 4, 0xC000 << 4, 0xC000 << 4, 0x0000 << 4, 0x0000 << 4};
      // for(int i = 0; i < 10; i++){
      //   uhADCxConvertedData[i] = channel_select[i % 8];
      // }
    }
    adc1_first_half_ready = 1;
  }
  if(hspi->Instance == SPI3) {
    //HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)uhADCxConvertedData, 21);
    adc2_first_half_ready = 1;
  }
}
     
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    if(hspi->Instance == SPI1) {
        packet_ID++;
        //HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)uhADCxConvertedData, (uint8_t*)adc1_rx_data, 15000);
        adc1_second_half_ready = 1;
    }
    if(hspi->Instance == SPI3) {
        //HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)uhADCxConvertedData, 21);
        adc2_second_half_ready = 1;
    }
}


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
void ADC_handle(void);


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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // uint32_t pseudodelay = 0;
  // while(pseudodelay++ < 0xFFFFFF)
  // {
  //   __NOP();
  // }
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_LWIP_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_TIM23_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  // filepath: Core/Src/main.c
  // Enable NVIC for TIM8 update interrupt

  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = (uint32_t)uhADCxConvertedData;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;  // Adjust size as needed (64*2=128B, so 256B region)
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  HAL_Delay(1000);

  udpClient_connect();
  HAL_Delay(1000);
  HAL_GPIO_WritePin(RX_EN_SIG_GPIO_Port, RX_EN_SIG_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RX_EN_REF_GPIO_Port, RX_EN_REF_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);

  

  fill_waveform_buffers(0);   // 62.5ns between channels = 1/16MHz
  DMA_Waveform_Init();
  TIM3_Waveform_Init();

  //HAL_TIM_Base_Start_IT(&htim3);

  while(1){
    

  }
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  while(1){};
  uint32_t channel_select[8] = { 0x4000 << 4, 0x4000 << 4, 0x8000 << 4, 0x8000 << 4, 0xC000 << 4, 0xC000 << 4, 0x0000 << 4, 0x0000 << 4};
 
 for(int i = 0; i < ADC_CONVERTED_DATA_BUFFER_SIZE; i ++){
    uhADCxConvertedData[i] = channel_select[i % 8];
  }

  //stuff some data with 0 to prevent garbage when clocks stabilize, then send configs
  uhADCxConvertedData[0] = 0;
  uhADCxConvertedData[1] = 0;
  uhADCxConvertedData[2] = 0;
  uhADCxConvertedData[3] = 0;
  uhADCxConvertedData[4] = 0x1062 <<4;
  uhADCxConvertedData[5] = 0x3FF << 4;
  uhADCxConvertedData[6] = 0x1005 << 4;
  uhADCxConvertedData[7] = 0x3FF << 4;

  uhADCxConvertedData[8] = 0x1040 << 4;
  //uhADCxConvertedData[0] = 1 << 5;
  //uhADCxConvertedData[1] = 1 << 6;
  
  HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&uhADCxConvertedData, (uint8_t*)adc1_rx_data, ADC_CONVERTED_DATA_BUFFER_SIZE);
  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*)&uhADCxConvertedData, (uint8_t*)adc2_rx_data, ADC_CONVERTED_DATA_BUFFER_SIZE);

  HAL_Delay(100);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  TIM1->CCR1 = 10;
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  TIM8->CCR1 = 5;
  uint8_t settling_delay = 0;
  while(settling_delay < 200){
    if(adc1_first_half_ready){
      adc1_first_half_ready = 0;
      settling_delay++;
    }
  }

  if(!(adc1_rx_data[0] & (1 << 18))){
    a1_first = 1;
    uint32_t channel_select[8] = { 0x4000 << 4, 0x4000 << 4, 0x8000 << 4, 0x8000 << 4, 0xC000 << 4, 0xC000 << 4, 0x0000 << 4, 0x0000 << 4};
 
    for(int i = 0; i < ADC_CONVERTED_DATA_BUFFER_SIZE; i ++){
        uhADCxConvertedData[i] = channel_select[i % 8];
    }
  }else{
    a1_first = 0;
    uint32_t channel_select[8] = {0x4000 << 4, 0x8000 << 4, 0x8000 << 4, 0xC000 << 4, 0xC000 << 4, 0x0000 << 4, 0x0000 << 4, 0x4000 << 4};
 
    for(int i = 0; i < ADC_CONVERTED_DATA_BUFFER_SIZE; i ++){
        uhADCxConvertedData[i] = channel_select[i % 8];
    }

  }

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    ADC_handle();
    MX_LWIP_Process();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_20BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_20BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 3;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 255;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM23 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM23_Init(void)
{

  /* USER CODE BEGIN TIM23_Init 0 */

  /* USER CODE END TIM23_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM23_Init 1 */

  /* USER CODE END TIM23_Init 1 */
  htim23.Instance = TIM23;
  htim23.Init.Prescaler = 3;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim23.Init.Period = 255;
  htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim23) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim23, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM23_Init 2 */

  /* USER CODE END TIM23_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6
                          |GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TX_EN_Pin|RX_EN_REF_Pin|RX_EN_SIG_Pin|ADC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PWM1_Pin|PWM2_Pin|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF0 PF1 PF2 PF6
                           PF12 PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_EN_Pin RX_EN_REF_Pin RX_EN_SIG_Pin ADC_CS_Pin */
  GPIO_InitStruct.Pin = TX_EN_Pin|RX_EN_REF_Pin|RX_EN_SIG_Pin|ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PWM1_Pin PWM2_Pin PD14 PD15 */
  GPIO_InitStruct.Pin = PWM1_Pin|PWM2_Pin|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ADC_handle(){
    if(adc1_first_half_ready && adc2_first_half_ready){
      adc1_first_half_ready = 0;
      adc2_first_half_ready = 0;


      uint8_t a2_first = 0;

      int16_t adc1_shifted[ADC_CONVERTED_DATA_BUFFER_SIZE/2];
      int16_t adc2_shifted[ADC_CONVERTED_DATA_BUFFER_SIZE/2];
      if(!a1_first){
        adc1_shifted[0] = adc1_rx_data[ADC_CONVERTED_DATA_BUFFER_SIZE-1] >> 2;
        adc2_shifted[0] = adc2_rx_data[ADC_CONVERTED_DATA_BUFFER_SIZE-1] >> 2;
        for(int i = 1; i < ADC_CONVERTED_DATA_BUFFER_SIZE/2; i++){
          adc1_shifted[i] = adc1_rx_data[i-1] >> 2;
          adc2_shifted[i] = adc2_rx_data[i-1] >> 2;
        }
      }else{
        adc1_shifted[0] = adc1_rx_data[0] >> 2;
        adc2_shifted[0] = adc2_rx_data[0] >> 2;
        for(int i = 1; i < ADC_CONVERTED_DATA_BUFFER_SIZE/2; i++){
          adc1_shifted[i] = adc1_rx_data[i] >> 2;
          adc2_shifted[i] = adc2_rx_data[i] >> 2;
        }
      }
      const int samples_per_channel = ADC_CONVERTED_DATA_BUFFER_SIZE / 16;
      for(int channel = 0; channel < 16; channel++){
        int src_channel = (channel < 8) ? channel : channel - 8;
        int base_index = channel * samples_per_channel;

        if(channel < 8){
          for(int i = 0; i < samples_per_channel; i++){
            adc_data[base_index + i + 1] = (adc1_shifted[src_channel + (i * 8)]);
          }
        } else {
          for(int i = 0; i < samples_per_channel; i++){
            adc_data[base_index + i + 1] = (adc2_shifted[src_channel + (i * 8)]);
          }
        }
      }
      adc_data[0] = packet_ID;
      udpClient_send((int16_t*)adc_data, ADC_CONVERTED_DATA_BUFFER_SIZE + 1);


    }else if(adc1_second_half_ready && adc2_second_half_ready){
      adc1_second_half_ready = 0;
      adc2_second_half_ready = 0;


      int16_t adc1_shifted[ADC_CONVERTED_DATA_BUFFER_SIZE/2];
      int16_t adc2_shifted[ADC_CONVERTED_DATA_BUFFER_SIZE/2];
      if(!a1_first){
        adc1_shifted[0] = adc1_rx_data[(ADC_CONVERTED_DATA_BUFFER_SIZE/2)-1] >> 2;
        adc2_shifted[0] = adc2_rx_data[(ADC_CONVERTED_DATA_BUFFER_SIZE/2)-1] >> 2;
        for(int i = 1; i < ADC_CONVERTED_DATA_BUFFER_SIZE/2; i++){
          adc1_shifted[i] = adc1_rx_data[((ADC_CONVERTED_DATA_BUFFER_SIZE/2)+i)-1] >> 2;
          adc2_shifted[i] = adc2_rx_data[((ADC_CONVERTED_DATA_BUFFER_SIZE/2)+i)-1] >> 2;
        }
      }else{
        adc1_shifted[0] = adc1_rx_data[ADC_CONVERTED_DATA_BUFFER_SIZE/2] >> 2;
        adc2_shifted[0] = adc2_rx_data[ADC_CONVERTED_DATA_BUFFER_SIZE/2] >> 2;
        for(int i = 1; i < ADC_CONVERTED_DATA_BUFFER_SIZE/2; i++){
          adc1_shifted[i] = adc1_rx_data[((ADC_CONVERTED_DATA_BUFFER_SIZE/2)+i)] >> 2;
          adc2_shifted[i] = adc2_rx_data[((ADC_CONVERTED_DATA_BUFFER_SIZE/2)+i)] >> 2;
        }
      }
      const int samples_per_channel = ADC_CONVERTED_DATA_BUFFER_SIZE / 16;
      for(int channel = 0; channel < 16; channel++){
        int src_channel = (channel < 8) ? channel : channel - 8;
        int base_index = channel * samples_per_channel;

        if(channel < 8){
          for(int i = 0; i < samples_per_channel; i++){
            adc_data[base_index + i + 1] = (adc1_shifted[src_channel + (i * 8)]);
          }
        } else {
          for(int i = 0; i < samples_per_channel; i++){
            adc_data[base_index + i + 1] = (adc2_shifted[src_channel + (i * 8)]);
          }
        }
      }
      adc_data[0] = packet_ID;
      udpClient_send((int16_t*)adc_data, ADC_CONVERTED_DATA_BUFFER_SIZE + 1);
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

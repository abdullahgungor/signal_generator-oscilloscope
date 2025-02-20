/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except  in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "usbd_cdc_if.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAVE_TABLE_SIZE 100
#define MAX_DAC_VALUE 255

uint8_t waveTable[WAVE_TABLE_SIZE];
uint16_t currentIndex = 0;
uint8_t RxBuffer[10];
volatile uint8_t waveFrequency = 0;  // 0 -> 100 Hz, 1 -> 200 Hz
volatile uint8_t waveType = 0;       // 0 -> Sinüs, 1 -> Testere, 2 -> Üçgen, 3 -> PWM
volatile uint8_t pwmDutyCycle = 50;  // PWM Duty Cycle (varsayılan %50)
uint16_t adcBuffer[10];
volatile uint8_t tim2Counter = 0; // TIM2 kesmesi sayaç
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void generateSineTable(void);
void generateSawtoothTable(void);
void generateTriangleTable(void);
void updateWaveType(uint8_t type);
void setTimerFrequency(uint8_t frequency);
void processReceivedData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  startADC();
  /* USER CODE BEGIN 2 */
    setTimerFrequency(waveFrequency); 
    updateWaveType(waveType);
    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (waveType == 3) {
            if (currentIndex < (pwmDutyCycle * WAVE_TABLE_SIZE / 100)) {
                GPIOA->ODR = MAX_DAC_VALUE;
            } else {
                GPIOA->ODR = 0;
            }
        } else {
            GPIOA->ODR = waveTable[currentIndex];
        }

        currentIndex = (currentIndex + 1) % WAVE_TABLE_SIZE;
        tim2Counter++;
        uint8_t tim2Frequency;
        if(waveFrequency == 0){
        	tim2Frequency = 20;
        }
        else {
        	tim2Frequency = 40;
        }
        uint16_t counterLimit = tim2Frequency / 10;
        if (tim2Counter >= counterLimit) {
                    tim2Counter = 0;
                    memcpy(adcBuffer, adcValue, sizeof(adcValue));
                    CDC_Transmit_FS((uint8_t*)adcBuffer, sizeof(adcBuffer));
                }
}
}
void generateSineTable(void) {
    for (int i = 0; i < WAVE_TABLE_SIZE; i++) {
        waveTable[i] = (uint8_t)((MAX_DAC_VALUE / 2.0) * (1.0 + sin(2 * M_PI * i / WAVE_TABLE_SIZE)));
    }
}

void generateSawtoothTable(void) {
    for (int i = 0; i < WAVE_TABLE_SIZE; i++) {
        waveTable[i] = (uint8_t)((MAX_DAC_VALUE * i) / WAVE_TABLE_SIZE);
    }
}

void generateTriangleTable(void) {
    int midpoint = WAVE_TABLE_SIZE / 2;
    for (int i = 0; i < midpoint; i++) {
        waveTable[i] = (uint8_t)((MAX_DAC_VALUE * i) / midpoint);
    }
    for (int i = midpoint; i < WAVE_TABLE_SIZE; i++) {
        waveTable[i] = (uint8_t)(MAX_DAC_VALUE - ((MAX_DAC_VALUE * (i - midpoint)) / midpoint));
    }
}

void updateWaveType(uint8_t type) {
    switch (type) {
        case 0:
            generateSineTable();
            break;
        case 1:
            generateSawtoothTable();
            break;
        case 2;
            generateTriangleTable();
            break;
        case 3:
            break;
        default:
            generateSineTable();
    }
}

void setTimerFrequency(uint8_t frequency) {
    if (frequency == 0) {
        htim2.Init.Prescaler = 71;
        htim2.Init.Period = 99;
    } else if (frequency == 1) {
        htim2.Init.Prescaler = 71;
        htim2.Init.Period = 49;
    }
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
}

void processReceivedData(void) {
    switch (RxBuffer[0]) {
        case 's':
            waveType = 0;
            updateWaveType(waveType);
            break;
        case 't':
            waveType = 1;
            updateWaveType(waveType);
            break;
        case 'u':
            waveType = 2;
            updateWaveType(waveType);
            break;
        case 'p':
            waveType = 3;
            updateWaveType(waveType);
            break;
        case '1':
            waveFrequency = 0;
            setTimerFrequency(waveFrequency);
            break;
        case '2':
            waveFrequency = 1;
            setTimerFrequency(waveFrequency);
            break;
        case 'a':
            pwmDutyCycle = 10;
            break;
        case 'b':
            pwmDutyCycle = 20;
            break;
        case 'c':
            pwmDutyCycle = 30;
            break;
        case 'd':
            pwmDutyCycle = 40;
            break;
        case 'e':
            pwmDutyCycle = 50;
            break;
        case 'f':
            pwmDutyCycle = 60;
            break;
        case 'g':
            pwmDutyCycle = 70;
            break;
        case 'h':
            pwmDutyCycle = 80;
            break;
        case 'i':
            pwmDutyCycle = 90;
            break;
        default:
            break;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* Kullanıcı kendi hata yönetimini ekleyebilir */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

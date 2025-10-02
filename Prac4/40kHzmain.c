/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
#include "EEE3096S_Prac4_WAV_LUTS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS         4096u         // LUT length (per waveform period) - matches WAV LUTs
#define TIM2CLK    16000000u     // per .ioc (16 MHz HCLK to TIM2)

// Different playback frequencies for different waveforms
#define FSIGNAL_BASIC    1000u   // 1 kHz for basic waveforms (sine, saw, triangle)
#define FSIGNAL_PIANO    (1.0f/18.86f)  // Play piano sample at original duration (18.86s)
#define FSIGNAL_GUITAR   (1.0f/10.58f)  // Play guitar sample at original duration (10.58s)
#define FSIGNAL_DRUM     (1.0f/11.34f)  // Play drum sample at original duration (11.34s)

// We'll calculate TIM2_TICKS dynamically based on current waveform
// For now, default to basic waveforms
#define TIM2_TICKS_BASIC (TIM2CLK / (128 * FSIGNAL_BASIC))   // Basic waveforms use 128 samples
#define TIM2_TICKS_PIANO 4324//(TIM2CLK / (NS * FSIGNAL_PIANO))    // Piano uses full NS samples
#define TIM2_TICKS_GUITAR  4324// (TIM2CLK / (NS * FSIGNAL_GUITAR))  // Guitar uses full NS samples
#define TIM2_TICKS_DRUM  10000
//(TIM2CLK / (NS * FSIGNAL_DRUM))      // Drum uses full NS samples

// Note: Basic waveforms still use 128 samples (declared below)

uint16_t sine_LUT[128] = {2048, 2148, 2248, 2348, 2447, 2545, 2642, 2737, 2831, 2923, 3013, 3100, 3185, 3267, 3346, 3423, 3495, 3565, 3630, 3692, 3750, 3804, 3853, 3898, 3939, 3975, 4007, 4034, 4056, 4073, 4085, 4093, 4095, 4093, 4085, 4073, 4056, 4034, 4007, 3975, 3939, 3898, 3853, 3804, 3750, 3692, 3630, 3565, 3495, 3423, 3346, 3267, 3185, 3100, 3013, 2923, 2831, 2737, 2642, 2545, 2447, 2348, 2248, 2148, 2048, 1947, 1847, 1747, 1648, 1550, 1453, 1358, 1264, 1172, 1082, 995, 910, 828, 749, 672, 600, 530, 465, 403, 345, 291, 242, 197, 156, 120, 88, 61, 39, 22, 10, 2, 0, 2, 10, 22, 39, 61, 88, 120, 156, 197, 242, 291, 345, 403, 465, 530, 600, 672, 749, 828, 910, 995, 1082, 1172, 1264, 1358, 1453, 1550, 1648, 1747, 1847, 1947};
uint16_t saw_LUT[128]  = {0, 32, 64, 97, 129, 161, 193, 226, 258, 290, 322, 355, 387, 419, 451, 484, 516, 548, 580, 613, 645, 677, 709, 742, 774, 806, 838, 871, 903, 935, 967, 1000, 1032, 1064, 1096, 1129, 1161, 1193, 1225, 1258, 1290, 1322, 1354, 1386, 1419, 1451, 1483, 1515, 1548, 1580, 1612, 1644, 1677, 1709, 1741, 1773, 1806, 1838, 1870, 1902, 1935, 1967, 1999, 2031, 2064, 2096, 2128, 2160, 2193, 2225, 2257, 2289, 2322, 2354, 2386, 2418, 2451, 2483, 2515, 2547, 2580, 2612, 2644, 2676, 2709, 2741, 2773, 2805, 2837, 2870, 2902, 2934, 2966, 2999, 3031, 3063, 3095, 3128, 3160, 3192, 3224, 3257, 3289, 3321, 3353, 3386, 3418, 3450, 3482, 3515, 3547, 3579, 3611, 3644, 3676, 3708, 3740, 3773, 3805, 3837, 3869, 3902, 3934, 3966, 3998, 4031, 4063, 4095};
uint16_t tri_LUT[128]  = {0, 65, 130, 195, 260, 325, 390, 455, 520, 585, 650, 715, 780, 845, 910, 975, 1040, 1105, 1170, 1235, 1300, 1365, 1430, 1495, 1560, 1625, 1690, 1755, 1820, 1885, 1950, 2015, 2080, 2145, 2210, 2275, 2340, 2405, 2470, 2535, 2600, 2665, 2730, 2795, 2860, 2925, 2990, 3055, 3120, 3185, 3250, 3315, 3380, 3445, 3510, 3575, 3640, 3705, 3770, 3835, 3900, 3965, 4030, 4095, 4095, 4030, 3965, 3900, 3835, 3770, 3705, 3640, 3575, 3510, 3445, 3380, 3315, 3250, 3185, 3120, 3055, 2990, 2925, 2860, 2795, 2730, 2665, 2600, 2535, 2470, 2405, 2340, 2275, 2210, 2145, 2080, 2015, 1950, 1885, 1820, 1755, 1690, 1625, 1560, 1495, 1430, 1365, 1300, 1235, 1170, 1105, 1040, 975, 910, 845, 780, 715, 650, 585, 520, 455, 390, 325, 260, 195, 130, 65, 0};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

// Waveform selection variables
typedef enum {
    WAVEFORM_SINE = 0,
    WAVEFORM_SAWTOOTH,
    WAVEFORM_TRIANGLE,
    WAVEFORM_PIANO,
    WAVEFORM_GUITAR,
    WAVEFORM_DRUM,
    WAVEFORM_COUNT
} waveform_t;

static waveform_t current_waveform = WAVEFORM_SINE;
static uint16_t *current_lut = sine_LUT;

// Function to get LUT pointer and size for current waveform
uint16_t* get_current_lut(void) {
    switch(current_waveform) {
        case WAVEFORM_SINE:     return sine_LUT;
        case WAVEFORM_SAWTOOTH: return saw_LUT;
        case WAVEFORM_TRIANGLE: return tri_LUT;
        case WAVEFORM_PIANO:    return piano_wav_LUT;
        case WAVEFORM_GUITAR:   return guitar_wav_LUT;
        case WAVEFORM_DRUM:     return drum_wav_LUT;
        default:                return sine_LUT;
    }
}

uint32_t get_current_lut_size(void) {
    switch(current_waveform) {
        case WAVEFORM_SINE:
        case WAVEFORM_SAWTOOTH:
        case WAVEFORM_TRIANGLE:
            return 128;  // Basic waveforms use 128 samples
        case WAVEFORM_PIANO:
        case WAVEFORM_GUITAR:
        case WAVEFORM_DRUM:
            return NS;   // WAV files use NS (4096) samples
        default:
            return 128;
    }
}

uint32_t get_current_tim2_ticks(void) {
    switch(current_waveform) {
        case WAVEFORM_SINE:
        case WAVEFORM_SAWTOOTH:
        case WAVEFORM_TRIANGLE:
            return TIM2_TICKS_BASIC;   // Fast playback for basic waveforms
        case WAVEFORM_PIANO:
            return TIM2_TICKS_PIANO;   // Original duration playback
        case WAVEFORM_GUITAR:
            return TIM2_TICKS_GUITAR;  // Original duration playback
        case WAVEFORM_DRUM:
            return TIM2_TICKS_DRUM;    // Original duration playback
        default:
            return TIM2_TICKS_BASIC;
    }
}





// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = 0; // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Task 4: Initialize all peripherals for audio playback

  // Initialize LCD
  init_LCD();

  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1. Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  // Configure DMA for the initial waveform (sine)
  current_lut = get_current_lut();
  uint32_t lut_size = get_current_lut_size();

  HAL_DMA_Start_IT(&hdma_tim2_ch1,
                   (uint32_t)current_lut,      // Source: LUT array
                   DestAddress,                // Destination: TIM3->CCR3
                   lut_size);                  // Number of transfers

  // TODO: Write current waveform to LCD(Sine is the first waveform)
  lcd_command(CLEAR);
  lcd_putstring("Sine Wave");

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);  // Enable TIM2 CH1 DMA requests

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim2.Init.Period = 4294967295;
  htim2.Init.Period = 400;// Initial just to trigger. Reset upon button press
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  // Ensure no reference to TIM_SMCR_ETPS_0 or other undefined macros
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;         // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;    // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;        // Memory address increments
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;            // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim3.Init.Period = 65535;
  htim3.Init.Period = 400; 	// 40khz PWM
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // -------------------------------
  // LCD pins configuration
  // -------------------------------
  // Configure PC14 (RS) and PC15 (E) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure PB8 (D4) and PB9 (D5) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA12 (D6) and PA15 (D7) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Set all LCD pins LOW initially
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);


  // -------------------------------
  // Button0 configuration (PA0)
  // -------------------------------
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Use pull-up resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI line 0 interrupt priority
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){

	// TODO: Debounce using HAL_GetTick()
	static uint32_t last_interrupt_time = 0;
	uint32_t current_time = HAL_GetTick();

	// Debounce: ignore interrupts that occur within 200ms of the last one
	if (current_time - last_interrupt_time < 200) {
		HAL_GPIO_EXTI_IRQHandler(Button0_Pin);
		return;
	}
	last_interrupt_time = current_time;

	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes

	// Cycle to next waveform
	current_waveform = (current_waveform + 1) % WAVEFORM_COUNT;
	current_waveform = 5;
	current_lut = get_current_lut();

	// Update DMA source
	// TODO: Stop DMA, reconfigure source, restart DMA

	// 1. Stop current DMA transfer
	__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
	HAL_DMA_Abort(&hdma_tim2_ch1);

	// 2. Update TIM2 period for new waveform timing
	uint32_t new_tim2_ticks = get_current_tim2_ticks();
	//new_tim2_ticks = TIM2_TICKS_DRUM;
	__HAL_TIM_SET_AUTORELOAD(&htim2, new_tim2_ticks - 1);

	// 3. Start DMA with new LUT
	uint32_t lut_size = get_current_lut_size();
	HAL_DMA_Start_IT(&hdma_tim2_ch1,
	                 (uint32_t)current_lut,
	                 DestAddress,
	                 lut_size);

	// 4. Re-enable DMA transfers
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

	// Update LCD display

	switch(current_waveform) {
	    case WAVEFORM_SINE:
	        lcd_command(CLEAR);
	        lcd_putstring("Sine Wave");
	        break;
	    case WAVEFORM_SAWTOOTH:
	        lcd_command(CLEAR);
	        lcd_putstring("Sawtooth Wave");
	        break;
	    case WAVEFORM_TRIANGLE:
	        lcd_command(CLEAR);
	        lcd_putstring("Triangle Wave");
	        break;
	    case WAVEFORM_PIANO:
	        lcd_command(CLEAR);
	        lcd_putstring("Piano Sample");
	        break;
	    case WAVEFORM_GUITAR:
	        lcd_command(CLEAR);
	        lcd_putstring("Guitar Sample");
	        break;
	    case WAVEFORM_DRUM:
	        lcd_command(CLEAR);
	        lcd_putstring("Drum Sample");
	        break;


	}

	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
#include "EEE3096S_Prac4_WAV_LUTS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS         4096u         // LUT length (per waveform period) - matches WAV LUTs
#define TIM2CLK    16000000u     // per .ioc (16 MHz HCLK to TIM2)

// Different playback frequencies for different waveforms
#define FSIGNAL_BASIC    1000u   // 1 kHz for basic waveforms (sine, saw, triangle)
#define FSIGNAL_PIANO    (1.0f/18.86f)  // Play piano sample at original duration (18.86s)
#define FSIGNAL_GUITAR   (1.0f/10.58f)  // Play guitar sample at original duration (10.58s)
#define FSIGNAL_DRUM     (1.0f/11.34f)  // Play drum sample at original duration (11.34s)

// We'll calculate TIM2_TICKS dynamically based on current waveform
// For now, default to basic waveforms
#define TIM2_TICKS_BASIC (TIM2CLK / (128 * FSIGNAL_BASIC))   // Basic waveforms use 128 samples
#define TIM2_TICKS_PIANO 4324//(TIM2CLK / (NS * FSIGNAL_PIANO))    // Piano uses full NS samples
#define TIM2_TICKS_GUITAR  4324// (TIM2CLK / (NS * FSIGNAL_GUITAR))  // Guitar uses full NS samples
#define TIM2_TICKS_DRUM  10000
//(TIM2CLK / (NS * FSIGNAL_DRUM))      // Drum uses full NS samples

// Note: Basic waveforms still use 128 samples (declared below)

uint16_t sine_LUT[128] = {2048, 2148, 2248, 2348, 2447, 2545, 2642, 2737, 2831, 2923, 3013, 3100, 3185, 3267, 3346, 3423, 3495, 3565, 3630, 3692, 3750, 3804, 3853, 3898, 3939, 3975, 4007, 4034, 4056, 4073, 4085, 4093, 4095, 4093, 4085, 4073, 4056, 4034, 4007, 3975, 3939, 3898, 3853, 3804, 3750, 3692, 3630, 3565, 3495, 3423, 3346, 3267, 3185, 3100, 3013, 2923, 2831, 2737, 2642, 2545, 2447, 2348, 2248, 2148, 2048, 1947, 1847, 1747, 1648, 1550, 1453, 1358, 1264, 1172, 1082, 995, 910, 828, 749, 672, 600, 530, 465, 403, 345, 291, 242, 197, 156, 120, 88, 61, 39, 22, 10, 2, 0, 2, 10, 22, 39, 61, 88, 120, 156, 197, 242, 291, 345, 403, 465, 530, 600, 672, 749, 828, 910, 995, 1082, 1172, 1264, 1358, 1453, 1550, 1648, 1747, 1847, 1947};
uint16_t saw_LUT[128]  = {0, 32, 64, 97, 129, 161, 193, 226, 258, 290, 322, 355, 387, 419, 451, 484, 516, 548, 580, 613, 645, 677, 709, 742, 774, 806, 838, 871, 903, 935, 967, 1000, 1032, 1064, 1096, 1129, 1161, 1193, 1225, 1258, 1290, 1322, 1354, 1386, 1419, 1451, 1483, 1515, 1548, 1580, 1612, 1644, 1677, 1709, 1741, 1773, 1806, 1838, 1870, 1902, 1935, 1967, 1999, 2031, 2064, 2096, 2128, 2160, 2193, 2225, 2257, 2289, 2322, 2354, 2386, 2418, 2451, 2483, 2515, 2547, 2580, 2612, 2644, 2676, 2709, 2741, 2773, 2805, 2837, 2870, 2902, 2934, 2966, 2999, 3031, 3063, 3095, 3128, 3160, 3192, 3224, 3257, 3289, 3321, 3353, 3386, 3418, 3450, 3482, 3515, 3547, 3579, 3611, 3644, 3676, 3708, 3740, 3773, 3805, 3837, 3869, 3902, 3934, 3966, 3998, 4031, 4063, 4095};
uint16_t tri_LUT[128]  = {0, 65, 130, 195, 260, 325, 390, 455, 520, 585, 650, 715, 780, 845, 910, 975, 1040, 1105, 1170, 1235, 1300, 1365, 1430, 1495, 1560, 1625, 1690, 1755, 1820, 1885, 1950, 2015, 2080, 2145, 2210, 2275, 2340, 2405, 2470, 2535, 2600, 2665, 2730, 2795, 2860, 2925, 2990, 3055, 3120, 3185, 3250, 3315, 3380, 3445, 3510, 3575, 3640, 3705, 3770, 3835, 3900, 3965, 4030, 4095, 4095, 4030, 3965, 3900, 3835, 3770, 3705, 3640, 3575, 3510, 3445, 3380, 3315, 3250, 3185, 3120, 3055, 2990, 2925, 2860, 2795, 2730, 2665, 2600, 2535, 2470, 2405, 2340, 2275, 2210, 2145, 2080, 2015, 1950, 1885, 1820, 1755, 1690, 1625, 1560, 1495, 1430, 1365, 1300, 1235, 1170, 1105, 1040, 975, 910, 845, 780, 715, 650, 585, 520, 455, 390, 325, 260, 195, 130, 65, 0};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

// Waveform selection variables
typedef enum {
    WAVEFORM_SINE = 0,
    WAVEFORM_SAWTOOTH,
    WAVEFORM_TRIANGLE,
    WAVEFORM_PIANO,
    WAVEFORM_GUITAR,
    WAVEFORM_DRUM,
    WAVEFORM_COUNT
} waveform_t;

static waveform_t current_waveform = WAVEFORM_SINE;
static uint16_t *current_lut = sine_LUT;

// Function to get LUT pointer and size for current waveform
uint16_t* get_current_lut(void) {
    switch(current_waveform) {
        case WAVEFORM_SINE:     return sine_LUT;
        case WAVEFORM_SAWTOOTH: return saw_LUT;
        case WAVEFORM_TRIANGLE: return tri_LUT;
        case WAVEFORM_PIANO:    return piano_wav_LUT;
        case WAVEFORM_GUITAR:   return guitar_wav_LUT;
        case WAVEFORM_DRUM:     return drum_wav_LUT;
        default:                return sine_LUT;
    }
}

uint32_t get_current_lut_size(void) {
    switch(current_waveform) {
        case WAVEFORM_SINE:
        case WAVEFORM_SAWTOOTH:
        case WAVEFORM_TRIANGLE:
            return 128;  // Basic waveforms use 128 samples
        case WAVEFORM_PIANO:
        case WAVEFORM_GUITAR:
        case WAVEFORM_DRUM:
            return NS;   // WAV files use NS (4096) samples
        default:
            return 128;
    }
}

uint32_t get_current_tim2_ticks(void) {
    switch(current_waveform) {
        case WAVEFORM_SINE:
        case WAVEFORM_SAWTOOTH:
        case WAVEFORM_TRIANGLE:
            return TIM2_TICKS_BASIC;   // Fast playback for basic waveforms
        case WAVEFORM_PIANO:
            return TIM2_TICKS_PIANO;   // Original duration playback
        case WAVEFORM_GUITAR:
            return TIM2_TICKS_GUITAR;  // Original duration playback
        case WAVEFORM_DRUM:
            return TIM2_TICKS_DRUM;    // Original duration playback
        default:
            return TIM2_TICKS_BASIC;
    }
}





// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = 0; // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Task 4: Initialize all peripherals for audio playback

  // Initialize LCD
  init_LCD();

  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1. Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  // Configure DMA for the initial waveform (sine)
  current_lut = get_current_lut();
  uint32_t lut_size = get_current_lut_size();

  HAL_DMA_Start_IT(&hdma_tim2_ch1,
                   (uint32_t)current_lut,      // Source: LUT array
                   DestAddress,                // Destination: TIM3->CCR3
                   lut_size);                  // Number of transfers

  // TODO: Write current waveform to LCD(Sine is the first waveform)
  lcd_command(CLEAR);
  lcd_putstring("Sine Wave");

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);  // Enable TIM2 CH1 DMA requests

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim2.Init.Period = 4294967295;
  htim2.Init.Period = 400;// Initial just to trigger. Reset upon button press
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  // Ensure no reference to TIM_SMCR_ETPS_0 or other undefined macros
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;         // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;    // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;        // Memory address increments
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;            // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim3.Init.Period = 65535;
  htim3.Init.Period = 400; 	// 40khz PWM
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // -------------------------------
  // LCD pins configuration
  // -------------------------------
  // Configure PC14 (RS) and PC15 (E) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure PB8 (D4) and PB9 (D5) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA12 (D6) and PA15 (D7) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Set all LCD pins LOW initially
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);


  // -------------------------------
  // Button0 configuration (PA0)
  // -------------------------------
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Use pull-up resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI line 0 interrupt priority
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){

	// TODO: Debounce using HAL_GetTick()
	static uint32_t last_interrupt_time = 0;
	uint32_t current_time = HAL_GetTick();

	// Debounce: ignore interrupts that occur within 200ms of the last one
	if (current_time - last_interrupt_time < 200) {
		HAL_GPIO_EXTI_IRQHandler(Button0_Pin);
		return;
	}
	last_interrupt_time = current_time;

	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes

	// Cycle to next waveform
	current_waveform = (current_waveform + 1) % WAVEFORM_COUNT;
	current_waveform = 5;
	current_lut = get_current_lut();

	// Update DMA source
	// TODO: Stop DMA, reconfigure source, restart DMA

	// 1. Stop current DMA transfer
	__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
	HAL_DMA_Abort(&hdma_tim2_ch1);

	// 2. Update TIM2 period for new waveform timing
	uint32_t new_tim2_ticks = get_current_tim2_ticks();
	//new_tim2_ticks = TIM2_TICKS_DRUM;
	__HAL_TIM_SET_AUTORELOAD(&htim2, new_tim2_ticks - 1);

	// 3. Start DMA with new LUT
	uint32_t lut_size = get_current_lut_size();
	HAL_DMA_Start_IT(&hdma_tim2_ch1,
	                 (uint32_t)current_lut,
	                 DestAddress,
	                 lut_size);

	// 4. Re-enable DMA transfers
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

	// Update LCD display

	switch(current_waveform) {
	    case WAVEFORM_SINE:
	        lcd_command(CLEAR);
	        lcd_putstring("Sine Wave");
	        break;
	    case WAVEFORM_SAWTOOTH:
	        lcd_command(CLEAR);
	        lcd_putstring("Sawtooth Wave");
	        break;
	    case WAVEFORM_TRIANGLE:
	        lcd_command(CLEAR);
	        lcd_putstring("Triangle Wave");
	        break;
	    case WAVEFORM_PIANO:
	        lcd_command(CLEAR);
	        lcd_putstring("Piano Sample");
	        break;
	    case WAVEFORM_GUITAR:
	        lcd_command(CLEAR);
	        lcd_putstring("Guitar Sample");
	        break;
	    case WAVEFORM_DRUM:
	        lcd_command(CLEAR);
	        lcd_putstring("Drum Sample");
	        break;


	}

	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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



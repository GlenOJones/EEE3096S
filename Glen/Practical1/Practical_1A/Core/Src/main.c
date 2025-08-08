/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdint.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
#include <stdlib.h>  // for rand()
#include <stdbool.h>
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
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// TODO: Define input variables
#define SW0 GPIO_IDR_0
#define SW1 GPIO_IDR_1
#define SW2 GPIO_IDR_2
#define SW3 GPIO_IDR_3

uint32_t LastDebounceTime = 0;
uint8_t DebounceDelay =200;

volatile uint8_t display_mode=0;	//0 -> off, 1-> Mode 1, 2-> Mode 2, 3 -> Mode 3
volatile uint16_t current_speed= 0;  // delay time in ms

int8_t patterns1[10] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000, 0b01000000, 0b00100000};
int8_t patterns2[10] = {0b11111110, 0b11111101, 0b11111011, 0b11110111, 0b11101111, 0b11011111, 0b10111111, 0b01111111, 0b10111111, 0b11011111};
int8_t  mode = -1;
bool delay1 = false;

volatile uint16_t rand_on = 0;
volatile uint16_t rand_off = 0;
volatile uint16_t rand_LED = 0;

// Global variables for sparkle
volatile uint8_t mask;
volatile uint8_t counter = 0;
uint8_t time_off;
uint16_t time_on;
uint8_t slider;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Start timer TIM16
  HAL_TIM_Base_Start_IT(&htim16);

  init_LCD();
  lcd_command(CLEAR);
  lcd_putstring("2025 PRAC 1");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TODO: Check pushbuttons to change timer delay
	 // SWITCH 0 -> PRESSED
	  if (HAL_GPIO_ReadPin(GPIOA, SW0) == 0 && (HAL_GetTick() - LastDebounceTime > DebounceDelay)) {
	      lcd_command(CLEAR);
	      LastDebounceTime = HAL_GetTick(); // DEBOUNCE

	      if (current_speed == 500) {
	    	  // SWITCHING FROM 0.5S TO 1S
	          current_speed = 1000;
	          lcd_putstring("Speed: 1s");
	          __HAL_TIM_SET_AUTORELOAD(&htim16, 999);  // ASSUMING CORRECT 1KHZ
	      }
	      else {
	    	  // SWITCHING FROM 1S TO O.5S
	          current_speed = 500;
	          lcd_putstring("Speed: 0.5s");
	          __HAL_TIM_SET_AUTORELOAD(&htim16, 499);
	      }
	 }

	  if (HAL_GPIO_ReadPin(GPIOA, SW1) == 0 && (HAL_GetTick() - LastDebounceTime > DebounceDelay)){
	  		lcd_command(CLEAR);
	  		lcd_putstring("MODE: 1");
	  		display_mode = 1;
	  	}

	  if (HAL_GPIO_ReadPin(GPIOA, SW2) == 0 && (HAL_GetTick() - LastDebounceTime > DebounceDelay)){
	  		lcd_command(CLEAR);
	  		lcd_putstring("MODE: 2");
	  		display_mode=2;
	  	}
	  if (HAL_GPIO_ReadPin(GPIOA, SW3) == 0 && (HAL_GetTick() - LastDebounceTime > DebounceDelay)){
	  	  		lcd_command(CLEAR);
	  	  		lcd_putstring("MODE: 3");
	  	  		display_mode=3;
	  	  		counter=0;
	  	  	}




  }

    


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TIM16_IRQHandler(void)
{


    // 3) Static state for each mode’s sequence
    static uint8_t idx1 = 0, idx2 = 0;
    static bool dir1 = true, dir2 = true;

    switch (display_mode)
    {
    case 1:
    	__HAL_TIM_SET_AUTORELOAD(&htim16, current_speed-1);
        // Mode 1: one LED marches 0→7 then back
        LL_GPIO_WriteOutputPort(LED0_GPIO_Port, patterns1[idx1]);
        if (dir1) {
            if (++idx1 == 8) dir1 = false;
        } else {
            if (--idx1 == 0) dir1 = true;
        }
        break;

    case 2:
        // Mode 2: inverse of Mode 1 (all LEDs on except one)
    	__HAL_TIM_SET_AUTORELOAD(&htim16, current_speed-1);
        LL_GPIO_WriteOutputPort(LED0_GPIO_Port, patterns2[idx2]);
        if (dir2) {
            if (++idx2 == 8) dir2 = false;
        } else {
            if (--idx2 == 0) dir2 = true;
        }
        break;

    case 3:


       // lcd_putstring("ONE");
        // Mode 3: “sparkle” — random 8-bit pattern each interrupt

    	if (counter == 1){
    		// Initial display
    		//lcd_putstring("2");
    		LL_GPIO_WriteOutputPort(LED0_GPIO_Port, mask);
    		__HAL_TIM_SET_AUTORELOAD(&htim16, time_off);  // Time off delay initial, should be time_off

    	}
    	if(counter > 1 && counter <= 10){

    				// Normal iterations
    				// SETT ARR TO THE RANDOM GENERATED NUMBER
    				//lcd_putstring("3.");
    				slider = 0b11111110;


    				while(((( mask >> (counter-2)) & 1) == 0)  &&  (counter < 10)){
    					counter++;  // be wary of getting stuck in loop
    					//lcd_putstring("E");
    				}


    				LL_GPIO_WriteOutputPort(LED0_GPIO_Port, mask & (slider << (counter-2))); // Only when on LED found, 2 indices off
    				//LL_GPIO_WriteOutputPort(LED0_GPIO_Port, 0b11110000);
    				__HAL_TIM_SET_AUTORELOAD(&htim16, time_off);  // Time off delay

    	    	}



    	if(counter == 0){
			// If first iteration of pattern
			// DEFINING RANDOM NUMBERS
			mask = (uint8_t)(rand() & 0xFF); // LED output

			time_off = (uint16_t)rand() % 101; // Time off in turn - off sequence
			if (time_off == 0){
				time_off = mask % 101; // Ensuring not zero as it will crash delay
			}

			time_on = (uint16_t)(rand()) % 1500;
			if(time_on < 100){
				time_on = (uint16_t)1500-time_on;
			}

			slider = 0b11111110; // Reseting slider ( also an iterating value)
			LL_GPIO_WriteOutputPort(LED0_GPIO_Port, 0);  // Setting to zero output for current ARR time

			// done with first iteration (blank)
			//lcd_command(CLEAR);
			//lcd_putstring("1");

			//mask =  0b01100111; // HARDCODED FOR DEBUGGING
		    //time_off = (uint16_t)3;
			//time_on = (uint16_t)10000;

			// Displaying random vals
			char buff1[8] = {0};  // "65535" + null = 6 bytes (plus margin)
			char buff2[8] = {0};
			// 2. Format numbers (no manual null termination needed)
			snprintf(buff1, sizeof(buff1), "%u", (unsigned int)time_on);
			snprintf(buff2, sizeof(buff2), "%u", (unsigned int)time_off);
			lcd_command(CLEAR);
			lcd_command(0xC0); // line 2
			lcd_putstring(buff1);
			lcd_putstring(" , ");
			lcd_putstring(buff2);
			lcd_command(0x80); // line 1 again
			lcd_putstring("MODE: 3");


			__HAL_TIM_SET_AUTORELOAD(&htim16, time_on); // Setting ARR for ON delay (counts next count)

    	}

    	counter++;

    	if (counter >= 10){
    				// Resetting
    				counter = 0;
    				//lcd_putstring("R");
    				__HAL_TIM_SET_AUTORELOAD(&htim16, 400); // Setting ARR for OFF delay (counts next count)
    			}


		//Reset ARR
		// Output blank , break case 3
//			marker++;

//    	 if (marker == 8){
//    		uint_8 marker = 0;
//    		sparkleOn = false;
//    	}
//
			//ALSO SET ARRR in main

        // Looking for next on led
//

    	/*
		while(slider > 0){
			while(!(~slider<<1 & mask)){
				slider <<= 1;
			}

			mask &= slider;  // Turning off rightmost led
			LL_GPIO_WriteOutputPort(LED0_GPIO_Port, mask);
			HAL_Delay(time_off);

			// need to wait rand(100)

			//Interupt loop
		}

		*/
    	break;


    default:
        // No mode selected → turn all LEDs off
        LL_GPIO_WriteOutputPort(LED0_GPIO_Port, 0);
        break;
    }

    // 1) Acknowledge & clear TIM16 interrupt
        HAL_TIM_IRQHandler(&htim16);

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

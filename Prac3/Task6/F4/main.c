/* USER CODE BEGIN Header */
/**
 * Task 6 - Compiler Optimisations
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
#include <stdint.h>
#include "stm32f4xx.h"
#include <stddef.h>   // for size_t


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USE_TILING       1        // 0 = whole image (no big buffer), 1 = process in stripes
#define TILE_H           64       // stripe height when tiling (adjust to fit SRAM)
#define USE_FIXED_KERNEL 1        // 1 = use your fixed-point kernel; 0 = use double kernel
#define MAX_ITER         100      // Task 4 constraint
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define and initialise the global varibales required
/*
  start_time
  end_time
  execution_time
  checksum: should be uint64_t
  initial width and height maybe or you might opt for an array??
*/

// New Struct to store different resolutions

typedef struct { uint16_t w, h; } res_t;

////Task 4
///* Common progression up to Full HD */
//static const res_t kResList[] = {
//  {128, 128}, {256, 256}, {320, 240}, {640, 480}, {800, 600},
//  {1024, 768}, {1280, 720}, {1600, 900}, {1920, 1080}
//};

//Task 6
static const res_t kResList[] = {
  {128, 128}, {160, 160}, {192, 192}, {224, 224}, {256, 256}
};

static const size_t kNumRes = sizeof(kResList) / sizeof(kResList[0]);

typedef struct {
  uint16_t w, h;
  uint32_t ms;
  uint64_t checksum;
  uint32_t flash_bytes;   // NEW: code+rodata+data-init in flash
  uint32_t ram_bytes;     // NEW: data+bss in SRAM at runtime
  uint32_t bin_bytes;     // NEW: approx .bin size (use flash_bytes)
} result_t;


/* volatile so debugger/live expressions can always see it */
volatile result_t results[sizeof(kResList)/sizeof(kResList[0])];

uint64_t mandelbrot_tile_checksum(int x0, int y0, int tile_w, int tile_h,
                                  int total_w, int total_h, int max_it);
uint64_t mandelbrot_point_iter(int gx, int gy, int total_w, int total_h, int max_it,
                               int use_fixed);

uint32_t start_time = 0;
uint32_t end_time = 0;
uint32_t execution_time = 0;
uint64_t checksum = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

// Linker symbols provided by the STM32 default .ld script
extern uint8_t _sidata;   // start of .data init values in FLASH
extern uint8_t _sdata;    // start of .data in SRAM
extern uint8_t _edata;    // end   of .data in SRAM
extern uint8_t _sbss;     // start of .bss  in SRAM
extern uint8_t _ebss;     // end   of .bss  in SRAM

static inline uint32_t get_flash_used_bytes(void)
{
  // FLASH usage = size of .text/.rodata (from FLASH base up to _sidata)
  //             + size of .data initializers (edata - sdata)              // STM32F0 flash base
  uintptr_t sidata = (uintptr_t)&_sidata;
  uintptr_t sdata  = (uintptr_t)&_sdata;
  uintptr_t edata  = (uintptr_t)&_edata;

  uint32_t text_ro_bytes = (uint32_t)(sidata - (uintptr_t)FLASH_BASE);
  uint32_t data_init_bytes = (uint32_t)(edata - sdata);
  return text_ro_bytes + data_init_bytes;
}

static inline uint32_t get_ram_used_bytes(void)
{
  // RAM at runtime = .data (edata - sdata) + .bss (ebss - sbss)
  uintptr_t sdata = (uintptr_t)&_sdata;
  uintptr_t edata = (uintptr_t)&_edata;
  uintptr_t sbss  = (uintptr_t)&_sbss;
  uintptr_t ebss  = (uintptr_t)&_ebss;

  uint32_t data_bytes = (uint32_t)(edata - sdata);
  uint32_t bss_bytes  = (uint32_t)(ebss - sbss);
  return data_bytes + bss_bytes;
}


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
  /* USER CODE BEGIN 2 */

  //TODO: Turn on LED 0 to signify the start of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  uint32_t flash_bytes_once = get_flash_used_bytes();
  uint32_t ram_bytes_once   = get_ram_used_bytes();

  for (size_t r = 0; r < kNumRes; ++r) {  // use r < kNumRes (task 4) or 5 (task 6)
    const int W = kResList[r].w;
    const int H = kResList[r].h;
    checksum = 0;

    start_time = HAL_GetTick();

  #if USE_TILING
    /* Process the image in horizontal stripes of TILE_H */
    for (int y = 0; y < H; y += TILE_H) {
      const int th = ((y + TILE_H) <= H) ? TILE_H : (H - y);
      checksum += mandelbrot_tile_checksum(0, y, W, th, W, H, MAX_ITER);
      /* If you had to output pixels, you would stream this stripe here. */
    }
  #else
    /* No tiling: just loop all pixels (no framebuffer is fine if you only checksum) */
    for (int gy = 0; gy < H; ++gy) {
      for (int gx = 0; gx < W; ++gx) {
        checksum += mandelbrot_point_iter(gx, gy, W, H, MAX_ITER, USE_FIXED_KERNEL);
      }
    }
  #endif

    end_time = HAL_GetTick();
    execution_time = end_time - start_time;
    results[r].w = (uint16_t)W;
    results[r].h = (uint16_t)H;
    results[r].ms = execution_time;
    results[r].checksum = checksum;

    // NEW: same size values per build, copied into each row so Live Expressions shows them
    results[r].flash_bytes = flash_bytes_once;
    results[r].ram_bytes   = ram_bytes_once;
    results[r].bin_bytes   = flash_bytes_once;  // good approximation of .bin bytes

    // Record somewhere (UART/ITM/SWO): W, H, checksum, execution_time
  }
  /* Optional: blink per resolution, or breakpoints/log prints if you have UART */
     //TODO: Turn on LED 1 to signify the end of the operation
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
       HAL_Delay(100);

//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
  while (1) { /* idle */ }
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Mandelbroat using variable type integers and fixed point arithmetic
/* --- Per-pixel iterators that mirror the calculate_* functions exactly --- */
// Q16 fixed-point per-pixel iterator with constants precomputed outside the while-loop
static inline uint64_t mandelbrot_point_iter_fixed(int gx, int gy,
                                                   int total_w, int total_h,
                                                   int max_it)
{
  // ---- precompute Q16 constants (outside the iteration loop) ----
  const int32_t S          = 1 << 16;            // 65536
  const int32_t threeFiveS = (S * 7) >> 1;       // 3.5 * S  = 229376 (integer-only; no FP)
  const int32_t twoS       =  S << 1;            // 2.0 * S  = 131072
  const int32_t twoFiveS   = (S * 5) >> 1;       // 2.5 * S  = 163840
  const int32_t oneS       =  S;                 // 1.0 * S  = 65536
  const int64_t fourS2     = (int64_t)4 * (int64_t)S * (int64_t)S; // 4*S^2 (escape radius squared)

  // ---- map (gx, gy) to complex plane using Q16 (one division per axis) ----
  int32_t x0 = (int32_t)(((int64_t)gx * threeFiveS) / total_w) - twoFiveS; // ((gx/W)*3.5 - 2.5)*S
  int32_t y0 = (int32_t)(((int64_t)gy * twoS)       / total_h) - oneS;     // ((gy/H)*2.0 - 1.0)*S

  // ---- iteration state (Q16) ----
  int32_t xi = 0, yi = 0;
  int64_t xsq = 0, ysq = 0;   // keep squares in 64-bit to avoid overflow
  int i = 0;

  // ---- Mandelbrot iterate: use shifts (>>16) instead of divide-by-S ----
  while (i < max_it && (xsq + ysq) <= fourS2) {
    // tmp = (x^2 - y^2)/S + x0  (Q16)
    int32_t tmp = (int32_t)((xsq - ysq) >> 16) + x0;

    // yi = (2*x*y)/S + y0  (Q16)
    yi = (int32_t)((((int64_t)2 * xi * yi) >> 16) + y0);

    xi = tmp;
    ++i;

    // update squares for next escape check
    xsq = (int64_t)xi * xi;
    ysq = (int64_t)yi * yi;
  }

  return (uint64_t)i;
}


/* --- Keep this wrapper only if other code still calls mandelbrot_point_iter --- */
uint64_t mandelbrot_point_iter(int gx, int gy, int total_w, int total_h, int max_it,
                               int use_fixed)
{
  return use_fixed
    ? mandelbrot_point_iter_fixed(gx, gy, total_w, total_h, max_it)
    : mandelbrot_point_iter_double(gx, gy, total_w, total_h, max_it);
}

/* --- Tiled checksum that produces IDENTICAL sums to calculate_* over full image --- */
uint64_t mandelbrot_tile_checksum(int x0, int y0, int tile_w, int tile_h,
                                  int total_w, int total_h, int max_it)
{
  uint64_t sum = 0;
  for (int ty = 0; ty < tile_h; ++ty) {
    const int gy = y0 + ty;   // global y
    for (int tx = 0; tx < tile_w; ++tx) {
      const int gx = x0 + tx; // global x
#if USE_FIXED_KERNEL
      sum += mandelbrot_point_iter_fixed(gx, gy, total_w, total_h, max_it);
#else
      sum += mandelbrot_point_iter_double(gx, gy, total_w, total_h, max_it);
#endif
    }
  }
  return sum;
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

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"
#include "spi.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_spi.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Accelerometer and gyroscope control registers
#define LSM6DSL_CTRL1_XL 0x10
#define LSM6DSL_CTRL2_G 0x11
#define LSM6DSL_CTRL3_C 0x12
#define LSM6DSL_CTRL4_C 0x13
#define LSM6DSL_CTRL5_C 0x14
#define LSM6DSL_CTRL6_C 0x15
#define LSM6DSL_CTRL7_G 0x16
#define LSM6DSL_CTRL8_XL 0x17
#define LSM6DSL_CTRL9_XL 0x18
#define LSM6DSL_CTRL10_C 0x19

// Sensitivity conversions
// Angular Velocity (DPS) = Raw data * Sensitivity
// Ex:
// full scale range of 125, each raw unit corresponds to 4.375 DPS
// if we read 15, Angular Velocity = 15 * 4.375 = 65.625 DPS
#define LSM6DSL_GYRO_SENSITIVITY_FS_125DPS 4.375f
#define LSM6DSL_GYRO_SENSITIVITY_FS_250DPS 8.750f
#define LSM6DSL_GYRO_SENSITIVITY_FS_500DPS 17.500f
#define LSM6DSL_GYRO_SENSITIVITY_FS_1000DPS 35.000f
#define LSM6DSL_GYRO_SENSITIVITY_FS_2000DPS 70.000f

// Gyroscope output registers
#define LSM6DSL_OUTX_L_G 0x22
#define LSM6DSL_OUTX_H_G 0x23
#define LSM6DSL_OUTY_L_G 0x24
#define LSM6DSL_OUTY_H_G 0x25
#define LSM6DSL_OUTZ_L_G 0x26
#define LSM6DSL_OUTZ_H_G 0x27

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Full Scale Range configuration paramaters
// sets the degrees per second range we can acquire
typedef enum {
  LSM6DSL_250dps = 0,
  LSM6DSL_125dps = 1,
  LSM6DSL_500dps = 2,
  LSM6DSL_1000dps = 4,
  LSM6DSL_2000dps = 6
} lsm6dsl_fs_g_t;

// Output Data Rate configuration paramaters
// frequency at which the gyroscope outputs data (Hz)
typedef enum {
  LSM6DSL_GY_ODR_OFF = 0,
  LSM6DSL_GY_ODR_12Hz5 = 1,
  LSM6DSL_GY_ODR_26Hz = 2,
  LSM6DSL_GY_ODR_52Hz = 3,
  LSM6DSL_GY_ODR_104Hz = 4,
  LSM6DSL_GY_ODR_208Hz = 5,
  LSM6DSL_GY_ODR_416Hz = 6,
  LSM6DSL_GY_ODR_833Hz = 7,
  LSM6DSL_GY_ODR_1k66Hz = 8,
  LSM6DSL_GY_ODR_3k33Hz = 9,
  LSM6DSL_GY_ODR_6k66Hz = 10
} lsm6dsl_odr_g_t;

// Packed Raw Data
typedef union {
  uint8_t bytes[6];

  struct __attribute__((packed)) // ensure no padding
  {
    int16_t x;
    int16_t y;
    int16_t z;
  };
} LSM6DSL_AxesRaw_t;

// 3 axis data (post processing)
typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} LSM6DSL_Axes_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void lsm6_read(uint8_t address, uint8_t *rxData);
void lsm6_write(uint8_t address, uint8_t *txData);
void lsm6_init();
void lsm6_readGyro(LSM6DSL_AxesRaw_t *axes);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lsm6_read(uint8_t address, uint8_t *rxData) {
  uint8_t DataSize = 2;
  uint8_t TxData[] = {address | 0x80, 0};
  uint8_t RxData[] = {0, 0};

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, TxData, RxData, DataSize, 10000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  *rxData = RxData[1];
}

void lsm6_write(uint8_t address, uint8_t *txData) {
  uint8_t DataSize = 2;
  uint8_t TxData[] = {address, *txData};

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, TxData, DataSize, 10000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}

void lsm6_init() {
  uint8_t data = 0;

  // Important registers: 
  // CTRL2_G (11h)
  // ODR_G[7:4] Output Data Rate
  // FS_G[3:2] Full-Scale Selection
  // FS_125[1:1] Full scale == 125 dps
  // NULL [0:0] set zero
  data = (0b0011<< 4) | (0b11<< 2 | 0b00);
  lsm6_write((0x11), &data);


  // CTRL7_G (16h)
  // [7:7]  set to 0 to enable high performance mode
  // [6:6]  set to 1 to enable high-pass filter
  // [5:4]  high-pass filter cutoff frequency
  // [3:3]  digital hp filter reset  (0: disabled)
  // [2:2]  rounding function enable (0: disabled)
  // [1:0]  leave 0

  // filter table:
  // 00 - 0.0081 hz
  // 01 - 0.0324 hz
  // 10 - 2.07 hz
  // 11 - 16.32 hz

  data = (0b0 << 7) | (0b1 << 6) | (0b11 << 4) | 0b0000;
  lsm6_write((0x16),&data);

}

void lsm6_readGyro_RAW(LSM6DSL_AxesRaw_t *axes) {
  uint8_t addr[] = {0x22 | 0x80, 0, 0x23 | 0x80, 0, 0x24 | 0x80, 0,
                    0x25 | 0x80, 0, 0x26 | 0x80, 0, 0x27 | 0x80, 0};

  // gyro is addr 0x22 - 0x27
  // we want to continously read addr 0x22 - 0x27
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, addr, (uint8_t*)axes, 12, 10000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // SPI serial message
  uint8_t UARTmsg[50];

  // spi package
  uint8_t addr;
  uint8_t txData;
  uint8_t rxData;

  HAL_StatusTypeDef status;
  lsm6_init();

  LSM6DSL_AxesRaw_t gyro_raw;

  while (1) {

    // read WHO_AM_I
    // addr = 0x0F;
    // lsm6_read(addr, &rxData);

    lsm6_readGyro_RAW(&gyro_raw);

    // snprintf((char *)SPImsg, sizeof(SPImsg), "WHO_AM_I: 0x%x\n\r", rxData);

    snprintf((char *)UARTmsg, sizeof(UARTmsg), "%i %i %i\n\r", gyro_raw.x, gyro_raw.y, gyro_raw.z);

    HAL_UART_Transmit(&huart1, (uint8_t *)UARTmsg, strlen((char *)UARTmsg), 200);

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

    HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

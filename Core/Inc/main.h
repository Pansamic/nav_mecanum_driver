/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEDR_Pin GPIO_PIN_11
#define LEDR_GPIO_Port GPIOF
#define LEDB_Pin GPIO_PIN_12
#define LEDB_GPIO_Port GPIOF
#define LEDG_Pin GPIO_PIN_13
#define LEDG_GPIO_Port GPIOF
#define KEY3_Pin GPIO_PIN_14
#define KEY3_GPIO_Port GPIOF
#define KEY2_Pin GPIO_PIN_15
#define KEY2_GPIO_Port GPIOF
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOG
#define SPI2_CS1_Pin GPIO_PIN_1
#define SPI2_CS1_GPIO_Port GPIOG
#define SPI2_CS2_Pin GPIO_PIN_7
#define SPI2_CS2_GPIO_Port GPIOE
#define SPI2_CS3_Pin GPIO_PIN_8
#define SPI2_CS3_GPIO_Port GPIOE
#define M2IN1_Pin GPIO_PIN_3
#define M2IN1_GPIO_Port GPIOD
#define M2IN2_Pin GPIO_PIN_4
#define M2IN2_GPIO_Port GPIOD
#define M4IN1_Pin GPIO_PIN_5
#define M4IN1_GPIO_Port GPIOD
#define M4IN2_Pin GPIO_PIN_6
#define M4IN2_GPIO_Port GPIOD
#define M1IN2_Pin GPIO_PIN_7
#define M1IN2_GPIO_Port GPIOD
#define M1IN1_Pin GPIO_PIN_10
#define M1IN1_GPIO_Port GPIOG
#define M3IN1_Pin GPIO_PIN_11
#define M3IN1_GPIO_Port GPIOG
#define M3IN2_Pin GPIO_PIN_12
#define M3IN2_GPIO_Port GPIOG
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

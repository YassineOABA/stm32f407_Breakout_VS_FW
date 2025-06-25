/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define FLASH_WP_Pin GPIO_PIN_13
#define FLASH_WP_GPIO_Port GPIOC
#define FLASH_HLD_Pin GPIO_PIN_14
#define FLASH_HLD_GPIO_Port GPIOC
#define BMI160_NSS_Pin GPIO_PIN_4
#define BMI160_NSS_GPIO_Port GPIOA
#define BMI160_SCK_Pin GPIO_PIN_5
#define BMI160_SCK_GPIO_Port GPIOA
#define BMI160_MISO_Pin GPIO_PIN_6
#define BMI160_MISO_GPIO_Port GPIOA
#define BMI160_MOSI_Pin GPIO_PIN_7
#define BMI160_MOSI_GPIO_Port GPIOA
#define BMI160_INT1_Pin GPIO_PIN_4
#define BMI160_INT1_GPIO_Port GPIOC
#define BMI160_INT1_EXTI_IRQn EXTI4_IRQn
#define BMI160_INT2_Pin GPIO_PIN_5
#define BMI160_INT2_GPIO_Port GPIOC
#define FLASH_NSS_Pin GPIO_PIN_15
#define FLASH_NSS_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_10
#define FLASH_SCK_GPIO_Port GPIOC
#define FLASH_MISO_Pin GPIO_PIN_11
#define FLASH_MISO_GPIO_Port GPIOC
#define FLASH_MOSI_Pin GPIO_PIN_12
#define FLASH_MOSI_GPIO_Port GPIOC
#define DEBUG_LED_Pin GPIO_PIN_6
#define DEBUG_LED_GPIO_Port GPIOD
#define BME280_SCL_Pin GPIO_PIN_6
#define BME280_SCL_GPIO_Port GPIOB
#define BME280_SDA_Pin GPIO_PIN_7
#define BME280_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

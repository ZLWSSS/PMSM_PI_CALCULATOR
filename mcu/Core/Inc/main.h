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
#define V_BUS_SEN_Pin GPIO_PIN_2
#define V_BUS_SEN_GPIO_Port GPIOC
#define Temp_Sen_Pin GPIO_PIN_3
#define Temp_Sen_GPIO_Port GPIOC
#define Test_Pin_1_Pin GPIO_PIN_2
#define Test_Pin_1_GPIO_Port GPIOA
#define Test_Pin_2_Pin GPIO_PIN_3
#define Test_Pin_2_GPIO_Port GPIOA
#define DRV_CS_Pin GPIO_PIN_4
#define DRV_CS_GPIO_Port GPIOA
#define Blue_LED_Pin GPIO_PIN_5
#define Blue_LED_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define DRV_Enable_pin_Pin GPIO_PIN_11
#define DRV_Enable_pin_GPIO_Port GPIOA
#define AS5047_CS_Pin GPIO_PIN_15
#define AS5047_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

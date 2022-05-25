/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_5
#define LCD_RS_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_0
#define LCD_DC_GPIO_Port GPIOB
#define LCD_LED_Pin GPIO_PIN_1
#define LCD_LED_GPIO_Port GPIOB
#define SIM_PWR_Pin GPIO_PIN_2
#define SIM_PWR_GPIO_Port GPIOB
#define TCH_CS_Pin GPIO_PIN_12
#define TCH_CS_GPIO_Port GPIOB
#define TCH_IRQ_Pin GPIO_PIN_6
#define TCH_IRQ_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

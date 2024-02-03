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
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define PCTRL0_Pin GPIO_PIN_1
#define PCTRL0_GPIO_Port GPIOC
#define PCTRL1_Pin GPIO_PIN_2
#define PCTRL1_GPIO_Port GPIOC
#define PPS_Pin GPIO_PIN_3
#define PPS_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define Touch_EXT_Pin GPIO_PIN_5
#define Touch_EXT_GPIO_Port GPIOC
#define Touch_EXT_EXTI_IRQn EXTI9_5_IRQn
#define B0_Pin GPIO_PIN_0
#define B0_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_1
#define B1_GPIO_Port GPIOB
#define B2_Pin GPIO_PIN_2
#define B2_GPIO_Port GPIOB
#define B10_Pin GPIO_PIN_10
#define B10_GPIO_Port GPIOB
#define B11_Pin GPIO_PIN_11
#define B11_GPIO_Port GPIOB
#define B12_Pin GPIO_PIN_12
#define B12_GPIO_Port GPIOB
#define B13_Pin GPIO_PIN_13
#define B13_GPIO_Port GPIOB
#define B14_Pin GPIO_PIN_14
#define B14_GPIO_Port GPIOB
#define B15_Pin GPIO_PIN_15
#define B15_GPIO_Port GPIOB
#define BL_Pin GPIO_PIN_6
#define BL_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_7
#define CS_GPIO_Port GPIOC
#define RS_Pin GPIO_PIN_8
#define RS_GPIO_Port GPIOC
#define WR_Pin GPIO_PIN_9
#define WR_GPIO_Port GPIOC
#define RD_Pin GPIO_PIN_8
#define RD_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_9
#define RST_GPIO_Port GPIOA
#define B3_Pin GPIO_PIN_3
#define B3_GPIO_Port GPIOB
#define B4_Pin GPIO_PIN_4
#define B4_GPIO_Port GPIOB
#define B5_Pin GPIO_PIN_5
#define B5_GPIO_Port GPIOB
#define B6_Pin GPIO_PIN_6
#define B6_GPIO_Port GPIOB
#define B7_Pin GPIO_PIN_7
#define B7_GPIO_Port GPIOB
#define B8_Pin GPIO_PIN_8
#define B8_GPIO_Port GPIOB
#define B9_Pin GPIO_PIN_9
#define B9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

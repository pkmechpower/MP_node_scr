/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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
#define LED_RED_Pin GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOA
#define LED1_KATODA_Pin GPIO_PIN_2
#define LED1_KATODA_GPIO_Port GPIOA
#define LED2_KATODA_Pin GPIO_PIN_3
#define LED2_KATODA_GPIO_Port GPIOA
#define LED3_KATODA_Pin GPIO_PIN_4
#define LED3_KATODA_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_5
#define CLK_GPIO_Port GPIOA
#define SERIAL_OUTPUT_Pin GPIO_PIN_6
#define SERIAL_OUTPUT_GPIO_Port GPIOA
#define LED4_KATODA_Pin GPIO_PIN_7
#define LED4_KATODA_GPIO_Port GPIOA
#define PL_Pin GPIO_PIN_1
#define PL_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_8
#define CE_GPIO_Port GPIOA
#define SWITCH1_Pin GPIO_PIN_3
#define SWITCH1_GPIO_Port GPIOB
#define SWITCH2_Pin GPIO_PIN_4
#define SWITCH2_GPIO_Port GPIOB
#define SWITCH3_Pin GPIO_PIN_5
#define SWITCH3_GPIO_Port GPIOB
#define SWITCH4_Pin GPIO_PIN_6
#define SWITCH4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

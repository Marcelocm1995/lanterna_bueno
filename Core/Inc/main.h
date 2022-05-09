/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
{
  uint8_t State;
  uint16_t Timer;
}vBUT;

typedef enum
{
  NO_CLICK,
  SHORT_CLICK,
  MEDIUM_CLICK,
  LONG_CLICK
}BUTTON_STATES;

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
#define BAT_MON_Pin GPIO_PIN_0
#define BAT_MON_GPIO_Port GPIOA
#define PWM_LED_SUP_Pin GPIO_PIN_0
#define PWM_LED_SUP_GPIO_Port GPIOB
#define PWM_LED_INF_Pin GPIO_PIN_1
#define PWM_LED_INF_GPIO_Port GPIOB
#define BUT_MINUS_Pin GPIO_PIN_12
#define BUT_MINUS_GPIO_Port GPIOB
#define BUT_MINUS_EXTI_IRQn EXTI15_10_IRQn
#define BUT_PLUS_Pin GPIO_PIN_13
#define BUT_PLUS_GPIO_Port GPIOB
#define BUT_PLUS_EXTI_IRQn EXTI15_10_IRQn
#define BUT_MENU_Pin GPIO_PIN_14
#define BUT_MENU_GPIO_Port GPIOB
#define BUT_MENU_EXTI_IRQn EXTI15_10_IRQn
#define POWER_CMD_Pin GPIO_PIN_15
#define POWER_CMD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SHORT_CLICK_DEBOUNCE  100-1
#define MEDIUM_CLICK_DEBOUNCE 1500-1
#define LONG_CLICK_DEBOUNCE   3000-1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

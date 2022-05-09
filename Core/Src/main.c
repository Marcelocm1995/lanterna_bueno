/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ssd1306.h"
#include "button.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define POWER_CMD(a) HAL_GPIO_WritePin(POWER_CMD_GPIO_Port, POWER_CMD_Pin, a)
#define LIGHT_PWM(a) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, a)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t LIGHT_LEVEL = 0,
				STATE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	ssd1306_Init();
  ssd1306_FlipScreenVertically();
  ssd1306_Clear();
  ssd1306_SetColor(White);
	ssd1306_SetCursor (2,0);
	ssd1306_WriteString( "Bueno" , Font_16x26);
	ssd1306_SetCursor (30,0);
	ssd1306_WriteString( "Light" , Font_16x26);
	ssd1306_UpdateScreen();
	
	if(CheckButtonState() >= MEDIUM_CLICK)
	{
		POWER_CMD(1);
	}
	
	else
	{
		POWER_CMD(0);
	}
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	LIGHT_PWM(0);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if( (CheckButtonState() == SHORT_CLICK) && (STATE == 0) )
		{
			STATE = 1;
			LIGHT_LEVEL++;
			if(LIGHT_LEVEL > 4)
				LIGHT_LEVEL = 0;
			
			switch (LIGHT_LEVEL)
			{
				case 0: LIGHT_PWM(0);  ssd1306_SetCursor (2,0); ssd1306_WriteString( "Power 0" , Font_16x26); ssd1306_UpdateScreen();	break;		
				case 1: LIGHT_PWM(20); ssd1306_SetCursor (2,0); ssd1306_WriteString( "Power 1" , Font_16x26); ssd1306_UpdateScreen();	break;
				case 2: LIGHT_PWM(40); ssd1306_SetCursor (2,0); ssd1306_WriteString( "Power 2" , Font_16x26); ssd1306_UpdateScreen();	break;
				case 3: LIGHT_PWM(60); ssd1306_SetCursor (2,0); ssd1306_WriteString( "Power 3" , Font_16x26); ssd1306_UpdateScreen();	break;
				case 4: LIGHT_PWM(80); ssd1306_SetCursor (2,0); ssd1306_WriteString( "Power 4" , Font_16x26); ssd1306_UpdateScreen();	break;
			}
		}
		
		else if( (CheckButtonState() == LONG_CLICK) && (STATE == 0) )
		{
			STATE = 1;
			ssd1306_SetCursor (2,0); 
			ssd1306_WriteString( "Sleep" , Font_16x26);
			ssd1306_UpdateScreen();
			HAL_Delay(1000);
			POWER_CMD(0);
		}
		
		else
		{
			STATE = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

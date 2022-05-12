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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ssd1306.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  LV0 = 0,
  LV1,
  LV2,
  LV3,
  LV4
}LIGHT_LEVEL;

typedef enum
{
  CITY = 0,
  TRAIL,
  MENU_TYPE_QTD
}MENU_TYPE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Avg_Slope .0025
#define V25 0.76

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define POWER_CMD(a) HAL_GPIO_WritePin(POWER_CMD_GPIO_Port, POWER_CMD_Pin, a)
#define LIGHT_PWM_SUP(a) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, a)
#define LIGHT_PWM_INF(a) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, a)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char Str_LCD[64];

uint8_t SUP_LIGHT_LEVEL = 0,
        INF_LIGHT_LEVEL = 0,
        MENU_MODE = CITY;

uint16_t ADC_CONVERSION_TIMER = 0,
         UptadeLightLevelTimer = 0,
         RefreshDisplayTimer = 0,
				 ADC_VAL[2];

float TEMP,
			BAT_PCT;

vBUT MENU_BUTTON,
     PLUS_BUTTON,
     MINUS_BUTTON;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

float Map (float inVal, float inMin, float inMax, float outMin, float outMax);

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	ssd1306_Init();
  ssd1306_FlipScreenVertically();
  ssd1306_Clear();
  ssd1306_SetColor(White);
	ssd1306_SetCursor(2,0);
	ssd1306_WriteString("Bueno", Font_16x26);
	ssd1306_SetCursor(30,0);
	ssd1306_WriteString("Light", Font_16x26);
	ssd1306_UpdateScreen();

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	LIGHT_PWM_SUP(SUP_LIGHT_LEVEL);
	LIGHT_PWM_INF(INF_LIGHT_LEVEL);

  while(MENU_BUTTON.State < MEDIUM_CLICK);

  POWER_CMD(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*Start Menu Button Routine*/
		if(MENU_BUTTON.State == LONG_CLICK)
		{
      POWER_CMD(0);
    }
    else if(MENU_BUTTON.State == MEDIUM_CLICK)
    {
      MENU_MODE++;
    }
    else if(MENU_BUTTON.State == SHORT_CLICK)
    {
      if(MENU_MODE == CITY)
      {
        if(SUP_LIGHT_LEVEL != LV0)
          SUP_LIGHT_LEVEL = LV0;
        else
          SUP_LIGHT_LEVEL = LV4;
      }
    }
    else
    {
      /*NO_CLICK (do nothing)*/
    }
    /*End Menu Button Routine*/

    /*Start Plus Button Routine*/
    if(PLUS_BUTTON.State == LONG_CLICK)
		{

    }
    else if(PLUS_BUTTON.State == MEDIUM_CLICK)
    {

    }
    else if(PLUS_BUTTON.State == SHORT_CLICK)
    {
      if(MENU_MODE == TRAIL)
        INF_LIGHT_LEVEL++;
    }
    else
    {
      /*NO_CLICK (do nothing)*/
    }
    /*End Plus Button Routine*/

    /*Start Minus Button Routine*/
    if(MINUS_BUTTON.State == LONG_CLICK)
		{

    }
    else if(MINUS_BUTTON.State == MEDIUM_CLICK)
    {

    }
    else if(MINUS_BUTTON.State == SHORT_CLICK)
    {
      if(MENU_MODE == TRAIL)
        INF_LIGHT_LEVEL--;
    }
    else
    {
      /*NO_CLICK (do nothing)*/
    }
    /*End Minus Button Routine*/

    /*Start of saturation light levels*/
    if(INF_LIGHT_LEVEL > LV4)
      INF_LIGHT_LEVEL = LV4;
    if(INF_LIGHT_LEVEL < LV0)
      INF_LIGHT_LEVEL = LV0;

    if(SUP_LIGHT_LEVEL > LV4)
      SUP_LIGHT_LEVEL = LV4;
    if(SUP_LIGHT_LEVEL < LV0)
      SUP_LIGHT_LEVEL = LV0;
    /*End of saturation light levels*/

    if(MENU_MODE > MENU_TYPE_QTD-1)
      MENU_MODE = CITY;

    if(RefreshDisplayTimer > 100)
    {
      RefreshDisplayTimer = 0;

      if(MENU_MODE == CITY)
			{
        ssd1306_SetCursor(2,0);
				ssd1306_WriteString("CITY", Font_7x10);
			}
      else if(MENU_MODE == TRAIL)
			{
        ssd1306_SetCursor(2,0);
				ssd1306_WriteString("TRAIL", Font_7x10);
			}
      else
      {
				/*do nothing*/
			}
      if(MENU_MODE == TRAIL)
      {
        if(SUP_LIGHT_LEVEL == LV0) sprintf(Str_LCD, "Level 0");
        else if(SUP_LIGHT_LEVEL == LV1) sprintf(Str_LCD, "Level 1");
        else if(SUP_LIGHT_LEVEL == LV2) sprintf(Str_LCD, "Level 2");
        else if(SUP_LIGHT_LEVEL == LV3) sprintf(Str_LCD, "Level 3");
        else if(SUP_LIGHT_LEVEL == LV4) sprintf(Str_LCD, "Level 4");
        else { /*do nothing*/ }
        ssd1306_SetCursor(2,64);
				ssd1306_WriteString(Str_LCD, Font_7x10);
      }
      else
      {
        ssd1306_SetCursor(2,64);
				ssd1306_WriteString("       ", Font_7x10);
      }

			ssd1306_UpdateScreen();
    }

    /*Start of Task for update light levels*/
    if(UptadeLightLevelTimer > 100)
    {
      UptadeLightLevelTimer = 0;

      if(SUP_LIGHT_LEVEL == LV0)
        LIGHT_PWM_SUP(0);
      else if(SUP_LIGHT_LEVEL == LV1)
        LIGHT_PWM_SUP(20);
      else if(SUP_LIGHT_LEVEL == LV2)
        LIGHT_PWM_SUP(40);
      else if(SUP_LIGHT_LEVEL == LV3)
        LIGHT_PWM_SUP(60);
      else if(SUP_LIGHT_LEVEL == LV4)
        LIGHT_PWM_SUP(80);
      else
        {/*do nothing*/}

      if(INF_LIGHT_LEVEL == LV0)
        LIGHT_PWM_INF(0);
      else if(INF_LIGHT_LEVEL == LV1)
        LIGHT_PWM_INF(20);
      else if(INF_LIGHT_LEVEL == LV2)
        LIGHT_PWM_INF(40);
      else if(INF_LIGHT_LEVEL == LV3)
        LIGHT_PWM_INF(60);
      else if(INF_LIGHT_LEVEL == LV4)
        LIGHT_PWM_INF(80);
      else
        {/*do nothing*/}
    }
	  /*End of Task for update light levels*/

		/*Reads Battery level and temperature periodically (every 2 seconds)*/
		if(ADC_CONVERSION_TIMER > 2000)
		{
			ADC_CONVERSION_TIMER = 0;

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);
			ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			HAL_ADC_PollForConversion(&hadc1, 1000);
			ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			TEMP = ((3.3*ADC_VAL[1]/4095 - V25)/Avg_Slope)+25;

			float BAT_VOLTAGE = 3.3*ADC_VAL[0]/4095;

			BAT_PCT = Map(BAT_VOLTAGE, 3.3, 4.1, 0, 100);

			sprintf(Str_LCD, "%.0f%", BAT_PCT);
			ssd1306_SetCursor (0,0); ssd1306_WriteString( Str_LCD, Font_7x10); ssd1306_UpdateScreen();
			sprintf(Str_LCD, "%.1fC", TEMP);
			ssd1306_SetCursor (0,0); ssd1306_WriteString( Str_LCD, Font_7x10); ssd1306_UpdateScreen();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float Map (float inVal, float inMin, float inMax, float outMin, float outMax)
{
	if(inVal<inMin)
		inVal=inMin;

	if(inVal>inMax)
		inVal=inMax;

	return ( (inVal - inMin)*(outMax - outMin)/(inMax - inMin) + outMin );
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

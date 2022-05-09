/**
  ******************************************************************************
  * @file    button.c
  * @brief   This file provides code for the configuration
  *          of the button instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Nucleo Racing.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "button.h"

uint32_t Button_Timer = 0;

uint8_t CheckButtonState()
{
	if(HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin) == BUTTON_TYPE)
	{
		Button_Timer = HAL_GetTick();
		while(HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin) == BUTTON_TYPE)
		{
			if(HAL_GetTick() > Button_Timer + 6000)
			break;
		}
		if(HAL_GetTick() > Button_Timer + 4000)
		return LONG_CLICK;
		
		else if(HAL_GetTick() > Button_Timer + 1500)
		return MEDIUM_CLICK;
		
		else if (HAL_GetTick() > Button_Timer + 100)
		return SHORT_CLICK;
		
		else
		return NO_PRESSED;
	}
	return NO_PRESSED;
}

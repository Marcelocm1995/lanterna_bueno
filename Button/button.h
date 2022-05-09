/**
  ******************************************************************************
  * @file    button.h
  * @brief   This file contains all the function prototypes for
  *          the button.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Nucleo Racing.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __button_H__
#define __button_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define LONG_CLICK				3
#define MEDIUM_CLICK			2
#define SHORT_CLICK 			1
#define NO_PRESSED				0

#define PULL_UP 0
#define PULL_DOWN 1

#define BUTTON_TYPE PULL_UP

extern uint32_t Button_Timer;

uint8_t CheckButtonState(void);

#endif /* __button_H__ */

/************************ (C) COPYRIGHT Nucleo Racing *****END OF FILE****/

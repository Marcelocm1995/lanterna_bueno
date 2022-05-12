/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE BEGIN EV */

extern vBUT MENU_BUTTON,
            PLUS_BUTTON,
            MINUS_BUTTON;

extern uint16_t ADC_CONVERSION_TIMER,
                UptadeLightLevelTimer,
                RefreshDisplayTimer;

uint8_t BUT_MENU_SIGNAL = 1, OLD_BUT_MENU_SIGNAL = 1, BUT_MENU_RISING_EDGE_DETECTED = 0,
        BUT_PLUS_SIGNAL = 1, OLD_BUT_PLUS_SIGNAL = 1, BUT_PLUS_RISING_EDGE_DETECTED = 0,
        BUT_MINUS_SIGNAL = 1, OLD_BUT_MINUS_SIGNAL = 1, BUT_MINUS_RISING_EDGE_DETECTED = 0;


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	ADC_CONVERSION_TIMER++;
  UptadeLightLevelTimer++;
  RefreshDisplayTimer++;

  BUT_MENU_SIGNAL = HAL_GPIO_ReadPin(BUT_MENU_GPIO_Port, BUT_MENU_Pin);
	BUT_PLUS_SIGNAL = HAL_GPIO_ReadPin(BUT_PLUS_GPIO_Port, BUT_PLUS_Pin);
  BUT_MINUS_SIGNAL = HAL_GPIO_ReadPin(BUT_MINUS_GPIO_Port, BUT_MINUS_Pin);

/******************************************************************************************************************/
	/*detecta borda de descida*/
	if( (OLD_BUT_MENU_SIGNAL == 1) && (BUT_MENU_SIGNAL == 0) )
	{
		OLD_BUT_MENU_SIGNAL = BUT_MENU_SIGNAL;
		MENU_BUTTON.StartTimer = 1;
	  BUT_MENU_RISING_EDGE_DETECTED = 0;
	}
	
	/*detecta borda de subida*/
	if( (OLD_BUT_MENU_SIGNAL == 0) && (BUT_MENU_SIGNAL == 1) ) 
	{
		OLD_BUT_MENU_SIGNAL = BUT_MENU_SIGNAL;
		MENU_BUTTON.StartTimer = 0;
		BUT_MENU_RISING_EDGE_DETECTED = 1;
	}
	
	if(MENU_BUTTON.StartTimer == 1)
	{
		MENU_BUTTON.Timer++;
	}	
	
  if( (MENU_BUTTON.Timer > LONG_CLICK_DEBOUNCE) && (BUT_MENU_RISING_EDGE_DETECTED == 1) )
  {
    MENU_BUTTON.State = LONG_CLICK;
		MENU_BUTTON.StartTimer = 0;
		MENU_BUTTON.Timer = 0;
  }
  else if( (MENU_BUTTON.Timer > MEDIUM_CLICK_DEBOUNCE) && (BUT_MENU_RISING_EDGE_DETECTED == 1) )
  {
    MENU_BUTTON.State = MEDIUM_CLICK;
		MENU_BUTTON.StartTimer = 0;
		MENU_BUTTON.Timer = 0;
  }
  else if( (MENU_BUTTON.Timer > SHORT_CLICK_DEBOUNCE) && (BUT_MENU_RISING_EDGE_DETECTED == 1) )
  {
    MENU_BUTTON.State = SHORT_CLICK;
		MENU_BUTTON.StartTimer = 0;
		MENU_BUTTON.Timer = 0;
  }
  else
  {
    MENU_BUTTON.State = NO_CLICK;
  }
/*-----------------------------------------------------------------------------------*/

/******************************************************************************************************************/
	/*detecta borda de descida*/
	if( (OLD_BUT_PLUS_SIGNAL == 1) && (BUT_PLUS_SIGNAL == 0) )
	{
		OLD_BUT_PLUS_SIGNAL = BUT_PLUS_SIGNAL;
		PLUS_BUTTON.StartTimer = 1;
	  BUT_PLUS_RISING_EDGE_DETECTED = 0;
	}
	
	/*detecta borda de subida*/
	if( (OLD_BUT_PLUS_SIGNAL == 0) && (BUT_PLUS_SIGNAL == 1) ) 
	{
		OLD_BUT_PLUS_SIGNAL = BUT_PLUS_SIGNAL;
		PLUS_BUTTON.StartTimer = 0;
		BUT_PLUS_RISING_EDGE_DETECTED = 1;
	}
	
	if(PLUS_BUTTON.StartTimer == 1)
	{
		PLUS_BUTTON.Timer++;
	}	
	
  if( (PLUS_BUTTON.Timer > LONG_CLICK_DEBOUNCE) && (BUT_PLUS_RISING_EDGE_DETECTED == 1) )
  {
    PLUS_BUTTON.State = LONG_CLICK;
		PLUS_BUTTON.StartTimer = 0;
		PLUS_BUTTON.Timer = 0;
  }
  else if( (PLUS_BUTTON.Timer > MEDIUM_CLICK_DEBOUNCE) && (BUT_PLUS_RISING_EDGE_DETECTED == 1) )
  {
    PLUS_BUTTON.State = MEDIUM_CLICK;
		PLUS_BUTTON.StartTimer = 0;
		PLUS_BUTTON.Timer = 0;
  }
  else if( (PLUS_BUTTON.Timer > SHORT_CLICK_DEBOUNCE) && (BUT_PLUS_RISING_EDGE_DETECTED == 1) )
  {
    PLUS_BUTTON.State = SHORT_CLICK;
		PLUS_BUTTON.StartTimer = 0;
		PLUS_BUTTON.Timer = 0;
  }
  else
  {
    PLUS_BUTTON.State = NO_CLICK;
  }
/*-----------------------------------------------------------------------------------*/

/******************************************************************************************************************/
	/*detecta borda de descida*/
	if( (OLD_BUT_MINUS_SIGNAL == 1) && (BUT_MINUS_SIGNAL == 0) )
	{
		OLD_BUT_MINUS_SIGNAL = BUT_MINUS_SIGNAL;
		MINUS_BUTTON.StartTimer = 1;
	  BUT_MINUS_RISING_EDGE_DETECTED = 0;
	}
	
	/*detecta borda de subida*/
	if( (OLD_BUT_MINUS_SIGNAL == 0) && (BUT_MINUS_SIGNAL == 1) ) 
	{
		OLD_BUT_MINUS_SIGNAL = BUT_MINUS_SIGNAL;
		MINUS_BUTTON.StartTimer = 0;
		BUT_MINUS_RISING_EDGE_DETECTED = 1;
	}
	
	if(MINUS_BUTTON.StartTimer == 1)
	{
		MINUS_BUTTON.Timer++;
	}	
	
  if( (MINUS_BUTTON.Timer > LONG_CLICK_DEBOUNCE) && (BUT_MINUS_RISING_EDGE_DETECTED == 1) )
  {
    MINUS_BUTTON.State = LONG_CLICK;
		MINUS_BUTTON.StartTimer = 0;
		MINUS_BUTTON.Timer = 0;
  }
  else if( (MINUS_BUTTON.Timer > MEDIUM_CLICK_DEBOUNCE) && (BUT_MINUS_RISING_EDGE_DETECTED == 1) )
  {
    MINUS_BUTTON.State = MEDIUM_CLICK;
		MINUS_BUTTON.StartTimer = 0;
		MINUS_BUTTON.Timer = 0;
  }
  else if( (MINUS_BUTTON.Timer > SHORT_CLICK_DEBOUNCE) && (BUT_MINUS_RISING_EDGE_DETECTED == 1) )
  {
    MINUS_BUTTON.State = SHORT_CLICK;
		MINUS_BUTTON.StartTimer = 0;
		MINUS_BUTTON.Timer = 0;
  }
  else
  {
    MINUS_BUTTON.State = NO_CLICK;
  }
/*-----------------------------------------------------------------------------------*/
	
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

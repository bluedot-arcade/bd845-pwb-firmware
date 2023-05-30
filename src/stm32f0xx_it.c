/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 BlueDot Arcade.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"
#include "stm32f0xx_it.h"

extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  while (1)
  {

  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {

  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(COMM_TEST_Pin);
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}
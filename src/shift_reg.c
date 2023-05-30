/**
  ******************************************************************************
  * @file           : shift_reg.c
  * @brief          : Implements the shift register utility.
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
 
#include "shift_reg.h"

void ShiftReg_Init(ShiftReg_TypeDef* ShiftReg)
{
  HAL_GPIO_WritePin(ShiftReg->Ser_Port, ShiftReg->Ser_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ShiftReg->Srclk_Port, ShiftReg->Srclk_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ShiftReg->Rclk_Port, ShiftReg->Rclk_Pin, GPIO_PIN_RESET);
}

void ShiftReg_ShiftBit(ShiftReg_TypeDef* ShiftReg, uint8_t bit)
{
  HAL_GPIO_WritePin(ShiftReg->Ser_Port, ShiftReg->Ser_Pin, bit & 0x01);
  HAL_GPIO_WritePin(ShiftReg->Srclk_Port, ShiftReg->Srclk_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ShiftReg->Srclk_Port, ShiftReg->Srclk_Pin, GPIO_PIN_RESET);
}

void ShiftReg_ShiftByte(ShiftReg_TypeDef* ShiftReg, uint8_t data) 
{
  if(ShiftReg->BitOrder == MSBFIRST) 
  {
    for(int8_t i = 7; i >= 0; i--)
    {
      ShiftReg_ShiftBit(ShiftReg, (data >> i));
    }
  }
  else 
  {
    for(int8_t i = 0; i < 8; i++)
    {
      ShiftReg_ShiftBit(ShiftReg, (data >> i));
    }
  }
}

void ShiftReg_Latch(ShiftReg_TypeDef* ShiftReg)
{
  HAL_GPIO_WritePin(ShiftReg->Rclk_Port, ShiftReg->Rclk_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ShiftReg->Rclk_Port, ShiftReg->Rclk_Pin, GPIO_PIN_RESET);
}

void ShiftReg_WriteByte(ShiftReg_TypeDef* ShiftReg, uint8_t data)
{
  ShiftReg_ShiftByte(ShiftReg, data);
  ShiftReg_Latch(ShiftReg);
}
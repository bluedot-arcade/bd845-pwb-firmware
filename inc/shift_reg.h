/**
  ******************************************************************************
  * @file           : shift_reg.h
  * @brief          : Header for shift_reg.c file.
  *                   This file contains the shift register utility defines.
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
#ifndef __SHIFT_REG_H
#define __SHIFT_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_hal.h"

/* Exported types ------------------------------------------------------------*/

typedef enum {
  LSBFIRST,
  MSBFIRST,
} ShiftReg_BitOrder;

typedef struct {
  ShiftReg_BitOrder BitOrder;
  GPIO_TypeDef *Ser_Port;
  uint16_t Ser_Pin;
  GPIO_TypeDef *Rclk_Port;
  uint16_t Rclk_Pin;
  GPIO_TypeDef *Srclk_Port;
  uint16_t Srclk_Pin;
} ShiftReg_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/

void ShiftReg_Init(ShiftReg_TypeDef* ShiftReg);

void ShiftReg_ShiftBit(ShiftReg_TypeDef* ShiftReg, uint8_t bit);

void ShiftReg_ShiftByte(ShiftReg_TypeDef* ShiftReg, uint8_t data);

void ShiftReg_Latch(ShiftReg_TypeDef* ShiftReg);

/** Shift a byte and subsequently latch the register. */
void ShiftReg_WriteByte(ShiftReg_TypeDef* ShiftReg, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __SHIFT_REG_H */

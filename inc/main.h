/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include "shift_reg.h"
#include "version.h"

void Error_Handler(void);

#define SERIAL_CMD_DDR_INIT    0x0C90
#define SERIAL_CMD_REQ_VERSION 0x065A
#define SERIAL_CMD_SENSOR_MASK 0x0B24

#define SERIAL_STATE_IDLE        0
#define SERIAL_STATE_DDR_INIT    1
#define SERIAL_STATE_REQ_VERSION 2
#define SERIAL_STATE_SENSOR_MASK 3

#define OUTPUTS_SER_Pin GPIO_PIN_13
#define OUTPUTS_SER_GPIO_Port GPIOC
#define OUTPUTS_RCLK_Pin GPIO_PIN_14
#define OUTPUTS_RCLK_GPIO_Port GPIOC
#define OUTPUTS_SRCLK_Pin GPIO_PIN_15
#define OUTPUTS_SRCLK_GPIO_Port GPIOC
#define LIGHTS_SRCLK_Pin GPIO_PIN_0
#define LIGHTS_SRCLK_GPIO_Port GPIOA
#define LIGHTS_RCLK_Pin GPIO_PIN_1
#define LIGHTS_RCLK_GPIO_Port GPIOA
#define LIGHTS_SER_Pin GPIO_PIN_2
#define LIGHTS_SER_GPIO_Port GPIOA
#define PANEL_C_S4_Pin GPIO_PIN_3
#define PANEL_C_S4_GPIO_Port GPIOA
#define PANEL_C_S3_Pin GPIO_PIN_4
#define PANEL_C_S3_GPIO_Port GPIOA
#define PANEL_C_S2_Pin GPIO_PIN_5
#define PANEL_C_S2_GPIO_Port GPIOA
#define PANEL_C_S1_Pin GPIO_PIN_6
#define PANEL_C_S1_GPIO_Port GPIOA
#define PANEL_R_S4_Pin GPIO_PIN_7
#define PANEL_R_S4_GPIO_Port GPIOA
#define PANEL_R_S3_Pin GPIO_PIN_0
#define PANEL_R_S3_GPIO_Port GPIOB
#define PANEL_R_S2_Pin GPIO_PIN_1
#define PANEL_R_S2_GPIO_Port GPIOB
#define PANEL_R_S1_Pin GPIO_PIN_2
#define PANEL_R_S1_GPIO_Port GPIOB
#define PANEL_L_S4_Pin GPIO_PIN_10
#define PANEL_L_S4_GPIO_Port GPIOB
#define PANEL_L_S3_Pin GPIO_PIN_11
#define PANEL_L_S3_GPIO_Port GPIOB
#define PANEL_L_S2_Pin GPIO_PIN_12
#define PANEL_L_S2_GPIO_Port GPIOB
#define PANEL_L_S1_Pin GPIO_PIN_13
#define PANEL_L_S1_GPIO_Port GPIOB
#define PANEL_D_S4_Pin GPIO_PIN_14
#define PANEL_D_S4_GPIO_Port GPIOB
#define PANEL_D_S3_Pin GPIO_PIN_15
#define PANEL_D_S3_GPIO_Port GPIOB
#define PANEL_D_S2_Pin GPIO_PIN_8
#define PANEL_D_S2_GPIO_Port GPIOA
#define PANEL_D_S1_Pin GPIO_PIN_9
#define PANEL_D_S1_GPIO_Port GPIOA
#define PANEL_U_S4_Pin GPIO_PIN_10
#define PANEL_U_S4_GPIO_Port GPIOA
#define PANEL_U_S3_Pin GPIO_PIN_11
#define PANEL_U_S3_GPIO_Port GPIOA
#define PANEL_U_S2_Pin GPIO_PIN_12
#define PANEL_U_S2_GPIO_Port GPIOA
#define PANEL_U_S1_Pin GPIO_PIN_6
#define PANEL_U_S1_GPIO_Port GPIOF
#define OPT_LIGHT_Pin GPIO_PIN_7
#define OPT_LIGHT_GPIO_Port GPIOF
#define OPT_DEBOUNCE_Pin GPIO_PIN_15
#define OPT_DEBOUNCE_GPIO_Port GPIOA
#define OPT_LEGACY_Pin GPIO_PIN_3
#define OPT_LEGACY_GPIO_Port GPIOB
#define COMM_TEST_Pin GPIO_PIN_4
#define COMM_TEST_GPIO_Port GPIOB
#define COMM_FL5_Pin GPIO_PIN_5
#define COMM_FL5_GPIO_Port GPIOB
#define COMM_FL4_Pin GPIO_PIN_6
#define COMM_FL4_GPIO_Port GPIOB
#define COMM_FL3_Pin GPIO_PIN_7
#define COMM_FL3_GPIO_Port GPIOB
#define COMM_FL2_Pin GPIO_PIN_8
#define COMM_FL2_GPIO_Port GPIOB
#define COMM_FL1_Pin GPIO_PIN_9
#define COMM_FL1_GPIO_Port GPIOB

#define PANEL_U_S1 0x00000001U
#define PANEL_U_S2 0x00000002U
#define PANEL_U_S3 0x00000004U
#define PANEL_U_S4 0x00000008U
#define PANEL_U_OR 0x0000000FU

#define PANEL_D_S1 0x00000010U
#define PANEL_D_S2 0x00000020U
#define PANEL_D_S3 0x00000040U
#define PANEL_D_S4 0x00000080U
#define PANEL_D_OR 0x000000F0U

#define PANEL_L_S1 0x00000100U
#define PANEL_L_S2 0x00000200U
#define PANEL_L_S3 0x00000400U
#define PANEL_L_S4 0x00000800U
#define PANEL_L_OR 0x00000F00U

#define PANEL_R_S1 0x00001000U
#define PANEL_R_S2 0x00002000U
#define PANEL_R_S3 0x00004000U
#define PANEL_R_S4 0x00008000U
#define PANEL_R_OR 0x0000F000U

#define PANEL_C_S1 0x00010000U
#define PANEL_C_S2 0x00020000U
#define PANEL_C_S3 0x00040000U
#define PANEL_C_S4 0x00080000U
#define PANEL_C_OR 0x000F0000U

#define COMM_FL1	0x00100000U
#define COMM_FL2  0x00200000U
#define COMM_FL3  0x00400000U
#define COMM_FL4  0x00800000U
#define COMM_FL5  0x01000000U
#define COMM_TEST 0x02000000U

#define OPT_LIGHT 	 0x04000000U
#define OPT_DEBOUNCE 0x08000000U
#define OPT_LEGACY	 0x10000000U

#define PANEL_U_LIGHT 0x20U
#define PANEL_D_LIGHT 0x10U
#define PANEL_L_LIGHT 0x08U
#define PANEL_R_LIGHT 0x04U
#define PANEL_C_LIGHT 0x02U
#define STATUS_LED    0x01U

#define PANEL_U_OUT 0x10U
#define PANEL_D_OUT 0x08U
#define PANEL_L_OUT 0x04U
#define PANEL_R_OUT 0x02U
#define PANEL_C_OUT 0x01U

/* Input debouncing ticks of 100uS */
#define DEBOUNCE_TICKS 40

/* DDR Init timeout in ms */
#define SERIAL_TIMEOUT_TICKS 600

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

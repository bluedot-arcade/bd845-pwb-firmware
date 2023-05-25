/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "gpio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Check if an option is enabled */
#define IS_OPT_ON(opt) (Inputs_State & opt) 

/* Private variables ---------------------------------------------------------*/

ShiftReg_TypeDef Pads_ShiftReg;
ShiftReg_TypeDef Lights_ShiftReg;
uint32_t Inputs_State = 0;
uint8_t Lights_State = 0;
uint8_t Pads_State = 0; 

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
void Lights_Register_Init(void);
void Pads_Register_Init(void);
void Inputs_Poll(void);
void Pads_Update(void);
void Lights_Update(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  Lights_Register_Init();
  Pads_Register_Init();

  while (1)
  {
    Inputs_Poll();
    Pads_Update();
    Lights_Update();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Init lights shift register.
  * @retval None
  */
void Lights_Register_Init(void) {
  ShiftReg_TypeDef ShiftReg = {0};

  ShiftReg.BitOrder = MSBFIRST;
  ShiftReg.Ser_Port = LIGHTS_SER_GPIO_Port;
  ShiftReg.Ser_Pin = LIGHTS_SER_Pin;
  ShiftReg.Rclk_Port = LIGHTS_RCLK_GPIO_Port;
  ShiftReg.Rclk_Pin = LIGHTS_RCLK_Pin;
  ShiftReg.Srclk_Port = LIGHTS_SRCLK_GPIO_Port;
  ShiftReg.Srclk_Pin = LIGHTS_SRCLK_Pin;
  ShiftReg_Init(&ShiftReg);

  Lights_ShiftReg = ShiftReg;
}

/**
  * @brief Init pads shift register.
  * @retval None
  */
void Pads_Register_Init(void) {
  ShiftReg_TypeDef ShiftReg = {0};

  ShiftReg.BitOrder = MSBFIRST;
  ShiftReg.Ser_Port = PADS_SER_GPIO_Port;
  ShiftReg.Ser_Pin = PADS_SER_Pin;
  ShiftReg.Rclk_Port = PADS_RCLK_GPIO_Port;
  ShiftReg.Rclk_Pin = PADS_RCLK_Pin;
  ShiftReg.Srclk_Port = PADS_SRCLK_GPIO_Port;
  ShiftReg.Srclk_Pin = PADS_SRCLK_Pin;
  ShiftReg_Init(&ShiftReg);

  Pads_ShiftReg = ShiftReg;
}

/**
  * @brief Poll sensors, comms and options.
  * @retval None
  */
void Inputs_Poll(void) 
{
  uint32_t State = 0;

  // Poll CH1 sensors
  State |= HAL_GPIO_ReadPin(CH1_S1_GPIO_Port, CH1_S1_Pin);
  State |= HAL_GPIO_ReadPin(CH1_S2_GPIO_Port, CH1_S2_Pin) << 1;
  State |= HAL_GPIO_ReadPin(CH1_S3_GPIO_Port, CH1_S3_Pin) << 2;
  State |= HAL_GPIO_ReadPin(CH1_S4_GPIO_Port, CH1_S4_Pin) << 3;

  // Poll CH2 sensors
  State |= HAL_GPIO_ReadPin(CH2_S1_GPIO_Port, CH2_S1_Pin) << 4;
  State |= HAL_GPIO_ReadPin(CH2_S2_GPIO_Port, CH2_S2_Pin) << 5;
  State |= HAL_GPIO_ReadPin(CH2_S3_GPIO_Port, CH2_S3_Pin) << 6;
  State |= HAL_GPIO_ReadPin(CH2_S4_GPIO_Port, CH2_S4_Pin) << 7;

  // Poll CH3 sensors
  State |= HAL_GPIO_ReadPin(CH3_S1_GPIO_Port, CH3_S1_Pin) << 8;
  State |= HAL_GPIO_ReadPin(CH3_S2_GPIO_Port, CH3_S2_Pin) << 9;
  State |= HAL_GPIO_ReadPin(CH3_S3_GPIO_Port, CH3_S3_Pin) << 10;
  State |= HAL_GPIO_ReadPin(CH3_S4_GPIO_Port, CH3_S4_Pin) << 11;

  // Poll CH4 sensors
  State |= HAL_GPIO_ReadPin(CH4_S1_GPIO_Port, CH4_S1_Pin) << 12;
  State |= HAL_GPIO_ReadPin(CH4_S2_GPIO_Port, CH4_S2_Pin) << 13;
  State |= HAL_GPIO_ReadPin(CH4_S3_GPIO_Port, CH4_S3_Pin) << 14;
  State |= HAL_GPIO_ReadPin(CH4_S4_GPIO_Port, CH4_S4_Pin) << 15;

  // Poll CH5 sensors
  State |= HAL_GPIO_ReadPin(CH5_S1_GPIO_Port, CH5_S1_Pin) << 16;
  State |= HAL_GPIO_ReadPin(CH5_S2_GPIO_Port, CH5_S2_Pin) << 17;
  State |= HAL_GPIO_ReadPin(CH5_S3_GPIO_Port, CH5_S3_Pin) << 18;
  State |= HAL_GPIO_ReadPin(CH5_S4_GPIO_Port, CH5_S4_Pin) << 19;

  //Poll COMM states
	State |= HAL_GPIO_ReadPin(COMM_FL1_GPIO_Port, COMM_FL1_Pin) << 20;
	State |= HAL_GPIO_ReadPin(COMM_FL2_GPIO_Port, COMM_FL2_Pin) << 21;
	State |= HAL_GPIO_ReadPin(COMM_FL3_GPIO_Port, COMM_FL3_Pin) << 22;
	State |= HAL_GPIO_ReadPin(COMM_FL4_GPIO_Port, COMM_FL4_Pin) << 23;
	State |= HAL_GPIO_ReadPin(COMM_FL5_GPIO_Port, COMM_FL5_Pin) << 24;
	State |= HAL_GPIO_ReadPin(COMM_TEST_GPIO_Port, COMM_TEST_Pin) << 25;

  //Poll OPT states
	State |= ~HAL_GPIO_ReadPin(OPT_LIGHT_GPIO_Port, OPT_LIGHT_Pin) << 26;
	State |= ~HAL_GPIO_ReadPin(OPT_DEBOUNCE_GPIO_Port, OPT_DEBOUNCE_Pin) << 27;
	State |= ~HAL_GPIO_ReadPin(OPT_LEGACY_GPIO_Port, OPT_LEGACY_Pin) << 28;

  Inputs_State = ~State;
}

/**
  * @brief Update the pads output registers.
  * @retval None
  */
void Pads_Update(void)
{
  uint8_t State = 0;

  State |= (Inputs_State & CH1_OR) ? CH1_PAD : 0;
  State |= (Inputs_State & CH2_OR) ? CH2_PAD : 0;
  State |= (Inputs_State & CH3_OR) ? CH3_PAD : 0;
  State |= (Inputs_State & CH4_OR) ? CH4_PAD : 0;
  State |= (Inputs_State & CH5_OR) ? CH5_PAD : 0;

  Pads_State = State;
  ShiftReg_WriteByte(&Pads_ShiftReg, Pads_State);
}

/**
  * @brief Update the lights output registers.
  * @retval None
  */
void Lights_Update(void)
{
  uint8_t State = 0;

  if(IS_OPT_ON(OPT_LIGHT)) {
    State |= (Inputs_State & CH1_OR) ? CH1_LIGHT : 0;
    State |= (Inputs_State & CH2_OR) ? CH2_LIGHT : 0;
    State |= (Inputs_State & CH3_OR) ? CH3_LIGHT : 0;
    State |= (Inputs_State & CH4_OR) ? CH4_LIGHT : 0;
    State |= (Inputs_State & CH5_OR) ? CH5_LIGHT : 0;
  } else {
    State |= (Inputs_State & COMM_FL1) ? CH1_LIGHT : 0;
    State |= (Inputs_State & COMM_FL2) ? CH2_LIGHT : 0;
    State |= (Inputs_State & COMM_FL3) ? CH3_LIGHT : 0;
    State |= (Inputs_State & COMM_FL4) ? CH4_LIGHT : 0;
    State |= (Inputs_State & COMM_FL5) ? CH5_LIGHT : 0;
  }

  Lights_State = State;
  ShiftReg_WriteByte(&Lights_ShiftReg, Lights_State);
}

/**
  * @brief Called before the SysTick increment.
  * @retval None
  */
void Before_IncTick_Handler(void) 
{
  // Empty4
}

/**
  * @brief Called after the SysTick increment.
  * @retval None
  */
void After_IncTick_Handler(void)
{
  //TODO Implement debounce mechanism
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    //Blink status LED indefinitely
	  ShiftReg_WriteByte(&Lights_ShiftReg, 0x00);
	  HAL_Delay(500);
	  ShiftReg_WriteByte(&Lights_ShiftReg, STATUS_LED);
	  HAL_Delay(500);
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

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
#include "tim.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define DDR_INIT_CMD 0x0C90
#define DDR_STAGE_IDLE 0
#define DDR_STAGE_INIT 1

/* Private macro -------------------------------------------------------------*/

/* Check if an option is enabled */
#define IS_OPT_ON(opt) (Inputs_State & opt) 
#define CH1_COUNTER Pads_Counters[0]
#define CH2_COUNTER Pads_Counters[1]
#define CH3_COUNTER Pads_Counters[2]
#define CH4_COUNTER Pads_Counters[3]
#define CH5_COUNTER Pads_Counters[4]

/* Private consts ------------------------------------------------------------*/

static const uint8_t DDR_Pads_States[] =
{
  0x12, 0x00, 0x10, 0x02,
  0x12, 0x02, 0x12, 0x02,
  0x12, 0x02, 0x12, 0x02,
  0x12, 0x02, 0x12, 0x02,
  0x12, 0x02, 0x12, 0x02,
  0x12, 0x02, 0x00, 0x00
};

/* Private variables ---------------------------------------------------------*/

uint32_t Inputs_State = 0;

ShiftReg_TypeDef Pads_ShiftReg;
uint8_t Pads_State = 0;
uint8_t Pads_Counters[5];

ShiftReg_TypeDef Lights_ShiftReg;
uint8_t Lights_State = 0;

uint16_t DDR_Cmd = 0;
uint8_t DDR_State = DDR_STAGE_IDLE;
uint8_t DDR_Bit = 0;

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
  MX_TIM3_Init();
  
  /* Initialize shift registers */
  Lights_Register_Init();
  Pads_Register_Init();

  /* Start timers */
  TIM3_Start();

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
  ShiftReg_WriteByte(&ShiftReg, 0);

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
  ShiftReg_WriteByte(&ShiftReg, 0);

  Pads_ShiftReg = ShiftReg;
}

/**
  * @brief Poll sensors, comms and options.
  * @retval None
  */
void Inputs_Poll(void) 
{
  uint32_t State = 0;

  /* Poll CH1 sensors */
  State |= HAL_GPIO_ReadPin(CH1_S1_GPIO_Port, CH1_S1_Pin);
  State |= HAL_GPIO_ReadPin(CH1_S2_GPIO_Port, CH1_S2_Pin) << 1;
  State |= HAL_GPIO_ReadPin(CH1_S3_GPIO_Port, CH1_S3_Pin) << 2;
  State |= HAL_GPIO_ReadPin(CH1_S4_GPIO_Port, CH1_S4_Pin) << 3;

  /* Poll CH2 sensors */
  State |= HAL_GPIO_ReadPin(CH2_S1_GPIO_Port, CH2_S1_Pin) << 4;
  State |= HAL_GPIO_ReadPin(CH2_S2_GPIO_Port, CH2_S2_Pin) << 5;
  State |= HAL_GPIO_ReadPin(CH2_S3_GPIO_Port, CH2_S3_Pin) << 6;
  State |= HAL_GPIO_ReadPin(CH2_S4_GPIO_Port, CH2_S4_Pin) << 7;

  /* Poll CH3 sensors */
  State |= HAL_GPIO_ReadPin(CH3_S1_GPIO_Port, CH3_S1_Pin) << 8;
  State |= HAL_GPIO_ReadPin(CH3_S2_GPIO_Port, CH3_S2_Pin) << 9;
  State |= HAL_GPIO_ReadPin(CH3_S3_GPIO_Port, CH3_S3_Pin) << 10;
  State |= HAL_GPIO_ReadPin(CH3_S4_GPIO_Port, CH3_S4_Pin) << 11;

  /* Poll CH4 sensors */
  State |= HAL_GPIO_ReadPin(CH4_S1_GPIO_Port, CH4_S1_Pin) << 12;
  State |= HAL_GPIO_ReadPin(CH4_S2_GPIO_Port, CH4_S2_Pin) << 13;
  State |= HAL_GPIO_ReadPin(CH4_S3_GPIO_Port, CH4_S3_Pin) << 14;
  State |= HAL_GPIO_ReadPin(CH4_S4_GPIO_Port, CH4_S4_Pin) << 15;

  /* Poll CH5 sensors */
  State |= HAL_GPIO_ReadPin(CH5_S1_GPIO_Port, CH5_S1_Pin) << 16;
  State |= HAL_GPIO_ReadPin(CH5_S2_GPIO_Port, CH5_S2_Pin) << 17;
  State |= HAL_GPIO_ReadPin(CH5_S3_GPIO_Port, CH5_S3_Pin) << 18;
  State |= HAL_GPIO_ReadPin(CH5_S4_GPIO_Port, CH5_S4_Pin) << 19;

  /* Poll COMM states */
  State |= HAL_GPIO_ReadPin(COMM_FL1_GPIO_Port, COMM_FL1_Pin) << 20;
  State |= HAL_GPIO_ReadPin(COMM_FL2_GPIO_Port, COMM_FL2_Pin) << 21;
  State |= HAL_GPIO_ReadPin(COMM_FL3_GPIO_Port, COMM_FL3_Pin) << 22;
  State |= HAL_GPIO_ReadPin(COMM_FL4_GPIO_Port, COMM_FL4_Pin) << 23;
  State |= HAL_GPIO_ReadPin(COMM_FL5_GPIO_Port, COMM_FL5_Pin) << 24;
  State |= HAL_GPIO_ReadPin(COMM_TEST_GPIO_Port, COMM_TEST_Pin) << 25;

  /* All inputs except OPT are active low so they need to be inverted. */
  State = ~State & 0x03FFFFFF; 

  /* Poll OPT states */
  State |= HAL_GPIO_ReadPin(OPT_LIGHT_GPIO_Port, OPT_LIGHT_Pin) << 26;
  State |= HAL_GPIO_ReadPin(OPT_DEBOUNCE_GPIO_Port, OPT_DEBOUNCE_Pin) << 27;
  State |= HAL_GPIO_ReadPin(OPT_LEGACY_GPIO_Port, OPT_LEGACY_Pin) << 28;

  Inputs_State = State;
}

/**
  * @brief Update the pads output registers.
  * @retval None
  */
void Pads_Update(void)
{
  /* Do not update pads during DDR initialization check. */
  if(IS_OPT_ON(OPT_LEGACY) & DDR_State == DDR_STAGE_INIT)
    return;

  uint8_t State = 0;

  if(IS_OPT_ON(OPT_DEBOUNCE)) 
  {
    if(Inputs_State & CH1_OR) CH1_COUNTER = DEBOUNCE_TICKS;
    if(Inputs_State & CH2_OR) CH2_COUNTER = DEBOUNCE_TICKS;
    if(Inputs_State & CH3_OR) CH3_COUNTER = DEBOUNCE_TICKS;
    if(Inputs_State & CH4_OR) CH4_COUNTER = DEBOUNCE_TICKS;
    if(Inputs_State & CH5_OR) CH5_COUNTER = DEBOUNCE_TICKS;

    State |= CH1_COUNTER > 0 ? CH1_PAD : 0;
    State |= CH2_COUNTER > 0 ? CH2_PAD : 0;
    State |= CH3_COUNTER > 0 ? CH3_PAD : 0;
    State |= CH4_COUNTER > 0 ? CH4_PAD : 0;
    State |= CH5_COUNTER > 0 ? CH5_PAD : 0;
  }
  else
  {
    State |= (Inputs_State & CH1_OR) ? CH1_PAD : 0;
    State |= (Inputs_State & CH2_OR) ? CH2_PAD : 0;
    State |= (Inputs_State & CH3_OR) ? CH3_PAD : 0;
    State |= (Inputs_State & CH4_OR) ? CH4_PAD : 0;
    State |= (Inputs_State & CH5_OR) ? CH5_PAD : 0;
  }

  if(Pads_State != State)
  {
    Pads_State = State;
    ShiftReg_WriteByte(&Pads_ShiftReg, Pads_State);
  }
}

/**
  * @brief Update the lights output registers.
  * @retval None
  */
void Lights_Update(void)
{
  uint8_t State = 0;

  if(IS_OPT_ON(OPT_LIGHT)) 
  {
    State |= (Pads_State & CH1_PAD) ? CH1_LIGHT : 0;
    State |= (Pads_State & CH2_PAD) ? CH2_LIGHT : 0;
    State |= (Pads_State & CH3_PAD) ? CH3_LIGHT : 0;
    State |= (Pads_State & CH4_PAD) ? CH4_LIGHT : 0;
    State |= (Pads_State & CH5_PAD) ? CH5_LIGHT : 0;
  } 
  else 
  {
    State |= (Inputs_State & COMM_FL1) ? CH1_LIGHT : 0;
    State |= (Inputs_State & COMM_FL2) ? CH2_LIGHT : 0;
    State |= (Inputs_State & COMM_FL3) ? CH3_LIGHT : 0;
    State |= (Inputs_State & COMM_FL4) ? CH4_LIGHT : 0;
    State |= (Inputs_State & COMM_FL5) ? CH5_LIGHT : 0;
  }

  if(Lights_State != State)
  {
    Lights_State = State;
    ShiftReg_WriteByte(&Lights_ShiftReg, Lights_State);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  for(uint8_t i = 0; i < 5; i++) 
  {
    if(Pads_Counters[i] > 0) 
    {
      Pads_Counters[i]--;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /*
   * This code emulates the DDR Stage IO initialization.
   * 
   * When the DDR_INIT_CMD is received go into DDR_STAGE_INIT 
   * mode, send the reply through CH1 and CH4 outputs at each
   * clock cycle then return into DDR_STAGE_IDLE mode.
   * 
   * The DDR_INIT_CMD is sent serially LSB first by sampling on clock 
   * rising edge.
   * 
   * Serial data line: FL5 Pin.
   * Clock line:       TEST Pin (configured as ext interrupt).
   * 
   * This is based on how MAME emulates the KSYS573.
   * All games that boot on MAME should boot with this board too.
   * https://github.com/mamedev/mame/blob/master/src/mame/konami/ksys573.cpp
   */

  if(!IS_OPT_ON(OPT_LEGACY))
    return;

  uint8_t data = HAL_GPIO_ReadPin(COMM_FL5_GPIO_Port, COMM_FL5_Pin);

  DDR_Cmd = (DDR_Cmd >> 1) | (data << 12); 

  switch(DDR_State)
  {
    case DDR_STAGE_IDLE:
      if(DDR_Cmd == DDR_INIT_CMD)
      {
        DDR_State = DDR_STAGE_INIT;
        DDR_Bit = 0;
        ShiftReg_WriteByte(&Pads_ShiftReg, DDR_Pads_States[DDR_Bit]);
      }
      break;
    case DDR_STAGE_INIT:
      if(++DDR_Bit < 22)
      {
        ShiftReg_WriteByte(&Pads_ShiftReg, DDR_Pads_States[DDR_Bit]);
      }
      else
      {
        DDR_State = DDR_STAGE_IDLE;
        DDR_Bit = 0;
        ShiftReg_WriteByte(&Pads_ShiftReg, 0x0);
      }
      break;
  }
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

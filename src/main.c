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

#include "main.h"
#include "gpio.h"
#include "tim.h"

/* Check if an option is enabled */
#define IS_OPT_ON(opt) (Inputs_State & opt) 
#define PANEL_U_COUNTER Panel_Counters[0]
#define PANEL_D_COUNTER Panel_Counters[1]
#define PANEL_L_COUNTER Panel_Counters[2]
#define PANEL_R_COUNTER Panel_Counters[3]
#define PANEL_C_COUNTER Panel_Counters[4]

static const uint8_t DDR_Outputs_States[] =
{
  0x12, 0x00, 0x10, 0x02,
  0x12, 0x02, 0x12, 0x02,
  0x12, 0x02, 0x12, 0x02,
  0x12, 0x02, 0x12, 0x02,
  0x12, 0x02, 0x12, 0x02,
  0x12, 0x02, 0x00, 0x00
};

uint32_t Inputs_Mask = 0xFFFFFFFF;
uint32_t Inputs_State = 0;

ShiftReg_TypeDef Outputs_ShiftReg;
uint8_t Outputs_State = 0;
uint8_t Panel_Counters[5];

ShiftReg_TypeDef Lights_ShiftReg;
uint8_t Lights_State = 0;

uint8_t Serial_State = SERIAL_STATE_IDLE;
uint8_t Serial_Bit = 0;
uint16_t Serial_Cmd = 0;
uint32_t Serial_Init_Tick = 0;

void SystemClock_Config(void);
void Lights_Register_Init(void);
void Outputs_Register_Init(void);
void Inputs_Poll(void);
void Outputs_Update(void);
void Lights_Update(void);
void Debounce_Tick_Handler(void);
void Serial_Clock_Handler(void);
void Serial_Idle_Clock_Handler(void);
void Serial_DDR_Init_Clock_Handler(void);
void Serial_Req_Version_Clock_Handler(void);
void Serial_Sensor_Mask_Clock_Handler(void);
void Serial_Timeout_Check(void);

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
  Outputs_Register_Init();

  /* Start timers */
  TIM3_Start();

  while (1)
  {
    Inputs_Poll();

    /* Do not interfere with Serial communication. */
    if(Serial_State != SERIAL_STATE_IDLE)
    {
      Serial_Timeout_Check();
    } 
    else
    {
      Outputs_Update();
    }

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
void Outputs_Register_Init(void) {
  ShiftReg_TypeDef ShiftReg = {0};

  ShiftReg.BitOrder = MSBFIRST;
  ShiftReg.Ser_Port = OUTPUTS_SER_GPIO_Port;
  ShiftReg.Ser_Pin = OUTPUTS_SER_Pin;
  ShiftReg.Rclk_Port = OUTPUTS_RCLK_GPIO_Port;
  ShiftReg.Rclk_Pin = OUTPUTS_RCLK_Pin;
  ShiftReg.Srclk_Port = OUTPUTS_SRCLK_GPIO_Port;
  ShiftReg.Srclk_Pin = OUTPUTS_SRCLK_Pin;
  ShiftReg_Init(&ShiftReg);
  ShiftReg_WriteByte(&ShiftReg, 0);

  Outputs_ShiftReg = ShiftReg;
}

/**
  * @brief Poll sensors, comms and options.
  * @retval None
  */
void Inputs_Poll(void) 
{
  uint32_t State = 0;

  /* Poll PANEL_U sensors */
  State |= HAL_GPIO_ReadPin(PANEL_U_S1_GPIO_Port, PANEL_U_S1_Pin);
  State |= HAL_GPIO_ReadPin(PANEL_U_S2_GPIO_Port, PANEL_U_S2_Pin) << 1;
  State |= HAL_GPIO_ReadPin(PANEL_U_S3_GPIO_Port, PANEL_U_S3_Pin) << 2;
  State |= HAL_GPIO_ReadPin(PANEL_U_S4_GPIO_Port, PANEL_U_S4_Pin) << 3;

  /* Poll PANEL_D sensors */
  State |= HAL_GPIO_ReadPin(PANEL_D_S1_GPIO_Port, PANEL_D_S1_Pin) << 4;
  State |= HAL_GPIO_ReadPin(PANEL_D_S2_GPIO_Port, PANEL_D_S2_Pin) << 5;
  State |= HAL_GPIO_ReadPin(PANEL_D_S3_GPIO_Port, PANEL_D_S3_Pin) << 6;
  State |= HAL_GPIO_ReadPin(PANEL_D_S4_GPIO_Port, PANEL_D_S4_Pin) << 7;

  /* Poll PANEL_L sensors */
  State |= HAL_GPIO_ReadPin(PANEL_L_S1_GPIO_Port, PANEL_L_S1_Pin) << 8;
  State |= HAL_GPIO_ReadPin(PANEL_L_S2_GPIO_Port, PANEL_L_S2_Pin) << 9;
  State |= HAL_GPIO_ReadPin(PANEL_L_S3_GPIO_Port, PANEL_L_S3_Pin) << 10;
  State |= HAL_GPIO_ReadPin(PANEL_L_S4_GPIO_Port, PANEL_L_S4_Pin) << 11;

  /* Poll PANEL_R sensors */
  State |= HAL_GPIO_ReadPin(PANEL_R_S1_GPIO_Port, PANEL_R_S1_Pin) << 12;
  State |= HAL_GPIO_ReadPin(PANEL_R_S2_GPIO_Port, PANEL_R_S2_Pin) << 13;
  State |= HAL_GPIO_ReadPin(PANEL_R_S3_GPIO_Port, PANEL_R_S3_Pin) << 14;
  State |= HAL_GPIO_ReadPin(PANEL_R_S4_GPIO_Port, PANEL_R_S4_Pin) << 15;

  /* Poll PANEL_C sensors */
  State |= HAL_GPIO_ReadPin(PANEL_C_S1_GPIO_Port, PANEL_C_S1_Pin) << 16;
  State |= HAL_GPIO_ReadPin(PANEL_C_S2_GPIO_Port, PANEL_C_S2_Pin) << 17;
  State |= HAL_GPIO_ReadPin(PANEL_C_S3_GPIO_Port, PANEL_C_S3_Pin) << 18;
  State |= HAL_GPIO_ReadPin(PANEL_C_S4_GPIO_Port, PANEL_C_S4_Pin) << 19;

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

  Inputs_State = State & Inputs_Mask;
}

/**
  * @brief Update the pads output registers.
  * @retval None
  */
void Outputs_Update(void)
{
  uint8_t State = 0;

  if(IS_OPT_ON(OPT_DEBOUNCE)) 
  {
    if(Inputs_State & PANEL_U_OR) PANEL_U_COUNTER = DEBOUNCE_TICKS;
    if(Inputs_State & PANEL_D_OR) PANEL_D_COUNTER = DEBOUNCE_TICKS;
    if(Inputs_State & PANEL_L_OR) PANEL_L_COUNTER = DEBOUNCE_TICKS;
    if(Inputs_State & PANEL_R_OR) PANEL_R_COUNTER = DEBOUNCE_TICKS;
    if(Inputs_State & PANEL_C_OR) PANEL_C_COUNTER = DEBOUNCE_TICKS;

    State |= PANEL_U_COUNTER > 0 ? PANEL_U_OUT : 0;
    State |= PANEL_D_COUNTER > 0 ? PANEL_D_OUT : 0;
    State |= PANEL_L_COUNTER > 0 ? PANEL_L_OUT : 0;
    State |= PANEL_R_COUNTER > 0 ? PANEL_R_OUT : 0;
    State |= PANEL_C_COUNTER > 0 ? PANEL_C_OUT : 0;
  }
  else
  {
    State |= (Inputs_State & PANEL_U_OR) ? PANEL_U_OUT : 0;
    State |= (Inputs_State & PANEL_D_OR) ? PANEL_D_OUT : 0;
    State |= (Inputs_State & PANEL_L_OR) ? PANEL_L_OUT : 0;
    State |= (Inputs_State & PANEL_R_OR) ? PANEL_R_OUT : 0;
    State |= (Inputs_State & PANEL_C_OR) ? PANEL_C_OUT : 0;
  }

  if(Outputs_State != State)
  {
    Outputs_State = State;
    ShiftReg_WriteByte(&Outputs_ShiftReg, Outputs_State);
  }
}

/**
  * @brief Update the lights output registers.
  * @retval None
  */
void Lights_Update(void)
{
  /* Preserve STATUS_LED state */
  uint8_t State = Lights_State & STATUS_LED;

  if(IS_OPT_ON(OPT_LIGHT)) 
  {
    State |= (Outputs_State & PANEL_U_OUT) ? PANEL_U_LIGHT : 0;
    State |= (Outputs_State & PANEL_D_OUT) ? PANEL_D_LIGHT : 0;
    State |= (Outputs_State & PANEL_L_OUT) ? PANEL_L_LIGHT : 0;
    State |= (Outputs_State & PANEL_R_OUT) ? PANEL_R_LIGHT : 0;
    State |= (Outputs_State & PANEL_C_OUT) ? PANEL_C_LIGHT : 0;
  } 
  else 
  {
    State |= (Inputs_State & COMM_FL1) ? PANEL_U_LIGHT : 0;
    State |= (Inputs_State & COMM_FL2) ? PANEL_D_LIGHT : 0;
    State |= (Inputs_State & COMM_FL3) ? PANEL_L_LIGHT : 0;
    State |= (Inputs_State & COMM_FL4) ? PANEL_R_LIGHT : 0;
    State |= (Inputs_State & COMM_FL5) ? PANEL_C_LIGHT : 0;
  }

  if(Lights_State != State)
  {
    Lights_State = State;
    ShiftReg_WriteByte(&Lights_ShiftReg, Lights_State);
  }
}

/**
  * @brief Called when a debounce tick occurs and updates the counters.
  * @retval None
  */
void Debounce_Tick_Handler(void)
{
  for(uint8_t i = 0; i < 5; i++) 
  {
    if(Panel_Counters[i] > 0) 
      Panel_Counters[i]--;
  }
}

/**
  * @brief Called when a clock pulse rising edge has been 
  *        detected on the TEST line.
  * @retval None
  */
void Serial_Clock_Handler(void)
{
  switch(Serial_State)
  {
    case SERIAL_STATE_IDLE:
      Serial_Idle_Clock_Handler();
      break;
    case SERIAL_STATE_DDR_INIT:
      Serial_DDR_Init_Clock_Handler();
      break;
    case SERIAL_STATE_REQ_VERSION:
      Serial_Req_Version_Clock_Handler();
      break;
    case SERIAL_STATE_SENSOR_MASK:
      Serial_Sensor_Mask_Clock_Handler();
      break;
  }
}

/**
  * @brief Handles clock pulses when in SERIAL_STATE_IDLE.
  * @retval None
  */
void Serial_Idle_Clock_Handler(void)
{
  /* 
   * Commands are sent serially LSB first by sampling data line 
   * on clock ising edge.
   *
   * Serial data line: FL5 Pin.
   * Clock line:       TEST Pin (configured as ext interrupt).
   */

  uint8_t data = HAL_GPIO_ReadPin(COMM_FL5_GPIO_Port, COMM_FL5_Pin);
  
  /* Assemble 13-bits command */
  Serial_Cmd = (Serial_Cmd >> 1) | (data << 12); 

  switch(Serial_Cmd)
  {
    case SERIAL_CMD_DDR_INIT:
      Serial_DDR_Init_Clock_Handler();
      break;
    case SERIAL_CMD_REQ_VERSION:
      Serial_Req_Version_Clock_Handler();
      break;
    case SERIAL_CMD_SENSOR_MASK:
      Serial_Sensor_Mask_Clock_Handler();
      break;
  }
}

/**
  * This code emulates the DDR Stage IO initialization.
  * 
  * This is called when the SERIAL_CMD_DDR_INIT is received. Go into 
  * SERIAL_STATE_DDR_INIT state, send the reply through PANEL_U and PANEL_R 
  * outputs at each clock cycle, then return into SERIAL_STATE_IDLE mode.
  * 
  * This is based on how MAME emulates the KSYS573.
  * All games that boot on MAME should boot with this board too.
  * https://github.com/mamedev/mame/blob/master/src/mame/konami/ksys573.cpp
  * @retval None
  */
void Serial_DDR_Init_Clock_Handler(void) 
{
  if(!IS_OPT_ON(OPT_LEGACY))
    return;

  switch(Serial_State) {
    case SERIAL_STATE_IDLE:
      Serial_State = SERIAL_STATE_DDR_INIT;
      Serial_Init_Tick = HAL_GetTick();
      Serial_Bit = 0;
    case SERIAL_STATE_DDR_INIT:
      if(Serial_Bit < 22)
      {
        ShiftReg_WriteByte(&Outputs_ShiftReg, DDR_Outputs_States[Serial_Bit]);
        Serial_Bit++;
      }
      else
      {
        Serial_State = SERIAL_STATE_IDLE;
        ShiftReg_WriteByte(&Outputs_ShiftReg, 0);
      }
      break;
  }
}

/**
  * Handles clock pulses when firmware version is requested.
  * 
  * This is called when the SERIAL_CMD_REQ_VERSION is received. Go into 
  * SERIAL_STATE_REQ_VERSION state, send the reply through PANEL_U and PANEL_R 
  * outputs at each clock cycle, then return into SERIAL_STATE_IDLE mode.
  * 
  * @retval None
  */
void Serial_Req_Version_Clock_Handler(void) 
{
  switch(Serial_State) {
    case SERIAL_STATE_IDLE:
      Serial_State = SERIAL_STATE_REQ_VERSION;
      Serial_Init_Tick = HAL_GetTick();
      Serial_Bit = 0;
    case SERIAL_STATE_REQ_VERSION:
      /* Send version parts(MAJOR, MINOR, PATCH) as 8-bits unsigned ints, LSB first. */
      if(Serial_Bit < 24)
      {
        uint8_t versionPart = 0;
        switch(Serial_Bit / 8) 
        {
          case 0:
            versionPart = VERSION_MAJOR;
            break;
          case 1:
            versionPart = VERSION_MINOR;
            break;
          case 2:
            versionPart = VERSION_PATCH;
            break;
        }

        /* Send version part bit on PANEL_U and inverted on PANEL_R. */
        uint8_t outputs = (versionPart >> (Serial_Bit % 8)) & 0x01 ? PANEL_U_OUT : PANEL_R_OUT;
        ShiftReg_WriteByte(&Outputs_ShiftReg, outputs);
        Serial_Bit++;
      }
      else
      {
        Serial_State = SERIAL_STATE_IDLE;
        ShiftReg_WriteByte(&Outputs_ShiftReg, 0);
      }
      break;
  }
}

/**
  * Handles clock pulses when sensor mask is being sent.
  * 
  * This is called when the SERIAL_CMD_SENSOR_MASK is received. Go into 
  * SERIAL_STATE_SENSOR_MASK state, receive the sensor mask then return 
  * into SERIAL_STATE_IDLE mode.
  * 
  * @retval None
  */
void Serial_Sensor_Mask_Clock_Handler(void) {
  switch(Serial_State) {
    case SERIAL_STATE_IDLE:
      Serial_State = SERIAL_STATE_SENSOR_MASK;
      Serial_Init_Tick = HAL_GetTick();
      Serial_Bit = 0;
      Inputs_Mask = 0xFFF00000;
      break;
    case SERIAL_STATE_SENSOR_MASK:
      uint8_t data = ~HAL_GPIO_ReadPin(COMM_FL5_GPIO_Port, COMM_FL5_Pin) & 0x01;
      Inputs_Mask |= data << Serial_Bit;
      if(++Serial_Bit >= 20) 
        Serial_State = SERIAL_STATE_IDLE;
      break;
  }
}

/**
  * @brief Return to SERIAL_STATE_IDLE if command reply timed out.
  * @retval None
  */
void Serial_Timeout_Check(void)
{
  if(HAL_GetTick() - Serial_Init_Tick > SERIAL_TIMEOUT_TICKS)
  { 
    /* Command timed out. Perform rollback and reset to Serial_STATE_IDLE. */
    
    /* Rollback to all inputs enabled if command was SERIAL_CMD_SENSOR_MASK. */
    if(Serial_State == SERIAL_STATE_SENSOR_MASK)
      Inputs_Mask = 0xFFFFFFFF; 

    Serial_State = SERIAL_STATE_IDLE;
  } 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  Debounce_Tick_Handler();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Triggered by TEST Pin rising edge */
  Serial_Clock_Handler();
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
    /* Blink status LED indefinitely */
    ShiftReg_WriteByte(&Lights_ShiftReg, 0);
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

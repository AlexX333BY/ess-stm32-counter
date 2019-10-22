/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stdbool.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef long_press_timer;
TIM_HandleTypeDef increment_timer;

uint32_t displayed_number = 0;
const uint32_t MAX_DISPLAYED_NUMBER = 0x7f3;
const uint16_t LONG_PRESS_TIME = 3000;
const uint16_t NUMBER_INCREMENT_TIME = 1000;
const uint16_t INCREMENT_PIN = GPIO_PIN_0,
  RESET_PIN = GPIO_PIN_1,
  OVERFLOW_SIGNAL_PIN = GPIO_PIN_7;
bool long_press_timer_reached_timeout = false;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void IncrementDisplay(void);
void ResetDisplay(void);

/* Private user code ---------------------------------------------------------*/
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  long_press_timer.Instance = TIM1;
  long_press_timer.Init.Prescaler = 8000 - 1;
  long_press_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  long_press_timer.Init.Period = LONG_PRESS_TIME;
  long_press_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  long_press_timer.Init.RepetitionCounter = 0;
  long_press_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&long_press_timer) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&long_press_timer, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&long_press_timer, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  increment_timer.Instance = TIM2;
  increment_timer.Init.Prescaler = 8000 - 1;
  increment_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  increment_timer.Init.Period = NUMBER_INCREMENT_TIME;
  increment_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  increment_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&increment_timer) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&increment_timer, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&increment_timer, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 
                           PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = INCREMENT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = OVERFLOW_SIGNAL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &long_press_timer) {
    HAL_TIM_Base_Start_IT(&increment_timer);
    HAL_TIM_Base_Stop_IT(&long_press_timer);
    long_press_timer_reached_timeout = true;
  } else if (htim == &increment_timer) {
    IncrementDisplay();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INCREMENT_PIN) {
    /* Increment button */
    if (HAL_GPIO_ReadPin(GPIOB, INCREMENT_PIN) == GPIO_PIN_RESET) {
      /* Button was pressed */
      HAL_TIM_Base_Start_IT(&long_press_timer);
    } else {
      /* Button was released */
      if (!long_press_timer_reached_timeout) {
        HAL_TIM_Base_Stop_IT(&long_press_timer);
        IncrementDisplay();
      } else {
        long_press_timer_reached_timeout = false;
      }
    }
  } else if (GPIO_Pin == RESET_PIN) {
    /* Reset button was pressed */
    ResetDisplay();
  }
}

void IncrementDisplay(void)
{
  displayed_number = (displayed_number + 1) % (MAX_DISPLAYED_NUMBER + 1);
  if (displayed_number == 0) {
    HAL_GPIO_WritePin(GPIOB, OVERFLOW_SIGNAL_PIN, GPIO_PIN_SET);
  } else if (displayed_number == 1) {
    HAL_GPIO_WritePin(GPIOB, OVERFLOW_SIGNAL_PIN, GPIO_PIN_RESET);
  }

  uint16_t cur_number_pin = GPIO_PIN_0;
  uint32_t shifted_displayed_number = displayed_number;
  const uint8_t number_pins_count = 12;
  for (uint8_t i = 0; i < number_pins_count; ++i) {
    HAL_GPIO_WritePin(GPIOA, cur_number_pin, shifted_displayed_number % 2 == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    cur_number_pin <<= 1;
    shifted_displayed_number >>= 1;
  }
}
void ResetDisplay(void)
{
  HAL_TIM_Base_Stop_IT(&long_press_timer);
  HAL_TIM_Base_Stop_IT(&increment_timer);
  HAL_GPIO_WritePin(GPIOB, OVERFLOW_SIGNAL_PIN, GPIO_PIN_RESET);
  displayed_number = 0;
  long_press_timer_reached_timeout = false;

  uint16_t cur_number_pin = GPIO_PIN_0;
  const uint8_t number_pins_count = 12;
  for (uint8_t i = 0; i < number_pins_count; ++i) {
    HAL_GPIO_WritePin(GPIOA, cur_number_pin, GPIO_PIN_RESET);
    cur_number_pin <<= 1;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

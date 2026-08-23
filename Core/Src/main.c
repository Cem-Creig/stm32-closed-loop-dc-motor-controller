/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNTS_PER_MOTOR_REV       12.0f // encoder counts for one motor turn
#define CONTROL_PERIOD_MS          150U // update motor speed every 150 ms
#define GEAR_RATIO                 235.0f
#define COUNTS_PER_OUTPUT_REV      (COUNTS_PER_MOTOR_REV * GEAR_RATIO) // encoder counts for one output shaft turn

#define KP_PWM_PER_RPM             0.50f // P control reacts to current speed error
#define PWM_MIN_RUNNING_PERCENT    20.0f
#define PWM_MAX_RUNNING_PERCENT    80.0f
#define KI_PWM_PER_RPM_SECOND      0.20f // I control corrects error left over time
#define INTEGRAL_ERROR_LIMIT_RPM_S 25.0f // stops I control growing too large

#define STEP_HOLD_MS               10000U // keep each target speed for 10 seconds
#define MOTOR_TEST_DURATION_MS     (4U * STEP_HOLD_MS) // run four speed steps for 40 seconds

#define TARGET_LOW_RPM             30.0f // low speed target
#define TARGET_MEDIUM_RPM          40.0f // medium speed target
#define TARGET_HIGH_RPM            50.0f // high speed target

#define PWM_FF_LOW_PERCENT         43.0f // starting PWM found for 30 RPM
#define PWM_FF_MEDIUM_PERCENT      55.0f // starting PWM found for 40 RPM
#define PWM_FF_HIGH_PERCENT        66.0f // starting PWM found for 50 RPM
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int32_t encoder_count = 0; // current encoder reading
int32_t previous_encoder_count = 0; // encoder reading from last update

float target_rpm = 0.0f; // speed the motor should reach
float actual_rpm = 0.0f; // speed measured from encoder
float speed_error_rpm = 0.0f; // difference between target and actual speed
float pwm_command_percent = 0.0f; // PWM worked out by controller
float integral_error_rpm_s = 0.0f; // speed error saved over time
float integral_term_percent = 0.0f; // I control correction in PWM percent

uint32_t previous_sample_time_ms = 0;
uint32_t motor_start_time_ms = 0;

uint8_t current_pwm_duty = 0U; // PWM currently sent to motor
uint8_t motor_test_running = 0U; // 1 while automatic test is running

float pwm_feedforward_percent = 0.0f; // starting PWM for current target speed
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void set_motor_pwm(uint8_t duty_percent);
static void motor_forward(void);
static void motor_reverse(void);
static void motor_stop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void set_motor_pwm(uint8_t duty_percent) // changes duty percent into timer compare value
{
  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }

  uint32_t compare_value =
      ((uint32_t)duty_percent * 4200U) / 100U;

  __HAL_TIM_SET_COMPARE(&htim3,
                        TIM_CHANNEL_1,
                        compare_value);
}

static void motor_forward(void) // sets driver pins for forward direction
{

  set_motor_pwm(0U);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

static void motor_reverse(void) // sets driver pins for reverse direction
{

  set_motor_pwm(0U);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

static void motor_stop(void) // stops PWM and disables motor driver
{

  set_motor_pwm(0U);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_5,
                    GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  motor_stop(); // keep motor off during setup

  // start the encoder counter
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  // start the PWM output
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }


  target_rpm = TARGET_LOW_RPM; // start with 30 RPM target
  pwm_feedforward_percent = PWM_FF_LOW_PERCENT; // use starting PWM found for 30 RPM


  printf("timestamp_ms,target_rpm,actual_rpm,error_rpm,i_term_percent,pwm_duty\r\n"); // send CSV headings to Python


  HAL_Delay(3000);


  __HAL_TIM_SET_COUNTER(&htim2, 0); // clear old encoder count before test
  previous_encoder_count = 0;
  previous_sample_time_ms = HAL_GetTick();
  integral_error_rpm_s = 0.0f;
  integral_term_percent = 0.0f;


  motor_forward(); // choose forward direction

  pwm_command_percent = pwm_feedforward_percent;

  current_pwm_duty =
      (uint8_t)(pwm_command_percent + 0.5f);

  set_motor_pwm(current_pwm_duty); // start motor with low speed PWM

  motor_start_time_ms = HAL_GetTick(); // save when the 40 second test starts
  motor_test_running = 1U; // allow automatic speed test to run
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint32_t now_ms = HAL_GetTick(); // read current time

	  if (motor_test_running != 0U) // only choose targets while test is running
	  {
	      uint32_t test_elapsed_ms =
	          now_ms - motor_start_time_ms;

	      if (test_elapsed_ms < STEP_HOLD_MS)
	      {
	          target_rpm = TARGET_LOW_RPM; // run at 30 RPM for first 10 seconds
	          pwm_feedforward_percent =
	              PWM_FF_LOW_PERCENT;
	      }
	      else if (test_elapsed_ms <
	               (2U * STEP_HOLD_MS))
	      {
	          target_rpm = TARGET_MEDIUM_RPM; // run at 40 RPM for next 10 seconds
	          pwm_feedforward_percent =
	              PWM_FF_MEDIUM_PERCENT;
	      }
	      else if (test_elapsed_ms <
	               (3U * STEP_HOLD_MS))
	      {
	          target_rpm = TARGET_HIGH_RPM; // run at 50 RPM for next 10 seconds
	          pwm_feedforward_percent =
	              PWM_FF_HIGH_PERCENT;
	      }
	      else
	      {
	          target_rpm = TARGET_LOW_RPM; // return to 30 RPM for final 10 seconds
	          pwm_feedforward_percent =
	              PWM_FF_LOW_PERCENT;
	      }
	  }
	  // measure speed every 150 ms and update PI while running
	      if ((now_ms - previous_sample_time_ms) >= CONTROL_PERIOD_MS)
	      {
	    	  // read current encoder count
	          encoder_count =
	              (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

	          // find encoder movement since last update
	          int32_t delta_counts =
	              encoder_count - previous_encoder_count;

	          uint32_t elapsed_ms =
	              now_ms - previous_sample_time_ms;

	          float elapsed_seconds =
	              (float)elapsed_ms / 1000.0f;

	          // change encoder movement into output shaft RPM
	          actual_rpm =
	              ((float)delta_counts * 60000.0f) /
	              (COUNTS_PER_OUTPUT_REV * (float)elapsed_ms);

	          speed_error_rpm =
	              target_rpm - actual_rpm;

	          if (motor_test_running != 0U)
	          {
	              // add speed error over time for I control
	              integral_error_rpm_s +=
	                  speed_error_rpm * elapsed_seconds;

	              // keep saved error between safe limits
	              if (integral_error_rpm_s >
	                  INTEGRAL_ERROR_LIMIT_RPM_S)
	              {
	                  integral_error_rpm_s =
	                      INTEGRAL_ERROR_LIMIT_RPM_S;
	              }
	              else if (integral_error_rpm_s <
	                       (-INTEGRAL_ERROR_LIMIT_RPM_S))
	              {
	                  integral_error_rpm_s =
	                      -INTEGRAL_ERROR_LIMIT_RPM_S;
	              }


	              integral_term_percent =
	                  KI_PWM_PER_RPM_SECOND *
	                  integral_error_rpm_s;

	              // use starting PWM plus P and I corrections
	              pwm_command_percent =
	                  pwm_feedforward_percent +
	                  (KP_PWM_PER_RPM * speed_error_rpm) +
	                  integral_term_percent;
	              // keep running PWM between 20 and 80 percent
	              if (pwm_command_percent >
	                  PWM_MAX_RUNNING_PERCENT)
	              {
	                  pwm_command_percent =
	                      PWM_MAX_RUNNING_PERCENT;
	              }
	              else if (pwm_command_percent <
	                       PWM_MIN_RUNNING_PERCENT)
	              {
	                  pwm_command_percent =
	                      PWM_MIN_RUNNING_PERCENT;
	              }


	              current_pwm_duty =
	                  (uint8_t)(pwm_command_percent + 0.5f);

	              set_motor_pwm(current_pwm_duty);
	          }
	          // send one CSV row to Python
	          printf("%lu,%.1f,%.1f,%.1f,%.2f,%u\r\n",
	                 (unsigned long)now_ms,
	                 target_rpm,
	                 actual_rpm,
	                 speed_error_rpm,
	                 integral_term_percent,
	                 (unsigned int)current_pwm_duty);

	          previous_encoder_count = encoder_count; // save encoder count for next update
	          previous_sample_time_ms = now_ms; // save time for next update
	      }
	      // stop motor after 40 seconds
	      if ((motor_test_running != 0U) &&
	          ((now_ms - motor_start_time_ms) >=
	           MOTOR_TEST_DURATION_MS))
	      {
	          motor_stop();

	          target_rpm = 0.0f; // show 0 RPM target after motor stops
	          pwm_feedforward_percent = 0.0f; // clear starting PWM after test

	          pwm_command_percent = 0.0f;
	          integral_error_rpm_s = 0.0f;
	          integral_term_percent = 0.0f;
	          current_pwm_duty = 0U;
	          motor_test_running = 0U;
	      }
	  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;

  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,(uint16_t)len,HAL_MAX_DELAY);

  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

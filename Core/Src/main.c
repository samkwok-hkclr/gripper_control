/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "stdbool.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Stopping criteria: check every 20ms
const uint16_t EXECUTION_TIMEOUT = 50 - 1; // timeout: 1000ms

uint32_t running = 0;

const uint16_t FULL_DUTY_CYCLE = 100;
const uint16_t DUTY_CYCLE_UPPER_BOUND = 55;
const uint16_t DUTY_CYCLE_LOWER_BOUND = 85;
const uint16_t MAX_DIFF = 10;
const uint16_t MIN_DIFF_PER_CYCLE = 2;

uint16_t left_duty_cycle = 55; // Initial duty cycle
uint16_t right_duty_cycle = 55; // Initial duty cycle

uint8_t sensor_states[3] = {0, 0, 0};
uint8_t resp[8] = {0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE};
uint16_t wrong_command[4] = {0, 0, 0, 0};
bool left_enabled = false;
bool right_enabled = false;
uint16_t left_steps_diff = 0;
uint16_t left_steps_sum = 0;
uint16_t right_steps_diff = 0;
uint16_t right_steps_sum = 0;
uint16_t left_period_cnt = 0;
uint16_t right_period_cnt = 0;
uint16_t left_cycle = 0;
uint16_t right_cycle = 0;
uint16_t execution_time_cnt = 0;
uint8_t uart_recv = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, &uart_recv, 1);

  set_duty_cycle(FULL_DUTY_CYCLE, FULL_DUTY_CYCLE);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_3);

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  running++;
	  HAL_Delay(800);
	  HAL_IWDG_Refresh(&hiwdg);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */

inline uint16_t calc_duty_cycle(const uint16_t x)
{
	// if x > diff_max, then y = DUTY_CYCLE_UPPER_BOUND
	// if x < 0, then y = DUTY_CYCLE_LOWER_BOUND
	// y = (D_upper - D_lower) / diff_max * x + DUTY_CYCLE_UPPER_BOUND

	if (x > MAX_DIFF)
		return DUTY_CYCLE_UPPER_BOUND;
	else if (x < 0)
		return DUTY_CYCLE_LOWER_BOUND;

	uint16_t y = (DUTY_CYCLE_UPPER_BOUND - DUTY_CYCLE_LOWER_BOUND) / MAX_DIFF * x + DUTY_CYCLE_LOWER_BOUND;

	if (y > DUTY_CYCLE_LOWER_BOUND)
		return DUTY_CYCLE_LOWER_BOUND;
	else if (y < DUTY_CYCLE_UPPER_BOUND)
		return DUTY_CYCLE_UPPER_BOUND;

	return y;
}

inline void set_duty_cycle(const uint16_t left_value, const uint16_t right_value)
{
	htim2.Instance->CCR1 = left_value;
	htim2.Instance->CCR3 = right_value;
}

inline void init_values()
{
	left_steps_sum = 0;
	left_steps_diff = 0;
	right_steps_sum = 0;
	right_steps_diff = 0;
	left_period_cnt = 0;
	right_period_cnt = 0;
	execution_time_cnt = 0;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (left_enabled && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		left_steps_sum += 1;
		left_steps_diff += 1;
	}
	if (right_enabled && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		right_steps_sum += 1;
		right_steps_diff += 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim == (&htim4))
	{
		if (execution_time_cnt > 1)
		{
//			left_cycle = left_enabled ? calc_duty_cycle(left_steps_diff) : FULL_DUTY_CYCLE;
			left_cycle = left_enabled ? calc_duty_cycle(left_steps_diff) - 3 : FULL_DUTY_CYCLE;
			right_cycle = right_enabled ? calc_duty_cycle(right_steps_diff) : FULL_DUTY_CYCLE;
			set_duty_cycle(left_cycle, right_cycle);

			left_enabled = left_steps_diff > MIN_DIFF_PER_CYCLE;
			right_enabled = right_steps_diff > MIN_DIFF_PER_CYCLE;
		}

		if ((execution_time_cnt > EXECUTION_TIMEOUT) || (!left_enabled && !right_enabled))
		{
			// stop motors
			left_enabled = false;
			right_enabled = false;
			set_duty_cycle(FULL_DUTY_CYCLE, FULL_DUTY_CYCLE);

			// |   0-1   |   2-3   |   4   |   5   |   6-7   |
			// | l_steps | r_steps |  l/r_mv_flag  | tim_cnt |
			resp[0] = (uint8_t)(left_steps_sum & 0x00FF);
			resp[1] = (uint8_t)((left_steps_sum & 0xFF00) >> 8);
			resp[2] = (uint8_t)(right_steps_sum & 0x00FF);
			resp[3] = (uint8_t)((right_steps_sum & 0xFF00) >> 8);
			resp[4] = (uint8_t)left_steps_diff;
			resp[5] = (uint8_t)right_steps_diff;

			resp[6] = (uint8_t)(execution_time_cnt & 0x00FF);
			resp[7] = (uint8_t)((execution_time_cnt & 0xFF00) >> 8);

			init_values();

			HAL_TIM_Base_Stop_IT(&htim4);
			HAL_UART_Transmit(&huart1, (uint8_t*)&resp, 8, 0xFFFF);
			HAL_UART_Receive_IT(&huart1, &uart_recv, 1);
		}

		execution_time_cnt += 1;

		left_steps_diff = 0;
		right_steps_diff = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	if (uart_recv == 0)
	{
		set_duty_cycle(FULL_DUTY_CYCLE, FULL_DUTY_CYCLE);
		HAL_UART_Receive_IT(&huart1, &uart_recv, 1);
	}
	else if (uart_recv == 1) //make finger vertical
	{
		// set two finger rotate direction
		init_values();
		HAL_GPIO_WritePin(Left_Direction_GPIO_Port, Left_Direction_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Right_Direction_GPIO_Port, Right_Direction_Pin, GPIO_PIN_SET);

		// set moving flags
		left_enabled = true;
		right_enabled = true;
		// start rotating...
		set_duty_cycle(DUTY_CYCLE_UPPER_BOUND, DUTY_CYCLE_UPPER_BOUND);

		// start timer to check finish signs
		HAL_TIM_Base_Start_IT(&htim4);
	}
	else if (uart_recv == 2) //make finger horizon
	{
		// set two finger rotate direction
		init_values();
		HAL_GPIO_WritePin(Left_Direction_GPIO_Port, Left_Direction_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Right_Direction_GPIO_Port, Right_Direction_Pin, GPIO_PIN_RESET);

		// set moving flags
		left_enabled = true;
		right_enabled = true;
		// start rotating...
		set_duty_cycle(DUTY_CYCLE_UPPER_BOUND, DUTY_CYCLE_UPPER_BOUND);

		// start timer to check finish signs
		HAL_TIM_Base_Start_IT(&htim4);
	}
	else if (uart_recv == 3) //read sensors info
	{
		sensor_states[0]= (uint8_t)HAL_GPIO_ReadPin(Through_Sensor_GPIO_Port, Through_Sensor_Pin);
		sensor_states[1]= (uint8_t)HAL_GPIO_ReadPin(Forward_Left_GPIO_Port, Forward_Left_Pin);
		sensor_states[2]= (uint8_t)HAL_GPIO_ReadPin(Forward_Right_GPIO_Port, Forward_Right_Pin);

		//enable receive interrupt for next serial input
		HAL_UART_Transmit(&huart1, sensor_states, 3, 0xFFFF);
		HAL_UART_Receive_IT(&huart1, &uart_recv, 1);
	}
	else
	{
		//enable receive interrupt for next serial input
		HAL_UART_Transmit(&huart1, (uint8_t*) &wrong_command, 8, 100); // 0xFFFF
		HAL_UART_Receive_IT(&huart1, &uart_recv, 1);
	}
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

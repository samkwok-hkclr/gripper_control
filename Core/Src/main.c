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

// finish signs: check every 20ms
const uint16_t LEFT_SIGN = 5; // not used
const uint16_t RIGHT_SIGN = 5; // not used
const uint16_t TIMEOUT_THD = 30; // timeout: 600ms

uint16_t sensor_states[3] = {0, 0, 0};
uint16_t time_lapse[4] = {0, 0, 0, 0};
uint16_t wrong_command[4] = {0, 0, 0, 0};
uint16_t left_moving = 0;
uint16_t right_moving = 0;
uint16_t left_count = 0;
uint16_t right_count = 0;
uint16_t left_counter_prev = 0;
uint16_t left_counter_now = 200;
uint16_t right_counter_prev = 0;
uint16_t right_counter_now = 200;
uint16_t finger_move_timeout_count = 0;
uint8_t uart_receive = 0;

const uint16_t FULL_DUTY_CYCLE = 100;
uint16_t left_duty_cycle = 90;
uint16_t right_duty_cycle = 90;

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

  HAL_UART_Receive_IT(&huart1, &uart_receive, 1);

  set_duty_cycle(FULL_DUTY_CYCLE, FULL_DUTY_CYCLE);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

inline void set_duty_cycle(const uint16_t left_value, const uint16_t right_value)
{
	htim2.Instance->CCR1 = left_value;
	htim3.Instance->CCR1 = right_value;
}

inline void init_values()
{
	left_counter_now = 200;
	left_counter_prev = 0;
	right_counter_now = 200;
	right_counter_prev = 0;
	left_count = 0;
	right_count = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim4))
	{
		// This stopping criteria is unstable.
//		if (left_moving == 1 && ((left_counter_now - left_counter_prev) < LEFT_SIGN))
//		{
//			set_duty_cycle(FULL_DUTY_CYCLE, htim3.Instance->CCR1);
//			left_moving = 0;
//		}
//		else if (left_moving == 1)
//		{
//			left_count += 1;
//			left_counter_prev = left_counter_now;
//		}

//		if (right_moving == 1 && ((right_counter_now - right_counter_prev) < RIGHT_SIGN))
//		{
//			set_duty_cycle(htim2.Instance->CCR1, FULL_DUTY_CYCLE);
//			right_moving = 0;
//		}
//		else if (right_moving == 1)
//		{
//			right_count += 1;
//			right_counter_prev = right_counter_now;
//		}

		finger_move_timeout_count += 1;
		if (finger_move_timeout_count > TIMEOUT_THD)
		{
			set_duty_cycle(FULL_DUTY_CYCLE, FULL_DUTY_CYCLE);
			time_lapse[0] = left_count;
			time_lapse[1] = right_count;
			time_lapse[2] = left_counter_now;
			time_lapse[3] = right_counter_now;

			init_values();

			left_count = 0;
			right_count = 0;
			HAL_TIM_Base_Stop_IT(&htim4);
			finger_move_timeout_count = 0;
			HAL_UART_Transmit(&huart1, (uint8_t*) &time_lapse, 8, 100); // 0xFFFF
			HAL_UART_Receive_IT(&huart1, &uart_receive, 1);
		}
		else if (right_moving == 0 && left_moving == 0)
		{
			time_lapse[0] = left_count;
			time_lapse[1] = right_count;
			time_lapse[2] = left_counter_now;
			time_lapse[3] = right_counter_now;

			init_values();

			HAL_TIM_Base_Stop_IT(&htim4);
			finger_move_timeout_count = 0;
			HAL_UART_Transmit(&huart1, (uint8_t*) &time_lapse, 8, 100); // 0xFFFF
			HAL_UART_Receive_IT(&huart1, &uart_receive, 1);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Left_Feedback_Pin)
	{
		if (left_moving == 1)
		{
			left_counter_now += 1;
		}
	}
	else if (GPIO_Pin == Right_Feedback_Pin)
	{
		if (right_moving == 1)
		{
			right_counter_now += 1;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	if (uart_receive == 0)
	{
		set_duty_cycle(FULL_DUTY_CYCLE, FULL_DUTY_CYCLE);
		HAL_UART_Receive_IT(&huart1, &uart_receive, 1);
	}
	else if (uart_receive == 1) //make finger vertical
	{
		// set two finger rotate direction
		HAL_GPIO_WritePin(Left_Direction_GPIO_Port, Left_Direction_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Right_Direction_GPIO_Port, Right_Direction_Pin, GPIO_PIN_SET);
		// start rotating...
		set_duty_cycle(left_duty_cycle, right_duty_cycle);
		// set moving flags
		left_moving = 1;
		right_moving = 1;
		// start timer to check finish signs
		HAL_TIM_Base_Start_IT(&htim4);
	}

	else if (uart_receive == 2) //make finger horizon
	{
		// set two finger rotate direction
		HAL_GPIO_WritePin(Left_Direction_GPIO_Port, Left_Direction_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Right_Direction_GPIO_Port, Right_Direction_Pin, GPIO_PIN_RESET);
		// start rotating...
		set_duty_cycle(left_duty_cycle, right_duty_cycle);
		// set moving flags
		left_moving = 1;
		right_moving = 1;
		// start timer to check finish signs
		HAL_TIM_Base_Start_IT(&htim4);
	}
	else if (uart_receive == 3) //read sensors info
	{
		sensor_states[0]= HAL_GPIO_ReadPin(Forward_Left_GPIO_Port, Forward_Left_Pin);
		sensor_states[1]= HAL_GPIO_ReadPin(Forward_Right_GPIO_Port, Forward_Right_Pin);
		sensor_states[2]= HAL_GPIO_ReadPin(Through_Sensor_GPIO_Port, Through_Sensor_Pin);
		//enable receive interrupt for next serial input
		HAL_UART_Transmit(&huart1, (uint8_t*) &sensor_states, 6, 100); // 0xFFFF
		HAL_UART_Receive_IT(&huart1, &uart_receive, 1);
	}
	else
	{
		//enable receive interrupt for next serial input
		HAL_UART_Transmit(&huart1, (uint8_t*) &wrong_command, 8, 100); // 0xFFFF
		HAL_UART_Receive_IT(&huart1, &uart_receive, 1);
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

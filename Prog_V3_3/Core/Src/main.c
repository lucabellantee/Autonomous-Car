/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "servo_motor.h"
#include "PID.h"
#include "DC_motor.h"
#include "bno055_stm32.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int flag = 0;
int n = 0;
int n1 = 0;
PID pid, pid_servo;
double u;
double u1;
float y_ref = 0;
uint8_t direction;
double duty;
double old_duty = 0;
double delta_duty = 0;
int flag_Tc = 0;
int n_ref = 0;
int counts = 0;
double delta_angle;
double diff_angle;
int diff_count;
int ref_count;
int dir;
float speed;
double old_delta_angle;
int old_dir = 0;
int ENCODER_PPR = 2048;
int GEARBOX_RATIO = 1;
int ENCODER_COUNTING_MODE = 4;
double dt = 0.1;
double dt_servo = 0.01;
int flag10_ms = 0;
int conta = 0;
double curvatura_riferimento = 0;
double yaw_ref = 0;
double raggio_riferimento = 1.2;
float m_s = 1;
double steering_angle = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim11);
	init_PID(&pid, dt, MAX_V_ENCODER, -MAX_V_ENCODER);
	tune_PID(&pid, 0.001, 0.0018, 0);

	init_PID(&pid_servo, dt_servo, MAX_ANGOLO, MIN_ANGOLO);
	tune_PID(&pid_servo, 1.25, 250, 0);
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, BNO055_I2C_ADDR << 1,
			5, 1000);
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (flag_Tc == 1) {
			flag_Tc = 0;

			ref_count = TIM2->ARR / 2;
			diff_count = counts - ref_count;
			diff_angle = (diff_count * 360)
					/ ((double) (ENCODER_PPR * ENCODER_COUNTING_MODE
							* GEARBOX_RATIO));
			speed = diff_angle / dt;

			direction = Ref2Direction(y_ref);

			y_ref = m_s / GIRI_MIN_CONV;

			float speed2 = speed;
			float y_ref2 = y_ref;
			if (speed2 < 0)
				speed2 = -speed2;
			if (y_ref2 < 0)
				y_ref2 = -y_ref2;

			u = PID_controller(&pid, speed2 * 60 / 360, y_ref2);
			delta_duty = Voltage2Duty(u);
			duty = old_duty + delta_duty;
			set_PWM_and_dir((uint32_t) duty, direction);
		}

		if(flag10_ms == 1){
			flag10_ms = 0;
			bno055_vector_t v = bno055_getVectorGyroscope();
			double yaw_rate = v.z;
			double yaw_rate_rad = (yaw_rate*PI)/180;
			if (raggio_riferimento == 0){
				u1 = PID_controller(&pid_servo, yaw_rate, 0);
				servo_motor(-u1);
			} else {
				yaw_ref = m_s/(raggio_riferimento);
				double yaw_rate2 = yaw_rate_rad;
				double yaw_ref2 = yaw_ref;
				if (yaw_rate2 < 0)
				yaw_rate2 = -yaw_rate2;
				if (yaw_ref2 < 0)
				yaw_ref2= -yaw_ref2;

				u1 = PID_controller(&pid_servo, yaw_rate2, yaw_ref2);

				if(raggio_riferimento >= 0 && u1 > 0)
					u1*=-1;
				if(raggio_riferimento <  0 && u1 < 0)
					u1*=-1;
				servo_motor((int)u1);
			}
		}

		 /* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* USER CODE END 3 */
	}
}

	/**
	 * @brief System Clock Configuration
	 * @retval None
	 */
	void SystemClock_Config(void) {
		RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
		RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/** Initializes the CPU, AHB and APB buses clocks
		 */
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
				| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)
				!= HAL_OK) {
			Error_Handler();
		}
	}

	/**
	 * @brief I2C1 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_I2C1_Init(void) {

		/* USER CODE BEGIN I2C1_Init 0 */

		/* USER CODE END I2C1_Init 0 */

		/* USER CODE BEGIN I2C1_Init 1 */

		/* USER CODE END I2C1_Init 1 */
		hi2c1.Instance = I2C1;
		hi2c1.Init.ClockSpeed = 100000;
		hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
		hi2c1.Init.OwnAddress1 = 0;
		hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c1.Init.OwnAddress2 = 0;
		hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN I2C1_Init 2 */

		/* USER CODE END I2C1_Init 2 */

	}

	/**
	 * @brief TIM1 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM1_Init(void) {

		/* USER CODE BEGIN TIM1_Init 0 */

		/* USER CODE END TIM1_Init 0 */

		TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
		TIM_MasterConfigTypeDef sMasterConfig = { 0 };
		TIM_OC_InitTypeDef sConfigOC = { 0 };
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

		/* USER CODE BEGIN TIM1_Init 1 */

		/* USER CODE END TIM1_Init 1 */
		htim1.Instance = TIM1;
		htim1.Init.Prescaler = 1681 - 1;
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.Period = 1001 - 1;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.RepetitionCounter = 0;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
			Error_Handler();
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
				!= HAL_OK) {
			Error_Handler();
		}
		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 0;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM1_Init 2 */

		/* USER CODE END TIM1_Init 2 */
		HAL_TIM_MspPostInit(&htim1);

	}

	/**
	 * @brief TIM2 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM2_Init(void) {

		/* USER CODE BEGIN TIM2_Init 0 */

		/* USER CODE END TIM2_Init 0 */

		TIM_Encoder_InitTypeDef sConfig = { 0 };
		TIM_MasterConfigTypeDef sMasterConfig = { 0 };

		/* USER CODE BEGIN TIM2_Init 1 */

		/* USER CODE END TIM2_Init 1 */
		htim2.Instance = TIM2;
		htim2.Init.Prescaler = 0;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = 4294967295 - 1;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
		sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
		sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
		sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
		sConfig.IC1Filter = 10;
		sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
		sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
		sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
		sConfig.IC2Filter = 0;
		if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM2_Init 2 */

		/* USER CODE END TIM2_Init 2 */

	}

	/**
	 * @brief TIM10 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM10_Init(void) {

		/* USER CODE BEGIN TIM10_Init 0 */

		/* USER CODE END TIM10_Init 0 */

		TIM_OC_InitTypeDef sConfigOC = { 0 };

		/* USER CODE BEGIN TIM10_Init 1 */

		/* USER CODE END TIM10_Init 1 */
		htim10.Instance = TIM10;
		htim10.Init.Prescaler = 84 - 1;
		htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim10.Init.Period = 1000 - 1;
		htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
			Error_Handler();
		}
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM10_Init 2 */

		/* USER CODE END TIM10_Init 2 */
		HAL_TIM_MspPostInit(&htim10);

	}

	/**
	 * @brief TIM11 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM11_Init(void) {

		/* USER CODE BEGIN TIM11_Init 0 */

		/* USER CODE END TIM11_Init 0 */

		/* USER CODE BEGIN TIM11_Init 1 */

		/* USER CODE END TIM11_Init 1 */
		htim11.Instance = TIM11;
		htim11.Init.Prescaler = 120 - 1;
		htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim11.Init.Period = 7000 - 1;
		htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM11_Init 2 */

		/* USER CODE END TIM11_Init 2 */

	}

	/**
	 * @brief USART2 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_USART2_UART_Init(void) {

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
		if (HAL_UART_Init(&huart2) != HAL_OK) {
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
	static void MX_GPIO_Init(void) {
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOH_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pin : DIR_Pin */
		GPIO_InitStruct.Pin = DIR_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

	}

	/* USER CODE BEGIN 4 */

//giansimone, walter
	int __io_putchar(int ch) {
		HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
		return ch;
	}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		if (htim == &htim11) {
			flag10_ms = 1;
			n++;
			n1++;

			if (n1 == 200) {
				raggio_riferimento = 0;
			}

			if (n1 == 400) {
				raggio_riferimento = -1.4;
			}

			if (n == 10) {
				counts = TIM2->CNT;
				flag_Tc = 1;
				TIM2->CNT = TIM2->ARR / 2;
				n = 0;
			}

		}
	}

	/* USER CODE END 4 */

	/**
	 * @brief  This function is executed in case of error occurrence.
	 * @retval None
	 */
	void Error_Handler(void) {
		/* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		__disable_irq();
		while (1) {
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

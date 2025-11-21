/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "filter.h"
#include "crsf.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_FALLBACK_SECONDS           0.004f
#define DT_MAX_SECONDS                0.02f
#define THROTTLE_ACTIVE_THRESHOLD_US  1050.0f
#define ARM_CHANNEL                   4        // CRSF channel 5 (0-based index)
#define LOOP_PERIOD_US                2000U    // 500 Hz
#define LOOP_DT_SECONDS 			  (LOOP_PERIOD_US * 1e-6f)
#define ESC_LOW                       1000U
#define ESC_HIGH                      2000U
#define CRSF_TIMEOUT_MS   			  500

#define MAX_RATE_DPS 500.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CRSF_t crsf;
uint8_t rxBuf[128];
icm_scaled_t imu;

PID_t pid_roll = { .kp = 0.1f, .ki = 0.06f, .kd = 0.005f, .integrator = 0.0f,
		.prev_error = 0.0f, .d_filter = 0.0f, .d_cutoff = 0.05f, // Low-pass smoothing factor (~50Hz)
		.out_limit = 400.0f, .i_limit = 200.0f };

PID_t pid_pitch = { .kp = 0.1f, .ki = 0.06f, .kd = 0.005f, .integrator = 0.0f,
		.prev_error = 0.0f, .d_filter = 0.0f, .d_cutoff = 0.05f, .out_limit =
				400.0f, .i_limit = 200.0f };

PID_t pid_yaw = { .kp = 0.05f, .ki = 0.01f, .kd = 0.0f, .integrator = 0.0f,
		.prev_error = 0.0f, .d_filter = 0.0f, .d_cutoff = 0.05f, .out_limit =
				400.0f, .i_limit = 200.0f };

static float gx_f = 0;
static float gy_f = 0;
static float gz_f = 0;
const float alpha = 0.25f;

uint16_t m11 = 0;
uint16_t m12 = 0;
uint16_t m13 = 0;
uint16_t m14 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void DWT_Init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t micros(void) {
	return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

//static float compute_dt_seconds(uint32_t now_us, uint32_t last_us) {
//	uint32_t delta =
//			(now_us >= last_us) ?
//					(now_us - last_us) : ((0xFFFFFFFFU - last_us) + now_us);
//	float dt = (float) delta / 1e6f;
//	if (dt > DT_MAX_SECONDS)
//		dt = DT_MAX_SECONDS;
//	else if (dt < 0.001f)
//		dt = DT_FALLBACK_SECONDS;
//	return dt;
//}

void print(const char *s) {
	HAL_UART_Transmit(&huart4, (uint8_t*) s, strlen(s), HAL_MAX_DELAY);
}
//
//void print_icm_data(icm_scaled_t *data) {
//	char buf[128];
//	snprintf(buf, sizeof(buf), "AX:%6d AY:%6d AZ:%6d mg | "
//			"GX:%6d GY:%6d GZ:%6d mdps | "
//			"TEMP:%3d C\r\n", (int) (data->ax * 1000), (int) (data->ay * 1000),
//			(int) (data->az * 1000), (int) (data->gx * 1000),
//			(int) (data->gy * 1000), (int) (data->gz * 1000),
//			(int) (data->temp));
//	print(buf);
//}
//
//void print_att(attitude_t *att) {
//	char buf[128];
//	snprintf(buf, sizeof(buf), "Roll: %.2f Pitch: %.2f\r\n",
//			att->roll * 57.2958f, att->pitch * 57.2958f);
//
//	print(buf);
//}

void ESC_Write_us(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t us) {
	__HAL_TIM_SET_COMPARE(htim, channel, us);
}

static float rc_to_rate(uint16_t ch) {
	float x = (float) ch - 1500.0f;  // -500..+500
	// deadband
	if (x > -5.0f && x < 5.0f)
		x = 0.0f;
	// map -500..+500 -> -1..+1
	float norm = x / 500.0f;
	if (norm > 1.0f)
		norm = 1.0f;
	if (norm < -1.0f)
		norm = -1.0f;
	return norm * MAX_RATE_DPS; // deg/s
}

static void idle_all(void) {
	ESC_Write_us(&htim2, TIM_CHANNEL_1, 1000);
	ESC_Write_us(&htim2, TIM_CHANNEL_2, 1000);
	ESC_Write_us(&htim2, TIM_CHANNEL_3, 1000);
	ESC_Write_us(&htim2, TIM_CHANNEL_4, 1000);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	DWT_Init();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_TIM13_Init();
	MX_TIM7_Init();
	MX_TIM5_Init();
	MX_UART4_Init();
	MX_USART2_UART_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	CRSF_Init(&crsf);  // <- sets neutral channels, clears state, valid=false
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxBuf, sizeof(rxBuf));
	__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
//
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // M1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // M2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // M3
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // M4

	/* IMU init */
	if (ICM_init(&hspi2, &imu)) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET); // status LED1

	float roll_input = 0;
	float pitch_input = 0;
	float yaw_input = 0;
	float throttle = 1000;

	float roll_error = 0, pitch_error = 0, yaw_error = 0;
	float roll_output = 0, pitch_output = 0, yaw_output = 0;

	int armed = 0;

	PID_Reset(&pid_roll);
	PID_Reset(&pid_pitch);
	PID_Reset(&pid_yaw);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_Delay(500); // let IMU settle
	for (int i = 0; i < 500; i++) {
		ICM_read_all(&hspi2, &imu);
		gx_f += alpha * (imu.gx - gx_f);
		gy_f += alpha * (imu.gy - gy_f);
		gz_f += alpha * (imu.gz - gz_f);
		HAL_Delay(1);
	}

	const float dt = LOOP_DT_SECONDS;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET); // status LED2

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		uint32_t loop_start = micros();
		// Reading RC INPUT
		const CRSF_Data_t *rc = CRSF_GetData(&crsf);

//		if ((HAL_GetTick() - rc->lastUpdate) > CRSF_TIMEOUT_MS) {
//			armed = 0;
//			roll_input = 0;
//			pitch_input = 0;
//			yaw_input = 0;
//			throttle = 1000;
//			PID_Reset(&pid_roll);
//			PID_Reset(&pid_pitch);
//			PID_Reset(&pid_yaw);
//			idle_all();
//			continue;
//		}
		if (rc->valid) {
			roll_input = rc_to_rate(rc->channels[0]);
			pitch_input = rc_to_rate(rc->channels[1]);
			yaw_input = rc_to_rate(rc->channels[3]);

			throttle = (float) rc->channels[2];
			if (throttle < 1050.0f) {
				throttle = 1050.0f;
			}

			if (rc->channels[4] > 1500 && throttle < 1100) {
				armed = 1;
			} else if (rc->channels[4] < 1500) {
				armed = 0;
			}
		} else {
			roll_input = 0;
			pitch_input = 0;
			yaw_input = 0;
			throttle = 1050;
		}

//		if (!armed) {
//			PID_Reset(&pid_roll);
//			PID_Reset(&pid_pitch);
//			PID_Reset(&pid_yaw);
//			idle_all();
//			continue;
//		}

		// Reading and filtering GYRO Data.
		ICM_read_all(&hspi2, &imu);
		gx_f += alpha * (imu.gx - gx_f);
		gy_f += alpha * (imu.gy - gy_f);
		gz_f += alpha * (imu.gz - gz_f);

		roll_error = roll_input - gx_f;
		pitch_error = pitch_input - gy_f;
		yaw_error = yaw_input - gz_f;

		roll_output = PID_Update(&pid_roll, roll_error, dt);
		pitch_output = PID_Update(&pid_pitch, pitch_error, dt);
		yaw_output = PID_Update(&pid_yaw, yaw_error, dt);

		float m1 = throttle + roll_output + pitch_output - yaw_output; // Front Right (CW)
		float m2 = throttle + roll_output - pitch_output + yaw_output; // Rear Right (CCW)
		float m3 = throttle - roll_output - pitch_output - yaw_output; // Rear Left  (CW)
		float m4 = throttle - roll_output + pitch_output + yaw_output; // Front Left (CCW)

		float maxMotor = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));
		float minMotor = fminf(fminf(m1, m2), fminf(m3, m4));

		float range = maxMotor - minMotor;

		// Prevent divide-by-zero (or extremely small range)
		if (fabsf(range) > 1e-6f) {
			if (maxMotor > ESC_HIGH) {
				float scale = (ESC_HIGH - minMotor) / range;
				m1 = (m1 - minMotor) * scale + minMotor;
				m2 = (m2 - minMotor) * scale + minMotor;
				m3 = (m3 - minMotor) * scale + minMotor;
				m4 = (m4 - minMotor) * scale + minMotor;
			} else if (minMotor < ESC_LOW) {
				float scale = (maxMotor - ESC_LOW) / range;
				m1 = (m1 - maxMotor) * scale + maxMotor;
				m2 = (m2 - maxMotor) * scale + maxMotor;
				m3 = (m3 - maxMotor) * scale + maxMotor;
				m4 = (m4 - maxMotor) * scale + maxMotor;
			}
		}

		m11 = (uint16_t) m1;
		if (m11 < 1025) {
			m11 = 1025;
		} else if (m11 > ESC_HIGH) {
			m11 = ESC_HIGH;
		}

		m12 = (uint16_t) m2;
		if (m12 < 1025) {
			m12 = 1025;
		} else if (m12 > ESC_HIGH) {
			m12 = ESC_HIGH;
		}

		m13 = (uint16_t) m3;
		if (m13 < 1025) {
			m13 = 1025;
		} else if (m13 > ESC_HIGH) {
			m13 = ESC_HIGH;
		}

		m14 = (uint16_t) m1;
		if (m14 < 1025) {
			m14 = 1025;
		} else if (m14 > ESC_HIGH) {
			m14 = ESC_HIGH;
		}

		ESC_Write_us(&htim2, TIM_CHANNEL_1, (uint16_t) m11); // M1: Front Right
		ESC_Write_us(&htim2, TIM_CHANNEL_2, (uint16_t) m12); // M2: Rear Right
		ESC_Write_us(&htim2, TIM_CHANNEL_3, (uint16_t) m13); // M3: Rear Left
		ESC_Write_us(&htim2, TIM_CHANNEL_4, (uint16_t) m14); // M4: Front Left

		while ((micros() - loop_start) < LOOP_PERIOD_US) {
			__NOP(); // wait until 2 ms passed
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 120;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART2) {
		uint32_t now = HAL_GetTick();  // pass current time in ms

		for (uint16_t i = 0; i < Size; i++) {
			CRSF_ParseByte(&crsf, rxBuf[i], now);
		}

		// Re-arm reception
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxBuf, sizeof(rxBuf));
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		// Clear typical error flags
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		// Re-arm reception
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxBuf, sizeof(rxBuf));
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM17 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM17) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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

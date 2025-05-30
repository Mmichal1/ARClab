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
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// --> include all necessary headers for
// printf() redirection
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pid.h"
#include "stdio.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t mv, dv, cs;
} LogMsg;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint8_t uartByte;
static QueueHandle_t xQueueMv;
static QueueHandle_t xQueueDv;
static QueueHandle_t xQueueLog;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 50);
	return len;
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&uartByte, 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    static uint16_t adcVal;
    adcVal = HAL_ADC_GetValue(hadc);
    BaseType_t xHigher = pdFALSE;
    xQueueOverwriteFromISR(xQueueMv, &adcVal, &xHigher);
    portYIELD_FROM_ISR(xHigher);
}

void measureTask(void *args) {
    TickType_t xLast = xTaskGetTickCount();
    uint16_t   adcVal;

    for (;;) {
        HAL_ADC_Start(&hadc1);

        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            adcVal = HAL_ADC_GetValue(&hadc1);
            xQueueOverwrite(xQueueMv, &adcVal);
        }

        HAL_ADC_Stop(&hadc1);

        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(10));
    }
}

void userTask(void *args) {
    TickType_t xLast = xTaskGetTickCount();
    const uint16_t step = 500;
	uint8_t c = uartByte;

    for (;;)
    {
		c = atoi((char*)&uartByte);
		if (c >= 0 && c <= 8) {
			uint16_t dv = c * step;
			xQueueOverwrite(xQueueDv, &dv);
		}
		vTaskDelayUntil(&xLast, pdMS_TO_TICKS(100));
    }
}

void controlTask(void *args) {
    TickType_t xLast = xTaskGetTickCount();
    PID_t pid;
    PID_Init(&pid,
             1.0f,      // kp
             0.2f,    	// ki
             1.0f,		// kd
             0.0f,		// out_min
             4095.0f);	// out_max

    uint16_t mv = 0, dv = 0, cs = 0;

    for (;;) {
        xQueueReceive(xQueueMv, &mv, 0);
        xQueueReceive(xQueueDv, &dv, 0);

        cs = PID_Update(&pid, (float)dv, (float)mv);

		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, cs);

        LogMsg log = {.mv = mv, .dv = dv, .cs = cs };
        xQueueSend(xQueueLog, &log, 0);

        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(10));
    }
}

void commTask(void *args) {
    LogMsg log;
    TickType_t xLast = xTaskGetTickCount();

    for (;;) {
        if (xQueueReceive(xQueueLog, &log, 0) == pdTRUE) {
            printf("mv:%4u  dv:%4u  cs:%4u\r\n", log.mv, log.dv, log.cs);
        }
        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(500));
    }
}
/* USER CODE END 0 */


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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM6_Init();
	MX_DAC1_Init();
	/* USER CODE BEGIN 2 */

	// enable UART receive in interrupt mode
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&uartByte, 1);
	// --> create all necessary synchronization mechanisms
	xQueueMv   = xQueueCreate(1, sizeof(uint16_t));
	xQueueDv   = xQueueCreate(1, sizeof(uint16_t));
	xQueueLog  = xQueueCreate(5, sizeof(LogMsg));

	// --> create all necessary tasks
	xTaskCreate(measureTask, "measure", 128, NULL, 3, NULL);
	xTaskCreate(controlTask, "control", 256, NULL, 3, NULL);
	xTaskCreate(userTask,    "user",    256, NULL, 2, NULL);
	xTaskCreate(commTask,    "comm",    256, NULL, 1, NULL);
	printf("Starting!\r\n");

	// --> start FreeRTOS scheduler
	vTaskStartScheduler();

	// --> start FreeRTOS scheduler

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

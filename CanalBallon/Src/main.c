/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "wwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Venus_GPS.h"
//#include "DHT22.h"
#include <string.h>
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
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define DHT_SUCCESS 0
#define DHT_TIMEOUT_ERROR 1
#define DHT_CHECKSUM_ERROR 2

#pragma GCC push_options
#pragma GCC optimize ("O0")
void delayUS_ASM(uint32_t us) {
	volatile uint32_t counter = us;
	while (counter--)
		;
}
#pragma GCC pop_options
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buff[20];
uint8_t tx_buff[] = {'1','2','3','4','5','\r','\n'};
int FLAG_GPS_DATA_RECEIVE = 0;

unsigned char readDHT22(float* temperature, float* humidity) {
	static unsigned char data[5] = { 0 }; // Chaine de char retourner par le DHT22
	unsigned long max_cycles = 1000000;
	unsigned long timeout = 0;
	char buffer[50];
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// Start Signal
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	delayUS_ASM(800);

	// MODE INPUT FLOAT
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	delayUS_ASM(50);

	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
		if (++timeout == max_cycles)
			return DHT_TIMEOUT_ERROR;
	}

	timeout = 0;
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
		if (++timeout == max_cycles)
			return DHT_TIMEOUT_ERROR;
	}

	for (unsigned char i = 0; i < 40; ++i) {
		unsigned long cycles_low = 0;
		unsigned long cycles_high = 0;

		/* Attente d'un état LOW */
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
			if (++cycles_low == max_cycles)
				return DHT_TIMEOUT_ERROR;
		}

		/* Attente d'un état HIGH */
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
			if (++cycles_high == max_cycles)
				return DHT_TIMEOUT_ERROR;
		}

		/* Si le temps haut est supérieur au temps bas c'est un "1", sinon c'est un "0" */
		data[i / 8] <<= 1;
		if (cycles_high > cycles_low) {
			data[i / 8] |= 1;
		}
	}
	sprintf(buffer, "d0 = %d d1 = %d\r\n", data[0], data[1]);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 1000);
	sprintf(buffer, "d2 = %d d3 = %d\r\n", data[2], data[3]);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 1000);

	unsigned int checksum = (data[0] + data[1] + data[2] + data[3]) & 0xff;
	if (data[4] != checksum)
		return DHT_CHECKSUM_ERROR; /* Erreur de checksum */

	/* Calcul des valeurs */
	*humidity = data[0];
	*humidity *= 256;
	*humidity += data[1];
	*humidity *= 0.1;

	*temperature = data[2] & 0x7f;
	*temperature *= 256;
	*temperature += data[3];
	*temperature *= 0.1;
	if (data[2] & 0x80) {
		*temperature *= -1;
	}
	return DHT_SUCCESS;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	FLAG_GPS_DATA_RECEIVE = 1;
	HAL_UART_Transmit(&huart2,rx_buff,20,100);
  	HAL_UART_Receive_DMA(&huart1,rx_buff,20);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  //MX_WWDG_Init();
  //MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart1,rx_buff,20);
  Venus_GPS_configure_message(&huart1);

  float temp, hum;
  char buffer[50];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*if (FLAG_GPS_DATA_RECEIVE == 1) {
		  Venus_GPS_get_position(&huart1,rx_buff);
		  //HAL_UART_Transmit(&huart2,rx_buff,20,100);
		  FLAG_GPS_DATA_RECEIVE = 0;
	  }
	  else{
		  HAL_Delay(500);
	  }*/
	  switch (readDHT22(&temp, &hum)) {
         case DHT_SUCCESS:
            HAL_UART_Transmit(&huart2, "GOOD\r\n", strlen("GOOD\r\n"), 1000);
			sprintf(buffer, "temp = %d", (int) temp);
			HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 1000);
			break;
		 case DHT_TIMEOUT_ERROR:
			HAL_UART_Transmit(&huart2, "Erreur de timing\r\n", strlen("Erreur de timing\r\n"), 1000);
			break;
		 case DHT_CHECKSUM_ERROR:
			HAL_UART_Transmit(&huart2, "Erreur de checksum\r\n", strlen("Erreur de checksum\r\n"), 1000);
			break;
      }
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

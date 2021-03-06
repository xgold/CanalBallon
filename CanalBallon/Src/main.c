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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Venus_GPS.h"
#include "DHT22.h"
#include "com_LoRa.h"
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t GPS_DATA_RECEIVE = 100;

HAL_StatusTypeDef ok_test;
HAL_StatusTypeDef uart_transmit;
uint8_t rx_buff[100];
uint8_t gps_data_buffer[100];
uint8_t FLAG_GPS_DATA_RECEIVE = 0;
float temp_pin4, hum_pin4, temp_pin5, hum_pin5 = 0;
float north_metric, east_metric = 0;
int8_t inte_temp, exte_temp, north, east;
uint8_t inte_hum;


char* test_text;
char buffer1[15];
#if DEBUG
   char buffer1[15];
   char buffer2[15];
#endif

char TrameATTx[45];

status_etat mon_etat = INIT_SENSORS;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if DEBUG
	HAL_UART_Transmit(&huart2,&rx_buff[0],GPS_DATA_RECEIVE,10);
#endif

	if (FLAG_GPS_DATA_RECEIVE == 0) {
		strncpy((char *)gps_data_buffer, (char *)rx_buff, GPS_DATA_RECEIVE);
		FLAG_GPS_DATA_RECEIVE =+ 1;
	}

	HAL_UART_Receive_DMA(&huart1,&rx_buff[0],GPS_DATA_RECEIVE);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval void
  */
void main(void)
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
  MX_USART4_UART_Init();
  //MX_WWDG_Init();
  //MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(mon_etat){
	     case INIT_SENSORS:
	    	Venus_GPS_configure_message(&huart1);
	    	HAL_UART_Receive_DMA(&huart1,rx_buff,GPS_DATA_RECEIVE);
	    	ConfigLoRaClick();

	    	mon_etat = GET_SENSOR_DATA;
	    	break;

	  	 case GET_SENSOR_DATA:
	  		// Get DTH22 metrics
	  		if (readDHT22(GPIO_PIN_4, &temp_pin4, &hum_pin4) != DHT_SUCCESS) {
	  		   temp_pin4 = 0;
	  		   hum_pin4 = 0;
	  		}
	  		if (readDHT22(GPIO_PIN_5, &temp_pin5, &hum_pin5) != DHT_SUCCESS) {
	  		   temp_pin5 = 0;
	  		   hum_pin5 = 0;
     		}

	  		mon_etat = FORMAT_DATA;
	  	    break;

	  	 case FORMAT_DATA:
			// Format DTH22 data
#if DEBUG
			sprintf(buffer1, "temp 1 = %d\r\n", (int)temp_pin4);
			sprintf(buffer2, "temp 2 = %d\r\n", (int)temp_pin5);
			HAL_UART_Transmit(&huart2, buffer1, strlen(buffer1),10);
			HAL_UART_Transmit(&huart2, buffer2, strlen(buffer2),10);
#endif

			// Format GPS data
			if (FLAG_GPS_DATA_RECEIVE) {
				Venus_GPS_get_position(&gps_data_buffer[0], GPS_DATA_RECEIVE, &north_metric, &east_metric);
				FLAG_GPS_DATA_RECEIVE = 0;

			}

#if DEBUG
			sprintf(buffer1, "north metric = %d\r\n", (int)north_metric);
			sprintf(buffer2, "east metric = %d\r\n", (int)east_metric);
			HAL_UART_Transmit(&huart2, buffer1, strlen(buffer1),10);
			HAL_UART_Transmit(&huart2, buffer2, strlen(buffer2),10);
#endif

			mon_etat = SEND_DATA;
	  		break;

	  	 case SEND_DATA:
	  		inte_temp = (int)(temp_pin4+0.5);
	  		inte_hum = (int)(hum_pin4+0.5);
	  		exte_temp = (int)(temp_pin5+0.5);
	  		north = (int)(north_metric*100);
	  		east = (int)(east_metric*100);

	  	    uart_transmit = EnvoisLoRa(inte_temp, inte_hum, exte_temp, north, east, 0, TrameATTx);

	  	    if (uart_transmit == HAL_OK){
				// On attend le Ok et radio_tx_ok avant de re envoyer
				ok_test = AttenteLoRa("radio_tx_ok", 11);


				if (ok_test == HAL_OK){
					test_text = "HAL_OK";
				}
				else {
					test_text = "HAL_ERROR";
				}
				sprintf(buffer1, "%s\r\n", test_text);
				HAL_UART_Transmit(&huart2, buffer1, strlen(buffer1),100);
#if DEBUG
				HAL_UART_Transmit(&huart2, ok_test, strlen(ok_test),1000);
#endif
				if (ok_test != HAL_OK){
					ConfigLoRaClick();
				}
	  	    }
			else{
				ConfigLoRaClick();
			}
	  	    HAL_Delay(10000);
	  	    // test if uart_transmit is HAL_OK else radio is deconnected

	  	    mon_etat = GET_SENSOR_DATA;
	  		break;

	  	 default:
	  		break;

	  }
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

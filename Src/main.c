
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2; // destinataire PC
UART_HandleTypeDef huart1; // destinataire LoRa

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void ConfigLoRaClick(void);
void ConversionEnASCII(char TabChar[33], int T, int H, int P, int X, int Y, int Z);
char HexaChar(int NbrbaseDix);
void AjouterUneValeurDansTrame(char Trame[45],  char TabValeurASCII[33]);
void EnvoisLoRa(int T, int H, int P, int X, int Y, int Z, char Trame[45]);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{

	//char trame3[] = "radio tx 0A\r\n";
	char TrameATTx[45];
	int T, H, P, X, Y, Z;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ConfigLoRaClick();
  T = 0;
  H = 0;
  P = 0;
  X = 0;
  Y = 0;
  Z = 0;

  while (1)
  {
	  EnvoisLoRa(T, H, P, X, Y, Z, TrameATTx);
	  T = T + 1;
	  H = H + 1;
	  P = P + 10;
	  X = X + 10;
	  Y = Y + 10;
	  Z = Z + 10;
	  HAL_Delay(10000); //J'envois toutes les 1 sec


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

void ConfigLoRaClick(void)
{
	char trame[] = "sys factoryRESET\r\n";
	char trame1[] = "mac pause\r\n";
	char trame2[] = "radio set wdt 0\r\n";
	char trame3[] = "Configuration du Module LoRa : OK\r\n";

	//Remise a l'etat d'usine du module
	HAL_UART_Transmit(&huart1, (uint8_t*) trame, (uint16_t) sizeof(trame), HAL_MAX_DELAY);
	HAL_Delay(4000);

	//Verifiaction de la capacité de trasmision
	HAL_UART_Transmit(&huart1, (uint8_t*) trame1, (uint16_t) sizeof(trame1), HAL_MAX_DELAY);
	HAL_Delay(4000);

	//Wacth Dog desactive
	HAL_UART_Transmit(&huart1, (uint8_t*) trame2, (uint16_t) sizeof(trame2), HAL_MAX_DELAY);
	HAL_Delay(4000);

	HAL_UART_Transmit(&huart2, (uint8_t*) trame3, (uint16_t) sizeof(trame3), HAL_MAX_DELAY);
}

void ConversionEnASCII(char TabChar[33], int T, int H, int P, int X, int Y, int Z)
{
    int digit1, digit2, digit3, digit4, digit5, digit6, signe = 0;

    //Tranformation de la temperature en caractere
    if(T > 0)
    {
        signe = 10;
        digit1 = T / 100;
        digit2 = (T - (digit1*100)) / 10;
        digit3 = T - ((digit1 * 100) + (digit2 * 10));

        TabChar[0] = HexaChar(signe);
        TabChar[1] = HexaChar(digit1);
        TabChar[2] = HexaChar(digit2);
        TabChar[3] = HexaChar(digit3);
    }
    else
    {
        signe = 15;
        T = T * (-1);
        digit1 = T / 100;
        digit2 = (T - (digit1*100)) / 10;
        digit3 = T - ((digit1 * 100) + (digit2 * 10));

        //Insertion dans le tableau
        TabChar[0] = HexaChar(signe);
        TabChar[1] = HexaChar(digit1);
        TabChar[2] = HexaChar(digit2);
        TabChar[3] = HexaChar(digit3);
    }

    //Tranformation de l'humidite en caractere
    if(H > 0)
    {
        signe = 10;
        digit1 = H / 100;
        digit2 = (H - (digit1*100)) / 10;
        digit3 = H - ((digit1 * 100) + (digit2 * 10));

        TabChar[4] = HexaChar(signe);
        TabChar[5] = HexaChar(digit1);
        TabChar[6] = HexaChar(digit2);
        TabChar[7] = HexaChar(digit3);
    }
    else
    {
        signe = 15;
        H = H * (-1);
        digit1 = H / 100;
        digit2 = (H - (digit1*100)) / 10;
        digit3 = H - ((digit1 * 100) + (digit2 * 10));

        //Insertion dans le tableau
        TabChar[4] = HexaChar(signe);
        TabChar[5] = HexaChar(digit1);
        TabChar[6] = HexaChar(digit2);
        TabChar[7] = HexaChar(digit3);
    }

    //Tranformation de la pression en caractere
    if(P > 0)
    {
        signe = 10;
        digit1 = P / 100000;
        digit2 = (P - (digit1 * 100000)) / 10000;
        digit3 = (P - ((digit1 * 100000) + (digit2 * 10000)))/1000;
        digit4 = (P - ((digit1 * 100000) + (digit2 * 10000) + (digit3 * 1000))) / 100;
        digit5 = (P - ((digit1 * 100000) + (digit2 * 10000) + (digit3 * 1000) + (digit4 *100))) / 10;
        digit6 = P - ((digit1 * 100000) + (digit2 * 10000) + (digit3 * 1000) + (digit4 *100) + (digit5 * 10));

        TabChar[8] = HexaChar(signe);
        TabChar[9] = HexaChar(digit1);
        TabChar[10] = HexaChar(digit2);
        TabChar[11] = HexaChar(digit3);
        TabChar[12] = HexaChar(digit4);
        TabChar[13] = HexaChar(digit5);
        TabChar[14] = HexaChar(digit6);
    }
    else
    {
        signe = 15;
        H = H * (-1);
        digit1 = P / 100000;
        digit2 = (P - (digit1 * 100000)) / 10000;
        digit3 = (P - ((digit1 * 100000) + (digit2 * 10000)))/1000;
        digit4 = (P - ((digit1 * 100000) + (digit2 * 10000) + (digit3 * 1000))) / 100;
        digit5 = (P - ((digit1 * 100000) + (digit2 * 10000) + (digit3 * 1000) + (digit4 *100))) / 10;
        digit6 = P - ((digit1 * 100000) + (digit2 * 10000) + (digit3 * 1000) + (digit4 *100) + (digit5 * 10));

        TabChar[8] = HexaChar(signe);
        TabChar[9] = HexaChar(digit1);
        TabChar[10] = HexaChar(digit2);
        TabChar[11] = HexaChar(digit3);
        TabChar[12] = HexaChar(digit4);
        TabChar[13] = HexaChar(digit5);
        TabChar[14] = HexaChar(digit6);
    }

    //Tranformation de la positionX en caractere
    if(X > 0)
    {
        signe = 10;
        digit1 = X / 10000;
        digit2 = (X - (digit1 * 10000)) / 1000;
        digit3 = (X - ((digit1 * 10000) + (digit2 * 1000)))/100;
        digit4 = (X - ((digit1 * 10000) + (digit2 * 1000) + (digit3 * 100))) / 10;
        digit5 = (X - ((digit1 * 10000) + (digit2 * 1000) + (digit3 * 100) + (digit4 *10)));

        TabChar[15] = HexaChar(signe);
        TabChar[16] = HexaChar(digit1);
        TabChar[17] = HexaChar(digit2);
        TabChar[18] = HexaChar(digit3);
        TabChar[19] = HexaChar(digit4);
        TabChar[20] = HexaChar(digit5);
    }
    else
    {
        signe = 15;
        X = X * (-1);
        digit1 = X / 10000;
        digit2 = (X - (digit1 * 10000)) / 1000;
        digit3 = (X - ((digit1 * 10000) + (digit2 * 1000)))/100;
        digit4 = (X - ((digit1 * 10000) + (digit2 * 10000) + (digit3 * 100))) / 10;
        digit5 = (X - ((digit1 * 10000) + (digit2 * 10000) + (digit3 * 100) + (digit4 *100)));

        TabChar[15] = HexaChar(signe);
        TabChar[16] = HexaChar(digit1);
        TabChar[17] = HexaChar(digit2);
        TabChar[18] = HexaChar(digit3);
        TabChar[19] = HexaChar(digit4);
        TabChar[20] = HexaChar(digit5);
    }

    //Tranformation de la positionY en caractere
    if(Y > 0)
    {
        signe = 10;
        digit1 = Y / 10000;
        digit2 = (Y - (digit1 * 10000)) / 1000;
        digit3 = (Y - ((digit1 * 10000) + (digit2 * 1000)))/100;
        digit4 = (Y - ((digit1 * 10000) + (digit2 * 1000) + (digit3 * 100))) / 10;
        digit5 = (Y - ((digit1 * 10000) + (digit2 * 1000) + (digit3 * 100) + (digit4 *10)));

        TabChar[21] = HexaChar(signe);
        TabChar[22] = HexaChar(digit1);
        TabChar[23] = HexaChar(digit2);
        TabChar[24] = HexaChar(digit3);
        TabChar[25] = HexaChar(digit4);
        TabChar[26] = HexaChar(digit5);
    }
    else
    {
        signe = 15;
        Y = Y * (-1);
        digit1 = Y / 10000;
        digit2 = (Y - (digit1 * 10000)) / 1000;
        digit3 = (Y - ((digit1 * 10000) + (digit2 * 1000)))/100;
        digit4 = (Y - ((digit1 * 10000) + (digit2 * 10000) + (digit3 * 100))) / 10;
        digit5 = (Y - ((digit1 * 10000) + (digit2 * 10000) + (digit3 * 100) + (digit4 *100)));

        TabChar[21] = HexaChar(signe);
        TabChar[22] = HexaChar(digit1);
        TabChar[23] = HexaChar(digit2);
        TabChar[24] = HexaChar(digit3);
        TabChar[25] = HexaChar(digit4);
        TabChar[26] = HexaChar(digit5);
    }

    //Tranformation de la positionZ en caractere
    if(Z > 0)
    {
        signe = 10;
        digit1 = Z / 10000;
        digit2 = (Z - (digit1 * 10000)) / 1000;
        digit3 = (Z - ((digit1 * 10000) + (digit2 * 1000)))/100;
        digit4 = (Z - ((digit1 * 10000) + (digit2 * 1000) + (digit3 * 100))) / 10;
        digit5 = (Z - ((digit1 * 10000) + (digit2 * 1000) + (digit3 * 100) + (digit4 *10)));

        TabChar[27] = HexaChar(signe);
        TabChar[28] = HexaChar(digit1);
        TabChar[29] = HexaChar(digit2);
        TabChar[30] = HexaChar(digit3);
        TabChar[31] = HexaChar(digit4);
        TabChar[32] = HexaChar(digit5);
    }
    else
    {
        signe = 15;
        Z = Z * (-1);
        digit1 = Z / 10000;
        digit2 = (Z - (digit1 * 10000)) / 1000;
        digit3 = (Z - ((digit1 * 10000) + (digit2 * 1000)))/100;
        digit4 = (Z - ((digit1 * 10000) + (digit2 * 10000) + (digit3 * 100))) / 10;
        digit5 = (Z - ((digit1 * 10000) + (digit2 * 10000) + (digit3 * 100) + (digit4 *100)));

        TabChar[27] = HexaChar(signe);
        TabChar[28] = HexaChar(digit1);
        TabChar[29] = HexaChar(digit2);
        TabChar[30] = HexaChar(digit3);
        TabChar[31] = HexaChar(digit4);
        TabChar[32] = HexaChar(digit5);
    }
}

char HexaChar(int NbrbaseDix)
{
    char lettre;

    switch(NbrbaseDix)
    {
        case 0 :
            lettre = '0';
            break;
        case 1 :
            lettre = '1';
            break;
        case 2 :
            lettre = '2';
            break;
        case 3 :
            lettre = '3';
            break;
        case 4 :
            lettre = '4';
            break;
        case 5 :
            lettre = '5';
            break;
        case 6 :
            lettre = '6';
            break;
        case 7 :
            lettre = '7';
            break;
        case 8 :
            lettre = '8';
            break;
        case 9 :
            lettre = '9';
            break;
        case 10 :
            lettre = 'A';
            break;
        case 11 :
            lettre = 'B';
            break;
        case 12 :
            lettre = 'C';
            break;
        case 13 :
            lettre = 'D';
            break;
        case 14 :
            lettre = 'E';
            break;
        case 15 :
            lettre = 'F';
            break;
        default :
            lettre = '!';
            break;
    }

    return lettre;
}

void AjouterUneValeurDansTrame(char Trame[45],  char TabValeurASCII[33])
{
    char EnteteTrame[] = "radio tx ";
    int i = 0, j = 0;

    while(EnteteTrame[i] != '\0')
    {
        Trame[i] = EnteteTrame[i];
        //printf("%c", Trame[i]);
        i++;
    }

    for(j = 0; j < 33; j++)
    {
        Trame[j+i] = TabValeurASCII[j];
        //printf(" %d", j);
    }

    Trame[42] = 'B';
    Trame[43] = '\r';
    Trame[44] = '\n';

    printf("\n");
}

void EnvoisLoRa(int T, int H, int P, int X, int Y, int Z, char Trame[45])
{
	char TabMesure[33];

	ConversionEnASCII(TabMesure, T, H, P, X, Y, Z);
	AjouterUneValeurDansTrame(Trame, TabMesure);

	HAL_UART_Transmit(&huart1, (uint8_t*) Trame, 45*(uint16_t) sizeof(char), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) Trame, 45*(uint16_t) sizeof(char), HAL_MAX_DELAY);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

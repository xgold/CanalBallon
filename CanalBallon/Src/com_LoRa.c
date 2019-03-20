/*
 * com_LoRa.c
 *
 *  Created on: 14 févr. 2019
 *      Author: kevin
 */

#include "com_LoRa.h"

uint8_t buff[20] = {0};

void ConfigLoRaClick(void)
{
	char trame[] = "sys factoryRESET\r\n";
	char trame1[] = "mac pause\r\n";
	char trame2[] = "radio set wdt 0\r\n";
	char trame3[] = "Configuration du Module LoRa : OK\r\n";
	char trame4[] = "radio set freq 433100000\r\n";
	char trame5[] = "Frequence : 433,1 MHz\r\n";
	char trame6[] = "radio set pwr 10\r\n";
	char trame7[] = "Puissance emission : 10 dB\r\n";

	//Remise a l'etat d'usine du module
	HAL_UART_Transmit(&huart4, (uint8_t*) trame, (uint16_t) sizeof(trame), HAL_MAX_DELAY);
	HAL_Delay(4000);

	//Verifiaction de la capacite de trasmision
	HAL_UART_Transmit(&huart4, (uint8_t*) trame1, (uint16_t) sizeof(trame1), HAL_MAX_DELAY);
	HAL_Delay(4000);

	//Wacth Dog desactive
	HAL_UART_Transmit(&huart4, (uint8_t*) trame2, (uint16_t) sizeof(trame2), HAL_MAX_DELAY);
	HAL_Delay(4000);

	//Config en 433,1 MHz
	HAL_UART_Transmit(&huart4, (uint8_t*) trame4, (uint16_t) sizeof(trame4), HAL_MAX_DELAY);
	HAL_Delay(4000);

	//Reponse de la config 433,1 MHz
	#if DEBUG
	HAL_UART_Transmit(&huart2, (uint8_t*) trame5, (uint16_t) sizeof(trame5), HAL_MAX_DELAY);
	HAL_Delay(4000);

	//Config en PWR
	HAL_UART_Transmit(&huart4, (uint8_t*) trame6, (uint16_t) sizeof(trame6), HAL_MAX_DELAY);
	HAL_Delay(4000);

	//Reponse de la config PWR
	HAL_UART_Transmit(&huart2, (uint8_t*) trame7, (uint16_t) sizeof(trame7), HAL_MAX_DELAY);
	HAL_Delay(4000);

	HAL_UART_Transmit(&huart2, (uint8_t*) trame3, (uint16_t) sizeof(trame3), HAL_MAX_DELAY);
	#endif
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
            lettre = '0';
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

    //printf("\n");
}

HAL_StatusTypeDef EnvoisLoRa(int T, int H, int P, int X, int Y, int Z, char Trame[45])
{
	char TabMesure[33];

	ConversionEnASCII(TabMesure, T, H, P, X, Y, Z);
	AjouterUneValeurDansTrame(Trame, TabMesure);

	HAL_StatusTypeDef return_uart;
	return_uart = HAL_UART_Transmit(&huart4, (uint8_t*) Trame, 45*(uint16_t) sizeof(char), HAL_MAX_DELAY);

    #if DEBUG
		HAL_UART_Transmit(&huart2, (uint8_t*) Trame, 45*(uint16_t) sizeof(char), HAL_MAX_DELAY);
	#endif

	return return_uart;
}

HAL_StatusTypeDef AttenteLoRa(char* to_test, uint8_t size)
{
	for (int i =0; i<sizeof(buff);i++){
		buff[i] = 0;
	}
	volatile HAL_StatusTypeDef t = HAL_UART_Receive(&huart4, (uint8_t*) buff, (uint16_t*) sizeof(buff), 10000);
	uint8_t inc = 0;
	uint8_t compt = 0;
	uint8_t i_copie = 0;

	if (t != HAL_ERROR){
		for (int i = 0; i < size; i++){
			i_copie = i;
			while (buff[i_copie] == to_test[compt]){
				inc += 1;
				i_copie += 1;
				compt += 1;
				if (inc == size){
					return HAL_OK;
				}
			}
		}
		return HAL_ERROR;
	}

	return t;
}

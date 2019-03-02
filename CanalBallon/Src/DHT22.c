/*
 * DTH22.c
 *
 *  Created on: 1 févr. 2019
 *      Author: kevin
 */

#include "DHT22.h"

#pragma GCC push_options
#pragma GCC optimize ("O0")
void delayUS_ASM(uint32_t us) {
	volatile uint32_t counter = us;
	while (counter--)
		;
}
#pragma GCC pop_options


unsigned char readDHT22(uint32_t pin, float* temperature, float* humidity)
{
	static unsigned char data[5] = { 0 }; // Chaine de char retourner par le DHT22
	unsigned long max_cycles = 1000000;
	unsigned long timeout = 0;
	char buffer[50];
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// Start Signal
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET);
	delayUS_ASM(800);

	// MODE INPUT FLOAT
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	delayUS_ASM(50);

	while (HAL_GPIO_ReadPin(GPIOA, pin) == GPIO_PIN_RESET) {
		if (++timeout == max_cycles)
			return DHT_TIMEOUT_ERROR;
	}

	timeout = 0;
	while (HAL_GPIO_ReadPin(GPIOA, pin) == GPIO_PIN_SET) {
		if (++timeout == max_cycles)
			return DHT_TIMEOUT_ERROR;
	}

	for (unsigned char i = 0; i < 40; ++i) {
		unsigned long cycles_low = 0;
		unsigned long cycles_high = 0;

		/* Attente d'un état LOW */
		while (HAL_GPIO_ReadPin(GPIOA, pin) == GPIO_PIN_RESET) {
			if (++cycles_low == max_cycles)
				return DHT_TIMEOUT_ERROR;
		}

		/* Attente d'un état HIGH */
		while (HAL_GPIO_ReadPin(GPIOA, pin) == GPIO_PIN_SET) {
			if (++cycles_high == max_cycles)
				return DHT_TIMEOUT_ERROR;
		}

		/* Si le temps haut est supérieur au temps bas c'est un "1", sinon c'est un "0" */
		data[i / 8] <<= 1;
		if (cycles_high > cycles_low) {
			data[i / 8] |= 1;
		}
	}
	//sprintf(buffer, "d0 = %d d1 = %d\r\n", data[0], data[1]);
	//HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 1000);
	//sprintf(buffer, "d2 = %d d3 = %d\r\n", data[2], data[3]);
	//HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 1000);

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

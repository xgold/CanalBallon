/*
 * com_LoRa.h
 *
 *  Created on: 14 févr. 2019
 *      Author: kevin
 */

#ifndef COM_LORA_H_
#define COM_LORA_H_

#include "usart.h"

void ConfigLoRaClick(void);
void ConversionEnASCII(char TabChar[33], int T, int H, int P, int X, int Y, int Z);
char HexaChar(int NbrbaseDix);
void AjouterUneValeurDansTrame(char Trame[45],  char TabValeurASCII[33]);
HAL_StatusTypeDef EnvoisLoRa(int T, int H, int P, int X, int Y, int Z, char Trame[45]);

#endif /* COM_LORA_H_ */

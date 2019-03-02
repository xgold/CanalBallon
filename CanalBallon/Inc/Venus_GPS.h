/*
 * Venus_GPS.h
 *
 *  Created on: 2 janv. 2019
 *      Author: kevin
 */

#ifndef INC_VENUS_GPS_H_
#define INC_VENUS_GPS_H_

#include "stm32l0xx_hal.h"


void Venus_GPS_restart(UART_HandleTypeDef*);
void Venus_GPS_configure_message(UART_HandleTypeDef*);
void Venus_GPS_get_position(uint8_t*, uint8_t, float*, float*);


#endif /* INC_VENUS_GPS_H_ */

/*
 * Venus_GPS.c
 *
 *  Created on: 2 janv. 2019
 *      Author: kevin
 */

#include "Venus_GPS.h"

void construct_sending_message(uint16_t length_payload, uint8_t* tab);

uint8_t buffer[50] = {0};

struct nmea_message {
	uint8_t message_id;
	uint8_t GGA;
	uint8_t GSA;
	uint8_t GSV;
	uint8_t GLL;
	uint8_t RMC;
	uint8_t VTG;
	uint8_t ZDA;
	uint8_t attributes;
};

void Venus_GPS_restart(UART_HandleTypeDef *huart)
{
	//construct_sending_message(0x0002, );
	HAL_UART_Transmit(huart, buffer, sizeof(buffer), 50);
}

void Venus_GPS_configure_message(UART_HandleTypeDef *huart)
{
	struct nmea_message messages_interval;
	messages_interval.message_id = 0x08;
	messages_interval.GGA = 0x3C;
	messages_interval.GLL = 0x0;
	messages_interval.GSA = 0x0;
	messages_interval.GSV = 0x0;
	messages_interval.RMC = 0x0;
	messages_interval.VTG = 0x0;
	messages_interval.ZDA = 0x0;
	messages_interval.attributes = 0x0;

	uint8_t tab_interval[9] = {0};
	tab_interval[0] = messages_interval.message_id;
	tab_interval[1] = messages_interval.GGA;
	tab_interval[2] = messages_interval.GLL;
	tab_interval[3] = messages_interval.GSA;
	tab_interval[4] = messages_interval.GSV;
	tab_interval[5] = messages_interval.RMC;
	tab_interval[6] = messages_interval.VTG;
	tab_interval[7] = messages_interval.ZDA;
	tab_interval[8] = messages_interval.attributes;
	construct_sending_message(0x009, tab_interval);

	HAL_UART_Transmit(huart, buffer, 16, 50);

}

/*
 * Parse the buffer received to send to LoRa then
 */
void Venus_GPS_get_position(UART_HandleTypeDef *huart, uint8_t* buffer_to_parse)
{
	int trame = 0;
	// rechercher $GPGGA -> CR LF
    for (uint8_t i=0; i < strlen(buffer_to_parse)-1; i++){
    	if (buffer_to_parse[i]=='$') {
    		trame = 1;
    		buffer[0] = trame;
    		HAL_UART_Transmit(huart, buffer, 16, 50);
    	}
    }
	// separateur ,
	// exemple trame : $GPGGA,111636.932,2447.0949,N,12100.5223,E,1,11,0.8,118.2,M,,,,0000*02<CR><LF>

}



void construct_sending_message(uint16_t length_payload, uint8_t* tab)
{

	// Header
	buffer[0] = 0xA0;
	buffer[1] = 0xA1;

	// Length payload
	buffer[2] = (length_payload>>8);
	buffer[3] = length_payload;

	// Payload
	for (int i = 0; i < length_payload; i++){
		buffer[3+(i+1)] = tab[i];
	}

	uint8_t new_case = 3+(length_payload+1);

	// Checksumm
	uint8_t CS = 0;
	uint16_t N=(buffer[2]<<8) | buffer[3];
    for (int i=0; i < N; i++){
       CS = CS^buffer[i+4];
    }
    buffer[new_case] = CS;

    // Footer
    new_case = new_case+1;
	buffer[new_case] = 0x0D;
    new_case = new_case+1;
	buffer[new_case] = 0x0A;
}

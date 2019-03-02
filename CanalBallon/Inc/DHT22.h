/*
 * DTH22.h
 *
 *  Created on: 1 févr. 2019
 *      Author: kevin
 */

#ifndef DHT22_H_
#define DHT22_H_

#include <string.h>
#include "gpio.h"


#define DHT_SUCCESS 0
#define DHT_TIMEOUT_ERROR 1
#define DHT_CHECKSUM_ERROR 2

unsigned char readDHT22(uint32_t, float*, float*);


#endif /* DHT22_H_ */

/*
 * HC12.h
 *
 *  Created on: Mar 19, 2022
 *      Author: Allen
 */

#ifndef DRIVERS_HC12_H_
#define DRIVERS_HC12_H_

#include <stdint.h>
#include <stdbool.h>

void
HC12_Initialize();

void
HC12_SendData(uint8_t header, uint8_t *data);

#endif /* DRIVERS_HC12_H_ */

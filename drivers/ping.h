/*
 * ping.h
 *
 *  Created on: Apr 6, 2022
 *      Author: Allen
 */

#ifndef DRIVERS_PING_H_
#define DRIVERS_PING_H_

#define PING0
#define PING1
#define PING2
#define PING3

void
Ping_Initialize();

int
Ping_GetDistance(uint8_t sensor);


#endif /* DRIVERS_PING_H_ */

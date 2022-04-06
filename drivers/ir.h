/*
 * ir.h
 *
 *  Created on: Apr 6, 2022
 *      Author: Allen
 */

#ifndef DRIVERS_IR_H_
#define DRIVERS_IR_H_

#define IR0
#define IR1
#define IR2
#define IR3

void
IR_Initialize(void);

float
IR_getDistance(uint8_t device);

#endif /* DRIVERS_IR_H_ */

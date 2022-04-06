/*
 * opt3101.h
 *
 *  Created on: Apr 6, 2022
 *      Author: Allen
 */

#ifndef DRIVERS_OPT3101_H_
#define DRIVERS_OPT3101_H_

#define OPT3101_L
#define OPT3101_R

// Address is 1011 A2 A1 A0
#define OPT3101_I2C_ADDRESS 0b1011111

void
OPT3101_Initialize();

void
OPT3101_Update();

float
OPT3101_GetDistance();

#endif /* DRIVERS_OPT3101_H_ */

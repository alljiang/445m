/*
 * hcsr04.h
 *
 *  Created on: Apr 12, 2022
 *      Author: Allen
 */

#ifndef DRIVERS_HCSR04_H_
#define DRIVERS_HCSR04_H_

void
HCSR04_Initialize(void);

// return distance in 0.01cm
// returns negative for invalid
int
HCSR04_GetDistance(void);

void
HCSR04_StartMeasurement(void);

#endif /* DRIVERS_HCSR04_H_ */

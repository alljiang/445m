/*
 * interrupt.h
 *
 *  Created on: Feb 8, 2022
 *      Author: allji
 */

#ifndef DRIVERS_INTERRUPT_H_
#define DRIVERS_INTERRUPT_H_

void
Interrupt_Enable(uint8_t interruptNum);

void
Interrupt_SetPriority(uint8_t interruptNum, uint8_t priority);

void
Interrupt_SetSystemPriority(uint8_t interruptNum, uint8_t priority);

#endif /* DRIVERS_INTERRUPT_H_ */

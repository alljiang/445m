/*
 * InterruptFunctions.h
 *
 *  Created on: Feb 3, 2022
 *      Author: Allen
 */

#ifndef RTOS_INTERRUPTFUNCTIONS_H_
#define RTOS_INTERRUPTFUNCTIONS_H_

extern void DisableInterrupts(void);

extern void EnableInterrupts(void);

extern long StartCritical (void);

extern void EndCritical(long sr);

extern void WaitForInterrupt(void);

#endif /* RTOS_INTERRUPTFUNCTIONS_H_ */

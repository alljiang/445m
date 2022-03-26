
#ifndef TIMERS_H_
#define TIMERS_H_

#include <stdint.h>

extern void
SysTick_Init(unsigned long period);

extern void
SysTick_Handler(void);

extern void
Timer0Init(void);

extern void
Timer0IntHandler(void);

extern void
Timer1Init(uint32_t period, uint32_t priority);

extern void
Timer1IntHandler(void);

extern void
Timer2Init(uint32_t period, uint32_t priority);

extern void
Timer2IntHandler(void);

extern void
Timer3Init(uint32_t period, uint32_t priority);

extern void
Timer3IntHandler(void);

extern void
Timer4Init(uint32_t period, uint32_t priority);

extern void
Timer4IntHandler(void);

#endif

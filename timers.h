#include <stdint.h>

extern void
SysTick_Init(unsigned long period);

extern void
SysTick_Handler();

extern void
Timer0Init(void);

extern void
Timer0IntHandler(void);

extern void
Timer1Init(uint32_t period, uint32_t priority);

extern void
Timer1IntHandler(void);

#include <stdint.h>

void
SysTick_Init(unsigned long period);

void
SysTick_Handler();

void
Timer0Init(void);

void
Timer0IntHandler(void);

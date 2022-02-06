#include <bit-utils.h>
#include <timers.h>
#include "vware/tm4c123gh6pm.h"

#include <RTOS/OS.h>

extern uint64_t osTimeMs;

extern void
ContextSwitch(void);

void (*PeriodicTask)(void);

/*
 * Datasheet Bookmarks:
 * Interrupt table          104
 * NVIC Priority            152
 */

void
SysTick_Init(unsigned long period) {
    NVIC_ST_CTRL_R = 0;                                                     // disable SysTick
    NVIC_ST_CURRENT_R = 0;                                                  // clear current counter
    NVIC_SYS_PRI3_R = set_bit_field_u32(NVIC_SYS_PRI3_R, 21, 3, 7);         // Priority 1, System Interrupt 15
    NVIC_ST_RELOAD_R = period - 1;                                          // reload value
    NVIC_ST_CTRL_R = 0x00000007;                                            // enable, core clock and interrupt arm
}

void
SysTick_Handler() {
    OS_Scheduler();
    ContextSwitch();
}

void
Timer0Init(void) {
    uint32_t frequencyHz = 1000u;

    // Page 722 of datasheet
    SYSCTL_RCGCTIMER_R |= 0x1u;
    TIMER0_CTL_R = set_bit_field_u32(TIMER0_CTL_R, 0, 1, 0);            //  1) Disable timer
    TIMER0_CFG_R = 0;                                                   //  2) Write GPTMCFG with a value of 0
    TIMER0_TAMR_R = set_bit_field_u32(TIMER0_TAMR_R, 0, 2, 0b10);       //  3b) Set timer A to Periodic Mode
    TIMER0_TAMR_R = set_bit_field_u32(TIMER0_TAMR_R, 4, 1, 0b0);        //  4) Set timer A to count down
    TIMER0_TAILR_R = 80000000u / frequencyHz;                           //  5) Set reload value
    TIMER0_IMR_R = set_bit_field_u32(TIMER0_IMR_R, 0, 1, 1);            //  6) Enable timer A time-out interrupt
    NVIC_PRI4_R = set_bit_field_u32(NVIC_PRI4_R, 29, 3, 2u);            // Priority 2, Interrupt 19
    NVIC_EN0_R = set_bit_field_u32(NVIC_EN0_R, 19, 1, 1);
    TIMER0_CTL_R = set_bit_field_u32(TIMER0_CTL_R, 0, 1, 1);            //  7) Enable timer and start counting
}

//  1000 Hz Handler
void
Timer0IntHandler(void) {
    // Page 722 of datasheet
    TIMER0_ICR_R = set_bit_field_u32(TIMER0_ICR_R, 0, 1, 0b1); //  8) Clear interrupt

    osTimeMs++;

    OS_UpdateSleep();
}

void
Timer1Init(uint32_t period, uint32_t priority) {
    // Page 722 of datasheet
    SYSCTL_RCGCTIMER_R |= 0x2;
    TIMER1_CTL_R = set_bit_field_u32(TIMER1_CTL_R, 0, 1, 0);            //  1) Disable timer
    TIMER1_CFG_R = 0;                                                   //  2) Write GPTMCFG with a value of 0
    TIMER1_TAMR_R = set_bit_field_u32(TIMER1_TAMR_R, 0, 2, 0b10);       //  3b) Set timer A to Periodic Mode
    TIMER1_TAMR_R = set_bit_field_u32(TIMER1_TAMR_R, 4, 1, 0b0);        //  4) Set timer A to count down
    TIMER1_TAILR_R = period;                                            //  5) Set reload value
    TIMER1_IMR_R = set_bit_field_u32(TIMER1_IMR_R, 0, 1, 1);            //  6) Enable timer A time-out interrupt
    NVIC_PRI5_R = set_bit_field_u32(NVIC_PRI5_R, 13, 3, priority);      // Interrupt 21
    NVIC_EN0_R = set_bit_field_u32(NVIC_EN0_R, 21, 1, 1);
    TIMER1_CTL_R = set_bit_field_u32(TIMER1_CTL_R, 0, 1, 1);            //  7) Enable timer and start counting
}

void
Timer1IntHandler(void) {
    TIMER1_ICR_R = set_bit_field_u32(TIMER1_ICR_R, 0, 1, 0b1); // Clear interrupt

    (*PeriodicTask)();
}

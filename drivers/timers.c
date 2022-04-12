#include "bit-utils.h"
#include "interrupt.h"
#include "timers.h"
#include "vware/tm4c123gh6pm.h"
#include "RTOS/OS.h"
#include "Launchpad.h"

extern uint64_t osTimeMs;

extern void
ContextSwitch(void);

extern void
OS_CallBackgroundTask(uint8_t taskID);

/*
 * Datasheet Bookmarks:
 * Interrupt table          104
 * NVIC Priority            152
 */

void
SysTick_Init(unsigned long period) {
    NVIC_ST_CTRL_R = 0;                                                     // disable SysTick
    NVIC_ST_CURRENT_R = 0;                                                  // clear current counter
    Interrupt_SetSystemPriority(15, 6);
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
    volatile uint32_t delay;

    // Page 722 of datasheet
    SYSCTL_RCGCTIMER_R |= 0x1u;
    delay = SYSCTL_RCGCTIMER_R;                                         // allow time to finish activating
    TIMER0_CTL_R = set_bit_field_u32(TIMER0_CTL_R, 0, 1, 0);            //  1) Disable timer
    TIMER0_CFG_R = 0;                                                   //  2) Set to 32-bit mode
    TIMER0_TAMR_R = set_bit_field_u32(TIMER0_TAMR_R, 0, 2, 0b10);       //  3b) Set timer A to Periodic Mode
    TIMER0_TAMR_R = set_bit_field_u32(TIMER0_TAMR_R, 4, 1, 0b0);        //  4) Set timer A to count down
    TIMER0_TAILR_R = 80000000u / frequencyHz;                           //  5) Set reload value
    TIMER0_IMR_R = set_bit_field_u32(TIMER0_IMR_R, 0, 1, 1);            //  6) Enable timer A time-out interrupt
    Interrupt_SetPriority(19, 2);
    Interrupt_Enable(19);
    TIMER0_CTL_R = set_bit_field_u32(TIMER0_CTL_R, 0, 1, 1);            //  7) Enable timer and start counting
}

//  1000 Hz Handler
void
Timer0IntHandler(void) {
    // Page 722 of datasheet
    TIMER0_ICR_R = set_bit_field_u32(TIMER0_ICR_R, 0, 1, 0b1); //  8) Clear interrupt
}

void
Timer1Init(uint32_t period, uint32_t priority) {
    // Page 722 of datasheet
    SYSCTL_RCGCTIMER_R |= 0x2;
    TIMER1_CTL_R = set_bit_field_u32(TIMER1_CTL_R, 0, 1, 0);            //  1) Disable timer
    TIMER1_CFG_R = 0;                                                   //  2) Set to 32-bit mode
    TIMER1_TAMR_R = set_bit_field_u32(TIMER1_TAMR_R, 0, 2, 0b10);       //  3b) Set timer A to Periodic Mode
    TIMER1_TAMR_R = set_bit_field_u32(TIMER1_TAMR_R, 4, 1, 0b0);        //  4) Set timer A to count down
    TIMER1_TAILR_R = period;                                            //  5) Set reload value
    TIMER1_IMR_R = set_bit_field_u32(TIMER1_IMR_R, 0, 1, 1);            //  6) Enable timer A time-out interrupt
    Interrupt_SetPriority(21, priority);
    Interrupt_Enable(21);
    TIMER1_CTL_R = set_bit_field_u32(TIMER1_CTL_R, 0, 1, 1);            //  7) Enable timer and start counting
}

void
Timer1IntHandler(void) {
    TIMER1_ICR_R = set_bit_field_u32(TIMER1_ICR_R, 0, 1, 0b1); // Clear interrupt

    (*OS_CallBackgroundTask)(2);
}

// Using this as a OS high resolution system timer
void
Timer2Init(uint32_t period, uint32_t priority) {
    // Page 722 of datasheet
    SYSCTL_RCGCTIMER_R |= 0x4;
    TIMER2_CTL_R = set_bit_field_u32(TIMER2_CTL_R, 0, 1, 0);            //  1) Disable timer
    TIMER2_CFG_R = 0;                                                   //  2) Set to 32-bit mode
    TIMER2_TAMR_R = set_bit_field_u32(TIMER2_TAMR_R, 0, 2, 0b10);       //  3b) Set timer A to Periodic Mode
    TIMER2_TAMR_R = set_bit_field_u32(TIMER2_TAMR_R, 4, 1, 0b0);        //  4) Set timer A to count down
    TIMER2_TAILR_R = period;                                            //  5) Set reload value
    TIMER2_IMR_R = set_bit_field_u32(TIMER2_IMR_R, 0, 1, 1);            //  6) Disable timer A time-out interrupt
    Interrupt_SetPriority(23, priority);
    Interrupt_Enable(23);
    TIMER2_CTL_R = set_bit_field_u32(TIMER2_CTL_R, 0, 1, 1);            //  7) Enable timer and start counting
}

void
Timer2IntHandler(void) {
    TIMER2_ICR_R = set_bit_field_u32(TIMER2_ICR_R, 0, 1, 0b1); // Clear interrupt

    (*OS_CallBackgroundTask)(3);
}

// Using this as a OS high resolution system timer
void
Timer3Init(uint32_t period, uint32_t priority) {
    // Page 722 of datasheet
    SYSCTL_RCGCTIMER_R |= 0x8;
    TIMER3_CTL_R = set_bit_field_u32(TIMER3_CTL_R, 0, 1, 0);            //  1) Disable timer
    TIMER3_CFG_R = 0;                                                   //  2) Set to 32-bit mode
    TIMER3_TAMR_R = set_bit_field_u32(TIMER3_TAMR_R, 0, 2, 0b10);       //  3b) Set timer A to Periodic Mode
    TIMER3_TAMR_R = set_bit_field_u32(TIMER3_TAMR_R, 4, 1, 0b1);        //  4) Set timer A to count up
    TIMER3_TAILR_R = period;                                            //  5) Set reload value
    TIMER3_IMR_R = set_bit_field_u32(TIMER3_IMR_R, 0, 1, 1);            //  6) Enable timer A time-out interrupt
    Interrupt_SetPriority(35, priority);
    Interrupt_Enable(35);
    TIMER3_CTL_R = set_bit_field_u32(TIMER3_CTL_R, 0, 1, 1);            //  7) Enable timer and start counting
}

void
Timer3IntHandler(void) {
    TIMER3_ICR_R = set_bit_field_u32(TIMER3_ICR_R, 0, 1, 0b1); // Clear interrupt

    osTimeMs++;

    OS_UpdateSleep();
}

// Using this as a OS high resolution system timer
void
Timer4Init(uint32_t period, uint32_t priority) {
    // Page 722 of datasheet
    SYSCTL_RCGCTIMER_R |= 0x10;
    TIMER4_CTL_R = set_bit_field_u32(TIMER4_CTL_R, 0, 1, 0);            //  1) Disable timer
    TIMER4_TAV_R = 0;
    TIMER4_CFG_R = 0;                                                   //  2) Set to 32-bit mode
    TIMER4_TAMR_R = set_bit_field_u32(TIMER4_TAMR_R, 0, 2, 0b10);       //  3b) Set timer A to Periodic Mode
    TIMER4_TAMR_R = set_bit_field_u32(TIMER4_TAMR_R, 4, 1, 0b1);        //  4) Set timer A to count up
    TIMER4_TAILR_R = period;                                            //  5) Set reload value
    TIMER4_IMR_R = set_bit_field_u32(TIMER4_IMR_R, 0, 1, 0);            //  6) Enable timer A time-out interrupt
//    Interrupt_SetPriority(70, priority);
//    Interrupt_Enable(70);
    TIMER4_CTL_R = set_bit_field_u32(TIMER4_CTL_R, 0, 1, 1);            //  7) Enable timer and start counting
}

void
Timer4IntHandler(void) {
    TIMER4_ICR_R = set_bit_field_u32(TIMER4_ICR_R, 0, 1, 0b1); // Clear interrupt

}


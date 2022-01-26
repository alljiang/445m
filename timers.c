#include <timers.h>
#include "vware/tm4c123gh6pm.h"
#include "bit_utils.h"

extern uint64_t osTimeMs;

void
Timer0Init(void) {
    uint32_t frequencyHz = 1000u;

    // Page 722 of datasheet
    SYSCTL_RCGCTIMER_R |= 0x1u;
    TIMER0_CTL_R = set_bit_field_u32(TIMER0_CTL_R, 0, 1, 0); //  1) Disable timer
    TIMER0_CFG_R = 0;                     //  2) Write GPTMCFG with a value of 0
    TIMER0_TAMR_R = set_bit_field_u32(TIMER0_TAMR_R, 0, 2, 0b10); //  3b) Set timer A to Periodic Mode
    TIMER0_TAMR_R = set_bit_field_u32(TIMER0_TAMR_R, 4, 1, 0b0); //  4) Set timer A to count down
    TIMER0_TAILR_R = 80000000u / frequencyHz;            //  5) Set reload value
    TIMER0_IMR_R = set_bit_field_u32(TIMER0_IMR_R, 0, 1, 1); //  6) Enable timer A time-out interrupt
    TIMER0_CTL_R = set_bit_field_u32(TIMER0_CTL_R, 0, 1, 1); //  7) Enable timer and start counting

}

//  1000 Hz Handler
void
Timer0IntHandler(void) {
    // Page 722 of datasheet
    TIMER0_ICR_R = set_bit_field_u32(TIMER0_ICR_R, 0, 1, 0b1); //  8) Clear interrupt

    osTimeMs++;
}


#include <stdint.h>
#include "vware/tm4c123gh6pm.h"
#include "utils/bit-utils.h"

#include "launchpad.h"

#define SW1 2
#define SW2 1
#define PF4             (*((volatile uint32_t *)0x40025040))
#define PF3             (*((volatile uint32_t *)0x40025020))
#define PF2             (*((volatile uint32_t *)0x40025010))
#define PF1             (*((volatile uint32_t *)0x40025008))
#define PF0             (*((volatile uint32_t *)0x40025004))

// Must call GPIO_Initialize() before calling this function!
void
Launchpad_PortFInitialize() {
    GPIO_PORTF_AMSEL_R = 0x00;          // disable analog on PF
    GPIO_PORTF_PCTL_R = 0x00000000;     // PCTL GPIO on PF4-0
    GPIO_PORTF_DIR_R = 0x0E;            // PF4,PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;          // disable alt funct on PF7-0
    GPIO_PORTF_PUR_R = 0x11;            // enable pull-up on PF0 and PF4
    GPIO_PORTF_DEN_R = 0x1F;            // enable digital I/O on PF4-0
}

/*
 *  SW1                 PF4
 *  SW2                 PF0
 *  RGB LED RED         PF1
 *  RGB LED BLUE        PF2
 *  RGB LED GREEN       PF3
 */

void
Launchpad_ToggleLED(enum LED_Color color) {
    if(color == LED_RED) {
        PF1 ^= generate_bit_mask_u32(1, 1);
    } else if(color == LED_GREEN) {
        PF3 ^= generate_bit_mask_u32(3, 1);
    } else if(color == LED_BLUE) {
        PF2 ^= generate_bit_mask_u32(2, 1);
    }
}

void
Launchpad_SetLED(enum LED_Color color, bool state) {
    uint32_t mask;

    if(color == LED_RED) {
//        mask = generate_bit_mask_u32(1, 1);
        mask = 0x2;
        if(state) {
            PF1 |= mask;
        } else {
            PF1 &= ~mask;
        }
    } else if(color == LED_GREEN) {
//        mask = generate_bit_mask_u32(3, 1);
        mask = 0x8;
        if(state) {
            PF3 |= mask;
        } else {
            PF3 &= ~mask;
        }
    } else if(color == LED_BLUE) {
//        mask = generate_bit_mask_u32(2, 1);
        mask = 0x4;
        if(state) {
            PF2 |= mask;
        } else {
            PF2 &= ~mask;
        }
    }
}

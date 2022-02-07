/*
 * gpio.c
 */

#include <stdint.h>
#include <stdbool.h>
#include "bit-utils.h"
#include "vware/tm4c123gh6pm.h"
#include "RTOS/OS.h"

#include "gpio.h"

void
GPIO_PortAHandler(void) {
}

void
GPIO_PortBHandler(void) {
}

void
GPIO_PortCHandler(void) {
}

void
GPIO_PortDHandler(void) {
}

void
GPIO_PortEHandler(void) {
}

void
GPIO_PortFHandler(void) {
    GPIOPortF_Handler();
}

// Activates clocks on ports
void
GPIO_Initialize(void) {
    volatile unsigned long delay;

    SYSCTL_RCGCGPIO_R |= 0b111111;      // activate clock for ports A-F
    delay = SYSCTL_RCGCGPIO_R;

    GPIO_PORTF_LOCK_R = 0x4C4F434B;     // unlock GPIO Port F
    GPIO_PORTF_CR_R = 0x1F;             // allow changes to PF4-0

    GPIO_PortFInitialize();
}

void
GPIO_PortFInitialize(void) {
    GPIO_PORTF_AMSEL_R = 0x00;          // disable analog on PF
    GPIO_PORTF_PCTL_R = 0x00000000;     // PCTL GPIO on PF4-0
    GPIO_PORTF_DIR_R = 0x0E;            // PF4,PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;          // disable alt funct on PF7-0
    GPIO_PORTF_PUR_R = 0x11;            // enable pull-up on PF0 and PF4
    GPIO_PORTF_DEN_R = 0x1F;            // enable digital I/O on PF4-0
}

void
GPIO_EnableEdgeInterrupt(enum Port port, uint8_t pinNum,
        enum EdgeEvent risingEdge) {
    if (port == PORT_A) {
        NVIC_PRI0_R = set_bit_field_u32(NVIC_PRI0_R, 5, 3, 2u); // Priority 2, Interrupt 0

        NVIC_EN0_R = set_bit_field_u32(NVIC_EN0_R, 0, 1, 1);

        // set pin to input
        GPIO_PORTA_DIR_R = set_bit_field_u32(GPIO_PORTA_DIR_R, pinNum, 1, 0);

        // enable digital IO
        GPIO_PORTA_DEN_R = set_bit_field_u32(GPIO_PORTA_DEN_R, pinNum, 1, 1);

        // set falling edge event
        GPIO_PORTA_IEV_R = set_bit_field_u32(GPIO_PORTA_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        // 0 for edge, 1 for level
        GPIO_PORTA_IS_R = set_bit_field_u32(GPIO_PORTA_IS_R, pinNum, 1, 0);

        // 0 for use IEV reg, 1 for both edges trigger
        GPIO_PORTA_IBE_R = set_bit_field_u32(GPIO_PORTA_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        // 1 to enable interrupt
        GPIO_PORTA_IM_R = set_bit_field_u32(GPIO_PORTA_IM_R, pinNum, 1, 1);
    } else if (port == PORT_B) {
        NVIC_PRI0_R = set_bit_field_u32(NVIC_PRI0_R, 13, 3, 2u); // Priority 2, Interrupt 1

        NVIC_EN0_R = set_bit_field_u32(NVIC_EN0_R, 1, 1, 1);

        GPIO_PORTB_DIR_R = set_bit_field_u32(GPIO_PORTB_DIR_R, pinNum, 1, 0);

        GPIO_PORTB_DEN_R = set_bit_field_u32(GPIO_PORTB_DEN_R, pinNum, 1, 1);

        GPIO_PORTB_IEV_R = set_bit_field_u32(GPIO_PORTB_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTB_IS_R = set_bit_field_u32(GPIO_PORTB_IS_R, pinNum, 1, 0);

        GPIO_PORTB_IBE_R = set_bit_field_u32(GPIO_PORTB_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTB_IM_R = set_bit_field_u32(GPIO_PORTB_IM_R, pinNum, 1, 1);
    } else if (port == PORT_C) {
        NVIC_PRI0_R = set_bit_field_u32(NVIC_PRI0_R, 21, 3, 2u); // Priority 2, Interrupt 2

        NVIC_EN0_R = set_bit_field_u32(NVIC_EN0_R, 2, 1, 1);

        GPIO_PORTC_DIR_R = set_bit_field_u32(GPIO_PORTC_DIR_R, pinNum, 1, 0);

        GPIO_PORTC_DEN_R = set_bit_field_u32(GPIO_PORTC_DEN_R, pinNum, 1, 1);

        GPIO_PORTC_IEV_R = set_bit_field_u32(GPIO_PORTC_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTC_IS_R = set_bit_field_u32(GPIO_PORTC_IS_R, pinNum, 1, 0);

        GPIO_PORTC_IBE_R = set_bit_field_u32(GPIO_PORTC_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTB_IM_R = set_bit_field_u32(GPIO_PORTB_IM_R, pinNum, 1, 1);
    } else if (port == PORT_D) {
        NVIC_PRI0_R = set_bit_field_u32(NVIC_PRI0_R, 29, 3, 2u); // Priority 2, Interrupt 3

        NVIC_EN0_R = set_bit_field_u32(NVIC_EN0_R, 3, 1, 1);

        GPIO_PORTD_DIR_R = set_bit_field_u32(GPIO_PORTD_DIR_R, pinNum, 1, 0);

        GPIO_PORTD_DEN_R = set_bit_field_u32(GPIO_PORTD_DEN_R, pinNum, 1, 1);

        GPIO_PORTD_IEV_R = set_bit_field_u32(GPIO_PORTD_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTD_IS_R = set_bit_field_u32(GPIO_PORTD_IS_R, pinNum, 1, 0);

        GPIO_PORTD_IBE_R = set_bit_field_u32(GPIO_PORTD_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTB_IM_R = set_bit_field_u32(GPIO_PORTB_IM_R, pinNum, 1, 1);
    } else if (port == PORT_E) {
        NVIC_PRI1_R = set_bit_field_u32(NVIC_PRI1_R, 5, 3, 2u); // Priority 2, Interrupt 4

        NVIC_EN0_R = set_bit_field_u32(NVIC_EN0_R, 4, 1, 1);

        GPIO_PORTE_DIR_R = set_bit_field_u32(GPIO_PORTE_DIR_R, pinNum, 1, 0);

        GPIO_PORTE_DEN_R = set_bit_field_u32(GPIO_PORTE_DEN_R, pinNum, 1, 1);

        GPIO_PORTE_IEV_R = set_bit_field_u32(GPIO_PORTE_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTE_IS_R = set_bit_field_u32(GPIO_PORTE_IS_R, pinNum, 1, 0);

        GPIO_PORTE_IBE_R = set_bit_field_u32(GPIO_PORTE_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTE_IM_R = set_bit_field_u32(GPIO_PORTE_IM_R, pinNum, 1, 1);
    } else if (port == PORT_F) {
        NVIC_PRI7_R = set_bit_field_u32(NVIC_PRI7_R, 21, 3, 2u); // Priority 2, Interrupt 30

        NVIC_EN0_R = set_bit_field_u32(NVIC_EN0_R, 30, 1, 1);

        GPIO_PORTF_DIR_R = set_bit_field_u32(GPIO_PORTF_DIR_R, pinNum, 1, 0);

        GPIO_PORTF_DEN_R = set_bit_field_u32(GPIO_PORTF_DEN_R, pinNum, 1, 1);

        GPIO_PORTF_IEV_R = set_bit_field_u32(GPIO_PORTF_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTF_IS_R = set_bit_field_u32(GPIO_PORTF_IS_R, pinNum, 1, 0);

        GPIO_PORTF_IBE_R = set_bit_field_u32(GPIO_PORTF_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTF_IM_R = set_bit_field_u32(GPIO_PORTF_IM_R, pinNum, 1, 1);
    }
}

void GPIO_ClearInterruptStatus(enum Port port, uint8_t pinNum) {
    if (port == PORT_A) {
        GPIO_PORTA_ICR_R = set_bit_field_u32(GPIO_PORTA_ICR_R, pinNum, 1, 1);
    } else if(port == PORT_B) {
        GPIO_PORTB_ICR_R = set_bit_field_u32(GPIO_PORTB_ICR_R, pinNum, 1, 1);
    } else if(port == PORT_C) {
        GPIO_PORTC_ICR_R = set_bit_field_u32(GPIO_PORTC_ICR_R, pinNum, 1, 1);
    } else if(port == PORT_D) {
        GPIO_PORTD_ICR_R = set_bit_field_u32(GPIO_PORTD_ICR_R, pinNum, 1, 1);
    } else if(port == PORT_E) {
        GPIO_PORTE_ICR_R = set_bit_field_u32(GPIO_PORTE_ICR_R, pinNum, 1, 1);
    } else if(port == PORT_F) {
        GPIO_PORTF_ICR_R = set_bit_field_u32(GPIO_PORTF_ICR_R, pinNum, 1, 1);
    }

}

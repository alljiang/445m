/*
 * gpio.c
 */

#include <stdint.h>
#include <stdbool.h>
#include "bit-utils.h"
#include "vware/tm4c123gh6pm.h"
#include "RTOS/OS.h"
#include "interrupt.h"

#include "gpio.h"

extern void HCSR04_InterruptHandler(void);
extern void OPT3101_0_GPIOPortC_Handler(void);
extern void OPT3101_3_GPIOPortC_Handler(void);

void
GPIO_PortAHandler(void) {
}

void
GPIO_PortBHandler(void) {
    if (GPIO_PORTB_RIS_R & (1u << 6u)) {
        // PB6
        GPIO_ClearInterruptStatus(PORT_B, 6);
        HCSR04_InterruptHandler();
    }
}

void
GPIO_PortCHandler(void) {
    if (GPIO_PORTC_RIS_R & (1u << 4u)) {
        // PC4
        GPIO_ClearInterruptStatus(PORT_C, 4);
        OPT3101_0_GPIOPortC_Handler();
    } else if (GPIO_PORTC_RIS_R & (1u << 6u)) {
        // PC6
        GPIO_ClearInterruptStatus(PORT_C, 6);
        OPT3101_3_GPIOPortC_Handler();
    } else {
        GPIO_PORTC_ICR_R = 0xFF;
    }
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
}

void
GPIO_EnableEdgeInterrupt(enum Port port, uint8_t pinNum,
        enum EdgeEvent risingEdge, uint8_t priority) {
    if (priority > 7) {
        goto exit;
    }

    if (port == PORT_A) {
        Interrupt_SetPriority(0, priority);
        Interrupt_Enable(0);

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
        Interrupt_SetPriority(1, priority);
        Interrupt_Enable(1);

        GPIO_PORTB_DIR_R = set_bit_field_u32(GPIO_PORTB_DIR_R, pinNum, 1, 0);

        GPIO_PORTB_DEN_R = set_bit_field_u32(GPIO_PORTB_DEN_R, pinNum, 1, 1);

        GPIO_PORTB_IEV_R = set_bit_field_u32(GPIO_PORTB_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTB_IS_R = set_bit_field_u32(GPIO_PORTB_IS_R, pinNum, 1, 0);

        GPIO_PORTB_IBE_R = set_bit_field_u32(GPIO_PORTB_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTB_IM_R = set_bit_field_u32(GPIO_PORTB_IM_R, pinNum, 1, 1);
    } else if (port == PORT_C) {
        Interrupt_SetPriority(2, priority);
        Interrupt_Enable(2);

        GPIO_PORTC_DIR_R = set_bit_field_u32(GPIO_PORTC_DIR_R, pinNum, 1, 0);

        GPIO_PORTC_DEN_R = set_bit_field_u32(GPIO_PORTC_DEN_R, pinNum, 1, 1);

        GPIO_PORTC_IEV_R = set_bit_field_u32(GPIO_PORTC_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTC_IS_R = set_bit_field_u32(GPIO_PORTC_IS_R, pinNum, 1, 0);

        GPIO_PORTC_IBE_R = set_bit_field_u32(GPIO_PORTC_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTB_IM_R = set_bit_field_u32(GPIO_PORTB_IM_R, pinNum, 1, 1);
    } else if (port == PORT_D) {
        Interrupt_SetPriority(3, priority);
        Interrupt_Enable(3);

        GPIO_PORTD_DIR_R = set_bit_field_u32(GPIO_PORTD_DIR_R, pinNum, 1, 0);

        GPIO_PORTD_DEN_R = set_bit_field_u32(GPIO_PORTD_DEN_R, pinNum, 1, 1);

        GPIO_PORTD_IEV_R = set_bit_field_u32(GPIO_PORTD_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTD_IS_R = set_bit_field_u32(GPIO_PORTD_IS_R, pinNum, 1, 0);

        GPIO_PORTD_IBE_R = set_bit_field_u32(GPIO_PORTD_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTB_IM_R = set_bit_field_u32(GPIO_PORTB_IM_R, pinNum, 1, 1);
    } else if (port == PORT_E) {
        Interrupt_SetPriority(4, priority);
        Interrupt_Enable(4);

        GPIO_PORTE_DIR_R = set_bit_field_u32(GPIO_PORTE_DIR_R, pinNum, 1, 0);

        GPIO_PORTE_DEN_R = set_bit_field_u32(GPIO_PORTE_DEN_R, pinNum, 1, 1);

        GPIO_PORTE_IEV_R = set_bit_field_u32(GPIO_PORTE_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTE_IS_R = set_bit_field_u32(GPIO_PORTE_IS_R, pinNum, 1, 0);

        GPIO_PORTE_IBE_R = set_bit_field_u32(GPIO_PORTE_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTE_IM_R = set_bit_field_u32(GPIO_PORTE_IM_R, pinNum, 1, 1);
    } else if (port == PORT_F) {
        Interrupt_SetPriority(30, priority);
        Interrupt_Enable(30);

        GPIO_PORTF_DIR_R = set_bit_field_u32(GPIO_PORTF_DIR_R, pinNum, 1, 0);

        GPIO_PORTF_DEN_R = set_bit_field_u32(GPIO_PORTF_DEN_R, pinNum, 1, 1);

        GPIO_PORTF_IEV_R = set_bit_field_u32(GPIO_PORTF_IEV_R, pinNum, 1,
                risingEdge == RISING_EDGE);

        GPIO_PORTF_IS_R = set_bit_field_u32(GPIO_PORTF_IS_R, pinNum, 1, 0);

        GPIO_PORTF_IBE_R = set_bit_field_u32(GPIO_PORTF_IBE_R, pinNum, 1,
                risingEdge == BOTH_EDGES);

        GPIO_PORTF_IM_R = set_bit_field_u32(GPIO_PORTF_IM_R, pinNum, 1, 1);
    }

exit:
    return;

}

void
GPIO_ClearInterruptStatus(enum Port port, uint8_t pinNum) {
    if (port == PORT_A) {
        GPIO_PORTA_ICR_R = set_bit_field_u32(GPIO_PORTA_ICR_R, pinNum, 1, 1);
    } else if (port == PORT_B) {
        GPIO_PORTB_ICR_R = set_bit_field_u32(GPIO_PORTB_ICR_R, pinNum, 1, 1);
    } else if (port == PORT_C) {
        GPIO_PORTC_ICR_R = set_bit_field_u32(GPIO_PORTC_ICR_R, pinNum, 1, 1);
    } else if (port == PORT_D) {
        GPIO_PORTD_ICR_R = set_bit_field_u32(GPIO_PORTD_ICR_R, pinNum, 1, 1);
    } else if (port == PORT_E) {
        GPIO_PORTE_ICR_R = set_bit_field_u32(GPIO_PORTE_ICR_R, pinNum, 1, 1);
    } else if (port == PORT_F) {
        GPIO_PORTF_ICR_R = set_bit_field_u32(GPIO_PORTF_ICR_R, pinNum, 1, 1);
    }

}

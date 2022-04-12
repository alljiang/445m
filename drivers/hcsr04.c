#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "vware/hw_memmap.h"
#include "vware/hw_ints.h"
#include "vware/hw_nvic.h"
#include "vware/InterruptFunctions.h"
#include "vware/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "RTOS/OS.h"
#include "utils/utils.h"
#include "drivers/hcsr04.h"
#include "drivers/gpio.h"
#include "drivers/timers.h"

/*
 * TRIG: PB7
 * ECHO: PB6
 */

int hcsr04_distance = -1; // units of 0.01 cm
int state = -1;

void
HCSR04_Initialize() {
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6); // ECHO
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);  // TRIG
    state = -1;
}

// return distance in 0.01cm
// returns negative for invalid
int
HCSR04_GetDistance() {
    return hcsr04_distance;
}

void
HCSR04_StartMeasurement() {
    /*
     * Pulse Width Measurement between 150us and 25ms
     */

    state = 0; // send trig pulse

    // Create a pulse of 10us
    DisableInterrupts();

    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);
    delayMicroseconds(12);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);

    EnableInterrupts();


    // Wait for echo to go high
    state = 1; // awaiting rising edge
    GPIO_EnableEdgeInterrupt(PORT_B, 6, BOTH_EDGES, 1);
}

void HCSR04_CalculateDistance(uint32_t pulseWidth) {
    // pulseWidth is in units of 12.5ns
    hcsr04_distance = 17 * (pulseWidth / 12.5 / 10) * 0.1538 + 82;
}

void
HCSR04_InterruptHandler() {
    if (state == 1) {
        Timer4Init(0xFFFFFFFF, 7);
        state = 2; // awaiting falling edge
    } else if (state == 2) {
        if (state == 2) {
            HCSR04_CalculateDistance((uint32_t) TIMER4_TAV_R);
            state = -1; // finished
        }
    }
}

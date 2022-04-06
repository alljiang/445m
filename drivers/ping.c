#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "vware/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "RTOS/OS.h"
#include "utils/utils.h"
#include "vware/InterruptFunctions.h"
#include "drivers/ping.h"

/*
 * PING0: PB6
 * PING1: PB4
 * PING2: PB2
 * PING3: PC5
 */

void
Ping_Initialize() {
#ifdef PING0
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);
#endif

#ifdef PING1
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
#endif

#ifdef PING2
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
#endif

#ifdef PING3
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
#endif
}

// return distance in 0.01cm
// returns negative for invalid
int
Ping_GetDistance(uint8_t sensor) {
    int rv;
    uint32_t portBase, portPin;

    unsigned static long LastTime;
    uint32_t thisTime;
    uint32_t timeCount = 0; // units of 12.5ns

    switch (sensor) {
#ifdef PING0
        case 0:
            portBase = GPIO_PORTB_BASE;
            portPin = GPIO_PIN_6;
            break;
#endif

#ifdef PING1
        case 1:
            portBase = GPIO_PORTB_BASE;
            portPin = GPIO_PIN_4;
            break;
#endif

#ifdef PING2
        case 2:
            portBase = GPIO_PORTB_BASE;
            portPin = GPIO_PIN_2;
            break;
#endif

#ifdef PING3
        case 3:
            portBase = GPIO_PORTC_BASE;
            portPin = GPIO_PIN_5;
            break;
#endif

        default:
            portBase = NULL;
            portPin = NULL;
            break;

    }

    if (portBase == NULL || portPin == NULL) {
        rv = -1;
        goto exit;
    }

    // drive pin high for 2-5 us
    GPIOPinTypeGPIOOutput(portBase, portPin);
    delayMicroseconds(3);

    // change to input
    GPIOPinTypeGPIOInput(portBase, portPin);

    DisableInterrupts();

    // busy wait for input to go high, start counting
    while (GPIOPinRead(portBase, portPin) == 0);

    // busy wait for input to go low
    while (GPIOPinRead(portBase, portPin) == 1) {
        thisTime = OS_Time();       // current time, 12.5ns

        uint32_t diff = OS_TimeDifference(LastTime, thisTime);
        timeCount += diff;

        LastTime = thisTime;
    }

    EnableInterrupts();

    // convert and return
    /*
     * 115us (9200, 12.5ns) = 2cm
     * 18.5ms (1480000, 12.5ns) = 3000cm
     */
    rv = map(timeCount, 9200, 1480000, 2, 3000, 100);

exit:
    return rv;
}

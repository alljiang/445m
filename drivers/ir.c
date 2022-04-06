#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "vware/hw_memmap.h"
#include "driverlib/pin_map.h"

#include "RTOS/OS.h"
#include "RTOS/ADC.h"
#include "utils/utils.h"
#include "drivers/ir.h"

/*
 * IR0: PD3 AIN4
 * IR1: PD2 AIN5
 * IR2: PD1 AIN6
 * IR3: PD0 AIN7
 */

void
IR_Initialize(void) {
}

// get distance in cm
float
IR_getDistance(uint8_t device) {
    int rv = -1;
    int channel = -1;
    float raw;

    switch (device) {
#ifdef IR0
        case 0:
            channel = 0;
            break;
#endif

#ifdef IR1
        case 1:
            channel = 1;
            break;
#endif

#ifdef IR2
        case 2:
            channel = 2;
            break;
#endif

#ifdef IR3
        case 3:
            channel = 3;
            break;
#endif

        default:
            channel = -1;
            break;
    }

    if (channel == -1) {
        rv = -1;
        goto exit;
    }

    ADC_Init(channel);
    raw = ADC_In_Voltage();
    rv = 29.988 * pow(raw, -1.173);

exit:
    return rv;
}

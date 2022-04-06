#include <stdint.h>
#include <utils.h>
#include "RTOS/OS.h"

void
delayMicroseconds(int microseconds) {
    unsigned static long LastTime;
    uint32_t thisTime;

    microseconds *= 10; // change to units of 0.1 us

    while (microseconds > 0) {
        thisTime = OS_Time();       // current time, 12.5ns

        uint32_t diff = OS_TimeDifference(LastTime, thisTime); // units of 0.1 us
        microseconds -= diff;

        LastTime = thisTime;
    }
}

long
map(long x, long in_min, long in_max, long out_min, long out_max, int scale) {
    return scale * (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min * scale;
}

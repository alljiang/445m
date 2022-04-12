#include <stdint.h>
#include <utils.h>
#include "RTOS/OS.h"

void
delayMicroseconds(int microseconds) {
    unsigned static long LastTime;
    uint32_t thisTime;

   for (int i = 0; i < microseconds * 6; i++) {
       volatile int waste = 0;
   }
}

long
map(long x, long in_min, long in_max, long out_min, long out_max, int scale) {
    return scale * (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min * scale;
}

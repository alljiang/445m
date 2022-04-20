#include <stdint.h>
#include <utils.h>
#include "RTOS/OS.h"

void
delayMicroseconds(int microseconds) {
    for (int i = 0; i < microseconds * 6; i++) {
        volatile int waste = 0;
    }
}

long
map(long x, long in_min, long in_max, long out_min, long out_max, int scale) {
    return scale * (x - in_min) * (out_max - out_min) / (in_max - in_min)
            + out_min * scale;
}

int inline
limit(int num, int min, int max) {
    if (num < min) {
        num = min;
    } else if (num > max) {
        num = max;
    }
    return num;
}

int inline
min(int a, int b) {
    int rv = 0;

    if (a < b)
        rv = a;
    else
        rv = b;

    return rv;
}

int inline
max(int a, int b) {
    int rv = 0;

    if (a > b)
        rv = a;
    else
        rv = b;

    return rv;
}

int inline
addMagnitude(int num, int addition) {
    if (addition < 0) addition = -addition;

    if (num > 0)
        num += addition;
    else
        num -= addition;

    return num;
}

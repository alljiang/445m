/*
 * str_utils.c
 *
 *  Created on: Jan 26, 2022
 *      Author: Allen
 */

#include <stdint.h>

/*
 * Changes a number of variable length to a character.
 * Must pass in a char buffer that is long enough for
 * the value.
 */
int
itoa(int value, char *ptr) {
    int count = 0, temp;
    if (ptr == 0) return 0;
    if (value == 0) {
        *ptr = '0';
        return 1;
    }

    if (value < 0) {
        value *= (-1);
        *ptr++ = '-';
        count++;
    }
    for (temp = value; temp > 0; temp /= 10, ptr++);
    *ptr = '\0';
    for (temp = value; temp > 0; temp /= 10) {
        *--ptr = temp % 10 + '0';
        count++;
    }
    return count;
}

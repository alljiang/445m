/*
 * launchpad.h
 *
 *  Created on: Feb 10, 2022
 *      Author: Allen
 */

#ifndef DRIVERS_LAUNCHPAD_H_
#define DRIVERS_LAUNCHPAD_H_

#include <stdbool.h>

enum LED_Color {
    LED_RED, LED_GREEN, LED_BLUE
};

extern void
Launchpad_PortFInitialize(void);

extern void
Launchpad_ToggleLED(enum LED_Color color);

extern void
Launchpad_SetLED(enum LED_Color color, bool state);

#endif /* DRIVERS_LAUNCHPAD_H_ */

#include <stdint.h>
#include <stdbool.h>

#include "vware/hw_memmap.h"
#include "vware/hw_pwm.h"
#include "vware/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "utils/utils.h"

#include "drivers/motor.h"

/*
 * Motor R
 *  A: PB7          M0PWM1 Gen0
 *  B: PB6          M0PWM0 Gen0
 *
 * Motor L
 *  A: PB5          M0PWM3 Gen1
 *  B: PB4          M0PWM2 Gen1
 *
 * Servo A: PD0     M0PWM6 Gen3
 * Servo B: PD1     M0PWM7 Gen3   NOT IMPLEMENTED
 */

void
Motor_Initialize() {
    // Initialize PWM
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {
    }

    // Configure PWM Pins
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);

    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);

    PWMOutputInvert(PWM0_BASE,
            PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);

    Motor_setLeft(0);
    Motor_setRight(0);

}

// speed is between (-1000, 1000)
void
Motor_setRight(int speed) {
    int genPeriod, speedMagnitude, speedPeriod;

    if (speed < -1000 || speed > 1000) goto exit;

    speed = -speed;

    genPeriod = 10000;      // clock ticks
    speedMagnitude = speed;
    if (speedMagnitude < 0) speedMagnitude = -speedMagnitude;

    speedPeriod = speedMagnitude * 10;
    speedPeriod = speedPeriod;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, genPeriod);

    if (speed < 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, speedPeriod);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
    } else if (speed > 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, speedPeriod);
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
    } else {
        PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
    }

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

exit:
    return;
}

// speed is between (-1000, 1000)
void
Motor_setLeft(int speed) {
    int genPeriod, speedMagnitude, speedPeriod;

    if (speed < -1000 || speed > 1000) goto exit;

    genPeriod = 10000;      // clock ticks
    speedMagnitude = speed;
    if (speedMagnitude < 0) speedMagnitude = -speedMagnitude;

    speedPeriod = speedMagnitude * 10;
    speedPeriod = speedPeriod;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, genPeriod);

    if (speed < 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, speedPeriod);
        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
    } else if (speed > 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, speedPeriod);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
    } else {
        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
    }

    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

exit:
    return;
}

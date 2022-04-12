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
 * Motor L
 *  A: PB7          M0PWM1 Gen0
 *  B: PB6          M0PWM0 Gen0
 *
 * Motor R
 *  A: PB5          M0PWM3 Gen1
 *  B: PB4          M0PWM2 Gen1
 *
 * Servo A: PD0     M0PWM6 Gen3
 * Servo B: PD1     M0PWM7 Gen3   NOT IMPLEMENTED
 */

void
Motor_Initialize() {
    // Initialize PWM
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

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

    GPIOPinConfigure(GPIO_PD0_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);

    GPIOPinConfigure(GPIO_PD1_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);

    Motor_setLeft(0);
    Motor_setRight(0);
    Motor_setServo(90);

    PWMOutputState(PWM0_BASE,
            PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT
                    | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
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
    speedPeriod = 10000 - speedPeriod;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, genPeriod);

    if (speed > 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, speedPeriod);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    } else if (speed < 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, speedPeriod);
    }
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    PWMOutputState(PWM0_BASE,
    PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMOutputState(PWM0_BASE,
    PWM_OUT_6_BIT | PWM_OUT_7_BIT, false);
exit:
    return;
}

// speed is between (-1000, 1000)
void
Motor_setRight(int speed) {
    int genPeriod, speedMagnitude, speedPeriod;

    if (speed < -1000 || speed > 1000) goto exit;

    genPeriod = 312;      // clock ticks
    speedMagnitude = -speed;   // flip since on other side
    if (speedMagnitude < 0) speedMagnitude = -speedMagnitude;

    speedPeriod = speed * genPeriod / 1000;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, genPeriod);

    if (speed > 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, speedPeriod);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    } else if (speed < 0) {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, speedPeriod);
    }

    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

exit:
    return;
}

// angle is between 0, 180
void
Motor_setServo(int angle) {
    int genPeriod, sendPeriod;

    if (angle < 0 || angle > 180) goto exit;

    genPeriod = 50000;      // clock ticks, 20ms
    sendPeriod = map(angle, 0, 180, genPeriod / 20, genPeriod / 10, 1);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, genPeriod);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, sendPeriod);

    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    PWMOutputState(PWM0_BASE,
    PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

exit:
    return;
}

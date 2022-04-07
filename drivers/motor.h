/*
 * motor.h
 *
 *  Created on: Apr 6, 2022
 *      Author: Allen
 */

#ifndef DRIVERS_MOTOR_H_
#define DRIVERS_MOTOR_H_

/*
 * Motor L
 *  A: PB7          M0PWM1
 *  B: PB6          M0PWM0
 *
 * Motor R
 *  A: PB5          M0PWM3
 *  B: PB4          M0PWM2
 *
 * Servo A: PD0     M0PWM6
 * Servo B: PD1     M0PWM7
 */

void
Motors_Initialize();

// speed is between (-1000, 1000)
void
Motor_setLeft(int speed);

// speed is between (-1000, 1000)
void
Motor_setRight(int speed);

// angle is between 0, 180
void
Motor_setServo(int angle);

#endif /* DRIVERS_MOTOR_H_ */

// Lab6.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Lab 6

// Jonathan W. Valvano 3/29/17, valvano@mail.utexas.edu
// Andreas Gerstlauer 3/1/16, gerstl@ece.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for use by OS profile 
// PF1 is preemptive thread switch
// PF2 is first periodic background task (if any)
// PF3 is second periodic background task (if any)
// PC4 is PF4 button touch (SW1 task)

// Outputs for task profiling
// PD0 is idle task
// PD1 is button task

// Button inputs
// PF0 is SW2 task
// PF4 is SW1 button input

// Analog inputs
// PE3 Ain0 sampled at 2kHz, sequencer 3, by Interpreter, using software start

#include <stdint.h>
#include <stdbool.h> 
#include <stdio.h> 
#include <string.h>
#include <stdlib.h>
#include <vware/opt3101_i2c0.h>
#include <vware/opt3101_i2c3.h>
#include "vware/tm4c123gh6pm.h"
#include "vware/CortexM.h"
#include "vware/LaunchPad.h"
#include "vware/PLL.h"
#include "vware/LPF.h"
#include "vware/I2CB1.h"
#include "vware/I2C3.h"
#include "RTOS/UART0int.h"
#include "RTOS/ADC.h"
#include "RTOS/OS.h"
#include "RTOS/heap.h"
#include "RTOS/Interpreter.h"
#include "RTOS/ST7735.h"
#include "RTOS/can0.h"
#include "utils/utils.h"
#include "utils/uart-utils.h"

#include "drivers/ping.h"
#include "drivers/ir.h"
#include "drivers/gpio.h"
#include "drivers/launchpad.h"
#include "drivers/hcsr04.h"

// CAN IDs are set dynamically at time of CAN0_Open
// Reverse on other microcontroller
#define RCV_ID 2
#define XMT_ID 3

//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short
PID_stm32(short Error, short *Coeff);
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM

uint32_t NumCreated;   // number of foreground threads created
uint32_t IdleCount;    // CPU idle counter

uint8_t packet_opt3101[4];
uint8_t packet_ir[4];
uint8_t packet_hcsr04[4];

//---------------------User debugging-----------------------
extern int32_t MaxJitter;      // largest time jitter between interrupts in usec

#define PD0  (*((volatile uint32_t *)0x40007004))
#define PD1  (*((volatile uint32_t *)0x40007008))
#define PD2  (*((volatile uint32_t *)0x40007010))
#define PD3  (*((volatile uint32_t *)0x40007020))

//---------------------ROBOT CAR STUFFS---------------------
uint32_t opt3101_l[3];
uint32_t opt3101_r[3];
uint32_t opt3101_l_raw[3];
uint32_t opt3101_r_raw[3];

#define MOTOR_MAX 1000
#define MOTOR_MIN -1000

uint8_t packet_motor[4];
void
CANSendMotor(int left, int right) {
    // make left and right 12 bits each
    packet_motor[0] = 0; // type motor speed

    // total 24 bits 
    // 1: left[12:4]
    // 2: left[3:0] | right[12:8]
    // 3: right[7:0]
    packet_motor[1] = (left >> 4) & 0xFF;
    packet_motor[2] = ((left & 0x0F) << 4) | ((right >> 8) & 0x0F);
    packet_motor[3] = right & 0xFF;

    CAN0_SendData(packet_motor);
}

#define MAX_SENSOR_DISTANCE 8000    // bigger = more turning ability but less stability while going straightish

void
AcquireOPT3101(void) {

    while (1) {
        int n = 1, i;

        for (i = 0; i < 3 * n; i++) {
            int channel = 0;
            if (i >= n) channel = 1;
            if (i >= 2 * n) channel = 2;

            DisableInterrupts();
            OPT3101_0_StartMeasurementChannel(channel);
            EnableInterrupts();

            while (!OPT3101_0_CheckDistanceSensor()) {
                OS_Sleep(3);
            }

            DisableInterrupts();
            OPT3101_0_ReadMeasurement();
            EnableInterrupts();

            OS_Sleep(30); // TODO may or may not need

            DisableInterrupts();
            OPT3101_3_StartMeasurementChannel(channel);
            EnableInterrupts();

            while (!OPT3101_3_CheckDistanceSensor()) {
                OS_Sleep(3);
            }

            DisableInterrupts();
            OPT3101_3_ReadMeasurement();

            opt3101_l_raw[channel] = OPT3101_0_GetDistanceMillimeters() * 10; // units 0.01cm
            opt3101_r_raw[channel] = OPT3101_3_GetDistanceMillimeters() * 10; // units 0.01cm

            opt3101_l_raw[channel] = min(opt3101_l_raw[channel], MAX_SENSOR_DISTANCE);
            opt3101_r_raw[channel] = min(opt3101_r_raw[channel], MAX_SENSOR_DISTANCE);

            opt3101_l[channel] = opt3101_l[channel] * 0.6 + opt3101_l_raw[channel] * 0.4;
            opt3101_r[channel] = opt3101_r[channel] * 0.6 + opt3101_r_raw[channel] * 0.4;

            EnableInterrupts();

            OS_Sleep(30); // TODO may or may not need
        }
    }
}

const char *str_error = "ERROR ";

void
CANHandler(void) {
    uint8_t buffer[4];
    int data;

    while (1) {
        // Suspend OS if mail not available
        while (!CAN0_CheckMail()) {
            OS_Suspend();
        }
        CAN0_GetMail(buffer);

        data = (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];

        if (buffer[0] == 0) {
        } else if (buffer[0] == 1) {
        } else {
        }
    }
}

void
Idle(void) {
    IdleCount = 0;
    while (1) {
        IdleCount++;
        PD0 ^= 0x01;
        WaitForInterrupt();
    }
}

enum ControlState {
    STOP, GOGOGO, WAITING
};

#define DISTANCE_KP 0.11
#define DISTANCE_KI 0.02
#define DISTANCE_KD 0.30

#define ANGLE_KP 0.05
#define ANGLE_KI 0.0
#define ANGLE_KD 0.0

#define KI_SCALE 1000

void
Controller(void) {
    enum ControlState state = STOP, lastState = STOP, nextState = STOP;
    int speedL, speedR, feedForward = 0, speedForward = 950;
    int ll, lm, lr, rl, rm, rr;

    int distance_error = 0, angle_error = 0;
    int64_t distance_integral = 0, angle_integral = 0;
    int distance_lastError = 0, angle_lastError = 0;
    int distance_lastD = 0, angle_lastD = 0;

    int transitionTime = 0;

    ST7735_PlotClear(-500, 500);

    while (1) {
        int time = OS_MsTime();

        // get sensor readings
        ll = opt3101_l[0];
        lm = opt3101_l[1];
        lr = opt3101_l[2];
        rl = opt3101_r[0];
        rm = opt3101_r[1];
        rr = opt3101_r[2];

        // Calculate Average Distances
        int leftSum = 0, rightSum = 0, leftAvg, rightAvg;
        leftSum += ll + lm + lr;
        rightSum += rl + rm + rr;
        leftAvg = leftSum / 3;
        rightAvg = rightSum / 3;

        // Calculate Angle Differences
        int leftAngle = lm - ll;
        int rightAngle = rm - rl;

        Launchpad_SetLED(LED_RED, leftAngle > rightAngle);
        Launchpad_SetLED(LED_GREEN, leftAngle < rightAngle);

//        Launchpad_SetLED(LED_RED, leftDistance > rightDistance);
//        Launchpad_SetLED(LED_GREEN, leftDistance < rightDistance);

        if (state == STOP) {
            speedL = 0;
            speedR = 0;

            //TODO
            if (time > transitionTime) {
                transitionTime = time + 600;
                nextState = GOGOGO;
            }
        } else if (state == GOGOGO) {

            if (lastState != GOGOGO) {
                // first iteration, reset some parameters
                distance_integral = 0;
                distance_lastD = 0;
                distance_lastError = distance_error;

                angle_integral = 0;
                angle_lastD = 0;
                angle_lastError = angle_error;
            }

            ST7735_ClearColumn();

            // ========== Layer 1 - Distance PID ==========
            distance_error = rightAvg - leftAvg;

            int distance_p = DISTANCE_KP * distance_error;
            int distance_i = distance_integral / KI_SCALE;
            int distance_d = DISTANCE_KD
                    * (distance_error - distance_lastError);
            distance_lastD = distance_lastD * 0.4 + distance_d * 0.6;

            // update lastError
            distance_lastError = distance_error;

            // cap the I term
            distance_integral += DISTANCE_KI * distance_error;
            distance_integral = limit(distance_integral, MOTOR_MIN * KI_SCALE,
            MOTOR_MAX * KI_SCALE);

            int distance_output = distance_p + distance_i + distance_lastD;
            distance_output = limit(distance_output, MOTOR_MIN, MOTOR_MAX);

            speedL = addMagnitude(distance_output + speedForward, feedForward);
            speedR = addMagnitude(-distance_output + speedForward,
                    feedForward + 20);

            // ========== End of Layer 1 ==========
            // ========== Layer 2 - Angle PID ==========
            angle_error = rightAngle - leftAngle;

            int angle_p = DISTANCE_KP * angle_error;
            int angle_i = angle_integral / KI_SCALE;
            int angle_d = ANGLE_KD * (angle_error - angle_lastError);
            angle_lastD = angle_lastD * 0.4 + angle_d * 0.6;

            // update lastError
            angle_lastError = angle_error;

            // cap the I term
            angle_integral += ANGLE_KI * angle_error;
            angle_integral = limit(angle_integral, MOTOR_MIN * KI_SCALE,
                    MOTOR_MAX * KI_SCALE);

            int angle_output = angle_p + angle_i + angle_lastD;
            angle_output = limit(angle_output, MOTOR_MIN, MOTOR_MAX);

            speedL += angle_output;
            speedR -= angle_output;

            // ========== End of Layer 2 ==========
            // ========== Debug Plotting ==========

            // Distance
//            ST7735_Message(0, 0, "ER ", distance_error);
//            ST7735_Message(0, 1, "P  ", distance_p);
//            ST7735_Message(0, 2, "D  ", distance_lastD);
//            ST7735_PlotPoint(distance_output, ST7735_WHITE);
//            ST7735_PlotPoint(distance_p, ST7735_GREEN);
//            ST7735_PlotPoint(distance_integral, ST7735_YELLOW);
//            ST7735_PlotPoint(distance_lastD, ST7735_BLUE);

            // Angle
            ST7735_Message(0, 0, "ER ", angle_error);
            ST7735_Message(0, 1, "P  ", angle_p);
            ST7735_Message(0, 2, "D  ", angle_lastD);
            ST7735_PlotPoint(angle_output, ST7735_WHITE);
            ST7735_PlotPoint(angle_p, ST7735_GREEN);
            ST7735_PlotPoint(angle_integral, ST7735_YELLOW);
            ST7735_PlotPoint(angle_lastD, ST7735_BLUE);

            // Distance + Angle
//            ST7735_Message(0, 0, "D ER ", angle_error);
//            ST7735_Message(0, 1, "A ER ", angle_error);
//            ST7735_PlotPoint(distance_output, ST7735_GREEN);
//            ST7735_PlotPoint(distance_error, ST7735_MAGENTA);
//            ST7735_PlotPoint(angle_output, ST7735_BLUE);
//            ST7735_PlotPoint(angle_error, ST7735_CYAN);

            // ======= End of Debug Plotting =======

            ST7735_PlotNext();

//            if (time > transitionTime) {
//                transitionTime = time + 300;
//                nextState = WAITING;
//            }

        } else if (state == WAITING) {
            speedL = 0;
            speedR = 0;

            if (time > transitionTime) {
                transitionTime = time + 500;
                nextState = GOGOGO;
            }
        } else {
            while (1);
        }

//        UART_OutStringNonBlock("ll: ");
//        UART_OutUDecNonBlock(ll);
//        UART_OutStringNonBlock("\trr: ");
//        UART_OutUDecNonBlock(rr);
//        UART_OutStringNonBlock("\tl: ");
//        UART_OutUDecNonBlock(leftAvg);
//        UART_OutStringNonBlock("\tr: ");
//        UART_OutUDecNonBlock(rightAvg);
//        UART_OutStringNonBlock("\tL: ");
//        UART_OutUDecNonBlock(speedL);
//        UART_OutStringNonBlock("\tR: ");
//        UART_OutUDecNonBlock(speedR);
//        UART_OutStringNonBlock("\r\n");

        speedL = limit(speedL, MOTOR_MIN, MOTOR_MAX);
        speedR = limit(speedR, MOTOR_MIN, MOTOR_MAX);
//        CANSendMotor(speedL, speedR);

        lastState = state;
        state = nextState;
        OS_Sleep(20);
    }
}

void
Halt(void) {
    while (1) {
        CANSendMotor(0, 0);
    }
}

int
realmain(void) { // realmain
    OS_Init();        // initialize, disable interrupts

    CAN0_Open(RCV_ID, XMT_ID);

//    OS_AddPeriodicThread(&, 80000000 / 20, 2);   // 20 Hz
    OS_AddSW2Task(&Halt, 1);

    // create initial foreground threads
    NumCreated = 0;
    NumCreated += OS_AddThread(&AcquireOPT3101, 128, 2);
    NumCreated += OS_AddThread(&Interpreter, 128, 3);
    NumCreated += OS_AddThread(&Controller, 128, 3);
//    NumCreated += OS_AddThread(&CANHandler, 128, 3);
    NumCreated += OS_AddThread(&Idle, 128, 5);  // at lowest priority

    OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
    return 0;            // this never executes
}

//*******************Trampoline for selecting main to execute**********
int
main(void) { 			// main
    PLL_Init(Bus80MHz);
    GPIO_Initialize();
    Launchpad_PortFInitialize();
    ST7735_InitR(INITR_REDTAB);             // LCD initialization
    UART_Init();                              // serial I/O for interpreter

    I2C0_Init(200000, 80000000);
    I2C3_Init(200000, 80000000);

    OPT3101_0_Init(7);
    OPT3101_3_Init(7);
    OPT3101_0_Setup();
    OPT3101_3_Setup();
    OPT3101_0_CalibrateInternalCrosstalk();
    OPT3101_3_CalibrateInternalCrosstalk();

    realmain();
}

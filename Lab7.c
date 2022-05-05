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
uint32_t TxChannel_0, TxChannel_1;
uint32_t Amplitudes_0[3];
uint32_t Amplitudes_1[3];
uint32_t Distances_0[3];
uint32_t Distances_1[3];

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

#define MAX_SENSOR_DISTANCE 1100    // bigger = more turning ability but less stability while going straightish

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
    STOP, GOGOGO, WAITING, CRASHED
};

#define DISTANCE_KP_SLOW 0.025
#define DISTANCE_KI_SLOW 0.0017
#define DISTANCE_KD_SLOW 0.12

#define DISTANCE_KP 0.030
#define DISTANCE_KI 0.0021
#define DISTANCE_KD 0.12

#define ANGLE_KP 0.0
#define ANGLE_KI 0.0
#define ANGLE_KD 0.0

#define KI_SCALE 1000

#define CRASH_DETECT_THRESHOLD 60
#define SHORT_CRASH_DETECT_TIME 200
#define CRASH_DETECT_TIME 1600

char *str_ll = "ll: ";
char *str_lm = "lm: ";
char *str_lr = "lr: ";
char *str_rl = "rl: ";
char *str_rightm = "rm: ";
char *str_rr = "rr: ";

void
printTelemetryScreen() {
    ST7735_Message(0, 0, str_ll, Distances_0[2]);
    ST7735_Message(0, 1, str_lm, Distances_0[1]);
    ST7735_Message(0, 2, str_lr, Distances_0[0]);
    ST7735_Message(0, 3, str_rl, Distances_1[2]);
    ST7735_Message(0, 4, str_rightm, Distances_1[1]);
    ST7735_Message(0, 5, str_rr, Distances_1[0]);

    // get sensor readings
    int ll = min(Distances_0[2], MAX_SENSOR_DISTANCE);
    int lm = min(Distances_0[1], MAX_SENSOR_DISTANCE);
    int lr = min(Distances_0[0], MAX_SENSOR_DISTANCE);
    int rl = min(Distances_1[2], MAX_SENSOR_DISTANCE);
    int rm = min(Distances_1[1], MAX_SENSOR_DISTANCE);
    int rr = min(Distances_1[0], MAX_SENSOR_DISTANCE);

    // Calculate Average Distances
    int leftSum = 0, rightSum = 0, leftDistance, rightDistance;
    leftSum += ll + lm + lr;
    rightSum += rl + rm + rr;
    leftDistance = leftSum * 10 / 3;
    rightDistance = rightSum * 10 / 3;

    ST7735_Message(1, 0, "L ", leftDistance);
    ST7735_Message(1, 1, "R ", rightDistance);
    ST7735_Message(1, 2, "E ", rightDistance - leftDistance);
}

void
Controller(void) {
    enum ControlState state = GOGOGO, lastState = STOP, nextState = GOGOGO;
    int speedL, speedR, feedForward = 0, speedForward = 820; // 820
    int ll, lm, lr, rl, rm, rr;

    int distance_error = 0;
    int64_t distance_integral = 0;
    int distance_lastError = 0;
    int distance_lastD = 0;
    int distance_dError, distance_dErrorFiltered = 3000;
    int distance_dErrorLightlyFiltered = 3000;
    int time, crashDetectStartTime = -1;
    bool crashedOnLeftSide;
    bool last_slowMode = false;

    int transitionTime = 0;

    Launchpad_SetLED(LED_BLUE, true);

    // Wait for Button Press
    while (!Launchpad_SW1Pressed()) {
        printTelemetryScreen();
    }

    int startTime = OS_MsTime();

    ST7735_FillScreen(0);
    ST7735_PlotClear(-500, 500);

    while (1) {
        time = OS_MsTime() - startTime;

        // get sensor readings
        ll = min(Distances_0[2], MAX_SENSOR_DISTANCE);
        lm = min(Distances_0[1], MAX_SENSOR_DISTANCE);
        lr = min(Distances_0[0], MAX_SENSOR_DISTANCE);
        rl = min(Distances_1[2], MAX_SENSOR_DISTANCE);
        rm = min(Distances_1[1], MAX_SENSOR_DISTANCE);
        rr = min(Distances_1[0], MAX_SENSOR_DISTANCE);

        // Calculate Average Distances
        int leftSum = 0, rightSum = 0, leftDistance, rightDistance;
        leftSum += ll + lm + lr;
        rightSum += rl + rm + rr;
        leftDistance = leftSum * 10 / 3;
        rightDistance = rightSum * 10 / 3;
        int frontDistance = min(lr, rl) * 10;
        int midDistance = min(lm, rm) * 10;
        int backDistance = min(ll, rr) * 10;
        int nearestDistance = min(min(frontDistance, midDistance), backDistance);

        if (state == STOP) {
            speedL = 0;
            speedR = 0;

            Launchpad_SetLED(LED_RED, true);
            Launchpad_SetLED(LED_GREEN, true);
            Launchpad_SetLED(LED_BLUE, true);
        } else if (state == GOGOGO) {
            Launchpad_SetLED(LED_BLUE, false);
//            Launchpad_SetLED(LED_RED, leftDistance > rightDistance);
//            Launchpad_SetLED(LED_GREEN, leftDistance < rightDistance);

            if (lastState != GOGOGO) {
                // first iteration, reset some parameters
                distance_integral = 0;
                distance_lastD = 0;
                distance_lastError = distance_error;

                crashDetectStartTime = time;
            }

            ST7735_ClearColumn();

            // ========== Layer 1 - Distance PID ==========
            distance_error = rightDistance - leftDistance;
            distance_dError = distance_error - distance_lastError;

            // filtering for speed detection
            distance_dErrorFiltered = distance_dErrorFiltered * 0.8
                    + abs(distance_dError) * 0.2;
            // filtering for crash detection
            distance_dErrorLightlyFiltered = distance_dErrorLightlyFiltered * 0.6
                    + abs(distance_dError) * 0.4;

            int distance_p = distance_error;
            int distance_i = distance_integral / KI_SCALE;
            int distance_d = distance_dError;

            if (frontDistance < 8000 || midDistance < 2800) {
                speedForward = 780;
                Launchpad_SetLED(LED_RED, true);
                Launchpad_SetLED(LED_GREEN, false);

                if (!last_slowMode) {
                    last_slowMode = true;
//                    distance_integral = 0;
                }

                distance_p = DISTANCE_KP_SLOW * distance_p;
                distance_integral += DISTANCE_KI_SLOW * distance_error;
                distance_d = DISTANCE_KD_SLOW * distance_d;
            } else {
                if (distance_dErrorFiltered < 150) {
                    speedForward = 940;
                    Launchpad_SetLED(LED_RED, false);
                    Launchpad_SetLED(LED_GREEN, true);
                } else {
                    speedForward = 880;
                    Launchpad_SetLED(LED_RED, false);
                    Launchpad_SetLED(LED_GREEN, false);
                }

                if (last_slowMode) {
                    last_slowMode = false;
//                    distance_integral = 0;
                }

                distance_p = DISTANCE_KP * distance_p;
                distance_integral += DISTANCE_KI * distance_error;
                distance_d = DISTANCE_KD * distance_d;
            }

            distance_lastD = distance_lastD * 0.4 + distance_d * 0.6;

            // update lastError
            distance_lastError = distance_error;

            // cap the I term
            distance_integral = limit(distance_integral,
            MOTOR_MIN * KI_SCALE,
            MOTOR_MAX * KI_SCALE);

            int distance_output = distance_p + distance_i + distance_lastD;
            distance_output = limit(distance_output, MOTOR_MIN, MOTOR_MAX);

            int offset = distance_output + speedForward - 1000;
            if (offset < 0) offset = 0;

            speedL = addMagnitude(distance_output + speedForward - offset,
                    feedForward + 35);
            speedR = addMagnitude(-distance_output + speedForward - offset,
                    feedForward);

            // ========== End of Layer 1 ==========
            // ========== Layer 2 - Crash Detection ==========

            if (distance_dErrorFiltered > CRASH_DETECT_THRESHOLD) {
                crashDetectStartTime = time;
            } else if ((time - crashDetectStartTime > SHORT_CRASH_DETECT_TIME) && (nearestDistance < 350)) {
                // front crash detected!
                crashedOnLeftSide = distance_error > 0;

                transitionTime = time + 1000;
                nextState = CRASHED;
                Launchpad_SetLED(LED_BLUE, true);
                Launchpad_SetLED(LED_RED, true);
                Launchpad_SetLED(LED_GREEN, false);
            } else if (time - crashDetectStartTime > CRASH_DETECT_TIME) {
                // stuck for a while
                crashedOnLeftSide = distance_error > 0;

                transitionTime = time + 1000;
                nextState = CRASHED;
                Launchpad_SetLED(LED_BLUE, true);
                Launchpad_SetLED(LED_RED, false);
                Launchpad_SetLED(LED_GREEN, false);
            }

            // ========== End of Layer 2 ==========
            // ========== Debug Plotting ==========

            // Distance
            ST7735_Message(0, 0, "ER ", distance_error);
            ST7735_Message(0, 1, "P  ", distance_p);
            ST7735_Message(0, 2, "D  ", distance_lastD);
            ST7735_PlotPoint(distance_output, ST7735_WHITE);
            ST7735_PlotPoint(distance_p, ST7735_GREEN);
            ST7735_PlotPoint(distance_integral, ST7735_YELLOW);
            ST7735_PlotPoint(distance_lastD, ST7735_BLUE);

            // Crash Detection
//            ST7735_Message(0, 2, "DE  ", distance_dErrorFiltered);
//            ST7735_PlotPoint(distance_dErrorFiltered, ST7735_BLUE);

            // ======= End of Debug Plotting =======

            ST7735_PlotNext();

        } else if (state == CRASHED) {
            if (crashedOnLeftSide) {
//                speedL = -750;
//                speedR = -800;
            } else {
//                speedL = -800;
//                speedR = -750;
            }
            speedL = -820;
            speedR = -800;

            if (time > transitionTime) {
                nextState = GOGOGO;
            }
        } else if (state == WAITING) {
            speedL = 0;
            speedR = 0;

            if (time > transitionTime) {
                nextState = GOGOGO;
            }
        } else {
            while (1);
        }

        speedL = limit(speedL, MOTOR_MIN, MOTOR_MAX);
        speedR = limit(speedR, MOTOR_MIN, MOTOR_MAX);
        CANSendMotor(speedL, speedR);

        // stop condition at 180s?
        if (time > 180000) {
            nextState = STOP;
        }

        lastState = state;
        state = nextState;
        OS_Sleep(20);
    }
}

void
EmergencyStop(void) {
    while (1) {
        CANSendMotor(0, 0);
    }
}

int
realmain(void) { // realmain
    OS_Init();        // initialize, disable interrupts

    OS_AddSW2Task(&EmergencyStop, 1);

    // create initial foreground threads
    NumCreated = 0;
    NumCreated += OS_AddThread(&Interpreter, 128, 3);
    NumCreated += OS_AddThread(&Controller, 128, 3);
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

    CAN0_Open(RCV_ID, XMT_ID);
    CANSendMotor(0, 0);

    ST7735_InitR(INITR_REDTAB);             // LCD initialization
    UART_Init();                              // serial I/O for interpreter

    I2C0_Init(400000, 80000000);
    I2C3_Init(400000, 80000000);

    OPT3101_0_Init(7);
    OPT3101_3_Init(7);
    OPT3101_0_Setup();
    OPT3101_3_Setup();
    OPT3101_0_CalibrateInternalCrosstalk();
    OPT3101_3_CalibrateInternalCrosstalk();

    TxChannel_0 = 3;
    OPT3101_0_ArmInterrupts(&TxChannel_0, Distances_0, Amplitudes_0);
    OPT3101_0_StartMeasurementChannel(1);

    TxChannel_1 = 3;
    OPT3101_3_ArmInterrupts(&TxChannel_1, Distances_1, Amplitudes_1);
    OPT3101_3_StartMeasurementChannel(1);

    realmain();
}

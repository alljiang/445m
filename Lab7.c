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

#define MOTOR_MAX 1000
#define MOTOR_MIN -1000

void
PortD_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
    while ((SYSCTL_RCGCGPIO_R & 0x08) == 0) {
    };
    GPIO_PORTD_DIR_R |= 0x0F;        // make PD3-0 output heartbeats
    GPIO_PORTD_AFSEL_R &= ~0x0F;     // disable alt funct on PD3-0
    GPIO_PORTD_DEN_R |= 0x0F;        // enable digital I/O on PD3-0
    GPIO_PORTD_PCTL_R = ~0x0000FFFF;
    GPIO_PORTD_AMSEL_R &= ~0x0F;
    ;    // disable analog functionality on PD
}

uint8_t packet_motor[4];
void
CANSendMotor(int left, int right) {
    // make left and right 12 bits each
    packet_motor[0] = 1; // type motor speed

    // total 24 bits 
    // 1: left[12:4]
    // 2: left[3:0] | right[12:8]
    // 3: right[7:0]
    packet_motor[1] = (left >> 4) & 0xFF;
    packet_motor[2] = ((left & 0x0F) << 4) | ((right >> 8) & 0x0F);
    packet_motor[3] = right & 0xFF;

    CAN0_SendData(packet_ir);
}

void
AcquireOPT3101(void) {

    while (1) {
        int n = 1, i;

        for (i = 0; i < 3*n; i++) {
            int channel = 0;
            if (i >= n) channel = 1;
            if (i >= 2*n) channel = 2;

//            OPT3101_0_StartMeasurementChannel(channel);
            OPT3101_3_StartMeasurementChannel(channel);

//            while (!OPT3101_0_CheckDistanceSensor()) {
//                OS_Sleep(3);
//            }
            while (!OPT3101_3_CheckDistanceSensor()) {
                OS_Sleep(3);
            }

//            OPT3101_0_ReadMeasurement();
            OPT3101_3_ReadMeasurement();

//            opt3101_r[channel] = OPT3101_0_GetDistanceMillimeters() * 10; // units 0.01cm
            opt3101_l[channel] = OPT3101_3_GetDistanceMillimeters() * 10; // units 0.01cm

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
        STOP,
        GO_SIDE,
        GO_MIDDLE,
        GO_FRONT
};

#define GO_SIDE_KP 0.05
#define GO_SIDE_KI 0
#define GO_SIDE_KD 0

#define GO_SIDE_KI_SCALE 100

void
Controller(void) {
    enum ControlState state = STOP, lastState = STOP, nextState = STOP;
    int speedL, speedR, baseSpeed = 300;
    int ll, lm, lr, rl, rm, rr;

    int side_error = 0;
    int side_integral = 0;
    int side_lastError = 0;

    while (1) {

        // get sensor readings
        ll = opt3101_l[0];
        lm = opt3101_l[1];
        lr = opt3101_l[2];
        rl = opt3101_r[2];
        rm = opt3101_r[1];
        rr = opt3101_r[0];

        ST7735_Message(0, 0, "LL ", ll);
        ST7735_Message(0, 1, "LM ", lm);
        ST7735_Message(0, 2, "LR ", lr);
        ST7735_Message(0, 3, "RL ", rl);
        ST7735_Message(0, 4, "RM ", rm);
        ST7735_Message(0, 5, "RR ", rr);

        if (state == STOP) {
            speedL = 0;
            speedR = 0;
        } else if (state == GO_SIDE) {
            // setpoint is just 0
            side_error = ll - rr;

            if (lastState != GO_SIDE) {
                // first iteration, reset some parameters
                side_integral = 0;
                side_lastError = side_error;
            }

            int side_p = GO_SIDE_KP * side_error;
            int side_i = side_integral / GO_SIDE_KI_SCALE;
            int side_d = GO_SIDE_KD * (side_error - side_lastError);

            // update lastError
            side_lastError = side_error;

            // cap the I term
            side_integral += GO_SIDE_KI * side_error;
            side_integral = limit(side_integral, -MOTOR_MIN * GO_SIDE_KI_SCALE, MOTOR_MAX * GO_SIDE_KI_SCALE);

            int side_output = side_p + side_i + side_d;
            side_output = limit(side_output, -MOTOR_MIN, MOTOR_MAX);

            speedL = baseSpeed + side_output;
            speedR = baseSpeed - side_output;
        } else {
            while (1);
        }

//        CANSendMotor(speedL, speedR);

        lastState = state;
        state = nextState;
        OS_Sleep(20);
    }
}

int
realmain(void) { // realmain
    OS_Init();        // initialize, disable interrupts
    PortD_Init();     // debugging profile

    CAN0_Open(RCV_ID, XMT_ID);

//    OS_AddPeriodicThread(&, 80000000 / 20, 2);   // 20 Hz

    // create initial foreground threads
    NumCreated = 0;
    NumCreated += OS_AddThread(&AcquireOPT3101, 128, 2);
    NumCreated += OS_AddThread(&Interpreter, 128, 3);
//    NumCreated += OS_AddThread(&Controller, 128, 3);
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
    ST7735_InitR(INITR_GREENTAB);             // LCD initialization
    UART_Init();                              // serial I/O for interpreter

    I2C0_Init(400000, 80000000);
    I2C3_Init(400000, 80000000);

//    OPT3101_0_Init(7);
    OPT3101_3_Init(7);
//    OPT3101_0_Setup();
    OPT3101_3_Setup();
//    OPT3101_0_CalibrateInternalCrosstalk();
    OPT3101_3_CalibrateInternalCrosstalk();

    realmain();
}

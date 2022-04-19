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
#include "vware/tm4c123gh6pm.h"
#include "vware/CortexM.h"
#include "vware/LaunchPad.h"
#include "vware/PLL.h"
#include "vware/LPF.h"
#include "RTOS/UART0int.h"
#include "RTOS/ADC.h"
#include "RTOS/OS.h"
#include "RTOS/heap.h"
#include "RTOS/can0.h"

#include "drivers/gpio.h"
#include "drivers/launchpad.h"
#include "drivers/motor.h"

// CAN IDs are set dynamically at time of CAN0_Open
// Reverse on other microcontroller
#define RCV_ID 3
#define XMT_ID 2

//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short
PID_stm32(short Error, short *Coeff);
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM

uint32_t NumCreated;   // number of foreground threads created
uint32_t IdleCount;    // CPU idle counter

//---------------------User debugging-----------------------
extern int32_t MaxJitter;      // largest time jitter between interrupts in usec

#define PD0  (*((volatile uint32_t *)0x40007004))
#define PD1  (*((volatile uint32_t *)0x40007008))
#define PD2  (*((volatile uint32_t *)0x40007010))
#define PD3  (*((volatile uint32_t *)0x40007020))

int motorLeft, motorRight;

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

void
DriveMotors(void) {
    /*
     * Motor 1A: PB7
     * Motor 1B: PB6
     */

    Motor_Initialize();

    while (1) {

        Motor_setLeft(motorLeft);
        Motor_setRight(motorRight);

        OS_Sleep(10);
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

void
CANThread(void) {
    uint8_t buffer[4];
    while (1) {
        CAN0_GetMail(buffer);
        
        if (buffer[0] == 0) {
            motorLeft = (buffer[1] << 4) | (buffer[2] >> 4);
            motorRight = ((buffer[2] & 0x0F) << 8) | buffer[3];
        }
    }
}

int
realmain(void) { // realmain
    OS_Init();        // initialize, disable interrupts
    PortD_Init();     // debugging profile
    Heap_Init();      // initialize heap
    MaxJitter = 0;    // in 1us units

    // hardware init
    CAN0_Open(RCV_ID, XMT_ID);

    // create initial foreground threads
    NumCreated = 0;
    NumCreated += OS_AddThread(&CANThread, 128, 2);
    NumCreated += OS_AddThread(&DriveMotors, 128, 2);
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

    realmain();
}

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
#include "RTOS/Interpreter.h"
#include "RTOS/ST7735.h"
#include "RTOS/can0.h"

// CAN IDs are set dynamically at time of CAN0_Open
// Reverse on other microcontroller
#define RCV_ID 2
#define XMT_ID 4

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
Idle(void) {
    IdleCount = 0;
    while (1) {
        IdleCount++;
        PD0 ^= 0x01;
        WaitForInterrupt();
    }
}

int
realmain(void) { // realmain
    OS_Init();        // initialize, disable interrupts
    PortD_Init();     // debugging profile
    Heap_Init();      // initialize heap
    MaxJitter = 0;    // in 1us units

    // hardware init
    ADC_Init(0);  // sequencer 3, channel 0, PE3, sampling in Interpreter
    CAN0_Open(RCV_ID, XMT_ID);

    // create initial foreground threads
    NumCreated = 0;
    NumCreated += OS_AddThread(&Interpreter, 128, 2);
    NumCreated += OS_AddThread(&Idle, 128, 5);  // at lowest priority

    OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
    return 0;            // this never executes
}

//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 

//*****************Test project 0*************************
// This is the simplest configuration, 
// Just see if you can import your OS
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
uint32_t Count1;   // number of times thread1 loops
uint32_t Count2;   // number of times thread2 loops
uint32_t Count3;   // number of times thread3 loops
void
Thread1(void) {
    Count1 = 0;
    for (;;) {
        PD0 ^= 0x01;       // heartbeat
        Count1++;
    }
}
void
Thread2(void) {
    Count2 = 0;
    for (;;) {
        PD1 ^= 0x02;       // heartbeat
        Count2++;
    }
}
void
Thread3(void) {
    Count3 = 0;
    for (;;) {
        PD2 ^= 0x04;       // heartbeat
        Count3++;
    }
}

int
Testmain0(void) {  // Testmain0
    OS_Init();          // initialize, disable interrupts
    PortD_Init();       // profile user threads
    NumCreated = 0;
    NumCreated += OS_AddThread(&Thread1, 128, 1);
    NumCreated += OS_AddThread(&Thread2, 128, 2);
    NumCreated += OS_AddThread(&Thread3, 128, 3);
    // Count1 Count2 Count3 should be equal or off by one at all times
    OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
    return 0;            // this never executes
}

//*****************Test project 1*************************
// CAN test, exchange CAN messages with second instance
uint8_t XmtData[4];
uint8_t RcvData[4];
uint32_t RcvCount = 0;
uint8_t sequenceNum = 0;

// periodic background task to send CAN message
void
CANSendTask(void) {
    XmtData[0] = PF0 << 1;  // 0 or 2
    XmtData[1] = PF4 >> 2;  // 0 or 4
    XmtData[2] = 0;       // unassigned field
    XmtData[3] = sequenceNum;  // sequence count
    CAN0_SendData(XmtData);
    sequenceNum++;
}

// foreground receiver task 
void
CANReceiveTask(void) {
    while (1) {
        CAN0_GetMail(RcvData);
        RcvCount++;
        ST7735_Message(1, 0, "RcvCount   = ", RcvCount);
        ST7735_Message(1, 0, "RcvData[0] = ", RcvData[0]);
        ST7735_Message(1, 0, "RcvData[1] = ", RcvData[1]);
    }
}

void
ButtonWork1(void) {
    uint32_t myId = OS_Id();
    ST7735_Message(0, 1, "SequenceNum = ", sequenceNum);
    OS_Kill();  // done, OS does not return from a Kill
}

void
SW1Push1(void) {
    if (OS_MsTime() > 20) { // debounce
        if (OS_AddThread(&ButtonWork1, 128, 1)) {
            NumCreated++;
        }
        OS_ClearMsTime();  // at least 20ms between touches
    }
}

int
Testmain1(void) {   // Testmain1
    OS_Init();           // initialize, disable interrupts
    PortD_Init();

    // Initialize CAN with given IDs
    CAN0_Open(RCV_ID, XMT_ID);

    // attach background tasks
    OS_AddPeriodicThread(&CANSendTask, 80000000 / 10, 2);   // 10 Hz
    OS_AddSW1Task(&SW1Push1, 2);

    // create initial foreground threads
    NumCreated = 0;
    NumCreated += OS_AddThread(&Idle, 128, 3);
    NumCreated += OS_AddThread(&CANReceiveTask, 128, 2);

    OS_Launch(10 * TIME_1MS); // doesn't return, interrupts enabled in here
    return 0;               // this never executes
}

//*******************Trampoline for selecting main to execute**********
int
main(void) { 			// main
    Testmain1();
//    realmain();
}

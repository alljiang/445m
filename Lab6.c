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
#include "vware/opt3101.h"
#include "vware/I2CB1.h"
#include "RTOS/UART0int.h"
#include "RTOS/ADC.h"
#include "RTOS/OS.h"
#include "RTOS/heap.h"
#include "RTOS/Interpreter.h"
#include "RTOS/ST7735.h"
#include "RTOS/can0.h"

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
AcquireOPT3101(void) {
    uint32_t opt3101[3];

    while (1) {
        for (int i = 0; i < 3; i++) {
            OPT3101_StartMeasurementChannel(i);
            while (!OPT3101_CheckDistanceSensor());
            OPT3101_ReadMeasurement();
            opt3101[i] = OPT3101_GetDistanceMillimeters() * 10; // units 0.01cm

            packet_opt3101[0] = 2 + i; // type OPT3101 (ch0: 2, ch1: 3, ch2: 4)
            packet_opt3101[1] = (opt3101[i] >> 16) & 0xFF;
            packet_opt3101[2] = (opt3101[i] >> 8) & 0xFF;
            packet_opt3101[3] = (opt3101[i] >> 0) & 0xFF;

            CAN0_SendData(packet_opt3101);
            OS_Sleep(100 / 3);
        }
    }
}

void
AcquireIR(void) {
    int ir;

    while (1) {
        ir = IR_getDistance(0); // units 0.01cm

        packet_ir[0] = 1; // type IR
        packet_ir[1] = (ir >> 16) & 0xFF;
        packet_ir[2] = (ir >> 8) & 0xFF;
        packet_ir[3] = (ir >> 0) & 0xFF;

        CAN0_SendData(packet_ir);
        OS_Sleep(100);
    }
}

void
AcquireHCSR04(void) {
    int hcsr04;

    while (1) {
        hcsr04 = HCSR04_GetDistance();

        packet_hcsr04[0] = 0; // type hcsr04
        packet_hcsr04[1] = (hcsr04 >> 16) & 0xFF;
        packet_hcsr04[2] = (hcsr04 >> 8) & 0xFF;
        packet_hcsr04[3] = (hcsr04 >> 0) & 0xFF;

        CAN0_SendData(packet_hcsr04);
        OS_Sleep(80);
    }
}

const char *str_hcsr04 = "HCSR04: ";
const char *str_ir = "IR: ";
const char *str_opt_0 = "OPT0: ";
const char *str_opt_1 = "OPT1: ";
const char *str_opt_2 = "OPT2: ";
const char *str_error = "ERROR ";

void
CANHandler(void) {
    uint8_t buffer[4];
    int data;

    while (1) {
        // Suspend OS if mail not available
        CAN0_GetMail(buffer);

        data = (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];

        if (buffer[0] == 0) {
            // hcsr04
            ST7735_Message(0, 1, (char*) str_hcsr04, data);
        } else if (buffer[0] == 1) {
            // IR
            ST7735_Message(0, 2, (char*) str_ir, data);
        } else if (buffer[0] >= 2 && buffer[0] <= 4) {
            // OPT3101
            int channel = buffer[0] - 2;

            if (channel == 0) {
                ST7735_Message(0, 3, (char*) str_opt_0, data);
            } else if (channel == 1) {
                ST7735_Message(0, 4, (char*) str_opt_1, data);
            } else if (channel == 2) {
                ST7735_Message(0, 5, (char*) str_opt_2, data);
            }
        } else {
            ST7735_Message(0, 7, (char*) str_error, 0);
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

void
SampleIR(void) {
    while (1) {
        IR_Sample();
        OS_Sleep(10);
    }
}

int
realmain(void) { // realmain
    OS_Init();        // initialize, disable interrupts
//    OS_Fifo_Init(128);
    PortD_Init();     // debugging profile

    CAN0_Open(RCV_ID, XMT_ID);

    OS_AddPeriodicThread(&HCSR04_StartMeasurement, 80000000 / 20, 2);   // 20 Hz

    // create initial foreground threads
    NumCreated = 0;
    NumCreated += OS_AddThread(&Interpreter, 128, 2);
    NumCreated += OS_AddThread(&CANHandler, 128, 2);
    NumCreated += OS_AddThread(&AcquireOPT3101, 128, 2);
    NumCreated += OS_AddThread(&AcquireIR, 128, 2);
    NumCreated += OS_AddThread(&AcquireHCSR04, 128, 2);
    NumCreated += OS_AddThread(&SampleIR, 128, 2);
    NumCreated += OS_AddThread(&Idle, 128, 5);  // at lowest priority

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
        ST7735_Message(1, 1, "RcvData[0] = ", RcvData[0]);
        ST7735_Message(1, 2, "RcvData[1] = ", RcvData[1]);
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
    PLL_Init(Bus80MHz);
    GPIO_Initialize();
    Launchpad_PortFInitialize();
    ST7735_InitR(INITR_GREENTAB);             // LCD initialization
    UART_Init();                              // serial I/O for interpreter
    HCSR04_Initialize();

    IR_Initialize();

    I2C0_Init(400000, 80000000);
    OPT3101_Init(5);
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();

//    Testmain1();
    realmain();
}

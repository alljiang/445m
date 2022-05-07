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
#include <vware/opt3101_i2c3.h>
#include "vware/tm4c123gh6pm.h"
#include "vware/CortexM.h"
#include "vware/LaunchPad.h"
#include "vware/PLL.h"
#include "vware/LPF.h"
#include "vware/I2CB1.h"
#include "vware/I2C3.h"
#include "RTOS/ADC.h"
#include "RTOS/OS.h"
#include "RTOS/heap.h"
#include "RTOS/Interpreter.h"
#include "RTOS/ST7735.h"
#include "RTOS/can0.h"
#include "utils/utils.h"

#include "drivers/ping.h"
#include "drivers/HC12.h"
#include "drivers/gpio.h"
#include "drivers/launchpad.h"

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

//---------------------User debugging-----------------------
extern int32_t MaxJitter;      // largest time jitter between interrupts in usec

#define PD0  (*((volatile uint32_t *)0x40007004))
#define PD1  (*((volatile uint32_t *)0x40007008))
#define PD2  (*((volatile uint32_t *)0x40007010))
#define PD3  (*((volatile uint32_t *)0x40007020))

//---------------------ROBOT CAR STUFFS---------------------
int motor_left = 0;
int motor_right = 0;
uint32_t TxChannel_1;
uint32_t Amplitudes_1[3];
uint32_t Distances_1[3];

uint8_t rxBuffer[100];
uint8_t rxBufferStart;
uint8_t rxBufferLength;

int lastHeartbeat = -1;

uint8_t isConnected;
bool sensorRequested, heartbeatRequested;
int sensorRequestTime, heartbeatRequestTime;

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

uint8_t heartbeat_data[4] = {0x9A, 0xBC, 0xDE, 0xF0};

void
RequestHandler_Task(void) {

    while (1) {
        int time = OS_MsTime();

        if (heartbeatRequested && time - heartbeatRequestTime > 10) {
            HC12_SendData(1, heartbeat_data);
            heartbeatRequested = false;
        } else if (sensorRequested && time - sensorRequestTime > 10) {
            // get sensor readings
            uint8_t rl = min(Distances_1[2]/10, 0xFF);
            uint8_t rm = min(Distances_1[1]/10, 0xFF);
            uint8_t rr = min(Distances_1[0]/10, 0xFF);

            // send sensor data
            uint8_t sensor_data[4] = {rl, rm , rr, 0x0};
            HC12_SendData(3, sensor_data);
            sensorRequested = false;
        }

        isConnected = (OS_MsTime() - lastHeartbeat < 1000) && (lastHeartbeat > -1);

        OS_Sleep(2);
    }
}

void
ProcessHC12RxBuffer() {
    while (1) {
        if (rxBufferLength >= 6) {
            uint8_t header = rxBuffer[(rxBufferStart + 0) % sizeof(rxBuffer)];

            // verify header
            if (header > 15) {
                // invalid header, skip
                rxBufferStart = (rxBufferStart + 1) % sizeof(rxBuffer); // increment start
                rxBufferLength--;
                continue;
            }

            uint8_t checksum = rxBuffer[(rxBufferStart + 5) % sizeof(rxBuffer)];

            // calculate xor checksum
            uint8_t xorChecksum = header;
            for (int i = 1; i < 5; i++) {
                xorChecksum ^= rxBuffer[(rxBufferStart + i) % sizeof(rxBuffer)];
            }

            if (xorChecksum != checksum) {
                // checksum failed, skip
                rxBufferStart = (rxBufferStart + 1) % sizeof(rxBuffer); // increment start
                rxBufferLength--;
                continue;
            }

            if (header == 0) {
                // heartbeat, verify data
                if (rxBuffer[(rxBufferStart + 1) % sizeof(rxBuffer)] == 0x12 &&
                    rxBuffer[(rxBufferStart + 2) % sizeof(rxBuffer)] == 0x34 &&
                    rxBuffer[(rxBufferStart + 3) % sizeof(rxBuffer)] == 0x56 &&
                    rxBuffer[(rxBufferStart + 4) % sizeof(rxBuffer)] == 0x78) {

                    lastHeartbeat = OS_MsTime();
                    heartbeatRequested = true;
                    heartbeatRequestTime = OS_MsTime();
                }
            } else if (header == 2) {
                // motor speed
                int left = rxBuffer[(rxBufferStart + 1) % sizeof(rxBuffer)] << 8;
                left |= rxBuffer[(rxBufferStart + 2) % sizeof(rxBuffer)];
                int right = rxBuffer[(rxBufferStart + 3) % sizeof(rxBuffer)] << 8;
                right |= rxBuffer[(rxBufferStart + 4) % sizeof(rxBuffer)];

                motor_left = left;
                motor_right = right;
            } else if (header == 4) {
                // sensor, verify data
                if (rxBuffer[(rxBufferStart + 1) % sizeof(rxBuffer)] == 0xAA &&
                    rxBuffer[(rxBufferStart + 2) % sizeof(rxBuffer)] == 0xBB &&
                    rxBuffer[(rxBufferStart + 3) % sizeof(rxBuffer)] == 0xCC &&
                    rxBuffer[(rxBufferStart + 4) % sizeof(rxBuffer)] == 0xDD) {

                    sensorRequested = true;
                    sensorRequestTime = OS_MsTime();
                }
            }

            rxBufferStart = (rxBufferStart + 6) % sizeof(rxBuffer); // increment start
            rxBufferLength -= 6;
        } else {
            OS_Sleep(5);
        }
    }
}

enum ControlState {
    STOP, GOGOGO
};

void
Controller(void) {
    enum ControlState state = STOP, lastState = STOP, nextState = STOP;
    int speedL, speedR;
    int time;


    int startTime = OS_MsTime();

    while (1) {
        time = OS_MsTime() - startTime;

        if (state == STOP) {
            speedL = 0;
            speedR = 0;

            Launchpad_SetLED(LED_RED, true);
            Launchpad_SetLED(LED_GREEN, false);
            Launchpad_SetLED(LED_BLUE, false);

            if (isConnected) {
                nextState = GOGOGO;
            }

        } else if (state == GOGOGO) {

            speedL = motor_left;
            speedR = motor_right;

            if (speedL == 0 && speedR == 0) {
                Launchpad_SetLED(LED_RED, false);
                Launchpad_SetLED(LED_GREEN, true);
                Launchpad_SetLED(LED_BLUE, false);
            } else {
                Launchpad_SetLED(LED_RED, true);
                Launchpad_SetLED(LED_GREEN, true);
                Launchpad_SetLED(LED_BLUE, true);
            }

            if (!isConnected) {
                nextState = STOP;
            }
        } else {
            while (1);
        }

        speedL = limit(speedL, MOTOR_MIN, MOTOR_MAX);
        speedR = limit(speedR, MOTOR_MIN, MOTOR_MAX);
        CANSendMotor(speedL, speedR);

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
    NumCreated += OS_AddThread(&Controller, 128, 3);
    NumCreated += OS_AddThread(&RequestHandler_Task, 128, 3);
    NumCreated += OS_AddThread(&ProcessHC12RxBuffer, 128, 2);
    NumCreated += OS_AddThread(&Idle, 128, 5);  // at lowest priority

    OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
    return 0;            // this never executes
}

//*******************Trampoline for selecting main to execute**********
int
main(void) {            // main
    PLL_Init(Bus80MHz);
    GPIO_Initialize();
    Launchpad_PortFInitialize();

    CAN0_Open(RCV_ID, XMT_ID);
    CANSendMotor(0, 0);

    HC12_Initialize();

    I2C3_Init(400000, 80000000);

    OPT3101_3_Init(7);
    OPT3101_3_Setup();
    OPT3101_3_CalibrateInternalCrosstalk();

    TxChannel_1 = 3;
    OPT3101_3_ArmInterrupts(&TxChannel_1, Distances_1, Amplitudes_1);
    OPT3101_3_StartMeasurementChannel(1);

    realmain();
}

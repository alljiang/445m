//*****************************************************************************
// Lab4.c - user programs, File system, stream data onto disk
//*****************************************************************************

// Jonathan W. Valvano, valvano@mail.utexas.edu
// Jan 12, 2020
// EE445M/EE380L.12 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file to do Lab 4
// as long as the basic functionality is simular
// 1) runs on your Lab 2 or Lab 3
// 2) implements your own eFile.c system with no code pasted in from other sources
// 3) streams real-time data from robot onto disk
// 4) supports multiple file reads/writes
// 5) has an interpreter that demonstrates features
// 6) interactive with UART input, and switch input

// LED outputs to logic analyzer for use by OS profile 
// PF1 is preemptive thread switch
// PF2 is first periodic task (DAS samples PE3)
// PF3 is second periodic task (PID)
// PC4 is PF4 button touch (SW1 task)

// IR distance sensors
// J5/A3/PE3 analog channel 0  <- connect an IR distance sensor to J5 to get a realistic analog signal on PE3
// J6/A2/PE2 analog channel 1  
// J7/A1/PE1 analog channel 2
// J8/A0/PE0 analog channel 3  <- connect an IR distance sensor to J8 to get a realistic analog signal on PE0

// Button inputs
// PF0 is SW2 task (Lab3)
// PF4 is SW1 button input

// Analog inputs
// PE0 Ain3 sequencer 3, channel 3, J8/PE0, 100 Hz, sampling in DAS(), software start
// PE3 Ain0 sequencer 0, channel 0, J5/PE3, 50 Hz, timer-triggered sampling, processed by Producer

//******Sensor Board I/O*******************
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) connected to PB0
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// HC-SR04 Ultrasonic Range Finder 
// J9X  Trigger0 to PB7 output (10us pulse)
// J9X  Echo0    to PB6 T0CCP0
// J10X Trigger1 to PB5 output (10us pulse)
// J10X Echo1    to PB4 T1CCP0
// J11X Trigger2 to PB3 output (10us pulse)
// J11X Echo2    to PB2 T3CCP0
// J12X Trigger3 to PC5 output (10us pulse)
// J12X Echo3    to PF4 T2CCP0

// Ping))) Ultrasonic Range Finder 
// J9Y  Trigger/Echo0 to PB6 T0CCP0
// J10Y Trigger/Echo1 to PB4 T1CCP0
// J11Y Trigger/Echo2 to PB2 T3CCP0
// J12Y Trigger/Echo3 to PF4 T2CCP0

// IR distance sensors
// J5/A0/PE3
// J6/A1/PE2
// J7/A2/PE1
// J8/A3/PE0  

// ESP8266
// PB1 Reset
// PD6 Uart Rx <- Tx ESP8266
// PD7 Uart Tx -> Rx ESP8266

// Free pins (debugging)
// PF3, PF2, PF1 (color LED)
// PD3, PD2, PD1, PD0, PC4

#include <stdint.h>
#include <string.h> 
#include <stdio.h>
#include <stdbool.h>
#include "vware/tm4c123gh6pm.h"
#include "vware/CortexM.h"
#include "vware/LaunchPad.h"
#include "vware/LPF.h"
#include "vware/IRDistance.h"
#include "vware/PLL.h"
#include "vware/ADCT0ATrigger.h"
#include "RTOS/OS.h"
#include "RTOS/Interpreter.h"
#include "RTOS/UART0int.h"
#include "RTOS/eDisk.h"
#include "RTOS/eFile.h"
#include "RTOS/ADC.h"
#include "drivers/ILI9341.h"
#include "drivers/HC12.h"
#include "utils/bit-utils.h"
#include "utils/uart-utils.h"
#include "utils/utils.h"

#include "gpio.h"
#include "launchpad.h"

#define TIMESLICE 2*TIME_1MS  // thread switch time in system time units

uint32_t NumCreated;   // number of foreground threads created

uint8_t rxBuffer[100];
uint8_t rxBufferStart;
uint8_t rxBufferLength;

uint32_t joystickH, joystickV, triggerL, triggerR;
bool btnA, btnB, btnStart;


/*
 * [0]: HEADER ( Range [0, 15] )
 * [1]: DATA
 * [2]: DATA
 * [3]: DATA
 * [4]: DATA
 * [5]: XOR CHECKSUM
 *
 * Remote -> Car
 * Heartbeat/Request Heartbeat: [0, 0x12, 0x34, 0x56, 0x78]
 * Set Motor Speed: [2, <Left Upper 8 bits>, <Left Lower 8 bits>, <Right Upper 8 bits>, <Right Lower 8 bits>
 * Request Sensor Data: [4, 0xAA, 0xBB, 0xCC, 0xDD]
 *
 * Car -> Remote
 * Heartbeat: [1, 0x9A, 0xBC, 0xDE, 0xF0]
 * Sensor Data (mm): [3, L, R, M, 0x00]
 */

int lastHeartbeat = -1;
uint8_t sensor_l = 0, sensor_r = 0, sensor_m = 0;

uint8_t isConnected;
bool heartbeatRequested, sensorRequested;
int heartbeatRequestTime, sensorRequestTime;
int lastMotorSendTime;

uint32_t joystickH_raw, joystickV_raw, triggerL_raw, triggerR_raw;
int leftMotor, rightMotor;

uint8_t heartbeat_data[4] = {0x12, 0x34, 0x56, 0x78};
uint8_t sensorRequest_data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
void
Request_Task(void) {
    int time;

    while (1) {
        time = OS_MsTime();
        heartbeatRequested = false;
        sensorRequested = false;
        if (!sensorRequested && !heartbeatRequested) {
            if (time - heartbeatRequestTime > 50) {
                heartbeatRequested = true;
                heartbeatRequestTime = time;

                HC12_SendData(0, heartbeat_data);
            } else if (time - lastMotorSendTime > 50) {
                lastMotorSendTime = time;

                if (triggerL_raw < 1000) rightMotor = 900;
                else rightMotor = 0;

                if (triggerR_raw > 1500) leftMotor = 900;
                else leftMotor = 0;

//                if (triggerL_raw < 1000) leftMotor = 900;
//                else leftMotor = 0;

//                if (triggerR_raw > 1500) rightMotor = 900;
//                else rightMotor = 0;

                if (!btnA && !btnB) {
                    leftMotor = 0;
                    rightMotor = 0;
                } else if (btnA) {
//                    leftMotor = -leftMotor;
//                    rightMotor = -rightMotor;
                    int temp = leftMotor;
                    leftMotor = -rightMotor;
                    rightMotor = -temp;
                }

                uint8_t motorData[] = {(uint8_t)(leftMotor >> 8),
                                        (uint8_t)(leftMotor & 0xFF),
                                        (uint8_t)(rightMotor >> 8),
                                        (uint8_t)(rightMotor & 0xFF)};
                HC12_SendData(2, motorData);
            }
        }

        isConnected = (time - lastHeartbeat < 1000) && (lastHeartbeat > -1);

        OS_Sleep(5);
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

            UART_OutChar(header);
            UART_OutChar(rxBuffer[(rxBufferStart + 1) % sizeof(rxBuffer)]);
            UART_OutChar(rxBuffer[(rxBufferStart + 2) % sizeof(rxBuffer)]);
            UART_OutChar(rxBuffer[(rxBufferStart + 3) % sizeof(rxBuffer)]);
            UART_OutChar(rxBuffer[(rxBufferStart + 4) % sizeof(rxBuffer)]);
            UART_OutChar('\r');
            UART_OutChar('\n');

            if (header == 1) {
                if (rxBuffer[(rxBufferStart + 1) % sizeof(rxBuffer)] == 0x9A &&
                   rxBuffer[(rxBufferStart + 2) % sizeof(rxBuffer)] == 0xBC &&
                   rxBuffer[(rxBufferStart + 3) % sizeof(rxBuffer)] == 0xDE &&
                   rxBuffer[(rxBufferStart + 4) % sizeof(rxBuffer)] == 0xF0) {
                    heartbeatRequested = false;
                    lastHeartbeat = OS_MsTime();
               }
            } else if (header == 3) {
                sensorRequested = false;
                sensor_l = rxBuffer[(rxBufferStart + 1) % sizeof(rxBuffer)];
                sensor_m = rxBuffer[(rxBufferStart + 2) % sizeof(rxBuffer)];
                sensor_r = rxBuffer[(rxBufferStart + 3) % sizeof(rxBuffer)];
            }

            rxBufferStart = (rxBufferStart + 6) % sizeof(rxBuffer); // increment start
            rxBufferLength -= 6;
        } else {
            OS_Sleep(5);
        }
    }
}

void
HumanInputsTask(void) {
    // BTN A: PB0
    // BTN B: PB1
    // BTN Start: PA5
    // Joystick H: PD0 (ADC CH7)
    // Joystick V: PD1 (ADC CH6)
    // Trigger L: PE1 (ADC CH2)
    // Trigger R: PE2 (ADC CH1)

    // Set buttons to inputs
    GPIO_PORTB_DEN_R = set_bit_field_u32(GPIO_PORTB_DEN_R, 0, 2, 0b11);
    GPIO_PORTB_DIR_R = set_bit_field_u32(GPIO_PORTB_DIR_R, 0, 2, 0b00);
    GPIO_PORTA_DEN_R = set_bit_field_u32(GPIO_PORTA_DEN_R, 5, 1, 0b1);
    GPIO_PORTA_DIR_R = set_bit_field_u32(GPIO_PORTA_DIR_R, 5, 1, 0b0);

    while (1) {
        ADC_Init(7);
        joystickH_raw = ADC_In();

        ADC_Init(6);
        joystickV_raw = ADC_In();

        ADC_Init(2);
        triggerL_raw = ADC_In();

        ADC_Init(1);
        triggerR_raw = ADC_In();

        btnA = !(GPIO_PORTB_DATA_R & generate_bit_mask_u32(0, 1));
        btnB = !(GPIO_PORTB_DATA_R & generate_bit_mask_u32(1, 1));
        btnStart = !(GPIO_PORTB_DATA_R & generate_bit_mask_u32(5, 1));

        joystickH = joystickH_raw;
        joystickV = joystickV_raw;
        triggerL = triggerL_raw;
        triggerR = triggerR_raw;

        OS_Sleep(20);
    }
}

char str_connected[] = "Connected: ";
char str_left[] = "Left: ";
char str_mid[] = "Mid: ";
char str_right[] = "Right: ";
char str_motorL[] = "MotorL: ";
char str_motorR[] = "MotorR: ";

void
ScreenDisplayTask(void) {
    int time;
    while (1) {
        time = OS_MsTime();
        ILI9341_Message(0, 1, str_connected, isConnected);
        ILI9341_Message(1, 0, str_left, sensor_l);
        ILI9341_Message(1, 1, str_mid, sensor_m);
        ILI9341_Message(1, 2, str_right, sensor_r);

        ILI9341_Message(0, 4, str_motorL, leftMotor);
        ILI9341_Message(0, 5, str_motorR, rightMotor);
        OS_Sleep(100);
    }
}

void
Idle(void) {
    while (1);
}

//*******************Lab 4 main **********
int
realmain(void) {        // lab 4 real main
    OS_Init();           // initialize, disable interrupts

    NumCreated = 0;

    // initialize communication channels
    OS_Fifo_Init(64);

    NumCreated += OS_AddThread(&Interpreter, 128, 4);
    NumCreated += OS_AddThread(&HumanInputsTask, 128, 3);
    NumCreated += OS_AddThread(&ScreenDisplayTask, 128, 4);
    NumCreated += OS_AddThread(&Request_Task, 128, 3);
//    NumCreated += OS_AddThread(&ProcessHC12RxBuffer, 128, 2);
    NumCreated += OS_AddThread(&Idle, 128, 5); // runs when nothing useful to do

    OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
    return 0;             // this never executes
}

//*******************Trampoline for selecting main to execute**********
int
main(void) {            // main
    PLL_Init(Bus80MHz);
    GPIO_Initialize();
    Launchpad_PortFInitialize();
    ILI9341_Init();
    HC12_Initialize();
    UART_Init();                              // serial I/O for interpreter

    realmain();
}

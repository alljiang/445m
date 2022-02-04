//*****************************************************************************
// Lab1.c - main program
//*****************************************************************************

// Allen & Sophia Jiang
// TA: Jeagun Jung
// Lab 1
// Created: 1/20/22
// Last revision: 2/2/22
// EE445M/EE380L.6 
// Simply put, develop a TM4C123 project with 
// 1) an interpreter running via the UART link to the PC, 
// 2) an LCD that has two logically separate displays implemented on one physical display, 
// 3) a periodic interrupt that maintains time, and 
// 4) an ADC device driver that collects data using a second periodic interrupt. 
// There are a lot of specifications outlined in this lab, however, 
// you are free to modify specifications as long as the above four components are implemented and understood. 

// IR distance sensors
// J5/A3/PE3 analog channel 0  
// J6/A2/PE2 analog channel 1 
// J7/A1/PE1 analog channel 2
// J8/A0/PE0 analog channel 3 <- connect an IR distance sensor to J8 to get a realistic analog signal on PE0

// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) unconnected
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// Analog inputs
// PE0 Ain3 sampled at 10Hz, sequencer 3, by DAS, using software start in ISR

#include <stdint.h>
#include <RTOS/ADC.h>
#include <RTOS/Interpreter.h>
#include <RTOS/OS.h>
#include <RTOS/ST7735.h>
#include <RTOS/UART0int.h>
#include "vware/tm4c123gh6pm.h"
#include "vware/CortexM.h"
#include "vware/LaunchPad.h"
#include "vware/LPF.h"
#include "vware/PLL.h"
#include "vware/Timer4A.h"
#include "vware/InterruptFunctions.h"
#include "vware/IRDistance.h"

int32_t ADCdata, FilterOutput, Distance;
uint32_t FilterWork;

// periodic task
void
DAStask(void) {  // runs at 10Hz in background
    PF1 ^= 0x02;
    ADCdata = ADC_In();  // channel set when calling ADC_Init
    PF1 ^= 0x02;
    FilterOutput = Median(ADCdata); // 3-wide median filter
    Distance = IRDistance_Convert(FilterOutput, 0);
    FilterWork++;        // calculation finished
    PF1 ^= 0x02;
}

//*******************lab 1 main **********
//int
//main(void) {
//    PLL_Init(Bus80MHz);
//    UART_Init();                              // serial I/O for interpreter
//    ST7735_InitR(INITR_GREENTAB);             // LCD initialization
//    LaunchPad_Init();                         // debugging profile on PF1
//    ADC_Init(3); // channel 3 is PE0 <- connect an IR distance sensor to J8 to get a realistic analog signal
//    Timer4A_Init(&DAStask, 80000000 / 10, 1);     // 10 Hz sampling, priority=1
//    OS_ClearMsTime();             // start a periodic interrupt to maintain time
//    EnableInterrupts();
//
//    ST7735_DrawFastHLine(0, 78, 128, 0xFFFF);
//
//    Interpreter();           // finally, launch interpreter, should never return
//}


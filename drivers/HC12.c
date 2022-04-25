// HC-12
// Uses UART1
//
// RX: C4
// TX: C5
// SET: D2

#include "HC12.h"
#include "vware/tm4c123gh6pm.h"
#include "vware/hw_memmap.h"
#include "vware/hw_types.h"
#include "vware/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/UART.h"
#include "driverlib/sysctl.h"

extern void
Delay1ms(uint32_t n);

const char AT_HEADER[] = "AT+";
const char AT_DEFAULT[] = "DEFAULT";
const char AT_BAUD[] = "B9600";

extern uint8_t rxBuffer[100];
extern uint8_t rxBufferLength;

extern void
ProcessHC12RxBuffer(void);

void
UART1_OutString(const char *str) {
    int i = 0;
    while (str[i]) {
        UARTCharPut(UART1_BASE, str[i++]);
    }
}

void
HC12_Flush() {
    while (UARTCharsAvail(UART1_BASE)) {
        UARTCharGetNonBlocking(UART1_BASE);
    }
}

void
HC12_EnableRXInts() {
//    HC12_Flush();
    UARTIntClear(UART1_BASE, UART_INT_RX);
    UARTIntEnable(UART1_BASE, UART_INT_RX);
}

void
HC12_DisableRXInts() {
    HC12_Flush();
    UARTIntClear(UART1_BASE, UART_INT_RX);
    UARTIntDisable(UART1_BASE, UART_INT_RX);
}

void
HC12_SendATCommand(const char *command) {
    UART1_OutString(AT_HEADER);
    UART1_OutString(command);
}

void
UART1_Handler() {
    UARTIntClear(UART1_BASE, UART_INT_RX);

    while (UARTCharsAvail(UART1_BASE)) {
        uint8_t rx = (uint8_t) UARTCharGet(UART1_BASE);
        rxBuffer[rxBufferLength++] = rx;

//        if (rx == 4) {  // EOT
//            // chk
//
//            ProcessHC12RxBuffer();
//
//            rxBufferLength = 0;
//        }
    }

    // Null terminator
    rxBuffer[rxBufferLength] = 0;
    ProcessHC12RxBuffer();
}

void
HC12_Initialize() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1));

    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE));

    UARTIntRegister(UART1_BASE, &UART1_Handler);
    HC12_EnableRXInts();

    UARTEnable(UART1_BASE);

    rxBufferLength = 0;

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);

    // Set to command mode
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);

    // Let HC12 boot up first
    Delay1ms(500);

    HC12_SendATCommand(AT_BAUD);

    Delay1ms(500);

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);

    uint8_t c = UARTCharGet(UART1_BASE);
    // Allow HC12 to exit command mode
    Delay1ms(200);

}

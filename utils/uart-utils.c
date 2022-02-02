
#include <stdint.h>
#include <stdbool.h>
#include "RTOS_Labs_common/UART0int.h"

void UART_OutStringNonBlock(char *pt) {
    while(*pt){
        UART_OutCharNonBlock(*pt);
        pt++;
    }
}

void UART_OutUDecNonBlock(uint32_t n) {
    if(n >= 10){
        UART_OutUDecNonBlock(n/10);
        n = n%10;
    }
    UART_OutCharNonBlock(n+'0'); /* n is between 0 and 9 */
}


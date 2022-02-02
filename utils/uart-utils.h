
#ifndef UTILS_UART_UTILS_H_
#define UTILS_UART_UTILS_H_

/*
 * Outputs string to UART without blocking
 */
void UART_OutStringNonBlock(char *pt);

/*
 * Outputs number to UART without blocking
 */
void UART_OutUDecNonBlock(uint32_t n);

#endif /* UTILS_UART_UTILS_H_ */

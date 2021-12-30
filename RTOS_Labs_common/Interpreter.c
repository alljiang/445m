// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 1/18/20, valvano@mail.utexas.edu
#include <stdint.h>
#include <stdlib.h>
#include <string.h> 
#include <stdio.h>
#include "vware/ADCT0ATrigger.h"
#include "vware/ADCSWTrigger.h"
#include "RTOS_Labs_common/OS.h"
#include "RTOS_Labs_common/ADC.h"
#include "RTOS_Labs_common/ST7735.h"
#include "RTOS_Labs_common/UART0int.h"
#include "RTOS_Labs_common/eDisk.h"
#include "RTOS_Labs_common/eFile.h"


#define EQ(a, b) (strcmp((a), (b)) == 0)

const char help[] = "--------------------------\n"
        "Invalid command. Available commands:\n"
        "0) - \n"
        "1) - \n"
        "2) - \n"
        "3) - \n"
        "4) - \n"
        "5) - \n"
        "6) - \n"
        "7) - \n"
        "8) - \n"
        "9) - \n"
        "--------------------------\n";

// Print jitter histogram
void
Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]) {
    // write this for Lab 3 (the latest)

}

/*
 * Returns pointer to the next token in the buffer (delimited by space or newline).
 * Also modifies the buffer and replaces the delimited space/newline with a null terminator for easier parsing.
 * Updates buffer to point to next next token, or NULL if no more tokens.
 *
 * Returns NULL if no token is found.
 */
char*
get_next_token(char **buffer) {
    char *str = *buffer;
    char *rv = str;
    uint16_t i = 0;

    // Null buffer was passed in, no more tokens
    if (str == NULL) {
        rv = NULL;
        goto exit;
    }

    while (str[i] != '\n' && str[i] != ' ') {
        i++;
    }

    if (str[i] == '\n') {
        // No more tokens, so replace buffer with NULL
        *buffer = NULL;
    } else {
        // Update buffer to point to next token
        *buffer = str + i + 1;
    }

    // Replace delimiter with null terminator
    str[i] = '\0';

exit:
    return rv;
}

int
get_next_token_as_int(char **buffer) {
    char *token = get_next_token(buffer);
    int rv;

    if (token == NULL) {
        rv = -1;
        goto exit;
    }
    rv = strtol(token, NULL, 10);

exit:
    return rv;
}

void
Interpreter_Parse(char *buffer) {
    char *token;
    token = get_next_token(&buffer);

    //  ADD NEW COMMANDS HERE
    if (EQ("0", token)) {
        // Test timer
        OS_ClearMsTime();
    } else if (EQ("1", token)) {
        // Test timer
        UART_OutUDec((uint32_t)OS_MsTime());
    } else if (EQ("2", token)) {

    } else if (EQ("3", token)) {

    } else if (EQ("4", token)) {

    } else if (EQ("5", token)) {

    } else if (EQ("6", token)) {

    } else if (EQ("7", token)) {

    } else if (EQ("8", token)) {

    } else if (EQ("9", token)) {

    } else if (EQ("lcd", token)) {
        uint8_t screen = get_next_token_as_int(&buffer);
        uint8_t row = get_next_token_as_int(&buffer);
        char* str = get_next_token(&buffer);
        ST7735_Message(screen, row, str, 0);
    } else if (EQ("adc", token)) {
        char voltage_formatted_str[6];

        float adc_voltage = ADC_In_Voltage();
        sprintf(voltage_formatted_str, "%0.2fV", adc_voltage);
        UART_OutString((char*) voltage_formatted_str);
    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else {
        // invalid command, print help info
        UART_OutString((char*) help);
    }
}

// *********** Command line interpreter (shell) ************
void
Interpreter(void) {
    // write this
    char input;
    char buffer[50];
    uint8_t buffer_index = 0;

    while (1) {
        input = UART_InChar();
        UART_OutChar(input);

        if (input != '\n' || buffer_index >= sizeof(buffer)) {
            // still adding to buffer
            buffer[buffer_index] = input;
            buffer_index++;
        } else {
            // add null terminator to buffer
            buffer[buffer_index] = 0;
            buffer_index = 0;

            // parse buffer for valid command
            Interpreter_Parse(buffer);
        }

    }
}

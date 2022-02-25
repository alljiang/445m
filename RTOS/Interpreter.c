// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 1/18/20, valvano@mail.utexas.edu
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <RTOS/ADC.h>
#include <RTOS/eDisk.h>
#include <RTOS/eFile.h>
#include <RTOS/OS.h>
#include <RTOS/ST7735.h>
#include <RTOS/UART0int.h>
#include "vware/ADCT0ATrigger.h"
#include "vware/ADCSWTrigger.h"
#include <str-utils.h>
#include <uart-utils.h>

#define EQ(a, b) (strcmp((a), (b)) == 0)

volatile bool printed_Prompt;

const char help[] = "--------------------------\r\n"
        "Invalid command. Available commands:\r\n"
        "0) adc - Prints voltage readout of PE0\r\n"
        "1) lcd <display, 0/1> <line, 0-7> <word> - Prints line to LCD\r\n"
        "2) numcreated - Prints number of threads created\r\n"
        "3) maxjitter - Prints maximum time jitter\r\n"
        "4) - \r\n"
        "5) - \r\n"
        "6) - \r\n"
        "7) - \r\n"
        "8) - \r\n"
        "9) - \r\n"
        "--------------------------\r\n";

// Print jitter histogram
void
Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]) {
    // write this for Lab 3 (the latest)
    UART_OutString("\r\nMaxJitter: ");
    UART_OutUDec((uint32_t) MaxJitter);
    UART_OutString("\n\r");
    for(int i = 0; i < JitterSize; i++) {
        if(JitterHistogram[i] == 0) continue;
        UART_OutUDec((uint32_t) i);
        if(i == JitterSize-1) {
            UART_OutString("+: ");
        } else {
            UART_OutString(": ");
        }
        UART_OutUDec((uint32_t) JitterHistogram[i]);
        UART_OutString("\r\n");
    }
    UART_OutString("\r\n");
}

/*
 * Returns pointer to the next token in the buffer (delimited by space or newline).
 * Also modifies the buffer and replaces the delimited space/newline with a null terminator for easier parsing.
 * Updates buffer to point to next next token, or NULL if no more tokens.
 *
 * Returns NULL if no token is found.
 */
char*
getNextToken(char **buffer) {
    char *str = *buffer;
    char *rv = str;
    uint16_t i = 0;

    // Null buffer was passed in, no more tokens
    if (str == NULL) {
        rv = NULL;
        goto exit;
    }

    while (str[i] != '\n' && str[i] != ' ' && str[i] != 0) {
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
getNextTokenAsInt(char **buffer) {
    char *token = getNextToken(buffer);
    int rv;

    if (token == NULL) {
        rv = -1;
        goto exit;
    }
    rv = strtol(token, NULL, 10);

exit:
    return rv;
}

extern uint32_t NumCreated;
extern uint32_t MaxJitter;
void
Interpreter_Parse(char *buffer) {
    char *token;
    token = getNextToken(&buffer);

    printed_Prompt = false;

    //  ADD NEW COMMANDS HERE
    if (EQ("0", token)) {
        // Test timer
        OS_ClearMsTime();
    } else if (EQ("1", token)) {
        // Test timer
        UART_OutUDecNonBlock((uint32_t) OS_MsTime());
        UART_OutStringNonBlock("\r\n");
    } else if (EQ("2", token)) {

    } else if (EQ("3", token)) {

    } else if (EQ("4", token)) {

    } else if (EQ("5", token)) {

    } else if (EQ("6", token)) {

    } else if (EQ("7", token)) {

    } else if (EQ("8", token)) {

    } else if (EQ("9", token)) {

    } else if (EQ("lcd", token)) {
        uint8_t screen = getNextTokenAsInt(&buffer);
        uint8_t row = getNextTokenAsInt(&buffer);
        char *str = getNextToken(&buffer);
        ST7735_Message(screen, row, str, 0);
    } else if (EQ("adc", token)) {
        char voltage_formatted_str[6];
        char itoa_buf[3];

        float adc_voltage = ADC_In_Voltage();
        int fixed_pt = adc_voltage * 100;
        itoa(fixed_pt, itoa_buf);

        for (int i = 0; i < 3; i++) {
            if (itoa_buf[i] == 0 || itoa_buf[i] > '9' || itoa_buf[i] < '0') {
                voltage_formatted_str[i] = '0';
            } else {
                voltage_formatted_str[i] = itoa_buf[i];
            }
        }

        // scoot decimal digits down to make room for decimal point
        voltage_formatted_str[3] = voltage_formatted_str[2];
        voltage_formatted_str[2] = voltage_formatted_str[1];
        voltage_formatted_str[1] = '.';
        voltage_formatted_str[4] = 'V';
        voltage_formatted_str[5] = 0;

        UART_OutStringNonBlock(voltage_formatted_str);
        UART_OutStringNonBlock("\r\n");
    } else if (EQ("numcreated", token)) {
        char itoa_buf[5];
        itoa(NumCreated, itoa_buf);
        UART_OutStringNonBlock(itoa_buf);
        UART_OutStringNonBlock("\r\n");
    } else if (EQ("maxjitter", token)) {
        char itoa_buf[5];
        itoa(MaxJitter, itoa_buf);
        UART_OutStringNonBlock(itoa_buf);
        UART_OutStringNonBlock("\r\n");
    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else {
        // invalid command, print help info
        UART_OutStringNonBlock((char*) help);
    }
}

// *********** Command line interpreter (shell) ************
void
Interpreter(void) {
    // write this
    char buffer[30];
    char input;
    uint8_t buffer_index = 0;
    printed_Prompt = false;

    while (1) {
        if (!printed_Prompt) {
            UART_OutCharNonBlock('>');
            UART_OutCharNonBlock(' ');
            printed_Prompt = true;
        }
        do {
            input = UART_InCharNonBlock();
        } while (input == 0);
        UART_OutCharNonBlock(input);

        if (input == '\r') UART_OutCharNonBlock('\n');

        if (input != '\r' || buffer_index >= sizeof(buffer) - 1) {
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

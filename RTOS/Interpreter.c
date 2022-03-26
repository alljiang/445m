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
#include "utils/str-utils.h"
#include "utils/uart-utils.h"

#define EQ(a, b) (strcmp((a), (b)) == 0)

volatile bool printed_Prompt;

const char help[] = "--------------------------\r\n"
        "Invalid command. Available commands:\r\n"
        " 0) adc - Prints voltage readout of PE0\r\n"
        " 1) lcd <display, 0/1> <line, 0-7> <word> - Prints line to LCD\r\n"
        " 2) numcreated - Prints number of threads created\r\n"
        " 3) maxjitter - Prints maximum time jitter\r\n"
        " 4) ls - display SD card directory\r\n"
        " 5) format - format SD card\r\n"
        " 6) read <filename> - prints content of given filename\r\n"
        " 7) rm <filename> - deletes file\r\n"
        " 8) wopen <filename> - opens file for writing\r\n"
        " 9) write <data> - writes data to open file\r\n"
        "10) wclose - closes file from writing\r\n"
        "11) create <filename> - creates file\r\n"
        "12) wspam - writes 600 characters to open file\r\n"
        "13) - \r\n"
        "14) - \r\n"
        "15) - \r\n"
        "16) - \r\n"
        "17) - \r\n"
        "--------------------------\r\n";

// Print jitter histogram
void
Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]) {
    // write this for Lab 3 (the latest)
    UART_OutString("\r\nMaxJitter: ");
    UART_OutUDec((uint32_t) MaxJitter);
    UART_OutString("\n\r");
    for (int i = 0; i < JitterSize; i++) {
        if (JitterHistogram[i] == 0) continue;
        UART_OutUDec((uint32_t) i);
        if (i == JitterSize - 1) {
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

const char str_lcd[] = "lcd";
const char str_adc[] = "adc";
const char str_numcreated[] = "numcreated";
const char str_maxjitter[] = "maxjitter";
const char str_ls[] = "ls";
const char str_format[] = "format";
const char str_read[] = "read";
const char str_rm[] = "rm";
const char str_wopen[] = "wopen";
const char str_write[] = "write";
const char str_wclose[] = "wclose";
const char str_create[] = "create";
const char str_wspam[] = "wspam";
const char str_newline[] = "\r\n";

const char msg_format_success[] =
        "Successfully formatted SD card\r\n";
const char msg_format_fail[] =
        "Format SD card failed\r\n";
const char msg_read_fail[] =
        "Could not open file for reading\r\n";
const char msg_delete_fail[] =
        "Failed to delete file\r\n";
const char msg_delete_success[] =
        "Successfully deleted file\r\n";
const char msg_wopen_success[] =
        "Successfully opened file for writing\r\n";
const char msg_wopen_fail[] =
        "Failed to open file for writing\r\n";
const char msg_write_fail[] =
        "Failed to write file\r\n";
const char msg_write_success[] =
        "Write file success\r\n";
const char msg_wclose_success[] =
        "Successfully closed file from writing\r\n";
const char msg_wclose_fail[] =
        "Failed to close file from writing\r\n";
const char msg_create_success[] =
        "Successfully created file\r\n";
const char msg_create_fail[] =
        "Failed to create file\r\n";
const char msg_spam_success[] =
        "File spam success\r\n";

extern uint32_t NumCreated;
extern uint32_t MaxJitter;
void
Interpreter_Parse(char *buffer) {
    int rv;
    char *token;
    token = getNextToken(&buffer);

    printed_Prompt = false;

    //  ADD NEW COMMANDS HERE
    if (EQ(str_lcd, token)) {
        uint8_t screen = getNextTokenAsInt(&buffer);
        uint8_t row = getNextTokenAsInt(&buffer);
        char *str = getNextToken(&buffer);
        ST7735_Message(screen, row, str, 0);
    } else if (EQ(str_adc, token)) {
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
        UART_OutStringNonBlock((char*) str_newline);
    } else if (EQ(str_numcreated, token)) {
        char itoa_buf[5];
        itoa(NumCreated, itoa_buf);
        UART_OutStringNonBlock(itoa_buf);
        UART_OutStringNonBlock((char*) str_newline);
    } else if (EQ(str_maxjitter, token)) {
        char itoa_buf[5];
        itoa(MaxJitter, itoa_buf);
        UART_OutStringNonBlock(itoa_buf);
        UART_OutStringNonBlock((char*) str_newline);
    } else if (EQ(str_ls, token)) {
        //eFile_PrintDirectory(&UART_OutStringNonBlock);
    } else if (EQ(str_format, token)) {
        rv = eFile_Format();
        if (rv == 0) {
            UART_OutStringNonBlock((char*) msg_format_success);
        } else {
            UART_OutStringNonBlock((char*) msg_format_fail);
        }
    } else if (EQ(str_read, token)) {
        char *filename = getNextToken(&buffer);

        rv = eFile_ROpen(filename);
        if (rv != 0) {
            UART_OutStringNonBlock((char*) msg_read_fail);
        } else {
           char c;
           rv = eFile_ReadNext(&c);
           while (rv == 0) {
               UART_OutCharNonBlock(c);
               rv = eFile_ReadNext(&c);
           }
           eFile_RClose();
           UART_OutStringNonBlock((char*) str_newline);
        }
    } else if (EQ(str_rm, token)) {
        char *filename = getNextToken(&buffer);

        rv = eFile_Delete(filename);
        if (rv != 0) {
            UART_OutStringNonBlock((char*) msg_delete_fail);
        } else {
            UART_OutStringNonBlock((char*) msg_delete_success);
        }

    } else if (EQ(str_wopen, token)) {
        char *filename = getNextToken(&buffer);

        rv = eFile_WOpen(filename);
        if (rv == 0) {
            UART_OutStringNonBlock((char*) msg_wopen_success);
        } else {
            UART_OutStringNonBlock((char*) msg_wopen_fail);
        }
    } else if (EQ(str_write, token)) {
        int index = 0;
        char c = *(buffer + index++);
        while (c != 0) {
            rv = eFile_Write(c);
            if (rv != 0) {
                UART_OutStringNonBlock((char*) msg_write_fail);
                break;
            }

            c = *(buffer + index++);
        }
        if (rv == 0) UART_OutStringNonBlock((char*) msg_write_success);
    } else if (EQ(str_wclose, token)) {
        rv = eFile_WClose();
        if (rv == 0) {
            UART_OutStringNonBlock((char*) msg_wclose_success);
        } else {
            UART_OutStringNonBlock((char*) msg_wclose_fail);
        }
    } else if (EQ(str_create, token)) {
        char *filename = getNextToken(&buffer);

        rv = eFile_Create(filename);
        if (rv == 0) {
            UART_OutStringNonBlock((char*) msg_create_success);
        } else {
            UART_OutStringNonBlock((char*) msg_create_fail);
        }
    } else if (EQ(str_wspam, token)) {
        for (int i = 0; i < 600; i++) {
            rv = eFile_Write('e');
            if (rv != 0) {
                UART_OutStringNonBlock((char*) msg_write_fail);
                break;
            }
        }
        if (rv == 0) UART_OutStringNonBlock((char*) msg_spam_success);
    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

    } else if (EQ("", token)) {

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
    char input;
    char buffer[40];
    uint8_t buffer_index = 0;
    printed_Prompt = false;

    while (1) {
        if (!printed_Prompt) {
            UART_OutStringNonBlock("\r\n> ");
            printed_Prompt = true;
        }
        do {
            input = UART_InCharNonBlock();
        } while (input == 0);
        UART_OutCharNonBlock(input);

        if (input == '\r') UART_OutCharNonBlock('\n');


        if (buffer_index >= sizeof(buffer) - 1) {
            buffer_index = 0;
            printed_Prompt = false;
            continue;
        } else if (input != '\r') {
            // still adding to buffer
            buffer[buffer_index] = input;
            buffer_index++;
        } else {
            // add null terminator to buffer
            memset(buffer + buffer_index, 0, 40 - buffer_index);

            // parse buffer for valid command
            Interpreter_Parse(buffer);

            buffer_index = 0;
        }

    }
}

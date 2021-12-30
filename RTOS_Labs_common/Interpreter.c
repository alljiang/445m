// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 1/18/20, valvano@mail.utexas.edu
#include <stdint.h>
#include <string.h> 
#include <stdio.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "vware/ADCT0ATrigger.h"
#include "vware/ADCSWTrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"


#define EQ(a, b) (strcmp((a), (b)) == 0)


const char help[] = "--------------------------\n"
                    "Invalid command. Available commands:\n"
                    "1) - \n"
                    "2) - \n"
                    "--------------------------\n";


// Print jitter histogram
void Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]){
    // write this for Lab 3 (the latest)

}

void Interpreter_Parse(char* buffer) {
    //  ADD NEW COMMANDS HERE
    if(EQ(buffer, "1")) {

    } else if(EQ(buffer, "2")) {

    } else {
        // invalid command, print help info
        UART_OutString((char*)help);
    }
}

// *********** Command line interpreter (shell) ************
void Interpreter(void){ 
    // write this
    char input;
    char buffer[50];
    uint8_t buffer_index = 0;

    while(1) {
        input = UART_InChar();
        UART_OutChar(input);

        if(input != '\n' || buffer_index >= sizeof(buffer)) {
            //  still adding to buffer
            buffer[buffer_index] = input;
            buffer_index++;
        } else {
            //  add null terminator to buffer
            buffer[buffer_index] = 0;
            buffer_index = 0;

            //  parse buffer for valid command
            Interpreter_Parse(buffer);
        }

    }
}



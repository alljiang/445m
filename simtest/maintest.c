#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "eFile.h"
#include "sim-compat.h"

void UART_OutString(char *str) {
    printf("%s\n", str);
}

int main() {
    int rv = 0;

    rv = eFile_Init();
    if (rv == 1) printf("Bad\n");

    eFile_Mount();

    rv = eFile_Format();
    if (rv == 1) printf("Bad\n");

    rv = eFile_Create("asdf");
    if (rv == 1) printf("Bad\n");
    
    rv = eFile_Create("asdf");
    if (rv == 1) printf("Bad2\n");
    
    rv = eFile_WOpen("asdf");
    if (rv == 1) printf("Bad\n");
    
    rv = eFile_Write('a');
    if (rv == 1) printf("Bad\n");

    eFile_PrintDirectory(&UART_OutString);

    return 0;
}
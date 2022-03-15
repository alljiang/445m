#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "eFile.h"

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
    
    eFile_Mount();

    rv = eFile_Create("aa.txt");
    if (rv == 1) printf("Bad\n");
    
    eFile_Mount();

    rv = eFile_Create("bbb.txt");
    if (rv == 1) printf("Bad\n");
    
    eFile_Mount();

    rv = eFile_WOpen("aa.txt");
    if (rv == 1) printf("Bad\n");

    for (int i = 0; i < 600; i++) {
        rv = eFile_Write('a');
        if (rv == 1) printf("Bad\n");
    }
    rv = eFile_WClose();
    if (rv == 1) printf("Bad\n");

    rv = eFile_ROpen("aa.txt");
    for (int i = 0; i < 601; i++) {
        char c;
        rv = eFile_ReadNext(&c);
        if (rv == 1) printf("\n\nEOF at %d\n", i);
        else printf("%c", c);
    }
    printf("\n");
    rv = eFile_RClose();
    if (rv == 1) printf("Bad\n");

    rv = eFile_ROpen("aa.txt");
    for (int i = 0; i < 601; i++) {
        char c;
        rv = eFile_ReadNext(&c);
        if (rv == 1) printf("\n\nEOF at %d\n", i);
        else printf("%c", c);
    }
    printf("\n");
    rv = eFile_RClose();
    if (rv == 1) printf("Bad\n");

    eFile_PrintDirectory(&UART_OutString);

    printf("Deleting aa.txt\n");

    rv = eFile_Delete("aa.txt");

    eFile_PrintDirectory(&UART_OutString);

    return 0;
}
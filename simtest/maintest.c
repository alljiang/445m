#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "eFile.h"

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

    return 0;
}
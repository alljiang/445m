#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "eDisk.h"

BYTE disk[2*1024*1024];

DRESULT eDisk_ReadBlock (BYTE *buff, DWORD sector) {
    memcpy(buff, disk + sector * 512, 512);
    return 0;
}

DRESULT eDisk_WriteBlock (const BYTE *buff, DWORD sector) {
    memcpy(disk + sector * 512, buff, 512);
    return 0;
}

DSTATUS eDisk_Init(BYTE drive) {
    memset(disk, 0, sizeof(disk));
}
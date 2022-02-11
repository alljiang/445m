
#include <stdint.h>
#include <stdbool.h>
#include "InterruptFunctions.h"
#include "drivers/launchpad.h"

void DisableInterrupts(void) {
    asm("        CPSID  I");
}

void EnableInterrupts(void) {
    asm("        CPSIE  I");
}

long StartCritical (void) {
    asm("        MRS    R0, PRIMASK");
    asm("        CPSID  I");
    asm("        BX  LR");
    return -1;  // should not reach here
}

void EndCritical(long sr) {
    asm("        MSR    PRIMASK, R0");
}

void WaitForInterrupt(void) {
    asm("        WFI");
}

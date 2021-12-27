
void DisableInterrupts(void) {
    asm("        CPSID  I");
}

void EnableInterrupts(void) {
    asm("        CPSIE  I");
}

long StartCritical (void) {
    asm("        MRS    R0, PRIMASK");
    asm("        CPSID  I");
}

void EndCritical(long sr) {
    asm("        MSR    PRIMASK, R0");
}

void WaitForInterrupt(void) {
    asm("        WFI");
}

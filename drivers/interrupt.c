#include <stdint.h>
#include "bit-utils.h"
#include "interrupt.h"
#include "vware/tm4c123gh6pm.h"

void
Interrupt_Enable(uint8_t interruptNum) {
    if (interruptNum > 138) {
        return;
    }

    volatile uint32_t *nvicEnableBase = &NVIC_EN0_R;
    uint8_t registerOffset = interruptNum / 32u;
    volatile uint32_t *nvicEnableReg = nvicEnableBase + registerOffset * 0x4;

    uint8_t bitOffset = interruptNum % 32;

    *nvicEnableReg = set_bit_field_u32(*nvicEnableReg, bitOffset, 1u, 1u);
}

void
Interrupt_SetPriority(uint8_t interruptNum, uint8_t priority) {
    if (interruptNum > 138 || priority > 7) {
        return;
    }

    // TM4C123GH6PM Datasheet: Page 152
    volatile uint32_t *nvicPriorityBase = &NVIC_PRI0_R;
    uint8_t registerOffset = interruptNum / 4;
    volatile uint32_t *nvicPriorityReg = nvicPriorityBase
            + registerOffset * 0x4;

    uint8_t bitFieldIndex = interruptNum % 4;
    uint8_t bitOffset = 8 * bitFieldIndex + 5;

    *nvicPriorityReg = set_bit_field_u32(*nvicPriorityReg, bitOffset, 3,
            priority);

    return;
}

void
Interrupt_SetSystemPriority(uint8_t interruptNum, uint8_t priority) {
    /*
     * 1: Reset
     * 2: NMI
     * 3: Hard Fault
     * 4: MPU Fault
     * 5: Bus Fault
     * 6: Usage Fault
     * 7..10: Reserved
     * 11: SVCall
     * 12: Debug Monitor
     * 13: Reserved
     * 14: PendSV
     * 15: SysTick
     */

    if (interruptNum > 15 || priority > 7) {
        return;
    }

    // TM4C123GH6PM Datasheet: Page 170-172
    volatile uint32_t *nvicPriorityBase = &NVIC_SYS_PRI1_R;
    uint8_t registerOffset = interruptNum / 4 - 1;

    volatile uint32_t *nvicPriorityReg = nvicPriorityBase
            + registerOffset * 0x4;

    uint8_t bitFieldIndex = interruptNum % 4;
    uint8_t bitOffset = 8 * bitFieldIndex + 5;

    *nvicPriorityReg = set_bit_field_u32(*nvicPriorityReg, bitOffset, 3,
            priority);

    return;
}

/*
 * bit_utils.c
 *
 *  Created on: Dec 28, 2021
 *      Author: Allen
 */

#include <bit-utils.h>

/*
 * Replaces a field within a container and returns the new value
 */
uint32_t
set_bit_field_u32(uint32_t container, uint8_t fieldLSBIndex,
        uint8_t fieldLength, uint32_t replacementBits) {
    uint32_t mask = 0u;
    uint8_t i = fieldLength;

    //  create bit mask
    while (i-- > 0u) {
        mask = (mask << 1u) | 1u;
    }
    mask <<= fieldLSBIndex;

    container &= ~mask;
    container |= replacementBits << fieldLSBIndex;

    return container;
}


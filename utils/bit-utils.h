/*
 * bit_utils.h
 *
 *  Created on: Dec 28, 2021
 *      Author: Allen
 */

#ifndef UTILS_BIT_UTILS_H_
#define UTILS_BIT_UTILS_H_

#include <stdint.h>

/*
 * Replaces a field within a container and returns the new value
 */
uint32_t
set_bit_field_u32(uint32_t container, uint8_t fieldLSBIndex,
        uint8_t fieldLength, uint32_t replacementBits);

#endif /* UTILS_BIT_UTILS_H_ */

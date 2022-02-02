/*
 * str_utils.h
 *
 *  Created on: Jan 26, 2022
 *      Author: Allen
 */

#ifndef UTILS_STR_UTILS_H_
#define UTILS_STR_UTILS_H_

/*
 * Changes a number of variable length to a character.
 * Must pass in a char buffer that is long enough for
 * the value.
 */
int
itoa(int value,char *ptr);

#endif /* UTILS_STR_UTILS_H_ */

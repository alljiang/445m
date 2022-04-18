/*
 * delay-utils.h
 *
 *  Created on: Apr 6, 2022
 *      Author: Allen
 */

#ifndef UTILS_UTILS_H_
#define UTILS_UTILS_H_

void
delayMicroseconds(int microseconds);

long
map(long x, long in_min, long in_max, long out_min, long out_max, int scale);

int inline
limit(int num, int min, int max);

#endif /* UTILS_UTILS_H_ */

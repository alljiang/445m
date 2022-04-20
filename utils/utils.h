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

int inline
min(int a, int b);

int inline
max(int a, int b);

int inline
addMagnitude(int num, int addition);

#endif /* UTILS_UTILS_H_ */

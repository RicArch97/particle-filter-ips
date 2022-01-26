/* 
 * MicroStorm - BLE Tracking
 * include/util.h
 *
 * Copyright (c) 2021 Ricardo Steijn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <math.h>

#define clampf(v, minv, maxv)   (fmaxf(fminf(maxv, v), minv))
#define clampaf(a)              (fmodf((a), (2 * M_PI)) + ((a) < 0 ? (2 * M_PI) : 0))

#define US_TO_S(us)             (us / 1000000)  

unsigned long ble_util_mix(unsigned long a, unsigned long b, unsigned long c);
int ble_util_sample(int states);
float ble_util_sample_range(float min, float max);
float *ble_util_corput(int set_size, int base);
int *ble_util_prime_sieve(int set_size);
float ble_util_scale(float x, float a, float b, float c, float d);
float ble_util_timedelta(int64_t *start_us);

#endif
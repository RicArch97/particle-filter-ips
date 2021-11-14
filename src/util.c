/* 
 * MicroStorm - BLE Tracking
 * src/util.c
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>

/**
 * \brief Return a random float value between a range.
 * 
 * \param min Minimum number of the range.
 * \param max Maximum number of the range.
 * 
 * \return Random float between a range.
 */
float ble_util_rand_float(float min, float max)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    srand((tv.tv_usec ^ tv.tv_sec));
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

/**
 * \brief Return a van der corput sequence for a set size N.
 * https://en.wikipedia.org/wiki/Van_der_Corput_sequence
 * 
 * \param set_size Amount of numbers to be calculated.
 * 
 * \return Pointer to a dynmically allocated array with the values.
 * Returns NULL on error.
 */
float *ble_util_corput(int set_size, int base)
{
    float *sequence = malloc(set_size * sizeof(float));
    if (sequence == NULL)
        return NULL;
    // van der corput sequence
    for (int i = 0; i < set_size; i++) {
        int n = i;
        float q = 0, bk = (float)1/base;
        while (n > 0) {
            q += (n % base) * bk;
            n /= base;
            bk /= base;
        }
        sequence[i] = q;  
    }
    return sequence;
}

/**
 * \brief Calculate N prime numbers using Sieve of Eratosthenes.
 * This has a time complexity of O(n * log(log(n))) instead of O(n^2).
 * 
 * \param set_size Amount of numbers to be calculated.
 * 
 * \return Pointer to a dynamically allocated array with the values.
 * Returns NULL on error.
 */
int *ble_util_prime_sieve(int set_size) 
{
    int n = 10, count = 0, start = 2;
    int *primes = malloc(set_size * sizeof(int));
    uint8_t *prime = malloc((n + 1) * sizeof(uint8_t));
    if (primes == NULL || prime == NULL)
        return NULL;
    // set all values initially to 1 (true)
    memset(prime, 1, ((n + 1) * sizeof(uint8_t)));

    while (count < set_size) {
        for (int i = 2; i * i <= n; i++) {
            if (prime[i] == 1) {
                for (int j = i * i; j <= n; j += i) {
                    prime[j] = 0;
                }
            }
        }
        for (int i = start; i <= n; i++) {
            if (count == set_size)
                break;
            if (prime[i]) {
                primes[count++] = i;
            }
        }
        // we need more numbers
        if (count < set_size) {
            start = n;
            free(prime);
            n += 50;
            prime = malloc((n + 1) * sizeof(uint8_t));
            // return NULL instead of breaking since higher level
            // functions may rely on getting the correct amount
            if (prime == NULL)
                return NULL;
            memset(prime, 1, ((n + 1) * sizeof(uint8_t)));
        }
    }
    return primes;
}

/**
 * \brief Scale a value from one range to another range.
 * 
 * \param x Value.
 * \param a Lower bound of range 1
 * \param b Upper bound of range 1
 * \param c Lower bound of range 2
 * \param d Upper bound of range 2
 * 
 * \return Value mapped in the given range.
 */
float ble_util_scale(float x, float a, float b, float c, float d)
{
    return c + ((x - a) * (d - c) / (b - a));
}
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

#include <math.h>

#include "util.h"

/**
 * \brief Calculate the RSSI from dBm to meters.
 * 
 * \param rssi Measured rssi value.
 * \param tx_power Received power at 1 meters in dBm.
 * 
 * \return Measured distance in meters.
 */
double ble_util_rssi_to_meters(int rssi, int8_t tx_power)
{
    // RSSI = -10*n*log10(d/d0) + A0
    // with d0 measured at 1 meter:
    // d = 10^((A-RSSI)/(10*n))
    return pow(10, ((double)(tx_power - rssi) / (10 * BLE_ENV_FACTOR_IND)));
}
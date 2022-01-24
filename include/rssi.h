/* 
 * MicroStorm - BLE Tracking
 * include/rssi.h
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

#ifndef RSSI_H
#define RSSI_H

#define TX_POWER_ONE_METER  -60
#define SIGNAL_LOSS         41

#define BLE_ENV_FACTOR_IND  2   

// Kalman filter tuning parameters
// P is calculated every iteration and is initialized at 1
// R is the standard deviation in db in stable state
// Q is a factor determinated by trial and error
// The Q factor determines the responsiveness of the filter
#define ERROR_VARIANCE_P    1
#define MEASUREMENT_NOISE_R 20
#define PROCESS_NOISE_Q     0.005 

typedef struct {
    float state;
    float p_noise;
    float m_noise;
    float err_v;
} ble_rssi_state_t;

void ble_rssi_update(int measurement);

#endif
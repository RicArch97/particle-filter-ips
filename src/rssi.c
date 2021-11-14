/* 
 * MicroStorm - BLE Tracking
 * src/rssi.c
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

#include <stdio.h>
#include <math.h>

#include "rssi.h"

/**
 * \brief Calculate a new state from old state & Kalman gain.
 * 
 * \param s RSSI state data.
 * \param m Raw RSSI value in meters.
 */
void ble_rssi_kf_estimate(ble_rssi_state_t *s, float m)
{
    // error variance prediction
    // P' = P(t-1) + Q
    float err_v_p = s->err_v + s->p_noise;
    // calculate Kalman gain
    // K = P' / (P' + R)
    float k = err_v_p / (err_v_p + s->m_noise);
    // calculate next state
    // S = S'+ K * (m - S')
    float next_s = s->state + (k * (m - s->state));
    // calculate next error variance
    // P = (I - K) * P'
    float next_err_v = (1 - k) * err_v_p;

    // TODO
    // is there a way the measurement and process noise can be calculated dynamically?

    // update state estimate & error variance
    s->state = next_s;
    s->err_v = next_err_v;
}

/**
 * \brief Calculate the RSSI from dBm to meters.
 * 
 * \param rssi Measured rssi value.
 * \param tx_power Received power at 1 meters in dBm.
 * 
 * \return Measured distance in meters.
 */
float ble_rssi_to_meters(int rssi, int8_t tx_power)
{
    // RSSI = -10 * n * log10(d / d0) + A0
    // with d0 measured at 1 meter:
    // d = 10^((A - RSSI) / (10 * n))
    return powf(10, (float)(tx_power - rssi) / (10 * BLE_ENV_FACTOR_IND));
}

/**
 * \brief Process a new RSSI measurement.
 * 
 * \param measurement Measured RSSI value.
 */
void ble_rssi_update(int measurement)
{
    static ble_rssi_state_t rssi = {0};

    // initialization
    if (rssi.m_noise == 0) {
        rssi.state = measurement;
        rssi.err_v = 1;
        rssi.m_noise = 20;
        rssi.p_noise = 0.0005;
    }
    // smooth value using Kalman filter
    ble_rssi_kf_estimate(&rssi, (float)measurement);
    
    // distance calculation
    float rssi_m = ble_rssi_to_meters(rssi.state, TX_POWER_ONE_METER);
    
    // TODO
    // wrap value in mesh message and send to mesh
}

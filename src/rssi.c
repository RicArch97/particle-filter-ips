/* 
 * MicroStorm - BLE Tracking
 * src/rssi.c
 *
 * Copyright (c) 2022 Ricardo Steijn
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

#include <stdint.h>
#include <math.h>

#include "rssi.h"
#include "util.h"
#include "mqtt.h"
#include "particle.h"
#include "config.h"

/**
 * \brief Calculate a new state from old state & Kalman gain.
 * 
 * \param s RSSI state data.
 * \param m Raw RSSI value in meters.
 */
static void 
ble_rssi_kf_estimate(ble_rssi_state_t *s, float m)
{
    // error variance prediction
    // P' = P(t-1) + Q
    float err_v_p = s->err_v + s->p_noise;
    // calculate Kalman gain
    // K = P' / (P' + R)
    float k = err_v_p / (err_v_p + s->m_noise);
    // calculate next state & error variance
    // S = S'+ K * (m - S')
    // P = (I - K) * P'
    s->state = s->state + (k * (m - s->state));
    s->err_v = (1 - k) * err_v_p;
}

/**
 * \brief Calculate the RSSI from dBm to meters.
 * 
 * \param rssi Measured rssi value.
 * \param tx_power Received power at 1 meters in dBm.
 * 
 * \return Measured distance in meters.
 */
static float 
ble_rssi_to_meters(float kalman_rssi, int tx_power)
{
    // RSSI = -10 * n * log10(d / d0) + A0
    // with d0 measured at 1 meter:
    // d = 10^((A - RSSI) / (10 * n))
    return pow10f(((float)tx_power - kalman_rssi) / (10.0F * BLE_ENV_FACTOR_IND));
}

/**
 * \brief Filter high frequency parts from the smoothed RSSI data.
 * 
 * \param rssi_state Kalman smoothed RSSI state value.
 * 
 * \return Low-pass filtered RSSI value.
 */
static float 
ble_rssi_low_pass_filter(float kalman_rssi)
{
    static float prev = 0;
    static int64_t start_us = 0;
    // initialize
    if (prev == 0)
        prev = kalman_rssi;
    
    float dt = ble_util_timedelta(&start_us);
    // smoothing factor (0 < alpha < 1)
    float new = prev + ((dt / (kalman_rssi + dt)) * (kalman_rssi - prev));
    prev = new;

    return new;
}

/**
 * \brief Process a new RSSI measurement.
 * 
 * \param measurement Measured RSSI value.
 */
void 
ble_rssi_update(int measurement)
{
    static ble_rssi_state_t kalman_rssi = {0};

    // initialization
    if (kalman_rssi.m_noise == 0) {
        kalman_rssi.state = measurement;
        kalman_rssi.err_v = ERROR_VARIANCE_P;
        kalman_rssi.m_noise = MEASUREMENT_NOISE_R;
        kalman_rssi.p_noise = PROCESS_NOISE_Q;
    }
    // smooth value using Kalman filter
    ble_rssi_kf_estimate(&kalman_rssi, (float)measurement);
    // distance calculation
    float rssi_m = ble_rssi_to_meters(kalman_rssi.state, TX_POWER_ONE_METER);
    // low pass filter go get rid of high frequency spikes
    float filtered_rssi_m = ble_rssi_low_pass_filter(rssi_m);
    // store value or publish using MQTT
#ifdef HOST
    ble_particle_ap_t host_ap = {
        .id = ID,
        .node_distance = filtered_rssi_m,
        .pos = {.x = POS_X, .y = POS_Y}
    };
    ble_mqtt_store_ap_data(host_ap);
#elif defined(AP)
    // construct payload string
    char *payload;
    int ret = asprintf(&payload, "%d,%g,%g,%g", ID, filtered_rssi_m, POS_X, POS_Y);
    if (ret != ESP_FAIL) {
        if (ble_mqtt_get_state() == MQTT_STATE_CONNECTED)
            esp_mqtt_client_publish(ble_mqtt_get_client(), AP_TOPIC, payload, 0, 0, 0);
        free(payload);
    }
#endif
}
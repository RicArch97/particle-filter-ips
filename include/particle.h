/* 
 * MicroStorm - BLE Tracking
 * include/particle.h
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

#ifndef PARTICLE_H
#define PARTICLE_H

#include "main.h"
#include "mqtt.h"

#define PARTICLE_SET_SIZE       500
#define NO_OF_APS               4

#define AP_MEASUREMENT_NOISE    0.5
#define RATIO_COEFFICIENT       1.2

#define GAUSS_NOISE_X           0.05
#define GAUSS_NOISE_Y           0.05

typedef struct {
    float d_node;
    float d_particle;
} ble_particle_ap_dist_t;

typedef struct {
    struct {
        struct {
            float x;
            float y;
        } coord;
        float angle;
    } state;
    float weight;
} ble_particle_t;

int ble_particle_update(ble_mqtt_ap_t *ap, ble_mqtt_node_state_t *node, int size);

#endif
/* 
 * MicroStorm - BLE Tracking
 * include/particle.h
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

#ifndef PARTICLE_H
#define PARTICLE_H

#define PARTICLE_SET            400
#define NO_OF_APS               4

#define AP_MEASUREMENT_VAR      0.8
#define ORIENTATION_VAR         0.2
#define POSITION_VAR            0.1

#define RATIO_COEFFICIENT       0.95

typedef enum {
    MOTION_STATE_STOP,
    MOTION_STATE_MOVING,
    MOTION_STATE_COUNT
} ble_particle_motion_t;

typedef struct {
    struct {
        struct {
            float x;
            float y;
        } pos;
        float theta;
        ble_particle_motion_t motion;
    } state;
    float weight;
} ble_particle_t;

typedef struct {
    struct {
        float x;
        float y;
    } pos;
    float acceleration;
} ble_particle_node_t;

typedef struct {
    struct {
        float x;
        float y;
    } pos;
    int id;
    float node_distance;
} ble_particle_ap_t;

typedef struct {
    float d_node;
    float d_particle;
} ble_particle_ap_dist_t;

typedef struct {
    ble_particle_ap_t aps[NO_OF_APS];
    ble_particle_node_t node;
} ble_particle_data_t;

int ble_particle_update(ble_particle_data_t *data);

#endif
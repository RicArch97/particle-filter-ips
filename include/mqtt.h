/* 
 * MicroStorm - BLE Tracking
 * include/mqtt.h
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

#ifndef MQTT_H
#define MQTT_H

#include <mqtt_client.h>

#define TOPIC           "ap"
#define KEEPALIVE       60
#define RECONNECT       1000
#define NETWORK_TIMEOUT 20000

#define RECONNECT_MAX   3

#define PF_TASK_NAME    "Update particle filter"
#define PF_TASK_SIZE    90000
#define PF_TASK_PRIO    10

#define NO_OF_APS       4

typedef struct {
    struct {
        float x;
        float y;
    } pos;
    int id;
    float node_distance;
} ble_mqtt_ap_t;

typedef struct {
    struct {
        float x;
        float y;
    } coord;
    float speed;
    float angle;
} ble_mqtt_node_state_t;

typedef enum {
    MQTT_STATE_DISCONNECTED = 0,
    MQTT_STATE_CONNECTED = 1
} mqtt_state_t;

typedef struct {
    ble_mqtt_ap_t aps[NO_OF_APS];
    ble_mqtt_node_state_t *node;
} ble_mqtt_pf_data_t;

mqtt_state_t ble_mqtt_get_state(void);
void ble_mqtt_init(void);
esp_mqtt_client_handle_t ble_mqtt_get_client(void);
void ble_mqtt_store_ap_data(ble_mqtt_ap_t data);

#endif
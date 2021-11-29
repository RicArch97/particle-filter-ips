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

#define TOPIC_PREFIX    "ap"

typedef struct {
    struct {
        int x;
        int y;
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

void ble_mqtt_init(void);
esp_mqtt_client_handle_t ble_mqtt_get_client(void);
void ble_mqtt_set_ap_data(ble_mqtt_ap_t data);
char *ble_mqtt_create_topic_str(const char *prefix, int id);

#endif
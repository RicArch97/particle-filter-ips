/* 
 * MicroStorm - BLE Tracking
 * include/adv.h
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

#ifndef ADV_H
#define ADV_H

#include <stdint.h>

#define SHA1_LENGTH                     20

#define BLE_ADV_APPEARANCE              0x0547
#define BLE_MIN_ADV_INTERVAL            0x0020
#define BLE_MAX_ADV_INTERVAL            0x0040

#define EDDYSTONE_UUID                  0xFEAA
#define EDDYSTONE_UID_FRAME_TYPE        0x00
#define EDDYSTONE_UID_NSP_LEN           10
#define EDDYSTONE_UID_INST_LEN          6
#define EDDYSTONE_UID_SIZE              20

typedef struct {
    uint8_t frame_type;
    int8_t tx_power;
    uint8_t namespace_id[10];
    uint8_t instance_id[6];
    uint16_t reserved;
} __attribute__((packed)) eddystone_uid_t;

typedef struct {
    struct {
        uint8_t len;
        uint8_t type;
        uint8_t flags;
    } flags;
    struct {
        uint8_t len;
        uint8_t type;
        uint16_t uuid;
    } uuid;
    struct {
        uint8_t len;
        uint8_t type;
        uint16_t uuid;
        eddystone_uid_t uid_beacon;
    } uid_frame;
} __attribute__((packed)) eddystone_adv_packet_t;

typedef struct {
    struct {
        uint8_t len;
        uint8_t type;
        uint16_t appearance;
    } appearance;
    struct {
        uint8_t len;
        uint8_t type;
        uint8_t name[16];
    } local_name;
} __attribute__((packed)) eddystone_scan_rsp_packet_t;

eddystone_uid_t ble_adv_create_service_data(int8_t tx_power, int node_id);
void ble_adv_set_advertisement_data(eddystone_uid_t service_data);
void ble_adv_set_scan_response_data(int id);
void ble_adv_start();
void ble_adv_stop();

#endif
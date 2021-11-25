/* 
 * MicroStorm - BLE Tracking
 * include/scan.h
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

#ifndef SCAN_H
#define SCAN_H

#include <stdint.h>

#define BLE_SCAN_INTERVAL       0x0010
#define BLE_SCAN_WINDOW         0x0010

#define LOCAL_NAME_LEN          16

typedef struct {
    struct {
        uint8_t flags;
        uint16_t uuid;
        struct {
            int8_t tx_power;
            uint8_t namespace_id[10];
            char instance_id[7];
        } uid_beacon;
    } adv;
    struct {
        uint16_t appearance;
        char local_name[17];
    } scan_rsp;
} ble_scan_rst_pkt_t;

int ble_scan_decode_adv(const uint8_t *p_adv_data, uint8_t data_len, 
        ble_scan_rst_pkt_t *rst);
int ble_scan_decode_scan_rsp(const uint8_t *p_scan_rsp_data, uint8_t data_len, 
        ble_scan_rst_pkt_t *rst);
void ble_scan_start(uint32_t duration);
void ble_scan_stop(void);

#endif
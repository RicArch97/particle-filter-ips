/* 
 * MicroStorm - BLE Tracking
 * src/scan.c
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

#include <stdint.h>
#include <string.h>

#include <esp_log.h>
#include <esp_gap_ble_api.h>
#include <mbedtls/sha1.h>

#include "main.h"
#include "scan.h"
#include "controller.h"
#include "adv.h"

static const char *TAG = "scan";

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = BLE_SCAN_INTERVAL,
    .scan_window            = BLE_SCAN_WINDOW,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

/**
 * \brief Return a byte array of the hashed namespace id.
 * 
 * \param namespace Namespace id.
 * 
 * \return Pointer to the dynamically allocated byte array.
 * Returns NULL on error.
 */
uint8_t *get_namespace_hash(const char *namespace)
{
    uint8_t *hash = malloc(SHA1_LENGTH);
    if (hash == NULL)
        return NULL;

    uint8_t *input = (uint8_t*)namespace;
    if (mbedtls_sha1_ret(input, strlen(namespace), hash) != ESP_OK) {
        free(hash);
        return NULL;
    }
    return hash;
}

/**
 * \brief Read 16 bit little endian value and swap to big endian.
 * 
 * \param data The data to be read.
 * \param p_ctr The program counter pointing to the current byte.
 * 
 * \return 16 bit value with swapped endianess.
 */
uint16_t little_endian_read_u16(const uint8_t *data, uint8_t p_ctr)
{
    return ((uint16_t)data[p_ctr]) | (((uint16_t)data[(p_ctr)+1]) << 8);
}

/**
 * \brief Decode advertisement data into advertisement result packet.
 * 
 * \param p_adv_data Pointer to the advertisment data.
 * \param data_len Length of the advertisment data.
 * \param rst Result packet where the data is written.
 * 
 * \return 0 on success, -1 on failure.
 */
int ble_scan_decode_adv(const uint8_t *p_adv_data, uint8_t data_len, 
        ble_scan_rst_pkt_t *rst)
{
    if (data_len == 0 || p_adv_data == NULL || rst == NULL)
        return -1;

    uint8_t p_ctr = 0;
    uint16_t uuid, serv_data_type, frame_type;

    while (p_ctr < data_len) {
        p_ctr++;  // frame len
        uint8_t type = p_adv_data[p_ctr++];
        
        switch (type) {
        case ESP_BLE_AD_TYPE_FLAG:
            rst->adv.flags = p_adv_data[p_ctr++];
            break;
        case ESP_BLE_AD_TYPE_16SRV_CMPL:
            uuid = little_endian_read_u16(p_adv_data, p_ctr);
            p_ctr += 2;
            if (uuid != EDDYSTONE_UUID)
                return -1;
            rst->adv.uuid = uuid;
            break;
        case ESP_BLE_AD_TYPE_SERVICE_DATA:
            serv_data_type = little_endian_read_u16(p_adv_data, p_ctr);
            p_ctr += 2;
            if (serv_data_type != EDDYSTONE_UUID)
                return -1;
            // we need to decode 1 + 1 + 10 + 6 = 18 bytes from this point
            // which is the total size of eddystone uid minus 2 reserved bytes
            if ((data_len - p_ctr) < (EDDYSTONE_UID_SIZE - 2))
                return -1;
            frame_type = p_adv_data[p_ctr++];
            if (frame_type != EDDYSTONE_UID_FRAME_TYPE)
                return -1;
            rst->adv.uid_beacon.tx_power = p_adv_data[p_ctr++];
            // save namespace
            for (int i = 0; i < EDDYSTONE_UID_NSP_LEN; i++) {
                rst->adv.uid_beacon.namespace_id[i] = p_adv_data[p_ctr++];
            }
            uint8_t *nsp_hash = get_namespace_hash(COMPANY_NAME);
            if (nsp_hash == NULL)
                return -1;
            // compare namespace to the first 10 characters to our hash   
            if (memcmp(rst->adv.uid_beacon.namespace_id, nsp_hash, 
                    EDDYSTONE_UID_NSP_LEN) != ESP_OK) {
                free(nsp_hash);
                return -1;
            }
            free(nsp_hash);
            // check if instance id contains our prefix
            for (int i = 0; i < EDDYSTONE_UID_INST_LEN; i++) {
                rst->adv.uid_beacon.instance_id[i] = (char)p_adv_data[p_ctr++];
            }
            // null terminate the string
            rst->adv.uid_beacon.instance_id[EDDYSTONE_UID_INST_LEN] = '\0';
            if (strncmp(rst->adv.uid_beacon.instance_id, INSTANCE_PREFIX, 
                    strlen(INSTANCE_PREFIX)) != ESP_OK)
                return -1;
            break;
        default:
            break;
        }
    }
    // in case we broke out of the loop early at some point
    if (rst->adv.flags == 0 || rst->adv.uuid == 0 || 
        rst->adv.uid_beacon.tx_power == 0)
        return -1;

    return 0;
}

/**
 * \brief Decode scan response data into scan reponse result packet.
 * 
 * \param p_scan_rsp_data Pointer to the scan response data.
 * \param data_len Length of the scan response data.
 * \param rst Result packet where the data is written.
 * 
 * \return 0 on success, -1 on failure.
 */
int ble_scan_decode_scan_rsp(const uint8_t *p_scan_rsp_data, uint8_t data_len, 
        ble_scan_rst_pkt_t *rst)
{
    if (data_len == 0 || p_scan_rsp_data == NULL || rst == NULL)
        return -1;

    uint8_t p_ctr = 0;
    uint16_t appearance;

    while (p_ctr < data_len) {
        p_ctr++;  // frame len
        uint8_t type = p_scan_rsp_data[p_ctr++];
        
        switch (type) {
        case ESP_BLE_AD_TYPE_APPEARANCE:
            appearance = little_endian_read_u16(p_scan_rsp_data, p_ctr);
            p_ctr += 2;
            if (appearance != BLE_ADV_APPEARANCE)
                return -1;
            rst->scan_rsp.appearance = appearance;
            break;
        case ESP_BLE_AD_TYPE_NAME_CMPL:
            // our local name has a maximum length of 16 bytes
            // we don't have to check the local name
            for (int i = 0; i < LOCAL_NAME_LEN; i++) {
                rst->scan_rsp.local_name[i] = (char)p_scan_rsp_data[p_ctr++];
            }
            rst->scan_rsp.local_name[LOCAL_NAME_LEN] = '\0';
            break;
        default:
            break;  
        } 
    }
    // if we broke out of the loop early
    if (rst->scan_rsp.appearance == 0)
        return -1;

    return 0;
}

/**
 * \brief Set scan params & start scanning for nodes.
 * 
 * \param duration Duration is the scan in seconds. 0 means endless.
 */
void ble_scan_start(uint32_t duration)
{
    if (!ble_controller_enabled()) {
        ESP_LOGE(TAG, "Could not start scanning, BLE controller not enabled.");
        return;
    }

    esp_err_t set_params_err = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (set_params_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not set scan params; %s", esp_err_to_name(set_params_err));
    }
    esp_err_t start_scan_err = esp_ble_gap_start_scanning(duration);
    if (start_scan_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start scanning; %s", esp_err_to_name(start_scan_err));
    }
}

/**
 * \brief Stop advertising.
 */
void ble_scan_stop(void)
{
    esp_err_t stop_err = esp_ble_gap_stop_scanning();
    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not stop advertising; %s", esp_err_to_name(stop_err));
    }
}
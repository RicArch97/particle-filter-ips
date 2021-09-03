/* MicroStorm - BLE Tracking
 * src/beacon.c
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

#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "mbedtls/sha1.h"

#include "main.h"
#include "beacon.h"

static const char *TAG = "Beacon";

/**
 * \brief Create service data from an EddystoneUID struct.
 * 
 * \param buffer Destination buffer for the service data.
 * \param tx_power Received power at 0 meters in dBm.
 * \param id Identifier for this beacon.
 */
void ble_adv_create_service_data(uint8_t *p_buffer, int8_t tx_power, int id)
{
    eddystone_uid_t beacon = {0};

    beacon.frame_type = EDDYSTONE_UID_FRAME_TYPE;
    beacon.tx_power = tx_power;
\
    uint8_t *sha1_input = (uint8_t*)COMPANY_NAME;
    uint8_t sha1_output[SHA1_LENGTH];
    if (mbedtls_sha1_ret(sha1_input, strlen(COMPANY_NAME), sha1_output) == ESP_OK)
        memccpy(beacon.namespace_id, sha1_output, EDDYSTONE_UID_NSP_LEN, SHA1_LENGTH * sizeof(uint8_t));
    else
        ESP_LOGE(TAG, "SHA-1 hash generation failed");

    char id_buf[EDDYSTONE_UID_INST_LEN - strlen(INSTANCE_PREFIX)];
    sprintf(id_buf, "%d", id);
    char *instance_buf;
    if (asprintf(&instance_buf, "%s%s", INSTANCE_PREFIX, id_buf) != ESP_FAIL)
        strcpy(beacon.instance_id, instance_buf);
    else
        ESP_LOGE(TAG, "Could not allocate memory for instance data");
    
    beacon.reserved = 0x00;

    memcpy(p_buffer, (const uint8_t*)&beacon, sizeof(beacon));
    
    free(instance_buf);
}

/**
 * \brief Create BLE scan response data from specification.
 * The scan response provides more detailed data in reaction to a scan request.
 * 
 * \param p_service_data Pointer to a service data buffer.
 * \param service_data_len Length of the provided service data. Should be less than 20.
 */
void ble_adv_set_scan_response(uint8_t *p_service_data, uint8_t service_data_len)
{
    // packet size for BLE advertisement is 31 bytes
    // if more data is provided it will be cut off at 20 bytes
    if (service_data_len > EDDYSTONE_UID_SIZE)
        service_data_len = EDDYSTONE_UID_SIZE;

    service_data_len += 0x03;
    
    // [len] [type] [data]
    uint8_t raw_scan_rsp[] = {
        0x02, ESP_BLE_AD_TYPE_FLAG, (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
        0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, (EDDYSTONE_UUID & 0xFF), (EDDYSTONE_UUID >> 8),
        service_data_len, ESP_BLE_AD_TYPE_SERVICE_DATA, (EDDYSTONE_UUID & 0xFF), (EDDYSTONE_UUID >> 8)
    };
    
    uint8_t *raw_scan_rsp_total = malloc((sizeof(raw_scan_rsp) + service_data_len) * sizeof(uint8_t));

    memcpy(raw_scan_rsp_total, raw_scan_rsp, sizeof(raw_scan_rsp) * sizeof(uint8_t));
    memcpy(raw_scan_rsp_total + sizeof(raw_scan_rsp), p_service_data, service_data_len * sizeof(uint8_t));

    esp_err_t scan_rsp_err = esp_ble_gap_config_scan_rsp_data_raw(
        raw_scan_rsp_total, 
        sizeof(raw_scan_rsp) + service_data_len
    );
    if (scan_rsp_err != ESP_OK) {
        ESP_LOGE(TAG, "Setting raw scan response data unsuccessful; %s", esp_err_to_name(scan_rsp_err));
    }

    free(raw_scan_rsp_total);
}

/**
 * \brief Create BLE advertisement data from specification.
 * The adverisement is basic data that is always emitted and identifies the device.
 * 
 * \param id The id of the device.
 */
void ble_adv_set_advertisement_data(int id)
{
    uint8_t name_len = (uint8_t)(0x01 + strlen(COMPANY_NAME));

    // [len] [type] [data]
    uint8_t raw_adv[] = {
        0x03, ESP_BLE_AD_TYPE_APPEARANCE, (BLE_ADV_APPEARANCE & 0xFF), (BLE_ADV_APPEARANCE >> 8),
        name_len, ESP_BLE_AD_TYPE_NAME_CMPL
    };

    char id_buf[EDDYSTONE_UID_INST_LEN - strlen(INSTANCE_PREFIX)];
    sprintf(id_buf, "%d", id);
    char *full_name;
    if (asprintf(&full_name, "%s%s%s", COMPANY_NAME, INSTANCE_PREFIX, id_buf) == ESP_FAIL) {
        ESP_LOGE(TAG, "Could not allocate memory for advertisement data");
        return;
    }

    uint8_t *raw_adv_total = malloc((sizeof(raw_adv) + strlen(full_name)) * sizeof(uint8_t));

    memcpy(raw_adv_total, raw_adv, sizeof(raw_adv) * sizeof(uint8_t));
    memcpy(raw_adv_total + sizeof(raw_adv), full_name, strlen(full_name) * sizeof(char));

    esp_err_t adv_err = esp_ble_gap_config_adv_data_raw(
        raw_adv_total,
        sizeof(raw_adv) + strlen(full_name)
    );
    if (adv_err != ESP_OK) {
        ESP_LOGE(TAG, "Setting raw advertisement data unsuccessful; %s", esp_err_to_name(adv_err));
    }

    free(full_name);
    free(raw_adv_total);
}

/**
 * \brief Set advertising params & start advertising as Eddystone UID beacon.
 */
void ble_adv_start()
{
    esp_ble_adv_params_t adv_params = {0};

    adv_params.adv_int_min       = BLE_MIN_ADV_INTERVAL;
    adv_params.adv_int_max       = BLE_MAX_ADV_INTERVAL;
    adv_params.adv_type          = ADV_TYPE_SCAN_IND;
    adv_params.own_addr_type     = BLE_ADDR_TYPE_PUBLIC;
    adv_params.peer_addr_type    = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map       = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    esp_err_t start_err = esp_ble_gap_start_advertising(&adv_params);
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start advertising; %s", esp_err_to_name(start_err));
    }
}

/**
 * \brief Stop advertising.
 */
void ble_adv_stop()
{
    esp_err_t stop_err = esp_ble_gap_stop_advertising();
    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not stop advertising; %s", esp_err_to_name(stop_err));
    }
}
/* 
 * MicroStorm - BLE Tracking
 * src/adv.c
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

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <esp_log.h>
#include <esp_gap_ble_api.h>
#include <mbedtls/sha1.h>

#include "config.h"
#include "adv.h"
#include "controller.h"

static const char *TAG = "adv";

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = BLE_MIN_ADV_INTERVAL,
    .adv_int_max        = BLE_MAX_ADV_INTERVAL,
    .adv_type           = ADV_TYPE_SCAN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

/**
 * \brief Create service data from an EddystoneUID struct.
 * 
 * \param tx_power Received power at 0 meters in dBm.
 * \param id Identifier for this beacon.
 * 
 * \return Value of a uid service data struct.
 */
eddystone_uid_t 
ble_adv_create_service_data(int8_t tx_power, int id)
{
    eddystone_uid_t uid = {0};

    uid.frame_type = EDDYSTONE_UID_FRAME_TYPE;
    uid.tx_power = tx_power;
    
    uint8_t *sha1_input = (uint8_t*)COMPANY_NAME;
    uint8_t sha1_output[SHA1_LENGTH];
    if (mbedtls_sha1_ret(sha1_input, strlen(COMPANY_NAME), sha1_output) == ESP_OK)
        memccpy(uid.namespace_id, sha1_output, EDDYSTONE_UID_NSP_LEN, SHA1_LENGTH);
    else
        ESP_LOGE(TAG, "SHA-1 hash generation failed");

    char id_buf[2];
    sprintf(id_buf, "%d", id);
    char *instance_buf;
    if (asprintf(&instance_buf, "%s%s", INSTANCE_PREFIX, id_buf) != ESP_FAIL)
        memcpy(uid.instance_id, (const uint8_t*)instance_buf, strlen(instance_buf));
    else
        ESP_LOGE(TAG, "Could not allocate memory for instance data");
    
    uid.reserved = 0x0000;
    
    free(instance_buf);
    return uid;
}

/**
 * \brief Create BLE advertisment data from specification.
 * The adverisement is always emitted and identifies the device.
 * 
 * \param service_data eddystone_uid_t struct instance initiated with service data.
 */
void 
ble_adv_set_advertisement_data(eddystone_uid_t service_data)
{   
    eddystone_adv_packet_t ble_adv_packet = {
        .flags = {
            .len = 0x02,
            .type = ESP_BLE_AD_TYPE_FLAG,
            .flags = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
        },
        .uuid = {
            .len = 0x03,
            .type = ESP_BLE_AD_TYPE_16SRV_CMPL,
            .uuid = EDDYSTONE_UUID
        },
        .uid_frame = {
            .len = 0x03 + sizeof(service_data),
            .type = ESP_BLE_AD_TYPE_SERVICE_DATA,
            .uuid = EDDYSTONE_UUID,
            .uid_beacon = service_data
        }
    };

    uint8_t *p_raw_adv = malloc(sizeof(ble_adv_packet));
    if (p_raw_adv == NULL) {
        ESP_LOGE(TAG, "Could not allocate memory for adv packet");
        return;
    }
    memcpy(p_raw_adv, (const uint8_t*)&ble_adv_packet, sizeof(ble_adv_packet));

    esp_err_t adv_err = esp_ble_gap_config_adv_data_raw(
        p_raw_adv, sizeof(ble_adv_packet));
        
    if (adv_err != ESP_OK) {
        ESP_LOGE(TAG, "Configuring raw advertisement data unsuccessful; %s", 
            esp_err_to_name(adv_err));
    }

    free(p_raw_adv);
}

/**
 * \brief Create BLE scan reponse data from specification.
 * The scan response provides more detailed data in reaction to a scan request.
 * 
 * \param id The id of the device.
 */
void 
ble_adv_set_scan_response_data(int id)
{
    char id_buf[2];
    sprintf(id_buf, "%d", id);
    char *full_name;
    if (asprintf(&full_name, "%s%s%s", COMPANY_NAME, 
            INSTANCE_PREFIX, id_buf) == ESP_FAIL) {
        ESP_LOGE(TAG, "Could not allocate memory for scan response data");
        return;
    }

    eddystone_scan_rsp_packet_t ble_scan_rsp_packet = {
        .appearance = {
            .len = 0x03,
            .type = ESP_BLE_AD_TYPE_APPEARANCE,
            .appearance = BLE_ADV_APPEARANCE
        },
        .local_name = {
            .len = 0x01 + (uint8_t)strlen(full_name),
            .type = ESP_BLE_AD_TYPE_NAME_CMPL
        }
    };
    memcpy(ble_scan_rsp_packet.local_name.name, (const uint8_t*)full_name, 
        strlen(full_name));

    uint8_t *p_raw_scan_rsp = malloc(sizeof(ble_scan_rsp_packet));
    if (p_raw_scan_rsp == NULL) {
        ESP_LOGE(TAG, "Could not allocate memory for scan resp packet");
        free(full_name);
        return;
    }
    memcpy(p_raw_scan_rsp, (const uint8_t*)&ble_scan_rsp_packet, 
        sizeof(ble_scan_rsp_packet));

    esp_err_t scan_rsp_err = esp_ble_gap_config_scan_rsp_data_raw(
        p_raw_scan_rsp, sizeof(ble_scan_rsp_packet));

    if (scan_rsp_err != ESP_OK) {
        ESP_LOGE(TAG, "Configuring raw scan response data unsuccessful; %s", 
            esp_err_to_name(scan_rsp_err));
    }
    
    free(full_name);
    free(p_raw_scan_rsp);
}

/**
 * \brief Set advertising params & start advertising as Eddystone UID beacon.
 */
void 
ble_adv_start(void)
{
    if (!ble_controller_enabled()) {
        ESP_LOGE(TAG, "Could not start advertising, BLE controller not enabled.");
        return;
    }

    esp_err_t start_err = esp_ble_gap_start_advertising(&ble_adv_params);
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start advertising; %s", esp_err_to_name(start_err));
    }
}

/**
 * \brief Stop advertising.
 */
void 
ble_adv_stop(void)
{
    esp_err_t stop_err = esp_ble_gap_stop_advertising();
    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG, "Could not stop advertising; %s", esp_err_to_name(stop_err));
    }
}
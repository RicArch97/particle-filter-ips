/* 
 * MicroStorm - BLE Tracking
 * src/controller.c
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

#include <string.h>

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_log.h>
#include <esp_gap_ble_api.h>
#include <nvs_flash.h>

#include "controller.h"
#include "scan.h"
#include "rssi.h"

static const char *TAG = "controller";

/**
 * \brief Set up a callback to handle GAP events in the GAP event loop.
 * 
 * \param event The event type.
 * \param param GAP parameters that can be checked.
 */
static void 
ble_controller_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    int found_adv;

    ble_scan_rst_pkt_t result_pkt = {0};

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertisement data is set");
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan response data is set");
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if ((err = param->adv_start_cmpl.status) == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Advertising started");
        else
            ESP_LOGE(TAG, "Advertising start attempt unsuccessful; %s", esp_err_to_name(err));
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan params are set.");
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if ((err = param->scan_start_cmpl.status) == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Scanning started");
        else
            ESP_LOGE(TAG, "Scanning start attempt unsuccessful; %s", esp_err_to_name(err));
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        switch (param->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            found_adv = ble_scan_decode_adv(
                param->scan_rst.ble_adv, param->scan_rst.adv_data_len, &result_pkt);
            // if we didn't find matching advertsing data skip event   
            if (found_adv != ESP_OK)
                return;
            else
                ble_rssi_update(param->scan_rst.rssi);
            break;
        default:
            break;
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Advertising stopped");
        else
            ESP_LOGE(TAG, "Advertising stop attempt unsuccessful; %s", esp_err_to_name(err));
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Scanning stopped");
        else
            ESP_LOGE(TAG, "Scanning stop attempt unsuccessful; %s", esp_err_to_name(err));
        break;
    default:
        ESP_LOGW(TAG, "Unhandled GAP event; %d", event);
        break;
    }
}

/**
 * \brief Initialize the BLE environment.
 */
void 
ble_controller_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // free unused memory from heap
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ESP_ERROR_CHECK(esp_bluedroid_init());
    }
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        ESP_ERROR_CHECK(esp_bluedroid_enable());
    }

    esp_err_t gap_cb_err = esp_ble_gap_register_callback(ble_controller_gap_cb);
    if (gap_cb_err != ESP_OK)
        ESP_LOGE(TAG, "Could not register GAP callback; %s", esp_err_to_name(gap_cb_err));
}

/**
 * \brief Return if the BLE controller is enabled.
 * 
 * \return true when the controller is enabled, false when it is not.
 */
int 
ble_controller_enabled(void)
{
    return (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) ? 1 : 0;
}
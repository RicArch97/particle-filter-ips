/* MicroStorm - BLE Tracking
 * src/controller.c
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

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_log.h>
#include <esp_gap_ble_api.h>
#include <nvs_flash.h>

#include "controller.h"

static const char *TAG = "controller";

/**
 * \brief Set up a callback to handle GAP events.
 * 
 * \param event The event type.
 * \param param GAP parameters that can be checked.
 */
static void ble_controller_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertisement data is set");
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan response data is set");
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
                ESP_LOGI(TAG, "Advertising started");
            else
                ESP_LOGE(TAG, "Advertising attempt unsuccessful; %d", param->adv_start_cmpl.status);
            break;
        default:
            ESP_LOGW(TAG, "Unhandled event; %d", event);
            break;
    }
}

/**
 * \brief Initialize the BLE environment.
 * 
 * \param device_name identifier for this device.
 */
void ble_controller_init()
{
    esp_err_t nvs_init_err = nvs_flash_init();
    if (nvs_init_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash; %s", esp_err_to_name(nvs_init_err));
    }

    // free unused memory from heap
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_err_t bt_init_err = esp_bt_controller_init(&cfg);
        if (bt_init_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize BLE controller; %s", esp_err_to_name(bt_init_err));
            return;
        }
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        esp_err_t bt_enable_err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (bt_enable_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable BLE controller; %s", esp_err_to_name(bt_enable_err));
            return;
        }
    }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        esp_err_t bdrd_init_err = esp_bluedroid_init();
        if (bdrd_init_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize bluedroid; %s", esp_err_to_name(bdrd_init_err));
            return;
        }
    }
    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        esp_err_t bdrd_enable_err = esp_bluedroid_enable();
        if (bdrd_enable_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable bluedroid; %s", esp_err_to_name(bdrd_enable_err));
            return;
        }
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
bool ble_controller_enabled()
{
    return (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
}
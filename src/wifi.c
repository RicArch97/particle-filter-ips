/* 
 * MicroStorm - BLE Tracking
 * src/wifi.c
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

#include <stdint.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <freertos/event_groups.h>

#include "wifi.h"
#include "config.h"

static const char *TAG = "wifi";

static EventGroupHandle_t wifi_event_group;
static int conn_retries = 0;

/**
 * \brief Handler for Wifi events.
 * 
 * \param arg extra argument passed to the handler.
 * \param event_base Event base for the handler.
 * \param event_id The id for the received event.
 * \param event_data The data for the event.
 */
void ble_wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, 
        void *event_data)
{
    ip_event_got_ip_t *event;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wifi connecting");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (conn_retries < WIFI_MAX_CONN_RETRIES) {
            esp_wifi_connect();
            conn_retries++;
            ESP_LOGW(TAG, "Wifi disconnected, reconnection attempt %d/%d", 
                conn_retries, WIFI_MAX_CONN_RETRIES);
        } 
        else {
            ESP_LOGW(TAG, "Wifi connection failed");
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
        ESP_LOGI(TAG, "Wifi connection successful");
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        conn_retries = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * \brief Initialize wifi connection and event loop.
 */
void ble_wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    // create wifi init config (default)
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // subscribe to events
    esp_event_handler_instance_t inst_any_id;
    esp_event_handler_instance_t inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
        &ble_wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
        &ble_wifi_event_handler, NULL, &inst_got_ip));

    // configure wifi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PSK,
            .threshold = {.authmode = WIFI_AUTH_WPA2_PSK},
            .pmf_cfg = {.capable = 1, .required = 0}
        }
    };
    // station mode (client)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // wait until connection established
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, 
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    // check return value
    if (bits & WIFI_CONNECTED_BIT)
        ESP_LOGI(TAG, "Connected to SSID: %s", SSID);
    if (bits & WIFI_FAIL_BIT)
        ESP_LOGW(TAG, "Could not connect to SSID: %s", SSID);
}
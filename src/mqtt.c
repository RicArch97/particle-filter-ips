/* 
 * MicroStorm - BLE Tracking
 * src/mqtt.c
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

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>

#include "mqtt.h"
#include "main.h"
#include "particle.h"
#include "wifi.h"

static const char *TAG = "mqtt";

static esp_mqtt_client_handle_t client;
static mqtt_state_t mqtt_state = MQTT_STATE_DISCONNECTED;
static int reconnect_tries = 0;

#ifdef HOST
static ble_mqtt_ap_t ap_data[NO_OF_APS];
static ble_mqtt_node_state_t node_state;
static ble_mqtt_pf_data_t pf_data;
static SemaphoreHandle_t xSemaphore = NULL;
static int event_idx = 0;
#endif

/**
 * \brief Log error if it is non zero.
 * 
 * \param message Error message to be logged.
 * \param error_code Error code to be logged.
 */
void ble_mqtt_log_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/**
 * \brief Task that updates the particle filter with new data upon receiving new events.
 * 
 * \param pv_params Parameter provided to XTaskCreate.
 */ 
void ble_mqtt_update_pf_task(void *pv_params)
{
#ifdef HOST
    ble_mqtt_pf_data_t *data = (ble_mqtt_pf_data_t*)pv_params;
    // try to take the semaphore to write to memory
    // the node state is updated within tasks and only 1 task can access it at a time
    // if the semaphore cannot be taken for the duration of 10 ticks, skip this update
    if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
        // update particle filter
        if (ble_particle_update(data) != ESP_OK)
            ESP_LOGE(TAG, "Particle filter update failed");
        // return access to the resource
        xSemaphoreGive(xSemaphore);
    }
    else
        ESP_LOGW(TAG, "Unable to take semaphore for particle filter update");

    // delete task after it is done, as it should only run once
    vTaskDelete(NULL);
#endif
}

/**
 * \brief Handler for MQTT events in the MQTT event loop.
 * 
 * \param handler_args User data registered to the event.
 * \param base Event base for handler.
 * \param event_id The id for the received event.
 * \param event_data The data for the event.
 */
void ble_mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, 
        void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT client connected");
        mqtt_state = MQTT_STATE_CONNECTED;
#ifdef HOST
        // in case of receiving values realtime, QoS 0 provides the least overhead
        // if a value is lost, it doesn't matter as we get a new more up to date value later
        esp_mqtt_client_subscribe(client, TOPIC, 0);
#endif
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT client disconnected");
        mqtt_state = MQTT_STATE_DISCONNECTED;
        reconnect_tries++;
        // reconnection isn't possible, and we also didn't get a wifi disconnect event
        // wifi might be frozen so try to reconnecting wifi first
        if (reconnect_tries == RECONNECT_MAX) {
            esp_wifi_connect();
            reconnect_tries = 0;
        }
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "Subscribe successfull, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
#ifdef HOST
        // check that topic matches
        if (strncmp(event->topic, TOPIC, event->topic_len) != ESP_OK)
            break;

        char *data_buf = strndup(event->data, event->data_len);
        ble_mqtt_ap_t data = {0};
        // split data string, delimiter is comma
        // format: [id,distance,posx,posy]
        for (int i = 0; i < strlen(data_buf); i++) {
            // split ID
            char *id_p = strtok(data_buf, ",");
            if (id_p != NULL) {
                data.id = (int)strtol(id_p, &id_p, 10);
            }
            // split distance
            char *dist_p = strtok(NULL, ",");
            if (dist_p != NULL) {
                data.node_distance = strtof(dist_p, &dist_p);
            }
            // split posx
            char *posx_p = strtok(NULL, ",");
            if (posx_p != NULL) {
                data.pos.x = strtof(posx_p, &posx_p);
            }
            // split posy
            char *posy_p = strtok(NULL, ",");
            if (posy_p != NULL) {
                data.pos.y = strtof(posy_p, &posy_p);
            }
        }
        free(data_buf);
        ble_mqtt_store_ap_data(data);

        // check if we have a value for each AP
        if (event_idx == NO_OF_APS) {
            memcpy(pf_data.aps, ap_data, sizeof(ap_data));
            pf_data.node = &node_state;
            // create task for particle update to prevent exceeding watchdog timer
            TaskHandle_t xHandle = NULL;
            xTaskCreate(ble_mqtt_update_pf_task, PF_TASK_NAME, PF_TASK_SIZE, 
                &pf_data, PF_TASK_PRIO, &xHandle);
            // reset counter & clear buffer
            event_idx = 0;
            memset(ap_data, 0, sizeof(ap_data));
        }        
#endif
        break;
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "Attempting to connect to MQTT broker");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "Connect return: (%s)", 
            strerror(event->error_handle->connect_return_code));
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ble_mqtt_log_if_nonzero("Reported from esp-tls", 
                event->error_handle->esp_tls_last_esp_err);
            ble_mqtt_log_if_nonzero("reported from tls stack", 
                event->error_handle->esp_tls_stack_err);
            ble_mqtt_log_if_nonzero("Captured as transport's socket errno",  
                event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", 
                strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGW(TAG, "Unhandled MQTT event; %d", event->event_id);
        break;
    }
}

/**
 * \brief Get the MQTT client state.
 * 
 * \return MQTT_STATE_DISCONNECTED or MQTT_STATE_CONNECTED.
 */
mqtt_state_t ble_mqtt_get_state(void)
{
    return mqtt_state;
}

/**
 * \brief Initialize MQTT broker connection and event loop.
 */
void ble_mqtt_init(void)
{   
    // connect to wifi
    ble_wifi_init();

    // create client
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = BROKER_HOST,
        .port = BROKER_PORT,
        .transport = MQTT_TRANSPORT_OVER_TCP,
        .username = BROKER_USERNAME,
        .password = BROKER_PASSWORD,
        .keepalive = KEEPALIVE,
        .reconnect_timeout_ms = RECONNECT,
        .network_timeout_ms = NETWORK_TIMEOUT
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    
    // subscribe to events
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, 
        ble_mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
#ifdef HOST
    // initialize node data position (center of the given area by default)
    // the other elements are 0 by default
    node_state.coord.x = AREA_X / 2;
    node_state.coord.y = AREA_Y / 2;

    // initialize mutex semaphore
    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL) {
        ESP_ERROR_CHECK(esp_mqtt_client_stop(client));
        ESP_ERROR_CHECK(esp_wifi_stop());
        ESP_LOGE(TAG, "Unable to create semaphore, closing connections");
    }
#endif
}

/**
 * \brief Return the active MQTT client instance.
 * 
 * \return Current active MQTT client instance.
 */
esp_mqtt_client_handle_t ble_mqtt_get_client(void)
{
    return client;
}

/**
 * \brief Cache new AP data.
 * The HOST AP caches the data directly instead of publishing via MQTT.
 * This function is only relevant for the HOST device.
 * 
 * \param data Struct holding the pre-processed RSSI and position.
 */
void ble_mqtt_store_ap_data(ble_mqtt_ap_t data)
{
#ifdef HOST
    for (int i = 0; i < NO_OF_APS; i++) {
        // we already cached an event from the HOST AP
        // replace it with the newer data for better accuracy
        if (ap_data[i].id == data.id) {
            ap_data[i] = data;
            return;
        }
    }
    // safeguard
    if (event_idx == NO_OF_APS)
        return;
    else
        ap_data[event_idx++] = data;
#endif
}
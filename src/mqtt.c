/* 
 * MicroStorm - BLE Tracking
 * src/mqtt.c
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
#include <freertos/FreeRTOS.h>

#include "mqtt.h"
#include "config.h"
#include "particle.h"
#include "wifi.h"

static const char *TAG = "mqtt";

static esp_mqtt_client_handle_t client;
static ble_mqtt_state_t mqtt_state = MQTT_STATE_DISCONNECTED;
static int reconnect_tries = 0;

#ifdef HOST
static ble_particle_ap_t ap_data[NO_OF_APS];
static ble_particle_data_t pf_data;
static SemaphoreHandle_t xSemaphore = NULL;
static ble_mqtt_task_t extra_task = TASK_NONE;
static int event_idx = 0;
#endif

/**
 * \brief Log error if it is non zero.
 * 
 * \param message Error message to be logged.
 * \param error_code Error code to be logged.
 */
static void 
ble_mqtt_log_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

#ifdef HOST
/**
 * \brief Write the node position to STDOUT.
 */
static void 
ble_mqtt_node_print(void)
{
    printf("%g,%g\n", pf_data.node.pos.x, pf_data.node.pos.y);
}

/**
 * \brief Publish the node state to a MQTT topic "node".
 */
static void 
ble_mqtt_node_publish(void)
{
    char *payload;
    int ret = asprintf(&payload, "%g,%g", pf_data.node.pos.x, pf_data.node.pos.y);
    if (ret != ESP_FAIL) {
        if (ble_mqtt_get_state() == MQTT_STATE_CONNECTED)
            esp_mqtt_client_publish(ble_mqtt_get_client(), NODE_TOPIC, payload, 0, 0, 0);
        free(payload);
    }
}

/**
 * \brief Task that updates the particle filter with new data upon receiving new events.
 * 
 * \param pv_params Parameter provided to XTaskCreate.
 */ 
static void 
ble_mqtt_update_pf_task(void *pv_params)
{
    // try to take the semaphore to write a new node state
    // poll the semaphore (don't block) because values are received fast
    if (xSemaphoreTake(xSemaphore, (TickType_t)0) == pdTRUE) {
        // update particle filter
        int ret = ble_particle_update(&pf_data);
        // return access to the resource
        xSemaphoreGive(xSemaphore);
        // execute extra task only after particle filter was updated
        if (ret == ESP_OK) {
            switch (extra_task) {
            case TASK_PRINT_NODE_STATE:
                ble_mqtt_node_print();
                break;
            case TASK_PUBLISH_NODE_STATE:
                ble_mqtt_node_publish();
                break;
            default:
                break;
            }
        }
        else
            ESP_LOGE(TAG, "Particle filter update failed");
    }
    // delete task after it is done, as it should only run once
    vTaskDelete(NULL);
}

/**
 * \brief Set a task to be executed when the particle filter is updated.
 * 
 * \param task TASK_PRINT_NODE_STATE, TASK_PUBLISH_NODE_STATE or TASK_NONE.
 */
void 
ble_mqtt_set_task(ble_mqtt_task_t task)
{
    extra_task = task;
}

/**
 * \brief Cache new AP data.
 * The HOST AP caches the data directly instead of publishing via MQTT.
 * This function is only relevant for the HOST device.
 * 
 * \param data Struct holding the pre-processed RSSI and position.
 */
void 
ble_mqtt_store_ap_data(ble_particle_ap_t data)
{
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
}
#endif

/**
 * \brief Handler for MQTT events in the MQTT event loop.
 * 
 * \param handler_args User data registered to the event.
 * \param base Event base for handler.
 * \param event_id The id for the received event.
 * \param event_data The data for the event.
 */
static void 
ble_mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, 
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
        esp_mqtt_client_subscribe(client, AP_TOPIC, 0);
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
        if (strncmp(event->topic, AP_TOPIC, event->topic_len) != ESP_OK)
            break;

        char *data_buf = strndup(event->data, event->data_len);
        ble_particle_ap_t data = {0};
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
            // create task for particle update to prevent exceeding watchdog timer
            TaskHandle_t xHandle;
            xTaskCreate(ble_mqtt_update_pf_task, PF_TASK_NAME, PF_TASK_SIZE, 
                NULL, PF_TASK_PRIO, &xHandle);
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
 * \brief Initialize MQTT broker connection and event loop.
 */
void 
ble_mqtt_init(void)
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
 * \brief Get the MQTT client state.
 * 
 * \return MQTT_STATE_DISCONNECTED or MQTT_STATE_CONNECTED.
 */
ble_mqtt_state_t 
ble_mqtt_get_state(void)
{
    return mqtt_state;
}

/**
 * \brief Return the active MQTT client instance.
 * 
 * \return Current active MQTT client instance.
 */
esp_mqtt_client_handle_t 
ble_mqtt_get_client(void)
{
    return client;
}
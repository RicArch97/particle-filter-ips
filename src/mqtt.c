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

#include "mqtt.h"
#include "main.h"
#include "particle.h"

static const char *TAG = "mqtt";

static ble_mqtt_ap_t ap_data[NO_OF_APS];
static esp_mqtt_client_handle_t client;
static ble_mqtt_node_state_t node_state;
static int event_idx = 0;

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
#ifdef HOST
        // in case of receiving values realtime, QoS 0 provides the least overhead
        // if a value is lost, it doesn't matter as we get a new more up to date value later
        esp_mqtt_client_subscribe(client, TOPIC, 0);
#endif
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT client disconnected");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "Subscribe successfull, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
#ifdef HOST
        // check that topic matches
        if (strcmp(event->topic, TOPIC) != ESP_OK)
            break;
        // no commas, incorrect data
        if (strchr(event->data, ',') == NULL)
            break;

        char *tmp_buf = strdup(event->data);
        ble_mqtt_ap_t data = {0};
        // split data string, delimiter is comma
        // format: [id,distance,posx,posy]
        for (int i = 0; i < strlen(tmp_buf); i++) {
            // split ID
            char *id_p = strtok(tmp_buf, ",");
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
        ble_mqtt_store_ap_data(data);

        // check if we have a value for each AP
        if (event_idx == (NO_OF_APS - 1)) {
            if (ble_particle_update(ap_data, &node_state, NO_OF_APS) != ESP_OK)
                ESP_LOGE(TAG, "Particle filter update failed");
            // clear cache
            memset(ap_data, 0, sizeof(ap_data));
        }        
#endif
        break;
    case MQTT_EVENT_ERROR:
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("Reported from esp-tls", 
                event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", 
                event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("Captured as transport's socket errno",  
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
void ble_mqtt_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // TODO
    // connect to wifi
    
    // create client
    esp_mqtt_client_config_t mqtt_cfg = {.uri = BROKER_ADDRESS};
    client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, 
        ble_mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    
    // initialize node data position (center of the given area by default)
    // the other elements are 0 by default
#ifdef HOST
    node_state.coord.x = AREA_X / 2;
    node_state.coord.y = AREA_Y / 2;
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
 * 
 * \param data Struct holding the pre-processed RSSI and position.
 */
void ble_mqtt_store_ap_data(ble_mqtt_ap_t data)
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
    if (event_idx == (NO_OF_APS - 1))
        return;
    else
        ap_data[event_idx++] = data;
}
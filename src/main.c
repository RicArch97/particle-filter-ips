/* 
 * MicroStorm - BLE Tracking
 * src/main.c
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

#include "main.h"
#include "adv.h"
#include "controller.h"
#include "scan.h"
#include "rssi.h"
#include "mqtt.h"

void app_main(void)
{
    // initalize ble controller
    ble_controller_init();
#ifdef NODE
    // set advertisment data
    ble_adv_set_advertisement_data(
       ble_adv_create_service_data((TX_POWER_ONE_METER + SIGNAL_LOSS), ID));
    // start advertising between 32-48 ms interval
    ble_adv_start();
#elif defined(AP) || defined(HOST)
    // connect to wifi & MQTT broker
    ble_mqtt_init();
    // start scanning indefinitely at 16 ms interval, with 16 ms duration
    ble_scan_start(0);
#endif
}
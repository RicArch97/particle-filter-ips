/* 
 * MicroStorm - BLE Tracking
 * include/main.h
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

#ifndef MAIN_H
#define MAIN_H

// purpose of this device; can be HOST, AP or NODE
// HOST is an AP and runs the particle filtering process
// AP just scans for the node and sends value to an MQTT topic
// NODE advertises eddystone UID packets
#define HOST

// ID of this device in range 1 to 4
#define ID              1
// area size (rectangle)
#define AREA_X          3
#define AREA_Y          2
// x and y postion of the beacon, within the given area size
#define POS_X           0
#define POS_Y           0             

// identifiers of the beacon
// instance prefix should be max 5 characters
#define COMPANY_NAME    "MicroStorm"
#define INSTANCE_PREFIX "Node"

// wifi connection
#define SSID            "<ssid>"
#define PSK             "<psk>"

// MQTT
#define BROKER_ADDRESS  "<addr>"

#endif
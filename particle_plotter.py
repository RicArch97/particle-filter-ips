"""
MicroStorm - BLE Tracking
particle_plotter.py - Script to plot particles generated by the HOST controller.

Copyright (c) 2021 Ricardo Steijn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from matplotlib.animation import FuncAnimation
from serial import Serial, SerialException

port = None
baudrate = 115200
x_limit = None
y_limit = None
amount = None


def get_args():
    """Get the arguments for this script."""
    description = "BLE-Tracking - particle plotter"
    arg = argparse.ArgumentParser(description=description)

    arg.add_argument(
        "-p", metavar="port", help="Serial port of the device, e.g. /dev/ttyUSB0", type=str, required=True
    )
    arg.add_argument(
        "-b", metavar="baudrate", help="Baudrate to use for the serial port", type=int,
    )
    arg.add_argument(
        "-s", metavar="size", help="Area size of the graph in format NUMxNUM", type=str, required=True
    )
    arg.add_argument(
        "-a", metavar="amount", help="Amount of particles that are sampled", type=int, required= True
    )

    return arg


def parse_args(parser: argparse.ArgumentParser):
    """Parse the given arguments."""
    args = parser.parse_args()
    
    global port, x_limit, y_limit, amount
    # port
    port = str(args.p)
    # x and y limits
    limits = args.s.split('x')
    x_limit = float(limits[0])
    y_limit = float(limits[1])
    # particle amount
    amount = args.a

    if args.b:
        global baudrate
        baudrate = int(args.b)
        


def animate(i, p, n, ser: Serial, p_x: deque, p_y: deque, n_x: deque, n_y: deque):
    """Update plot on interval."""
    bytes = ser.readline()
    received_data = bytes.split(b',')
    if len(received_data) < 3:
        return
    
    # parse data
    prefix: str = received_data[0].decode("utf-8")
    # check if numbers only
    try:
        coord_x = float(received_data[1])
        coord_y = float(received_data[2])
    except ValueError:
        return

    # handle based on data prefix
    if prefix == "p":
        # check if list full, in that case overwrite first values
        if len(p_x) == p_x.maxlen:
            p_x.appendleft(coord_x)
        else:
            p_x.append(coord_x)
        if len(p_y) == p_y.maxlen:
            p_y.appendleft(coord_y)
        else:
            p_y.append(coord_y)
    elif prefix == "n":
        n_x.append(coord_x)
        n_y.append(coord_y)

    # add to scatter plots
    p.set_offsets(np.c_[p_x, p_y])
    n.set_offsets(np.c_[n_x, n_y])


def main():
    """Initialize serial port."""
    # get args
    parser = get_args()
    parse_args(parser)
   
    # setup serial
    ser = Serial(
        port=port,
        baudrate=baudrate
    )
    try:
        if ser.is_open:
            ser.close()
            ser.open()
        else:
            ser.open()
    except SerialException as err:
        print(f"unable to open serial port: {err}")
        sys.exit(1)

    print(ser)
    
    particles_x = deque([0], maxlen=amount)
    particles_y = deque([0], maxlen=amount)
    node_x = deque([x_limit / 2], maxlen=1)
    node_y = deque([y_limit / 2], maxlen=1)

    # create plot
    fig, ax = plt.subplots()
    particles = ax.scatter(particles_x, particles_y, color='blue')
    node = ax.scatter(node_x, node_y, color='red', s=100)
    plt.xlim(0, x_limit)
    plt.ylim(0, y_limit)

    # plot at very low interval, as particle come in very fast
    # note that this will likely run 1 logical cpu core at 100%
    ani = FuncAnimation(
        fig, animate, fargs=(
            particles, node, ser, particles_x, particles_y, node_x, node_y
        ), interval=1
    )
    plt.show()

try:    
    main()
except KeyboardInterrupt:
    print("keyboardinterrupt received, closing")
    sys.exit(1)
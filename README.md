# particle-filter-ips

Particle Filter Indoor Positioning System on ESP32 microcontrollers.

# Work in progress, don't use this yet

## Building and running

The easiest way to build and run is using the PlatformIO extension in VS Code. 
Simply open this project as a folder in VS Code and use the PlatformIO tasks to build, upload and monitor.

### PlatformIO from the commandline

First install PlatformIO Core for your operating system. This requires Python 3.6+ or above to be installed. 

#### Linux / Mac

Arch Linux users may wish to install the `platformio` package from the AUR.

For all other distro's and MacOS you can simply run:
```
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
```
to install PlatformIO Core.

#### Windows 10

Install Python from the Microsoft Store.

Download [this](https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py) file.
Simply click "save as" and save it in an appropriate location.

Open Command Prompt or PowerShell in the folder where you saved the file and run:
```
python get-platformio.py
```

#### Building and running

Head over to the project folder and use:
```
pio run
```
to build and upload the project.

To use the device monitor:
```
pio device monitor
```

This assumes you are using a Adafruit Huzzah32 Feather board. In case you want to build and upload to a different ESP32 board,
add matching entries to `platformio.ini`. For example:
```
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = espidf
monitor_speed = 115200
monitor_flags = 
    --raw
```
for a ESP32 DoIt Devkit board.

You can select this specific environment using the `-e` or `--environment` flag;
```
pio device monitor --environment esp32doit-devkit-v1
```

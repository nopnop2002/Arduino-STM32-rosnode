# bmp280 rosnode
A demo that captures data from an SPI sensor and includes it in ROS.   



# Hardware requirements
- STM32 Development Board like Bluepill/Blackpill  

- Inexpensive Chinese ST-LINK adapter   
![ST-LINK-1](https://user-images.githubusercontent.com/6020549/221065783-33508ebe-2454-4033-92f8-34c00fe0eb80.JPG)
![platformio-2](https://user-images.githubusercontent.com/6020549/221065793-a32da243-946a-4cf4-9655-1347a229d6eb.JPG)

- UART-USB converter module   

- BMP280 Digital pressure sensor   


# Build Firmware
Connect ST-LINK adapter and STM32.
```
+----------+         +----------+         +----------+
|  STM32   |         | ST-LINK  |  USB    |   Host   |
|      3.3V|---------|3.3V      |=========|          |
|      GND |---------|GND       |         |          |
|      PA13|---------|SWDIO     |         |          |
|      PA14|---------|SWCLK     |         |          |
+----------+         +----------+         +----------+
```

Build rosserial_arduino Library
```
$ git clone https://github.com/nopnop2002/Arduino-STM32-rosnode
$ cd Arduino-STM32-rosnode/bmp280
$ cd lib
$ rosrun rosserial_arduino make_libraries.py .
$ ls
README  ros_lib
```


Build Firmware for pill board
```
$ cd ..
$ pio run -e blackpill_f103c8 -t upload
```

Build Firmware for generic board
```
$ cd ..
$ pio run -e genericSTM32F103C8 -t upload
```


# Wireing to sensor
|BMP280||STM32||
|:---|:---|:---|:---|
|3V3||3.3V||
|GND||GND||
|SCL||PA5|SPI SCK|
|SDA||PA7|SPI MOSI|
|CSB||PA4|SPI CS|
|SDO||PA6|SPI MISO|


# Starting a ROS node

### Wireing
Connect STM32 and ROS HOST using UART-USB converter.

```
Build with pill board
+----------+         +----------+         +----------+
|  STM32   |   UART  | UART-USB |  USB    | ROS HOST |
|      PA9 |---------|RX        |=========|          |
|      PA10|---------|TX        |         |          |
|      GND |---------|GND       |         |          |
+----------+         +----------+         +----------+

Build with generic board
+----------+         +----------+         +----------+
|  STM32   |   UART  | UART-UART|  USB    | ROS HOST |
|      PA2 |---------|RX        |=========|          |
|      PA3 |---------|TX        |         |          |
|      GND |---------|GND       |         |          |
+----------+         +----------+         +----------+
```

When the PlatformIO host and ROS host are the same, it will be as follows:
```
+----------+         +----------+         +----------+
|  STM32   |         | ST-LINK  |  USB    | ROS Host |
|      3.3V|---------|3.3V      |=========|          |
|      GND |---------|GND       |         |          |
|      PA13|---------|SWDIO     |         |          |
|      PA14|---------|SWCLK     |         |          |
|          |         +----------+         |          |
|          |                              |          |
|          |         +----------+         |          |
|          |   UART  | UART-USB |  USB    |          |
|      PA9 |---------|RX        |=========|          |
|      PA10|---------|TX        |         |          |
|      GND |---------|GND       |         |          |
+----------+         +----------+         +----------+
```

### Terminal 1
```
$ roscore
```

### Terminal 2
```
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
[INFO] [1677197464.818522]: ROS Serial Python Node
[INFO] [1677197464.827333]: Connecting to /dev/ttyUSB0 at 115200 baud
[INFO] [1677197466.935209]: Requesting topics...
[INFO] [1677197467.339121]: Note: publish buffer size is 512 bytes
[INFO] [1677197467.341800]: Setup publisher on chatter [std_msgs/String]
[INFO] [1677197467.348600]: Note: subscribe buffer size is 512 bytes
[INFO] [1677197467.350585]: Setup subscriber on led [std_msgs/Bool]
```

### Terminal 3
```
$ rostopic echo /chatter
data: "Temperature = 14.63 *C"
---
data: "Pressure = 102229.39 Pa"
---
data: "Approx altitude = -75.03 m"
---


$ rostopic echo /temperature
data: 14.6499996185
---
data: 14.6499996185
---
data: 14.6499996185
---

$ rostopic echo /pressure
data: 102226.703125
---
data: 102226.875
---
data: 102227.046875
---


$ rostopic echo /altitude
data: -75.280960083
---
data: -75.280960083
---
data: -75.2669830322
---
```

### Visualization
```
rqt_plt rqt_plt /temperature
```

![temp_2023-02-24_18-03-12](https://user-images.githubusercontent.com/6020549/221138164-22929878-2e08-4728-8d91-b9552490e441.png)


# Using other board
The hardware SPI of STM32 is complicated, and the pin assignment of the SPI has different GPIO for each valiant.   
The SPI definition of each board cannot be understood without looking at the variant_generic.h of each board published [here](https://github.com/stm32duino/Arduino_Core_STM32/tree/main/variants).   

__STM32F4 is especially complicated.__   
It is necessary to confirm the mapping of the i2c gpio in advance.   
This is a sample of some boards.   
|Bord valiant|SS|MOSI|MISO|SCK|MCU|
|:---|:---|:---|:---|:---|:---|
|blackpill_f103c8|PB7|PB6|STM32F103C8|
|bluepill_f103c8|PA4|PA7|PA6|PA5|STM32F103C8|
|genericSTM32F103C8|PA4|PA7|PA6|PA5|STM32F103C8|
|blackpill_f401cc|PA4|PA7|PA6|PA5|STM32F401CC|
|genericSTM32F401CC|PA4|PA7|PA6|PA5|STM32F401CC
|blackpill_f411ce|PA4|PA7|PA6|PA5|STM32F411CE|
|genericSTM32F411CE|PB12|PA1|PA11|PA13|STM32F411CE|
|genericSTM32F405RG|PA4|PA7|PA6|PA5|STM32F405RG|
|black_f407vg|PB7(*)|PB5|PB4|PB3|STM32F407VGT|
|diymore_f407vgt|PB6|PB5|PB4|PB3|STM32F407VGT|
|genericSTM32F407VGT6|PA4|PA7|PA6|PA5|STM32F407VGT|
|disco_f407vg|PB12|PA7|PA6|PA5|STM32F407VGT|

(*) I can't use it because I'm using it with the onboard NRF24 connector.   

# Memory Usage
This demo can't ececute with ATMega328(Uno,Nano,Promini).   
Too little RAM.
```
$ pio run -e uno
Processing uno (platform: atmelavr; board: uno; framework: arduino)
--------------------------------------------------------------------------------
Verbose mode can be enabled via `-v, --verbose` option
CONFIGURATION: https://docs.platformio.org/page/boards/atmelavr/uno.html
PLATFORM: Atmel AVR (4.1.0) > Arduino Uno
HARDWARE: ATMEGA328P 16MHz, 2KB RAM, 31.50KB Flash
DEBUG: Current (avr-stub) External (avr-stub, simavr)
PACKAGES:
 - framework-arduino-avr @ 5.1.0
 - toolchain-atmelavr @ 1.70300.191015 (7.3.0)
LDF: Library Dependency Finder -> https://bit.ly/configure-pio-ldf
LDF Modes: Finder ~ chain, Compatibility ~ soft
Found 9 compatible libraries
Scanning dependencies...
Dependency Graph
|-- Adafruit BMP280 Library @ 2.6.6
|   |-- Adafruit Unified Sensor @ 1.1.7
|   |-- Adafruit BusIO @ 1.14.1
|   |   |-- Wire @ 1.0
|   |   |-- SPI @ 1.0
|   |-- Wire @ 1.0
|   |-- SPI @ 1.0
|-- SPI @ 1.0
|-- Wire @ 1.0
|-- ros_lib
|   |-- SPI @ 1.0
Building in release mode
Checking size .pio/build/uno/firmware.elf
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [========= ]  94.4% (used 1934 bytes from 2048 bytes)
Flash: [=======   ]  69.7% (used 22468 bytes from 32256 bytes)
```

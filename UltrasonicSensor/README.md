# UltrasonicSensor rosnode
A demo that captures data from an ultrasonic sensor and includes it in ROS.   

# Hardware requirements
- STM32 Development Board with 5V pin header.   
 BlackPill does not have a 5V pin header, so it cannot be used.   

- Inexpensive Chinese ST-LINK adapter   
![ST-LINK-1](https://user-images.githubusercontent.com/6020549/221065783-33508ebe-2454-4033-92f8-34c00fe0eb80.JPG)
![platformio-2](https://user-images.githubusercontent.com/6020549/221065793-a32da243-946a-4cf4-9655-1347a229d6eb.JPG)

- UART-USB converter module   

- Ultrasonic Sensor like HC-SR04/US-015/HY-SRF05   
![HY-SRF05-3](https://user-images.githubusercontent.com/6020549/221387700-94149b81-7b76-46b3-b8aa-5d9773995328.JPG)

# Wireing to sensor
|Ultrasonic||STM32|
|:---|:---|:---|
|5V||5V|
|GND||GND|
|Trig||PB6|
|Echo||PB7(*)|

(*)
Must be FT GPIO.   
STM32 has a terminal that can accept a 5V voltage signal.   
The specific terminal is described as "FT" (Five voltage Tolerant) in the datasheet.   
Even if the power supply voltage of the microcomputer is 3.6V or less, 5V voltage can be applied to this pin.   

![STM32F103-5V-FT](https://user-images.githubusercontent.com/6020549/221387743-dda3d5dd-442c-4344-b305-b7ecca230eba.jpg)

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
$ cd Arduino-STM32-rosnode/UltrasonicSensor
$ cd lib
$ rosrun rosserial_arduino make_libraries.py .
$ ls
README  ros_lib
```


Build Firmware for pill board
```
$ cd ..
$ pio run -e bluepill_f103c8 -t upload
```

Build Firmware for generic board
```
$ cd ..
$ pio run -e genericSTM32F103C8 -t upload
```


# Starting a ROS node

### Wireing
Disconnect ST-Link from STM32.   
Connect STM32 and ROS HOST using UART-USB converter.   
Power the STM32 from USB.   
Because the sensor needs 5V power supply.   

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
data: "Duration:39838.00 microsec"
---
data: "Distance:1380.39 cm"
---

$ rostopic echo /duration
data: 39836.5
---
data: 39837.0
---

$ rostopic echo /distance
data: 1380.24804688
---
data: 1380.35205078
---
data: 1380.31738281
---
```


### Visualization
```
rqt_plt rqt_plt /distance
```

![ultrasonic_2023-02-26_10-25-38](https://user-images.githubusercontent.com/6020549/221387503-599cce07-74f4-428a-9783-5f289bcfef18.png)

# Memory Usage
This demo can ececute with ATMega328(Uno,Nano,Promini).   
```
rd: uno; framework: arduino)
----------------------------------------------------------------------------------------------------------------------
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
Found 6 compatible libraries
Scanning dependencies...
Dependency Graph
|-- ros_lib
|   |-- SPI @ 1.0
Building in release mode
Checking size .pio/build/uno/firmware.elf
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [=======   ]  73.1% (used 1498 bytes from 2048 bytes)
Flash: [====      ]  38.5% (used 12424 bytes from 32256 bytes)
```

# Speed of sound
The speed of sound is affected by temperature and can be calculated with 331.5+0.61\*temperature[m/sec].   
If the temperature is 25 degrees, it will be 331.5+0.61\*25=346.75[m/sec].   
This example assuming a temperature of 25 degrees.   
If you need more accuracy, you should use a digital thermometer to measure the temperature and use the measured temperature to calculate the distance.   


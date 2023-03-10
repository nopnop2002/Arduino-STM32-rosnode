# mpu6050-madgwick rosnode
A demo that captures data from an i2c 6DoF imu and includes it in ROS.   



# Hardware requirements
- STM32 Development Board like Bluepill/Blackpill  

- Inexpensive Chinese ST-LINK adapter   
![ST-LINK-1](https://user-images.githubusercontent.com/6020549/221065783-33508ebe-2454-4033-92f8-34c00fe0eb80.JPG)
![platformio-2](https://user-images.githubusercontent.com/6020549/221065793-a32da243-946a-4cf4-9655-1347a229d6eb.JPG)

- UART-USB converter module   

- MPU6050 6DoF MotionTracking device   
 MPU6050 has an internal processing function called DMP (Digital Motion Processor).   
 But this sample doesn't use DMP, just 6DoF data.   
 Conversion from 6DoF data to quaternion uses imu_filter_madgwick on the host side.   

I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) library to get data from MPU6050.   
ros code is based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_raw).   

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
$ cd Arduino-STM32-rosnode/mpu6050-madgwick
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
|MPU6050||STM32|
|:---|:---|:---|
|3V3||3.3V|
|GND||GND|
|SCL||PB6|
|SDA||PB7|


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
---
data: "accel_sensitivity=16384.00 gyro_sensitivity=   16.40"
---
data: "dataSteps=974"
---
data: "ax_avr:  -0.01 ay_avr:   0.01 az_avr:   1.01"
---
data: "gx_avr:   0.00 gy_avr:  -0.00 gz_avr:  -0.00"
---

$ rostopic echo /imu/data_raw
header:
  seq: 1299
  stamp:
    secs: 1677280248
    nsecs: 177066884
  frame_id: "imu"
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.42724609375
  y: -0.30517578125
  z: 1.0986328125
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 0.04833984375
  y: -0.01220703125
  z: -0.99072265625
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

### Conversion to Quatanion using madgwick filters
Learn more about filters [here](http://wiki.ros.org/imu_filter_madgwick).
```
$ sudo apt-get install ros-melodic-imu-tools

$ rosrun imu_filter_madgwick imu_filter_node _use_mag:=false
```

### Show Quatanion
```
$ rostopic echo /imu/data
header:
  seq: 176
  stamp:
    secs: 1677107817
    nsecs: 473739017
  frame_id: "imu"
orientation:
  x: 0.626041507241
  y: -0.778182179409
  z: -0.0456474441657
  w: 0.0205143292187
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: -0.30517578125
  y: 0.0
  z: 0.06103515625
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.00048828125
  y: -0.00634765625
  z: -0.97119140625
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

### View Quatanion using rviz
Using rviz_imu_plugin you can directly see the result of ImuFilter.   
- start rviz   
- Add->rviz_imu_plugin->Imu   
- Global Options->Fixed Frame->imu   
- Imu->Topic->/imu/data   

![rviz_2023-02-26_09-00-11](https://user-images.githubusercontent.com/6020549/221385300-b6805bde-5b48-44fd-baf9-dbdaebacf4fb.png)
![mpu9250_2023-03-01_18-09-38](https://user-images.githubusercontent.com/6020549/222094528-186cfc32-4d70-415e-a509-3a0a200bd80b.png)


# Using other board
The hardware I2C of STM32 is complicated, and the pin assignment of the I2C has different GPIO for each valiant.   
For example, I2C of blackpill_f401cc is assigned PB7/PB6, but I2C of genericSTM32F401CC is assigned PB3/PB10.   
The I2C definition of each board cannot be understood without looking at the variant_generic.h of each board published [here](https://github.com/stm32duino/Arduino_Core_STM32/tree/main/variants).   

__STM32F4 is especially complicated.__   
It is necessary to confirm the mapping of the i2c gpio in advance.   
This is a sample of some boards.   
|Bord valiant|SDA|SCL|MCU|
|:---|:---|:---|:---|
|blackpill_f103c8|PB7|PB6|STM32F103C8|
|bluepill_f103c8|PB7|PB6|STM32F103C8|
|genericSTM32F103C8|PB7|PB6|STM32F103C8|
|blackpill_f401cc|PB7|PB6|STM32F401CC|
|genericSTM32F401CC|PB3|PB10|STM32F401CC
|blackpill_f411ce|PB7|PB6|STM32F411CE|
|genericSTM32F411CE|PB3|PB10|STM32F411CE|
|genericSTM32F405RG|PB7|PB6|STM32F405RG|
|black_f407vg|PB9|PB8|STM32F407VGT|
|diymore_f407vgt|PB7|PB6|STM32F407VGT|
|genericSTM32F407VGT6|PB9|PB8|STM32F407VGT|
|disco_f407vg|PB7|PB8|STM32F407VGT|


# Memory Usage
This demo can't run on ATMega328(Uno,Nano,Promini).   
Too little RAM.
```
$ pio run -e uno
Processing uno (platform: atmelavr; board: uno; framework: arduino)
-----------------------------------------------------------------------------------------------------------
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
Found 8 compatible libraries
Scanning dependencies...
Dependency Graph
|-- I2Cdevlib-MPU6050 @ 1.0.0
|   |-- I2Cdevlib-Core @ 1.0.1
|   |   |-- Wire @ 1.0
|-- ros_lib
|   |-- SPI @ 1.0
Building in release mode
Checking size .pio/build/uno/firmware.elf
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [==========]  112.3% (used 2299 bytes from 2048 bytes)
Flash: [======    ]  64.9% (used 20928 bytes from 32256 bytes)
```

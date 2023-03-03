# mpu9250 rosnode
A demo that captures data from an i2c 9DoF imu and includes it in ROS.   



# Hardware requirements
- STM32 Development Board like Bluepill/Blackpill  

- Inexpensive Chinese ST-LINK adapter   
![ST-LINK-1](https://user-images.githubusercontent.com/6020549/221065783-33508ebe-2454-4033-92f8-34c00fe0eb80.JPG)
![platformio-2](https://user-images.githubusercontent.com/6020549/221065793-a32da243-946a-4cf4-9655-1347a229d6eb.JPG)

- UART-USB converter module   

- MPU9250 9DoF MotionTracking device   
 MPU9250 is a package that integrates the MPU6050 and a chip (AK8963) with a 3-axis magnetic sensor.   
 MPU9250 has an internal processing function called DMP (Digital Motion Processor).   
 But this sample doesn't use DMP, just 9DoF data.   
 Conversion from 9DoF data to quaternion uses imu_filter_madgwick on the host side.   

- AL8963 3-axis Electronic Compass   
 Sensitivity adjustment data for each axis is stored to fuse ROM on shipment.   
 This value can be used to calculate adjusted measurement data.   

I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) library to get data from MPU6050.   
I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/AK8963) library to get data from AK8963.   
IMU_Zero is based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/IMU_Zero).   
ros code is based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_raw).   

# Sensor calibration
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


Connect STM32 and HOST using a UART-USB converter to view calibration results.   
Calibration results are output to the serial port.   
Start a serial monitor on the host.   
I used TeraTerm.   
```
Build with pill board
+----------+         +----------+         +----------+
|  STM32   |   UART  | UART-USB |  USB    |   Host   |
|      PA9 |---------|RX        |=========|          |
|      PA10|---------|TX        |         |          |
|      GND |---------|GND       |         |          |
+----------+         +----------+         +----------+

Build with generic board
+----------+         +----------+         +----------+
|  STM32   |   UART  | UART-UART|  USB    |   Host   |
|      PA2 |---------|RX        |=========|          |
|      PA3 |---------|TX        |         |          |
|      GND |---------|GND       |         |          |
+----------+         +----------+         +----------+
```

Lay the sensor horizontally and perform the calibration.   
```
$ git clone https://github.com/nopnop2002/Arduino-STM32-rosnode
$ cd Arduino-STM32-rosnode/mpu9250/IMU_Zero
$ pio run -t upload
```

It will take a few minutes for the calibration to complete.   
The serial monitor should show results similar to the following.   
If -- done -- is displayed, it is completed.    
```
....................    XAccel                  YAccel                          ZAccel                  XGyro                   YGyro                   ZGyro
 [-2891,-2889] --> [-10,3]      [-445,-444] --> [-17,1] [697,698] --> [16381,16400]     [148,149] --> [-1,2]    [26,27] --> [-1,2]      [16,17] --> [-2,1]
.................... [-2890,-2889] --> [-5,3]   [-445,-444] --> [-17,1] [697,698] --> [16375,16400]     [148,149] --> [0,2]     [26,27] --> [0,2]       [16,17] --> [-2,1]
.................... [-2890,-2889] --> [-10,3]  [-445,-444] --> [-16,1] [697,698] --> [16373,16400]     [148,149] --> [0,2]     [26,27] --> [0,2]       [16,17] --> [-1,1]
-------------- done --------------
```

The last line is your offset value.   
We need 6 values XAccelOffset, YAccelOffset, ZAccelOffset, XGyroOffset, YGyroOffset, ZGyroOffset.
In this example:
- XAccelOffset = -2889   
- YAccelOffset = -444   
- ZAccelOffset = 698   
- XGyroOffset = 149   
- YGyroOffset = 27   
- ZGyroOffset = 17   


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
$ cd Arduino-STM32-rosnode/mpu9250
$ cd lib
$ rosrun rosserial_arduino make_libraries.py .
$ ls
README  ros_lib
```

Set your offset in your code   
```
$ cd ..
$ vi src/main.cpp
# Change the offset values below:
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-2889);
  mpu.setYAccelOffset(-444);
  mpu.setZAccelOffset(698);
  mpu.setXGyroOffset(149);
  mpu.setYGyroOffset(27);
  mpu.setZGyroOffset(17);
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
|MPU9250||STM32|
|:---|:---|:---|
|3V3||3.3V|
|GND||GND|
|SDA||PB7|
|SCL||PB6|


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
data: "roll:  0.54 pitch: -0.37 yaw: -7.96"
---
data: "magCalibration X: 1.21 Y: 1.20 Z: 1.16"
---
data: "intMagData=-14081 7680 2816"
---
data: "microTesla:-2557.68 1381.50 491.70"
---
data: "millGauss:-25566.41 13809.38 4915.00"
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


$ rostopic echo /imu/mag
header:
  seq: 1
  stamp:
    secs: 1677280291
    nsecs: 667997104
  frame_id: "imu"
magnetic_field:
  x: -3.6328125
  y: 51.56640625
  z: 5.8203125
magnetic_field_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

### Conversion to Quatanion
Learn more about filters [here](http://wiki.ros.org/imu_filter_madgwick).
```
$ sudo apt-get install ros-melodic-imu-tools

# don't use mag data
$ rosrun imu_filter_madgwick imu_filter_node _use_mag:=false

# use mag data
$ rosrun imu_filter_madgwick imu_filter_node _use_mag:=true
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
Found 8 compatible libraries
Scanning dependencies...
Dependency Graph
|-- I2Cdevlib-MPU6050 @ 1.0.0
|   |-- I2Cdevlib-Core @ 1.0.1
|   |   |-- Wire @ 1.0
|-- ros_lib
|   |-- SPI @ 1.0
|-- I2Cdevlib-Core @ 1.0.1
|   |-- Wire @ 1.0
Building in release mode
Checking size .pio/build/uno/firmware.elf
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [==========]  153.9% (used 3151 bytes from 2048 bytes)
Flash: [==========]  99.5% (used 32110 bytes from 32256 bytes)
```

# mpu6050-dmp rosnode
A demo that captures data from an i2c 6DoF imu and includes it in ROS.   



# Hardware requirements
- STM32 Development Board like Bluepill/Blackpill  

- Inexpensive Chinese ST-LINK adapter   
![ST-LINK-1](https://user-images.githubusercontent.com/6020549/221065783-33508ebe-2454-4033-92f8-34c00fe0eb80.JPG)
![platformio-2](https://user-images.githubusercontent.com/6020549/221065793-a32da243-946a-4cf4-9655-1347a229d6eb.JPG)

- UART-USB converter module   

- MotionTracking device with DMP function   
 MPU6000/6050/6500/6555/9150/9225/9250/9255 are abailable.   
 MPU9150/9225/9250/9255PU9250 is a package that integrates a 3-axis magnetic sensor (AK8963).   
 Since this sample uses DMP, it does not use a 3-axis magnetic sensor.   
 You can use these to get Euler angles and quaternions angles.   

I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) library to get data from MPU6050 using DMP.   
IMU_Zero is based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/IMU_Zero).   
ros code is based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6).   

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
$ cd Arduino-STM32-rosnode/mpu6050/IMU_Zero
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
$ cd Arduino-STM32-rosnode/mpu6050-dmp
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
$ pio run -e blackpill_f103c8 -t upload
```

Build Firmware for generic board
```
$ cd ..
$ pio run -e genericSTM32F103C8 -t upload
```


# Wireing to sensor
|MPU6050/MPU9250||STM32|
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
data: "device id is 0x71"
---
data: "roll:  0.09 pitch: -0.27 yaw:  0.41"
---
data: "q.x:  0.00 q.y:  0.00 q.z: -0.00 q.w:  1.00"
---


$ rostopic echo pose
header:
  seq: 2898
  stamp:
    secs: 1678673250
    nsecs:  48445880
  frame_id: "map"
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.00482177734375
    y: 0.001953125
    z: -0.0032958984375
    w: 0.999938964844
---
```


### View Quatanion using rviz
With rviz you can see the imu pose.   
- start rviz   
- Add->Pose   
- Pose->Topic->/pose   
- Pose->Shape->Axes   

![mpu6050_2023-02-27_11-09-30](https://user-images.githubusercontent.com/6020549/221465372-5d1eae00-faf4-4e69-9d5e-8cb496b5968b.png)
![mpu6050_2023-02-27_11-09-58](https://user-images.githubusercontent.com/6020549/221465379-80ccc688-031b-44f6-a2fe-fd519e743923.png)
![mpu6050_2023-02-27_11-10-22](https://user-images.githubusercontent.com/6020549/221465383-455a72c6-abca-4124-9613-dbb29965c343.png)


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
RAM:   [==========]  109.5% (used 2243 bytes from 2048 bytes)
Flash: [========  ]  79.2% (used 25532 bytes from 32256 bytes)
```

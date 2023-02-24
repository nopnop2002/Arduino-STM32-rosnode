# bmp085 rosnode
A demo that captures data from an i2c sensor and includes it in ROS.   



# Hardware requirements
- BMP085 Digital pressure sensor


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
$ cd Arduino-STM32-rosnode/bmp085
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
|BMP085||STM32|
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
data: "hello world! 2131011"
---
data: "hello world! 2131511"
---
data: "hello world! 2132011"
---


$ rostopic echo /temperature
data: 19.0
---
data: 19.0
---
data: 19.1000003815
---


$ rostopic echo /pressure
data: 101441
---
data: 101449
---
data: 101449
---


$ rostopic echo /altitude
data: -11.3164596558
---
data: -11.0673265457
---
data: -10.3188858032
---


$ rostopic echo /sealeve
data: 101452
---
data: 101453
---
data: 101456
---
```


# Using other board
STM32 SCA and SDA are valiant dependent.   
The i2c definition of each board cannot be understood without looking at the variant_generic.h of each board published [here](https://github.com/stm32duino/Arduino_Core_STM32/tree/main/variants).   
This is a sample of some boards.   
|Bord valiant|SDA|SCL|
|:---|:---|:---|
|blackpill_f103c8|PB7|PB6|
|bluepill_f103c8|PB7|PB6|
|genericSTM32F103xx|PB7|PB6|
|genericSTM32F303xx|PB7|PB6|
|blackpill_f401cc|PB7|PB6|
|genericSTM32F401CC|PB3|PB10|
|blackpill_f411ce|PB7|PB6|
|genericSTM32F411CE|PB3|PB10|
|genericSTM32F405RG|PB7|PB6|
|black_f407vg|PB9|PB8|
|diymore_f407vgt|PB7|PB6|
|genericSTM32F407VGT6|PB9|PB8|
|disco_f407vg|PB7|PB8|
# RotaryEncoder rosnode
A demo that captures data from an RotaryEncoder and includes it in ROS.   

# Hardware requirements
- STM32 Development Board with 5V pin header.   
 BlackPill does not have a 5V pin header, so it cannot be used.   

- Inexpensive Chinese ST-LINK adapter   
![ST-LINK-1](https://user-images.githubusercontent.com/6020549/221065783-33508ebe-2454-4033-92f8-34c00fe0eb80.JPG)
![platformio-2](https://user-images.githubusercontent.com/6020549/221065793-a32da243-946a-4cf4-9655-1347a229d6eb.JPG)

- UART-USB converter module   

- RotaryEncoder like EC11   
There are two types of rotary encoders, but their functions are the same.   

![rotary-encoder](https://user-images.githubusercontent.com/6020549/223601998-fad9af5d-0bae-4e27-9374-a9eff1055330.jpeg)

# Wireing to sensor
|RotaryEncoder||STM32|
|:---|:---|:---|
|Out A||PB6|
|Out B||PB7|
|Switch||PB8|
|VCC||3.3V|
|GND||GND|

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
$ cd Arduino-STM32-rosnode/RotaryEncoder
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
data: "InterruptCount=315 InterruptSwitch=0 Count=4 StatePinSW=0"
---
data: "InterruptCount=326 InterruptSwitch=0 Count=7 StatePinSW=0"
---

$ rostopic echo /position
layout:
  dim: []
  data_offset: 0
data: [6, 0]
---
layout:
  dim: []
  data_offset: 0
data: [5, 0]
---
```


### Visualization
```
rqt_plt rqt_plt /position/daa[0]
```

![RotaryEncoder_2023-03-08_11-10-18](https://user-images.githubusercontent.com/6020549/223601948-9b38c9c2-c667-4cc7-9cf4-e67772e2ee96.png)


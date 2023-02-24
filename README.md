# Arduino-STM32-rosnode
rosnode example using STM32.   

rosserial_arduino_lib allows you to communicate with Arduino environments such as ATMega328(UNO,Nano,ProMini) and ATMega2560(MEGA) via serial.   
Data from various sensors can be imported into ROS using Arduino.   
However, ATMega328 is not suitable as rosnode because memory is too small.   
ATMega2560 is fine, but the board is too big.

We can use compact and powerful STM32 Development Board as rosnode.   
STM32 has enough memory for rosnode and runs much faster than ATMega.   

# Development environment
- ROS Melodic   
- PlatformIO Core   
- Arduino_Core_STM32   


# Hardware requirements
- STM32 Development Board like bluepill/Blackpill  

- Inexpensive Chinese ST-LINK adapter   
![ST-LINK-1](https://user-images.githubusercontent.com/6020549/221065783-33508ebe-2454-4033-92f8-34c00fe0eb80.JPG)
![platformio-2](https://user-images.githubusercontent.com/6020549/221065793-a32da243-946a-4cf4-9655-1347a229d6eb.JPG)

- UART-USB converter module   

# Install PlatformIO Core
```
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U platformio
$ pio --version
PlatformIO Core, version 6.1.6
```

# Install dependent libraries
```
$ sudo apt install ros-melodic-rosserial-arduino ros-melodic-rosserial
```

# Grant access to USB ports and add rules
```
$ sudo usermod -a -G dialout {login_user_name}
$ sudo apt install curl
$ curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
$ ls /etc/udev/rules.d/99-platformio-udev.rules
/etc/udev/rules.d/99-platformio-udev.rules
$ sudo reboot
```

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
$ cd Arduino-STM32-rosnode/HelloWorld
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
```




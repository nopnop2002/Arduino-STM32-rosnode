# Arduino-STM32-rosnode
rosnode example using STM32.   

rosserial_arduino_lib allows you to communicate with Arduino environments such as ATMega328(UNO,Nano,ProMini) and ATMega2560(MEGA) via serial.   
Data from various sensors can be imported into ROS using Arduino.   
However, ATMega328 is not suitable as rosnode because memory is too small.   
ATMega2560 is fine, but the board is too big.

We can use compact and powerful STM32 Development Board as rosnode.   
STM32 has enough memory to run rosnode and is much faster than ATMega.   
Arduino environment has many libraries that we can use.

# Development environment
- ROS Melodic   
- PlatformIO Core   
- Arduino_Core_STM32   


# Hardware requirements
- STM32 Development Board like Bluepill/Blackpill  
 Supported boards is [here](https://github.com/stm32duino/Arduino_Core_STM32).   

- Inexpensive Chinese ST-LINK adapter   
 Available on eBay and AliExpress for $2.   
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

/dev/stlinkv2* device files are generated.   
```
$ ls /dev/stlinkv2*
/dev/stlinkv2_4
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
|      5V  |---------|5V        |         |          |
|      GND |---------|GND       |         |          |
+----------+         +----------+         +----------+

Build with generic board
+----------+         +----------+         +----------+
|  STM32   |   UART  | UART-UART|  USB    | ROS HOST |
|      PA2 |---------|RX        |=========|          |
|      PA3 |---------|TX        |         |          |
|      5V  |---------|5V        |         |          |
|      GND |---------|GND       |         |          |
+----------+         +----------+         +----------+
```

When the PlatformIO host and ROS host are the same, it will be as follows:   
___Do not supply power from both ST-LINK and UART-USB.___    
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
|      5V  |         |5V        |         |          |
|      GND |---------|GND       |         |          |
+----------+         +----------+         +----------+
```

If the quality of the UART wire is poor, you will get an error like this.   
```
[ERROR] [1677403411.453212]: Mismatched protocol version in packet ('\x12'): lost sync or rosserial_python is from different ros release than the rosserial client
[INFO] [1677403411.456146]: Protocol version of client is unrecognized, expected Rev 1 (rosserial 0.5+)
[ERROR] [1677403411.950751]: Mismatched protocol version in packet ('\xff'): lost sync or rosserial_python is from different ros release than the rosserial client
[INFO] [1677403411.953727]: Protocol version of client is Rev 0 (rosserial 0.4 and earlier), expected Rev 1 (rosserial 0.5+)
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
```

### Terminal 4
```
# Onboard led gose LOW
$ rostopic pub -1 /led std_msgs/Bool "data: false"
# Onboard led gose HOGH
$ rostopic pub -1 /led std_msgs/Bool "data: true"
```

There are boards that turn off when it goes LOW and boards that turn off when it goes HIGH.   
This is hardware implementation dependent.   
GPIO of onboard LED is set in platform.ini.   
Settings for boards other than BluePill and BlackPill are provisional.   
```
build_flags = '-D LED_BUILTIN=PC13'
```


# Using other board
rosserial_arduino_lib uses Serial object for ROS communication.   
The hardware serial of STM32 is complicated, and the pin assignment of the Serial object has different GPIO for each valiant.   
For example, blackpill_f103c8/bluepill_f103c8 Serial objects are assigned TX=PA9 RX=PA10, but genericSTM32F103C8 Serial objects are assigned TX=PA2 RX=PA3.   
In other words, even with the same STM32F103C8, the GPIO of Sirial changes depending on the build conditions.   
Additionally, the serial GPIO can be remapped to a alternate pins by code in your application.   

__STM32F4 is especially complicated.__   
This is a sample of some boards.   
|Bord valiant|TX of Serial|RX of Serial|MCU|
|:---|:---|:---|:---|
|blackpill_f103c8|PA9|PA10|STM32F103C8|
|bluepill_f103c8|PA9|PA10|STM32F103C8|
|genericSTM32F103C8|PA2|PA3|STM32F103C8|
|blackpill_f401cc|PA9|PA10|STM32F401CC|
|genericSTM32F401CC|PA2|PA3|STM32F401CC|
|blackpill_f411ce|PA9|PA10|STM32F411CE|
|genericSTM32F411CE|PA2|PA3|STM32F411CE|
|genericSTM32F405RG|PA0|PA1|STM32F405RG|
|diymore_f407vgt|PA9|PA10|STM32F407VGT|
|black_f407vg|PA2|PA3|STM32F407VGT|
|genericSTM32F407VGT6|PA0|PA1|STM32F407VGT|
|disco_f407vg|PA2|PA3|STM32F407VGT|

The UART definition of each board cannot be understood without looking at the variant_generic.h of each board published [here](https://github.com/stm32duino/Arduino_Core_STM32/tree/main/variants).   
Some definitions are intended only for specific boards, such as variant_DIYMORE_F407VGT.h and variant_BLACK_F407VX.h.   
It is necessary to confirm the mapping of the Serial object in advance.


# Default upload protocol
There are variants where the default upload protocol is not ST-LINK.   
[Here](https://docs.platformio.org/en/stable/boards/ststm32/genericSTM32F405RG.html) is the documentation for the generic STM32F405RG but the default upload protocol is serial.   
It would be a mistake to think that all STM32 have ST-LINK as the default protocol.   
If you look closely at the messages when writing, you can see that the upload protocol is serial.   
```
Configuring upload protocol...
AVAILABLE: blackmagic, dfu, jlink, serial, stlink
CURRENT: upload_protocol = serial
Looking for upload port...

Warning! Please install `99-platformio-udev.rules`.
More details: https://docs.platformio.org/page/faq.html#platformio-udev-rules

Error: Please specify `upload_port` for environment or use global `--upload-port` option.
For some development platforms it can b
```

If ST-LINK is not the default protocol, you need to specify the protocol as follows.
```
[env:genericSTM32F405RG]
platform = ststm32
board = genericSTM32F405RG
framework = arduino
upload_protocol = stlink
```


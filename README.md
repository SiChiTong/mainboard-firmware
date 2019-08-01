# RoboSub19 Electronics
ITU AUV RoboSub Electronics Firmware + Embedded


## ROS Integration
Ros integration is done through the rosserial libraries, a small modification has been made in the core libraries
such as ros.h for STM32F4ETHERNET support.

### What works
Serial connection with 57600 (or lower) baud rates have been tested and verified to work, above such speeds there are message losses, the possible solution is stated in the next steps.

### What does not work
Ethernet connection works but it is slow and does not act as expected. There should be a fix, todo. :wink



## USB Serial Port Latency Problem
The current version, will have too many checksum errors if baudrates higher than 57600 is used,
this might be due to latency problem.

It is reported that the /sys/bus/usb-serial/devices/ttyUSB0/latency_timer became 16 after Linux Kernel 4.10, which causes big latency while using usb-serial. This also significantly influences the performance of rosserial.

Several solution has been proposed:

$ setserial /dev/ttyUSB0 low_latency
direct edition: sudo bash -c "echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
add ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1" in udev rules.
To avoid the latency, I suggest to apply any of the above solution before running rosserial

- tongtybj
https://github.com/ros-drivers/rosserial/issues/383

The hardware serial port has been changed from PD5&PD6 to PD2&PC12, and currently 115200 baud rate works, but above that still the same problem.

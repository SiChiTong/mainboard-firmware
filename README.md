# Mainboard Firmware

## Overview

This repository is a source code of the firmware that is to be uploaded on Nucleo STM32F429ZI hardware. The STM32, is attached to a custum built board (PCB).

### License

The source code is released under [MIT License](mainboard-firmware/LICENSE).

### Authors & Maintainers
**Author:** Sencer Yazici, [senceryazici@gmail.com](mailto:senceryazici@gmail.com)
**Maintainers:** ITU Auv Electronics Sub-Team

### Working Environment
This repository consists packages of different platforms in a single folder. As a ros package, mainboard_firmware package is used with Ubuntu 18.04 and [ROS] Melodic. As a PlatformIO development package, it is used and developed with latest PlatformIO in VSCode. In order to function as a package for both platforms, [platformio.ini](mainboard-firmware/platformio.ini) and [package.xml](mainboard-firmware/package.xml) files must be kept.


## Connection & Hardware
As a microcontroller ST Nucleo F429ZI is used.
![Nucleo STM32F429ZI](https://os.mbed.com/media/cache/platforms/Nucleo144_perf_logo_1024_qTjTDC0.jpg.250x250_q85.jpg)

#### Todo: 
- add connection diagram


## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org)
- [PlatformIO] 
    Install VSCode (or supported editors), and install PlatformIO extension.
- [AutoPID], [ping-arduino]
    Provided under [lib/](mainboard-firmware/lib)

#### Building

To build from source, clone to repository to a catkin workspace, and use catkin build command

	cd catkin_ws/src
	git clone https://gitlab.com/itu-auv/electronics/mainboard-firmware.git
	cd ../
	catkin build 

This will build the ros package thus, you'll be able to use messages, launch files, configs etc. To upload the source code to the microcontroller, use PlatformIO's "Upload" button in VSCode, or your editor.

## Usage

After flashing the board, to establish the connection to the microcontroller:

	roslaunch mainboard_firmware start_jetson.launch

## Config files

* **default.yaml** Contains the configuration parameters for Microcontroller, PID and Thruster Allocation Matrix is provided here.

## Launch files
The main launch files will be named as start_<PLATFORM>, where the platform may be OS X, Jetson, Ubuntu etc.

* **start_jetson.launch:** Starts the necessary nodes to connect to the microcontroller and other features.

     Connection Argument

     - **`connection_type`** port and baud rate argument. Default: `/dev/ttyTHS0 _baud:=115200`.

* **start_osx.launch:** Starts the necessary nodes to connect to the microcontroller and other features.

     Connection Argument

     - **`connection_type`** port and baud rate argument. Default: `/dev/cu.usbserial _baud:=57600`.

## Nodes

### rosserial_python

Since microcontrollers using rosserial, aren't directly registered as node in ROS, a bridge node is implemented.
This node acts as a bridge between the node running in microcontroller and ROS.

#### Published Topics

* **`/turquoise/sensors/sonar/bottom`** ([sensor_msgs/Range])

	The range measurements computed from the bottom ping sonar ([ping-1d]) from BlueRobotics.

* **`/turquoise/battery_state`** ([sensor_msgs/BatteryState])

	The necessary information of battery and real time voltage and current readings.

* **`/turquoise/thrusters/current`** ([std_msgs/Float32MultiArray])

	The current readings in amperes (A) computed from the current sensors located on board for each thruster.

#### Subscribed Topics

* **`/turquoise/signal`** ([mainboard_firmware/Signal])

	The Signal messages that can control some features of the Microcontroller in real-time.
    List of signals that can be used:
    | Signal Type   | Signal Content| Description |
    |---|---|---|
    | pinmode_output| PIN_NUMBER | Registers the provided pin with PIN_NUMBER as output pin. |
    | digital_set | PIN_NUMBER | Sets the provided pin (HIGH) |
    | digital_reset | PIN_NUMBER | Resets the provided pin (LOW) |
    | digital_toggle | PIN_NUMBER | Toggles the provided pin |
    | change_indicator_freq | FREQUENCY| Changes the flasing rate of the indicator led. |

* **`/turquoise/cmd_vel`** ([geometry_msgs/Twist])

	The velocity references for each axis, Linear X,Y,Z and Rotational X,Y,Z that are used to command the vehicle. A minimum frequency of 2.5 Hz must be provided in order to work without timeout (time > 400 ms). If messages are sent with time > 400 ms, the motors would stop after each reference.

* **`/turquoise/odom`** ([mainboard_firmware/Odometry])

	The odometry of the vehicle, generated from [nav_msgs/Odometry] message, excluding the covariances since it makes the communication unstable due to long message.

* **`/turquoise/thrusters/input`** ([std_msgs/Int16MultiArray])

	Directly controls each motor with given PWM signals (not thrust). The range of \[1100 - 1900\] is allowed. ALLOW_DIRECT_CONTROL Macro must be defined on the firmware, in order for this option to work.


#### Macro's


- [main.h](mainboard-firmware/include/main.h)
    | MACRO | DESCRIPTION |
    |--|--|
    | ALLOW_DIRECT_CONTROL | Enables the feature of direct motor control from the topics when defined | 
    | ENABLE_SONARS | Enables the ping sonars when defined | 
    | DEBUG_PRINT |  Enables the ping sonars when defined | 

- [ros_ethernet.h](mainboard-firmware/include/ros_ethernet.h)
    | MACRO | DESCRIPTION |
    |--|--|
    | STM32ETHERNET | Defined to initialize ros.h with ethernet connection of STM32 | 
    | ROSSERIAL_ARDUINO_TCP | Defined to initialize ros.h with TCP communication mode | 

- [ros_serial.h](mainboard-firmware/include/ros_serial.h)
    | MACRO | DESCRIPTION |
    |--|--|
    | USE_STM32_HW_SERIAL | Defined to initialize ros.h with Hardware Serial communication mode | 


#### Parameters

##### ROS Parameters
* **`allocation_cx`** (FloatArray, default: "[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Cartesian (Linear) X.

* **`allocation_cy`** (FloatArray, default: "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -1.0]")

	The row of the Thruster Allocation Matrix corresponding to Cartesian (Linear) Y.

* **`allocation_cz`** (FloatArray, default: "[1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Cartesian (Linear) Z.

* **`allocation_rx`** (FloatArray, default: "[1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Rotational (Angular) x.

* **`allocation_ry`** (FloatArray, default: "[-1.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Rotational (Angular) Y.

* **`allocation_rz`** (FloatArray, default: "[0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0]")

	The row of the Thruster Allocation Matrix corresponding to Rotational (Angular) Z.


* **`pid_cx`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Cartesian (Linear) X.

* **`pid_cy`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Cartesian (Linear) Y.

* **`pid_cz`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Cartesian (Linear) Z.

* **`pid_rx`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Rotational (Angular) x.

* **`pid_ry`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Rotational (Angular) Y.

* **`pid_rz`** (FloatArray, default: "[1.0, 0.0, 0.0]")

	The PID Gains for Proportional Integral and Derivative respectively \[Kp, Ki, Kd\] corresponding to Rotational (Angular) Z.


##### Firmware Parameters

- [params.h](mainboard-firmware/include/params.h)
    | PARAMETER | DEFAULT | TYPE | DESCRIPTION |
    |--|--|--|--|
    | ACS712_30A_SENS_MV_PER_AMP | 66.0 | mV / A| 66 mV per 1.A | 
    | ACS712_20A_SENS_MV_PER_AMP | 100.0 | mV / A| 100 mV per 1.A |
    | ADC_READ_RESOLUTION_BIT | 12 | bit | 12 bits for analog read resolution |
    | ADC_READ_MAX_VALUE | 4096.0 | unit | 2^ADC_READ_RESOLUTION_BIT = 4096 |
    | ADC_READ_MAX_VOLTAGE | 3300.0 | mV | 12 bits for analog read resolution max 3.3V |
    | ADC_OFFSET_CURRENT_ERROR | 800.0 | mA | 800 mA offset error. |
    | INDICATOR_TIMER | TIM8 | Timer | Timer instance of indicator Led. (LED_GREEN)|
    | CURRENT_COUNT_TIMER | TIM9 | Timer |  Timer instance of current count. Reads current |
    | CURRENT_COUNT_INTERVAL | 0.1 | seconds | time between each measurement |
    | CURRENT_SENS_PIN | PC2 | Pin | ADC Pin |
    | CURRENT_SENS_MAX_AMPS | 60.0 | A | Max measurable amps for sensor |
    | VOLTAGE_SENS_PIN | PB1 | Pin | ADC Pin |
    | VOLTAGE_SENS_MAX_VOLTS | 18.0 | V | 4S Max |
    | ADC_TO_VOLTAGE_RATIO | 0.0085 | unit | Module parameter |
    | ADC_TO_CURRENT_RATIO | 0.0137134308511 | unit | Module parameter |
    | MIN_BATT_VOLTAGE | 9.0 | V | 3S Batt Min voltage |
    | LOW_BATT_VOLTAGE | 9.5 | V | Low Voltage warning |
    | PING_TIMEOUT | 1000 | ms | Timeout value for ping sonar |
    | ROS_BAUDRATE | 115200 | bits / second (bps)| Ros Communication Baud Rate |
    | DEBUG_BAUDRATE | 57600 | bits / second (bps) | Default Serial port baud rate. (For DEBUG use.) |
    | MOTOR_TIMEOUT | 400 | ms | the max time between each motor cmd allowed |

- [motor_config.h](mainboard-firmware/include/motor_config.h)
    | PARAMETER | DEFAULT | TYPE | DESCRIPTION |
    |--|--|--|--|
    | MOTOR_PULSE_RANGE | 400 | uS | the range of motor pulse starting from 1500 uS | 
    | DEFAULT_PULSE_WIDTH | 1500 | uS | Default motor off pulse |
    | MIN_PULSE_WIDTH | DEFAULT_PULSE_WIDTH - MOTOR_PULSE_RANGE | uS | Min uS pulse time |
    | MAX_PULSE_WIDTH | DEFAULT_PULSE_WIDTH + MOTOR_PULSE_RANGE | uS | Max uS pulse time |
    | POS_FIT_P1 | -0.1717 | unit | Value of a in, a\*s^2 + b\*s + c for Thrust-PWM Curve Fit |
    | POS_FIT_P2 | 18.2 | unit | Value of b in, a\*s^2 + b\*s + c for Thrust-PWM Curve Fit |
    | POS_FIT_P3 | 1543 | unit | Value of c in, a\*s^2 + b\*s + c for Thrust-PWM Curve Fit |
    | NEG_FIT_P1 | 0.6732 | unit | Value of a in, a\*s^2 + b\*s + c for Thrust-PWM Curve Fit |
    | NEG_FIT_P2 | 32.1 | unit | Value of b in, a\*s^2 + b\*s + c for Thrust-PWM Curve Fit |
    | NEG_FIT_P3 | 1460 | unit | Value of c in, a\*s^2 + b\*s + c for Thrust-PWM Curve Fit |

- [ros_serial.h](mainboard-firmware/include/ros_serial.h)
    | PARAMETER | DEFAULT | TYPE | DESCRIPTION |
    |--|--|--|--|
    | XAVIER_RX | PD2 | Pin | Rx Pin used to connect to the ROS Computer |
    | XAVIER_TX | PC12 | Pin | Tx Pin used to connect to the ROS Computer |

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://gitlab.com/itu-auv/electronics/mainboard-firmware/issues).
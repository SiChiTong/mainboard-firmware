#define XAVIER_RX                        PD6     // NUCLEO-F429ZI UART2-RX
#define XAVIER_TX                        PD5     // NUCLEO-F429ZI UART2-TX
#include "HardwareSerial.h"
HardwareSerial XavierSerial(XAVIER_RX, XAVIER_TX);
#define Serial1 XavierSerial
#define USE_STM32_HW_SERIAL


// #include <ros.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Bool.h>
// #include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
// #include "Servo.h"
// ros::NodeHandle nh;

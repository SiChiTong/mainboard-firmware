// MIT License

// Copyright (c) 2019 ITU AUV Team / Electronics

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#define XAVIER_RX                        PD2 //PD6     // NUCLEO-F429ZI UART2-RX
#define XAVIER_TX                        PC12 //PD5     // NUCLEO-F429ZI UART2-TX
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

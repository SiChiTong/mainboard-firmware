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

#include "Arduino.h"
#include "motor_config.h"
#include "error_codes.h"
#include "params.h"

#if defined(USE_ETHERNET)
    #include "ros_ethernet.h"
#else
    #include "ros_serial.h"
#endif

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Bool.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include "Servo.h"
#include <mainboard_firmware/Signal.h>
#include <sensor_msgs/Range.h>
#include "ping1d.h"

/* *************************** Callbacks *************************** */

void motor_callback(const std_msgs::Int16MultiArray& data);
void command_callback(const mainboard_firmware::Signal& data);
static void indicator_callback(stimer_t *htim);
void InitializeIndicatorTimer(uint32_t frequency);
void SetFrequencyOfIndicatorTimer(uint32_t frequency);
void InitializePeripheralPins();
void PublishInfo(String msg);
void EvaluateCommand(String type, String content);
void InitializePingSonarDevices();

/* *************************** Variables *************************** */

/* Node Handle
 */
ros::NodeHandle nh;

/* ESC init.
 */
Servo motors[8];

/* PING SONAR CONFIGURATION
 */
HardwareSerial hwserial_3(PD2, PC12);
static Ping1D ping_1 { hwserial_3 };

// New design
int current_sens_pins[8] = {PA0, PC0, PC0, PC0, PA3, PA6, PA7, PB6};
int aux_pinmap[3] = {PD12, PD13, PD14};

/* Variables
 */
uint32_t last_motor_update = millis();
bool last_state = false;  //motor disabled

/* Publisher Messages
 */
std_msgs::String info_msg;
std_msgs::String error_msg;
std_msgs::Float32MultiArray current_msg;
sensor_msgs::Range range_msg;

/* Publishers
 */
ros::Publisher diagnose_info("/turquoise/board/diagnose/info", &info_msg);
ros::Publisher diagnose_error("/turquoise/board/diagnose/error", &error_msg);

ros::Publisher motor_currents("/turquoise/thrusters/current", &current_msg);

ros::Publisher ping_1_pub("/turquoise/sensors/sonar/front", &range_msg);

/* Subscribers
 */
ros::Subscriber<std_msgs::Int16MultiArray> motor_subs("/turquoise/thrusters/input", motor_callback);
ros::Subscriber<mainboard_firmware::Signal> command_sub("/turquoise/board/cmd", command_callback);

/* *************************** Functions *************************** */

/* Initialize NodeHandle with ethernet or serial
 */
void InitNode()
{
    #if defined(USE_ETHERNET)
        Ethernet.begin(mac, ip);
        delay(1000);
        nh.getHardware()->setConnection(server, serverPort);
        nh.initNode();
    #else
        nh.getHardware()->setBaud(ROS_BAUDRATE);
        nh.initNode();
    #endif
}

void PublishInfo(String msg)
{
    if (INFO_STREAM_ENABLE)
    {
        info_msg.data = msg.c_str();
        diagnose_info.publish(&info_msg);
        info_msg.data = "";
    }
}

void EvaluateCommand(String type, String content)
{
    if (String(type) == "pinmode_output")
    {
        int pin = String(content).toInt();
        pinMode(pin, OUTPUT);
    }
    else if (String(type) == "digital_set")
    {
        int pin = String(content).toInt();
        digitalWrite(pin, HIGH);
    }
    else if (String(type) == "digital_reset")
    {
        int pin = String(content).toInt();
        digitalWrite(pin, LOW);
    }
    else if (String(type) == "digital_toggle")
    {
        int pin = String(content).toInt();
        digitalToggle(pin);
    }
    else if (String(type) == "change_indicator_freq")
    {
        int freq = String(content).toInt();
        SetFrequencyOfIndicatorTimer(freq);
    }
    else
    {
        nh.logerror(String("PRM_ERR:" + String(COMMAND_EVAL_FAILED)).c_str());
    }
}

void InitializeIndicatorTimer(uint32_t frequency)
{
    static stimer_t TimHandle;
    TimHandle.timer = INDICATOR_TIMER;

    double prescaler = 20000;
    double period = ( (getTimerClkFreq(INDICATOR_TIMER)) / (double)(prescaler+1) / frequency) - 1;

    TimerHandleInit(&TimHandle, (uint32_t)period, (uint32_t)prescaler);

    attachIntHandle(&TimHandle, indicator_callback);
}

void SetFrequencyOfIndicatorTimer(uint32_t frequency)
{
    double prescaler = 20000;
    double period = ( (getTimerClkFreq(INDICATOR_TIMER)) / (double)(prescaler+1) / frequency) - 1;

    INDICATOR_TIMER->ARR = (uint32_t)(period - 1);
    INDICATOR_TIMER->CNT = 0;
}

void SpinIndicatorTimer()
{
    bool state_change = (millis() - last_motor_update > MOTOR_TIMEOUT) ^ last_state;
    last_state = millis() - last_motor_update > MOTOR_TIMEOUT;

    if (state_change)
    {
        if (last_state == 1)
        {
            SetFrequencyOfIndicatorTimer(1);
        }
        else
        {
            SetFrequencyOfIndicatorTimer(10);
        }
    }
}

void InitializePeripheralPins()
{
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
}

void InitializeHardwareSerials()
{
    hwserial_3.begin(115200);
}

void InitializePingSonarDevices()
{
    bool init_state = true;
    init_state = init_state && ping_1.initialize(PING_PULSE_TIME);

    // if () PublishInfo("ping_1 device on hwserial_3(PD2, PC12) Initialized successfully");
    // else PublishInfo("ping_1 device on hwserial_3(PD2, PC12) failed to initialize.");

    // if (ping_2.initialize()) PublishInfo("ping_2 device on hwserial_3(PD2, PC12) Initialized successfully");
    // else PublishInfo("ping_2 device on hwserial_3(PD2, PC12) failed to initialize.");

    if (!init_state) nh.logerror(String("PRM_ERR:" + String(PING_SONAR_INIT_FAILED)).c_str());
}

void PublishPingSonarMeasurements()
{
    range_msg.max_range = 30;
    range_msg.min_range = 0.5;
    range_msg.field_of_view = 30.0;
    range_msg.radiation_type = range_msg.ULTRASOUND;
    if (ping_1.request(Ping1DNamespace::Distance_simple, PING_TIMEOUT))
    {
        range_msg.range = ping_1.distance() / 1000.0;
        ping_1_pub.publish(&range_msg);
    }
    else
    {
        nh.logerror(String("PRM_ERR:" + String(PING_SONAR_READ_FAILED)).c_str());
    }
}

void InitializeCurrentsMessage()
{
    current_msg.layout.dim = (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    current_msg.layout.dim[0].label = "motor_currents";
    current_msg.layout.dim[0].size = 8;
    current_msg.layout.dim[0].stride = 1*8;
    current_msg.layout.data_offset = 0;
    // current_msg.layout.dim_length = 1;
    current_msg.data_length = 8;
    current_msg.data = (float *)malloc(sizeof(float)*8);
}



// End of file.

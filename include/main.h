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


/* DEPRECEATED, DO NOT USE ETHERNET MODE.
 * Built in PlatformIO packages for ststm32
 * and framework-arduinoststm32 have been updated
 * to last version, and the last version does not 
 * support s_timer types and TypeDefs, they've 
 * implemented a new HardwareTimer class, 
 * thus Stm32Ethernet library is no longer 
 * working. 
 * HardwareTimer Library: 
 * https://github.com/stm32duino/wiki/wiki/HardwareTimer-library#Introduction
 * 
 * 
 * Open Pull Request here: https://github.com/stm32duino/STM32Ethernet
 */
// #define USE_ETHERNET
#define DEBUG_PRINT
#define ALLOW_DIRECT_CONTROL


/* *************************** Includes *************************** */
#include <Arduino.h>
#include <Servo.h>
#include <motor_config.h>
#include <error_codes.h>
#include <params.h>
#include <debugging.h>

#if defined(USE_ETHERNET)
    #include <ros_ethernet.h>
#else
    #include <ros_serial.h>
#endif

#include <HardwareTimer.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
#include <mainboard_firmware/Odometry.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <mainboard_firmware/Signal.h>
#include <sensor_msgs/Range.h>
#include <ping1d.h>
#include <transformations.h>

#include <AutoPID.h>
#include <math.h>
/* *************************** Includes *************************** */


/* *************************** Callbacks *************************** */
void cmd_vel_callback(const geometry_msgs::Twist& data);
void odometry_callback(const mainboard_firmware::Odometry& data);
void motor_callback(const std_msgs::Int16MultiArray& data);
void command_callback(const mainboard_firmware::Signal& data);
static void HardwareTimer_Callback(HardwareTimer* htim);
void InitNode();
void InitSubPub();
void GetThrusterAllocationMatrix();
void GetPIDControllerParameters();
void UpdatePIDControllerGains();
void PerformHaltModeCheck();
void InitMotors();
void ResetMotors();
void PublishInfo(String msg);
void RunPIDControllers(double* output, double dt);
void EvaluateCommand(String type, String content);
void InitializeIndicatorTimer(uint32_t frequency);
void TimeoutDetector();
void InitializePeripheralPins();
void InitializeHardwareSerials();
void InitializePingSonarDevices();
void PublishPingSonarMeasurements();
void PublishMotorCurrents(int);
void InitializeCurrentsMessage();



/* *************************** Variables *************************** */
/* Node Handle
 */
ros::NodeHandle nh;

/* Motors define.
 */
Servo motors[8];

/* HardwareTimers
 */
HardwareTimer *IndicatorTimer;
HardwareTimer *CurrentCounterTimer;
/* PING SONAR CONFIGURATION
 */
HardwareSerial hwserial_3(PD2, PC12);
static Ping1D ping_1 { hwserial_3 };

/* This section includes pinmaps for current sensors and auxilary channel pinmaps
 * currently, mainboard does not support such features
 * therefore these features are not tested yet.
 */
int current_sens_pins[8] = {PA0, PC0, PC0, PC0, PA3, PA6, PA7, PB6};
int aux_pinmap[3] = {PD12, PD13, PD14};

/* Global Variables
 */
uint32_t last_motor_update = millis();
bool last_motor_timeout_state = false;  //motor disabled
double current_consumption_mah = 0;
double main_current = 0;
double main_voltage = 0;

/**
 * @brief PID Controllers
 * 
 */
float thruster_allocation[6][8];
float pid_gains[6][3];
double v[6] = {0, 0, 0, 0, 0, 0}; // Velocity & Ang. Velocity in 6 DOF.
double n[6] = {0, 0, 0, 0, 0, 0}; // Position & Orientation in 6 DOF.
double velocity_setpoint[6] = {0, 0, 0, 0, 0, 0};
unsigned long last_odom_update = millis();
AutoPID controllers[6] = {
    AutoPID(-MOTOR_PULSE_RANGE, MOTOR_PULSE_RANGE, 1.0, 0.0, 0.0), // Pos_X
    AutoPID(-MOTOR_PULSE_RANGE, MOTOR_PULSE_RANGE, 1.0, 0.0, 0.0), // Pos_Y
    AutoPID(-MOTOR_PULSE_RANGE, MOTOR_PULSE_RANGE, 1.0, 0.0, 0.0), // Pos_Z
    AutoPID(-MOTOR_PULSE_RANGE, MOTOR_PULSE_RANGE, 1.0, 0.0, 0.0), // Rot_X
    AutoPID(-MOTOR_PULSE_RANGE, MOTOR_PULSE_RANGE, 1.0, 0.0, 0.0), // Rot_Y
    AutoPID(-MOTOR_PULSE_RANGE, MOTOR_PULSE_RANGE, 1.0, 0.0, 0.0) // Rot_Z
    };

/**
 * @brief Publisher Messages
 */
std_msgs::String info_msg;
std_msgs::String error_msg;
std_msgs::Float32MultiArray current_msg;
sensor_msgs::Range range_msg;

/**
 * @brief Publishers
 */
ros::Publisher motor_currents("/turquoise/thrusters/current", &current_msg);
ros::Publisher ping_1_pub("/turquoise/sensors/sonar/front", &range_msg);

/**
 * @brief Subscribers
 */
#if defined(ALLOW_DIRECT_CONTROL)
ros::Subscriber<std_msgs::Int16MultiArray> motor_subs("/turquoise/thrusters/input", &motor_callback);
#endif
ros::Subscriber<mainboard_firmware::Signal> command_sub("/turquoise/signal", &command_callback);
// ros::Subscriber<mainboard_firmware::Odometry> odom_subscriber("/turquoise/odom", odometry_callback);
ros::Subscriber<mainboard_firmware::Odometry> odom_subb("/turquoise/odom", &odometry_callback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/turquoise/cmd_vel", &cmd_vel_callback);

/* *************************** Functions *************************** */
/* Initialize NodeHandle with ethernet or serial
 */
void InitNode()
{
    #if defined(USE_ETHERNET)
        // DEPRECEATED, SEE LINE 24
        debugln("[ROS_CONN] ETHERNET_INTERFACE");

        Ethernet.begin(mac, ip);

        debug("[SELF_IP] ");
        debugln(Ethernet.localIP());
        debug("[HOST_IP] ");
        debugln(server);
        debug("[HOST_PORT] ");
        debugln(serverPort);
        
        delay(1000);

        nh.getHardware()->setConnection(server, serverPort);
        nh.initNode();

        debugln("[ROS_CONN] NODE_INIT_OK!");
    #else
        debugln("[ROS_CONN] SERIAL_INTERFACE");
        debug("[ROS_CONN] BAUD:");
        debugln(ROS_BAUDRATE);
        nh.getHardware()->setBaud(ROS_BAUDRATE);
        nh.initNode();

        debugln("[ROS_CONN] NODE_INIT_OK!");
    #endif
}

void PerformUARTControl()
{
    while (Serial.available())
    {
        String value = Serial.readStringUntil('\n');
        last_motor_update = millis();
        for (size_t i = 0; i < 8; i++)
        {
            motors[i].writeMicroseconds(value.toInt());
        }
    }
}

void InitSubPub()
{
    nh.advertise(motor_currents);
    nh.advertise(ping_1_pub);

    #if defined(ALLOW_DIRECT_CONTROL)
    nh.subscribe(motor_subs);
    #endif
    nh.subscribe(odom_subb);
    
    // FIXME: Does this lines effect odom callback ?
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(command_sub);

    debug("[PUB_SUB_INIT]");
}

void GetThrusterAllocationMatrix()
{
    char *alloc_param_names[6] = {"~allocation_cx", "~allocation_cy", "~allocation_cz",
        "~allocation_rx", "~allocation_ry", "~allocation_rz"};

    for (size_t i = 0; i < 6; i++)
    {
        if (!nh.getParam(alloc_param_names[i], thruster_allocation[i], 8)) 
        { 
            for (size_t j = 0; j < 8; j++)
            {
                thruster_allocation[i][j]= 0;
            }
        }
    }
}

void GetPIDControllerParameters()
{
    char *pid_param_names[6] = {"~pid_cx", "~pid_cy", "~pid_cz",
        "~pid_rx", "~pid_ry", "~pid_rz"};
    for (size_t i = 0; i < 6; i++)
    {
        if (!nh.getParam(pid_param_names[i], pid_gains[i], 3)) 
        { 
            for (size_t j = 0; j < 3; j++)
            {
                pid_gains[i][j]= 0.0;
            }
        }
    }
}

void UpdatePIDControllerGains()
{
    for (size_t i = 0; i < 6; i++)
    {
        controllers[i].setGains((double)pid_gains[i][0], (double)pid_gains[i][1], (double)pid_gains[i][2]);
    }
    
}

void PerformHaltModeCheck()
{
    // Connection is up, skip mode check.
    if (nh.connected()) return;

    debugln("[HALT_MODE] ON! (Possible Disconnection)");
    /* *** HALT MODE ON / LOST CONNECTION *** */
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    IndicatorTimer->pause();
    ResetMotors();
    /* *** HALT MODE ON / LOST CONNECTION *** */

    while (!nh.connected()) { nh.spinOnce(); delay(1); }

    /* *** RECOVERED CONNECTION, BACK TO NORMAL *** */
    IndicatorTimer->resume();
    digitalWrite(LED_RED, LOW);
    debugln("[HALT_MODE] OFF! (Normal Operation)");

    /* *** RECOVERED CONNECTION, BACK TO NORMAL *** */

}

/* Initialize and Attach motors to spesified pins.
 * Writes default DEFAULT_PULSE_WIDTH us pulse to motors.
 */
void InitMotors()
{
    debugln("[MOTOR_INIT] " + String(DEFAULT_PULSE_WIDTH) + " uS PULSE");
    for (size_t i = 0; i < 8; i++)
    {
        while (!motors[i].attached()) { motors[i].attach(motor_pinmap[i]); } 
        motors[i].writeMicroseconds(DEFAULT_PULSE_WIDTH);
    }
}

void ResetMotors()
{
    debugln("[MOTOR_RESET]");
    for (size_t i = 0; i < 8; i++)
    {
        motors[i].writeMicroseconds(DEFAULT_PULSE_WIDTH);
    }
}

void RunPIDControllers(double* output, unsigned long dt)
{
    for (size_t i = 0; i < 6; i++)
    {
        if (i == 3 || i == 4)
        {
            // Roll and Pitch Controllers are position controller.
            output[i] = controllers[i].run(0.0, n[i], dt);
        }
        else
        {
            // Others follow cmd_vel topic.
            output[i] = controllers[i].run(velocity_setpoint[i], v[i], dt);
        }
    }
}

void EvaluateCommand(String type, String content)
{
    debug("[COMMAND_RECV] -> [" + type + ":" + content + "]");

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
        IndicatorTimer->setOverflow(freq, HERTZ_FORMAT); // 10 Hz
    }
    else
    {
        nh.logerror(String("PRM_ERR:" + String(COMMAND_EVAL_FAILED)).c_str());
    }
}

/* Init, greed led indicator timer
 */
void InitializeIndicatorTimer(uint32_t frequency)
{
    IndicatorTimer = new HardwareTimer(INDICATOR_TIMER);
    IndicatorTimer->setOverflow(frequency, HERTZ_FORMAT); // 10 Hz
    IndicatorTimer->attachInterrupt(HardwareTimer_Callback);
    IndicatorTimer->resume();
    debugln("[INDICATOR_LED_TIMER_INIT]");
}

void InitializeCurrentCounterTimer()
{
    CurrentCounterTimer = new HardwareTimer(CURRENT_COUNT_TIMER);
    CurrentCounterTimer->setOverflow(CURRENT_COUNT_INTERVAL * 1000000.0 , MICROSEC_FORMAT); 
    CurrentCounterTimer->attachInterrupt(HardwareTimer_Callback);
    CurrentCounterTimer->resume();
    debugln("[CURRENT_COUNTING_TIMER_INIT]");
}

/* Motor Timeout Detector
 * Detects whether the timeout reached for the motor commands,
 * thus disables (1500 us pulse) the motors to prevent vehicle
 * from going crazy in case of network/connection loss. :)
 * This is the savior of your billion dollar machine.
 */
void TimeoutDetector()
{
    bool state_change = (millis() - last_motor_update > MOTOR_TIMEOUT) ^ last_motor_timeout_state;
    last_motor_timeout_state = millis() - last_motor_update > MOTOR_TIMEOUT;

    if (state_change)
    {
        if (last_motor_timeout_state == 1)
        {
            IndicatorTimer->setOverflow(1, HERTZ_FORMAT); // 10 Hz
            ResetMotors();
            ResetMotors();
        }
        else
        {
            IndicatorTimer->setOverflow(10, HERTZ_FORMAT); // 10 Hz
        }
    }
}

void InitializePeripheralPins()
{
    debugln("[I/O_PINMODE]");
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
}

void InitializeHardwareSerials()
{
    debugln("[HW_SERIAL_INIT]");
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

void PublishMotorCurrents(int readings_count=5)
{
    for (size_t i = 0; i < 8; i++)
    {
        double adc_val = 0;
        for (size_t j = 0; j < readings_count; j++)
        {
            adc_val += analogRead(current_sens_pins[i]);
        }
        adc_val /= (double)readings_count;

        double voltage_mv = ADC_READ_MAX_VOLTAGE * adc_val / ADC_READ_MAX_VALUE;
        double current_ma = 1000.0 * (voltage_mv - 2500.0) / ACS712_30A_SENS_MV_PER_AMP;
        current_msg.data[i] = (float)current_ma - ADC_OFFSET_CURRENT_ERROR;
    }
    debugln("[PUB] CURRENT_SENS");
    motor_currents.publish(&current_msg);
}

void InitializeCurrentsMessage()
{
    debugln("[CURRENT_SENS_MSG_INIT]");
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

// End of file. Copyright (c) 2019 ITU AUV Team / Electronics
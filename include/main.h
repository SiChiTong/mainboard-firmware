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

#define USE_ETHERNET
#define FIRMWARE_VERSION "1.5"
#define STANDALONE_USE
// #define RELEASE_MODE


#define ALLOW_DIRECT_CONTROL
#define ENABLE_CUSTOM_SERVO

#ifndef STANDALONE_USE
/** Below functions require spesific hardware and program 
 * won't initialize without them, if you are using stm32 board
 * standalone/no additional hardware, then enable STANDALONE_USE
 * to disable hardware spesific functions
 */
#define ENABLE_SONARS
#define ENABLE_PRES_SENSOR
#define ENABLE_KILLSWITCH
#endif

/* *************************** Includes *************************** */
// Core includes
#include <Arduino.h>
#include <motor_config.h>
#include <error_codes.h>
#include <params.h>
#include <debugging.h>

#if defined(ENABLE_CUSTOM_SERVO)
    #include <CustomServo.h>
#else
    #include <Servo.h>
#endif


#if defined(USE_ETHERNET)
    #include <ros_ethernet.h>
#else
    #include <ros_serial.h>
#endif

// Ros / Rosmsg includes
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <mainboard_firmware/Odometry.h>
#include <mainboard_firmware/Signal.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>

// Other includes
#include <HardwareTimer.h>
#include <ping1d.h>
#include <transformations.h>
#include <BatteryMonitor.h>
#include <math.h>
#include <Wire.h>
#include <MS5837.h>
#include <Controller6DOF.h>
#include <tm_stm32f4_cpu_load.h>
#include <DVL.h>
/* *************************** Includes *************************** */


/* *************************** Callbacks *************************** */
void cmd_vel_callback(const geometry_msgs::Twist& data);
void odometry_callback(const mainboard_firmware::Odometry& data);
void motor_callback(const std_msgs::Int16MultiArray& data);
void command_callback(const mainboard_firmware::Signal& data);
void cmd_depth_callback(const std_msgs::Float32& data);
void aux_callback(const std_msgs::Int16MultiArray& data);
void dvl_callback(const std_msgs::String& data);

static void HardwareTimer_Callback(HardwareTimer* htim);
void arming_service_callback(const std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& resp);
void dvl_state_service_callback(const std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& resp);
void InitNode();
void InitSubPub();
void GetThrusterAllocationMatrix();
void GetPIDControllerParameters();
void PerformHaltModeCheck();
void InitMotors();
void ResetMotors();
void PublishInfo(String msg);
void EvaluateCommand(String type, String content);
void InitializeIndicatorTimer(uint32_t frequency);
bool TimeoutDetector();
void InitializePeripheralPins();
void InitializeHardwareSerials();
void InitializePingSonarDevices();
void PublishPingSonarMeasurements();
void PublishMotorCurrents(int);
void InitializeCurrentsMessage();
bool CheckBattery();
void HandlePressureSensorRoutine();
void InitPressureSensor();
void InitializeTimers();
void InitController();
void HandleArmedPublish();
void LogStartUpInfo();
void InitAux();
bool KillSwitch_isKilled();
void SystemWatchdog();
void InitializeBatteryMonitor();
void HandlePingSonarRequests();
void InitializeDVL();

/* *************************** Variables *************************** */
/* Node Handle
 */
ros::NodeHandle nh;

/* Motors define.
 */
Servo motors[8];
Servo aux[AUX_LEN];

/* HardwareTimers
 */
HardwareTimer *IndicatorTimer;
HardwareTimer *CurrentCounterTimer;
HardwareTimer *PingSonarTimer;
HardwareTimer *PublishTimer;
HardwareTimer *PressureTimer;
HardwareTimer *PIDTimer;

/* PING SONAR CONFIGURATION
 */
HardwareSerial hwserial_3 = HardwareSerial(PE7, PE8);
static Ping1D bottom_sonar { hwserial_3 };

HardwareSerial dvl_serial = HardwareSerial(PE10, PE12);

BatteryMonitor *bms;
MS5837 pressure_sensor;
TM_CPULOAD_t cpu_load;
DVL *dvl;

/* This section includes pinmaps for current sensors and auxilary channel pinmaps
 * currently, mainboard does not support such features
 * therefore these features are not tested yet.
 */
int current_sens_pins[8] = {PA0, PC0, PC0, PC0, PA3, PA6, PA7, PB6};
int aux_pinmap[AUX_LEN] = {PC10};

/* Global Variables
 */
uint32_t last_motor_update = millis();
bool last_motor_timeout_state = false;  //motor disabled
bool motor_armed = true;
bool armed_publish_flag = false;

Controller6DOF controller;

/**
 * @brief Publisher Messages
 */
std_msgs::Float32MultiArray current_msg;
std_msgs::Float32 depth_msg;
sensor_msgs::Range range_msg;
sensor_msgs::BatteryState battery_msg;
std_msgs::Bool armed_msg;
std_msgs::Int8 cpu_load_msg;
std_msgs::String dvl_msg;

/**
 * @brief Publishers
 */
ros::Publisher motor_currents("/turquoise/thrusters/current", &current_msg);
ros::Publisher bottom_sonar_pub("/turquoise/sensors/sonar/bottom", &range_msg);
ros::Publisher battery_state("/turquoise/battery_state", &battery_msg);
ros::Publisher depth_pub("/turquoise/depth", &depth_msg);
ros::Publisher armed_pub("/turquoise/is_armed", &armed_msg);
ros::Publisher cpu_load_pub("/turquoise/nucleo/cpu_load", &cpu_load_msg);
ros::Publisher dvl_pub("/turquoise/dvl/from", &dvl_msg);

/**
 * @brief Subscribers
 */
#if defined(ALLOW_DIRECT_CONTROL)
ros::Subscriber<std_msgs::Int16MultiArray> motor_subs("/turquoise/thrusters/input", &motor_callback);
#endif
ros::Subscriber<mainboard_firmware::Signal> command_sub("/turquoise/signal", &command_callback);
ros::Subscriber<mainboard_firmware::Odometry> odom_subb("/turquoise/odom", &odometry_callback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/turquoise/cmd_vel", &cmd_vel_callback);
ros::Subscriber<std_msgs::Float32> cmd_depth_sub("/turquoise/cmd_depth", &cmd_depth_callback);
ros::Subscriber<std_msgs::Int16MultiArray> aux_sub("/turquoise/aux", &aux_callback);
ros::Subscriber<std_msgs::String> dvl_sub("/turquoise/dvl/to", &dvl_callback);

/**
 * @brief Services
 * 
 */
ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> arming_srv("/turquoise/set_arming", &arming_service_callback);
ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> dvl_srv("/turquoise/dvl/set_power", &dvl_state_service_callback);

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
    nh.advertise(depth_pub);
    nh.advertise(armed_pub);

    #ifdef ENABLE_CPU_LOAD_MEASURING
    nh.advertise(cpu_load_pub);
    #endif
    /* *** Sonar Publishers *** */
    #if defined(ENABLE_SONARS)
    nh.advertise(bottom_sonar_pub);
    #endif
    
    nh.advertise(battery_state);
    nh.advertise(dvl_pub);

    #if defined(ALLOW_DIRECT_CONTROL)
    nh.subscribe(motor_subs);
    #endif
    nh.subscribe(odom_subb);
    
    // FIXME: Does this lines effect odom callback ?
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(command_sub);
    nh.subscribe(aux_sub);
    nh.subscribe(cmd_depth_sub);
    nh.subscribe(dvl_sub);

    nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(arming_srv);
    nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(dvl_srv);

    debugln("[PUB_SUB_INIT]");
}

void GetThrusterAllocationMatrix()
{
    char *alloc_param_names[6] = {"~allocation_cx", "~allocation_cy", "~allocation_cz",
        "~allocation_rx", "~allocation_ry", "~allocation_rz"};

    for (size_t i = 0; i < 6; i++)
    {
        if (!nh.getParam(alloc_param_names[i], controller.thruster_allocation[i], 8)) 
        { 
            for (size_t j = 0; j < 8; j++)
            {
                controller.thruster_allocation[i][j]= 0;
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
        if (!nh.getParam(pid_param_names[i], controller.pid_gains[i], 3, 5000)) 
        { 
            for (size_t j = 0; j < 3; j++)
            {
                controller.pid_gains[i][j]= 0.0;
            }
        }
    }
    controller.update_gains();
}

bool KillSwitch_isKilled()
{
    #ifdef ENABLE_KILLSWITCH
    return digitalRead(KILLSWITCH_PIN);
    #else
    return false;
    #endif
}

void PerformHaltModeCheck()
{
    // Connection is up, skip mode check.
    if (nh.connected() && !KillSwitch_isKilled()) return;
    bool last_motor_armed = motor_armed;

    motor_armed = false;
    debugln("[HALT_MODE] ON! (Possible Disconnection)");
    /* *** HALT MODE ON / LOST CONNECTION *** */
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    IndicatorTimer->pause();
    PIDTimer->pause();
    PublishTimer->pause();

    ResetMotors();
    ResetMotors();
    /* *** HALT MODE ON / LOST CONNECTION *** */

    while (!nh.connected() || KillSwitch_isKilled()) {
        nh.spinOnce(); 
        delay(50); 
        digitalWrite(LED_RED, bms->getVoltage() < MIN_BATT_VOLTAGE);
    }

    // TODO, TEMP, FIX: this is temporary
    // since the stm is never rebooted, the stm must
    // exchange the new parameters with the Xavier / Host.
    GetPIDControllerParameters();
    LogStartUpInfo();

    /* *** RECOVERED CONNECTION, BACK TO NORMAL *** */
    motor_armed = last_motor_armed;
    IndicatorTimer->resume();
    PIDTimer->resume();
    PublishTimer->resume();

    // Display info
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

void InitAux()
{
    debugln("[AUX_INIT] " + String(DEFAULT_AUX_PULSE_WIDTH) + " uS PULSE");
    for (size_t i = 0; i < AUX_LEN; i++)
    {
        while (!aux[i].attached()) { aux[i].attach(aux_pinmap[i]); } 
        aux[i].writeMicroseconds(DEFAULT_AUX_PULSE_WIDTH);
    }
}

void InitializeDVL()
{
    dvl = new DVL(&dvl_pub);
    dvl->setDVLStream(&dvl_serial);
    dvl->setPowerPin(DVL_PIN);
    dvl->setPowerState(false);
    debugln("[ DVL_INIT ]");
}

void ResetMotors()
{
    debugln("[MOTOR_RESET]");
    for (size_t i = 0; i < 8; i++)
    {
        motors[i].writeMicroseconds(DEFAULT_PULSE_WIDTH);
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

void InitPressureSensor()
{
    #if defined(ENABLE_PRES_SENSOR)
    Wire.begin();
    pressure_sensor.setModel(MS5837::MS5837_30BA);
    pressure_sensor.setFluidDensity(FLUID_DENSITY); 
    pressure_sensor.setOverSampling(MS5837_CONVERT_OVERSAMPLING::OSR_8192); // set oversampling to OSR_8192
    while (!pressure_sensor.init())
    {
        debugln("[MS5837_INIT_ERR]");
        delay(1000);
    }
    #endif
}

void HandlePressureSensorRoutine()
{
    #if defined(ENABLE_PRES_SENSOR)
    int bar30_stat = pressure_sensor.readNonBlocking();
    if (bar30_stat == 1 && pressure_sensor.getPublishFlag())
    {
        // debugln(pressure_sensor.depth());
        // publish sensor.depth();
        depth_msg.data = pressure_sensor.depth();
        depth_pub.publish(&depth_msg);
        pressure_sensor.setPublishFlag(false);
    }
    #endif
}

void InitializeTimers()
{
    IndicatorTimer = new HardwareTimer(INDICATOR_TIMER);
    IndicatorTimer->setOverflow(10, HERTZ_FORMAT); // 10 Hz
    IndicatorTimer->attachInterrupt(HardwareTimer_Callback);
    IndicatorTimer->resume();
    debugln("[INDICATOR_LED_TIMER_INIT]");

    CurrentCounterTimer = new HardwareTimer(CURRENT_COUNT_TIMER);
    CurrentCounterTimer->setOverflow(CURRENT_COUNT_INTERVAL * 1000000.0 , MICROSEC_FORMAT); 
    CurrentCounterTimer->attachInterrupt(HardwareTimer_Callback);
    CurrentCounterTimer->resume();
    debugln("[CURRENT_COUNTING_TIMER_INIT]");

    #if defined(ENABLE_SONARS)
    PingSonarTimer = new HardwareTimer(PING_TIMER);
    PingSonarTimer->setOverflow(1000.0, MICROSEC_FORMAT); 
    PingSonarTimer->attachInterrupt(HardwareTimer_Callback);
    PingSonarTimer->resume();
    debugln("[CURRENT_COUNTING_TIMER_INIT]");
    #endif

    PublishTimer = new HardwareTimer(PUBLISH_TIMER);
    PublishTimer->setOverflow(GLOBAL_PUBLISH_RATE, HERTZ_FORMAT);
    PublishTimer->attachInterrupt(HardwareTimer_Callback);
    PublishTimer->resume();
    debugln("[PUBLISHER_TIMER_INIT]");

    #if defined(ENABLE_PRES_SENSOR)
    PressureTimer = new HardwareTimer(PRESSURE_TIMER);
    PressureTimer->setOverflow(PRESSURE_INTERVAL, MICROSEC_FORMAT);
    PressureTimer->attachInterrupt(HardwareTimer_Callback);
    PressureTimer->resume();
    debugln("[PRESSURE_TIMER_INIT]");
    #endif

    PIDTimer = new HardwareTimer(PID_TIMER);
    PIDTimer->setOverflow(PID_LOOP_RATE, HERTZ_FORMAT);
    PIDTimer->attachInterrupt(HardwareTimer_Callback);
    PIDTimer->resume();
    debugln("[PID_TIMER_INIT]");
}

void InitController()
{
    controller.set_dt(1000.0 / PID_LOOP_RATE);
}

/* Motor Timeout Detector
 * Detects whether the timeout reached for the motor commands,
 * thus disables (1500 us pulse) the motors to prevent vehicle
 * from going crazy in case of network/connection loss. :)
 * This is the savior of your billion dollar machine.
 */
bool TimeoutDetector()
{
    bool state_change = (millis() - last_motor_update > MOTOR_TIMEOUT) ^ last_motor_timeout_state;
    last_motor_timeout_state = millis() - last_motor_update > MOTOR_TIMEOUT;

    if (state_change)
    {
        if (last_motor_timeout_state == 1)
        {
            debugln("[ MOTOR_TIMEOUT ]");
            IndicatorTimer->setOverflow(1, HERTZ_FORMAT); // 10 Hz
            ResetMotors();
            ResetMotors();
            return false;
        }
        else
        {
            IndicatorTimer->setOverflow(10, HERTZ_FORMAT); // 10 Hz
            return true;
        }
    }
    return !last_motor_timeout_state;
}

bool CheckBattery()
{
    bool arm_status = true;
    if (bms->getVoltage() < MIN_BATT_VOLTAGE)
    {
        digitalWrite(LED_RED, HIGH);
        arm_status = false;
        ResetMotors();
        nh.logerror(String("[ MAIN_VOLTAGE_CRITICAL ] Voltage: " + String(bms->getVoltage())).c_str());
        debugln(String("[ MAIN_VOLTAGE_CRITICAL ] Voltage: " + String(bms->getVoltage())).c_str());
    }
    else if (bms->getVoltage() < LOW_BATT_VOLTAGE)
    {
        nh.logwarn(String("[ MAIN_VOLTAGE_LOW ] Voltage: " + String(bms->getVoltage())).c_str());
        debugln(String("[ MAIN_VOLTAGE_LOW ] Voltage: " + String(bms->getVoltage())).c_str());
    }
    else
    {
    }
    return arm_status;
}

void SystemWatchdog()
{
    #ifdef ENABLE_CPU_LOAD_MEASURING
    if (TM_CPULOAD_GoToSleepMode(&cpu_load) == 1)
    {
        cpu_load_msg.data = (uint8_t)cpu_load.Load;
        cpu_load_pub.publish(&cpu_load_msg);
    }
    #endif
    TimeoutDetector();
    bool _batt = CheckBattery();
    digitalWrite(LED_RED, !_batt);
    PerformHaltModeCheck();
    digitalWrite(LED_BLUE, motor_armed);
}

void InitializePeripheralPins()
{
    debugln("[I/O_PINMODE]");
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(CURRENT_SENS_PIN, INPUT);
    pinMode(VOLTAGE_SENS_PIN, INPUT);

    #ifdef ENABLE_KILLSWITCH
    pinMode(KILLSWITCH_PIN, INPUT_PULLUP);
    #endif
}

void InitializeHardwareSerials()
{
    debugln("[HW_SERIAL_INIT]");
    hwserial_3.begin(115200);
}

void InitializePingSonarDevices()
{
    #if defined(ENABLE_SONARS)
    while (!bottom_sonar.initialize())
    {
        debugln("[PING_INIT_ERR]");
        delay(1000);
    }
    bottom_sonar.set_timeout(PING_TIMEOUT);

    debugln("[PING_INIT_OK]");
    #endif
}

void InitializeBatteryMonitor()
{
    bms = new BatteryMonitor(&battery_state);
    bms->initBattery(BatteryMonitor::CUSTOM_LIION);
}

void HandlePingSonarRequests()
{
    #if defined(ENABLE_SONARS)
    if (bottom_sonar.PublishFlag)
    {
        bottom_sonar.PublishFlag = false;
        range_msg.max_range = 30;
        range_msg.min_range = 0.5;
        range_msg.field_of_view = 30.0;
        range_msg.radiation_type = range_msg.ULTRASOUND;
        range_msg.range = bottom_sonar.distance() / 1000.0;
        bottom_sonar_pub.publish(&range_msg);
    }
    bottom_sonar.requestOnly(Ping1DNamespace::Distance_simple);
    #endif
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

void HandleArmedPublish()
{
    if (armed_publish_flag)
    {
        armed_publish_flag = false;
        armed_msg.data = motor_armed;
        armed_pub.publish(&armed_msg);
    }
}

void LogStartUpInfo()
{
    nh.logwarn("Z Depth is only using 1 pressure sensor measurement.");
    nh.logwarn("CMD_VEL.linear.z, CMD_VEL.angular.x/y are not used.");
    nh.logwarn("Use cmd_depth topic, to set depth reference");

    #ifndef ENABLE_KILLSWITCH
    nh.logwarn("Killswitch is DISABLED, use with caution.");
    #endif

    #ifdef STANDALONE_USE
    nh.logwarn("STANDALONE_USE: Pressure, Sonar, and switch are disabled.");
    #else
    nh.loginfo("NORMAL_USE: Hardware requiring functions are enabled (sensor, sonar, etc.)")
    #endif

    nh.loginfo(String("Firmware version: " + String(FIRMWARE_VERSION)).c_str());
}

// End of file. Copyright (c) 2019 ITU AUV Team / Electronics
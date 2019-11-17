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

#include <main.h>

void odometry_callback(const mainboard_firmware::Odometry& data)
{
    debugln("[ODOMETRY]      " + String(millis()));
    n[0] = data.pose.position.x;
    n[1] = data.pose.position.y;
    n[2] = data.pose.position.z;
    geometry_msgs::Vector3 euler = EulerFromQuaternion(data.pose.orientation);
    n[3] = euler.x;
    n[4] = euler.y;
    n[5] = euler.z;

    v[0] = data.twist.linear.x;
    v[1] = data.twist.linear.y;
    v[2] = data.twist.linear.z;

    v[3] = data.twist.angular.x;
    v[4] = data.twist.angular.y;
    v[5] = data.twist.angular.z;

    unsigned long dt = millis() - last_odom_update;
    last_odom_update += dt;
    
    double controller_output[6];
    RunPIDControllers(controller_output, dt);

    debug("[PID_CONTROLLER]");

    for (size_t i = 0; i < 6; i++)
    {
        debug(controller_output[i]);
        debug(" ");
    }
    debugln("");

    // PID CONTROLLER OUTPUT TO THRUSTER CONFIGURATION
    
    float thrust_vector[8];
    float sum = 0;

    for (size_t i = 0; i < 8; i++)
    {
        sum = 0;
        for (size_t j = 0; j < 6; j++)
        {
            sum += controller_output[j] * thruster_allocation[j][i];
        }
        thrust_vector[i] = sum;
    }

    debug("[THRUST_VECTOR] ");

    for (size_t i = 0; i < 8; i++)
    {
        debug(thrust_vector[i]);
        debug(" ");
    }
    debugln("");

    // NORMAL-OPERATION
    if (motor_armed)
    {
        for (size_t i = 0; i < 8; i++)
        {
            int motor_pulse_us = get_pwm(thrust_vector[i], thruster_direction[i]);
            motor_pulse_us = constrain(motor_pulse_us, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
            motors[i].writeMicroseconds(motor_pulse_us);
        }
    }
}

void cmd_vel_callback(const geometry_msgs::Twist& data)
{
    last_motor_update = millis();
    
    velocity_setpoint[0] = data.linear.x;
    velocity_setpoint[1] = data.linear.y;
    velocity_setpoint[2] = data.linear.z;
    velocity_setpoint[3] = data.angular.x;
    velocity_setpoint[4] = data.angular.y;
    velocity_setpoint[5] = data.angular.z;

    debug("[CMD_VEL] [" + String(millis()) + "] ");
    for (size_t i = 0; i < 6; i++)
    {
        debug(velocity_setpoint[i]);
        debug(" ");
    }
    debugln("");

}

/* Motor value update callback
 * If this callback isn't fired in MOTOR_TIMEOUT milliseconds after the last one,
 * the motors will be set to their default pulse time.
 */
void motor_callback(const std_msgs::Int16MultiArray& data)
{
    debugln("[MOTOR_DIRECT] [" + String(millis()) + "]");
    last_motor_update = millis();
    if (motor_armed)
    {
        for (size_t i = 0; i < 8; i++)
        {
            int motor_pulse_us = (int)data.data[i];
            // TODO: Remove constrain since the servo library already constrains data based on the 
            // macros: MIN_PULSE_WIDTH, MAX_PULSE_WIDTH
            motor_pulse_us = constrain(motor_pulse_us, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
            motors[i].writeMicroseconds(motor_pulse_us);
        }
    }

}

void command_callback(const mainboard_firmware::Signal& data)
{
    EvaluateCommand(data.type, data.content);
}

static void HardwareTimer_Callback(HardwareTimer* htim)
{
    if (htim == IndicatorTimer)
    {
        digitalToggle(LED_BUILTIN);
    }
    else if (htim == CurrentCounterTimer)
    {
        main_voltage = (double)analogRead(VOLTAGE_SENS_PIN)*(double)ADC_TO_VOLTAGE_RATIO;

        main_current = (double)analogRead(CURRENT_SENS_PIN)*(double)ADC_TO_CURRENT_RATIO;
        
        current_consumption_mah -= (double)CURRENT_COUNT_INTERVAL * main_current;
    }
    else if (htim == PingSonarTimer)
    {
        if (bottom_sonar.isRequestPending())
        {
            bottom_sonar.checkMessage(Ping1DNamespace::Distance_simple);
        }
    }
    else
    {
        UNUSED(htim);
    }
}

/**
 * @brief Application entry point.
 */
void setup()
{
    Serial.begin(DEBUG_BAUDRATE);
    InitializeHardwareSerials();
    debugln("[ROS_INIT]");
    InitNode();
    InitSubPub();

    debugln("[ADC_RES]: " + String(ADC_READ_RESOLUTION_BIT));
    analogReadResolution(ADC_READ_RESOLUTION_BIT);
    InitMotors();
    InitializePeripheralPins();
    InitializeCurrentsMessage();
    InitializeBatteryStateMsg();
    InitializePingSonarDevices();
    pinMode(USER_BTN, INPUT);
    
    InitializeIndicatorTimer(1);
    InitializeTimers();
    /* ********** HALT OPERATION ********** */
    PerformHaltModeCheck();
    // CONN SUCCESS. INDICATE GREEN..
    debugln("[ROS_CONN] OK!");
    /* ********** HALT OPERATION ********** */


    debugln("[REQUEST_PARAMS]");
    /* ********** GET PARAMETERS ********** */
    nh.spinOnce();
    GetThrusterAllocationMatrix();
    debugln("[PARAM_OK] TAM");
    GetPIDControllerParameters();
    debugln("[PARAM_OK] PID_GAINS");
    UpdatePIDControllerGains();
    debugln("[UPDATE_PID_GAINS]");
    /* ********** GET PARAMETERS ********** */

    LogStartUpInfo();
    debugln("[MAIN_LOOP_START]");
}

void loop()
{
    PublishBatteryState();
    PerformUARTControl();
    HandlePingSonarRequests();

    // PublishMotorCurrents(2);
    nh.spinOnce();
    SystemWatchdog();
}

// End of file. Copyright (c) 2019 ITU AUV Team / Electronics
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

void odom_callback(const nav_msgs::Odometry& data)
{
    n[0] = data.pose.pose.position.x;
    n[1] = data.pose.pose.position.y;
    n[2] = data.pose.pose.position.z;
    geometry_msgs::Vector3 euler = EulerFromQuaternion(data.pose.pose.orientation);
    n[3] = euler.x;
    n[4] = euler.y;
    n[5] = euler.z;

    v[0] = data.twist.twist.linear.x;
    v[1] = data.twist.twist.linear.y;
    v[2] = data.twist.twist.linear.z;

    v[3] = data.twist.twist.angular.x;
    v[4] = data.twist.twist.angular.y;
    v[5] = data.twist.twist.angular.z;

    unsigned long dt = millis() - last_odom_update;
    last_odom_update += dt;
    
    double controller_output[6];
    RunPIDControllers(controller_output, dt);
    
    // PID CONTROLLER OUTPUT TO THRUSTER CONFIGURATION
    
    float thruster_vector[8];
    float sum = 0;

    for (size_t i = 0; i < 8; i++)
    {
        sum = 0;
        for (size_t j = 0; j < 6; j++)
        {
            sum += controller_output[j] * thruster_allocation[j][i];
        }
        thruster_vector[i] = sum;
    }

    debug("Calculated thruster vector:");
    for (size_t i = 0; i < 8; i++)
    {
        debug(thruster_vector[i]);
        debug(" ");
    }
    debugln("");

    if (millis() - last_motor_update > MOTOR_TIMEOUT)
    {
        // TIMEOUT
        debugln("Timeout.. Resetting motors.");
        ResetMotors();
    }
    else
    {
        // NORMAL-OPERATION
        for (size_t i = 0; i < 8; i++)
        {
            int motor_pulse_us = get_pwm(thruster_vector[i], thruster_direction[i]);
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

    debugln("Received Velocity Command.");
}

/* Motor value update callback
 * If this callback isn't fired in MOTOR_TIMEOUT milliseconds after the last one,
 * the motors will be set to their default pulse time.
 */
void motor_callback(const std_msgs::Int16MultiArray& data)
{
    if (millis() - last_motor_update > MOTOR_TIMEOUT)
    {
        for (size_t i = 0; i < 8; i++)
        {
            motors[i].writeMicroseconds(DEFAULT_PULSE_WIDTH);
        }
    }
    last_motor_update = millis();
    for (size_t i = 0; i < 8; i++)
    {
        int motor_pulse_us = (int)data.data[i];
        // TODO: Remove constrain since the servo library already constrains data based on the 
        // macros: MIN_PULSE_WIDTH, MAX_PULSE_WIDTH
        motor_pulse_us = constrain(motor_pulse_us, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        motors[i].writeMicroseconds(motor_pulse_us);
    }
}

void command_callback(const mainboard_firmware::Signal& data)
{
    EvaluateCommand(data.type, data.content);
}

static void indicator_callback(HardwareTimer* htim)
{
    // UNUSED(htim);
    digitalToggle(LED_BUILTIN);
}

/**
 * @brief Application entry point.
 */
void setup()
{
    Serial.begin(DEBUG_BAUDRATE);
    InitializeHardwareSerials();
    debugln("Initializing ROS");
    InitNode();
    InitSubPub();

    debugln("Analog Read Resolution:" + String(ADC_READ_RESOLUTION_BIT) 
        + String(" bits, range from 0-") + String(pow(2, ADC_READ_RESOLUTION_BIT)));
    analogReadResolution(ADC_READ_RESOLUTION_BIT);

    InitMotors();
    InitializePeripheralPins();
    InitializeCurrentsMessage();
    // InitializePingSonarDevices();
    pinMode(USER_BTN, INPUT);
    
    InitializeIndicatorTimer(1);

    /* ********** HALT OPERATION ********** */
    PerformHaltModeCheck();
    // CONN SUCCESS. INDICATE GREEN..
    debugln("Connected to ROS...");
    /* ********** HALT OPERATION ********** */


    debugln("Getting parameters...");
    /* ********** GET PARAMETERS ********** */
    nh.spinOnce();
    GetThrusterAllocationMatrix();
    debugln("Got TAM.");
    GetPIDControllerParameters();
    debugln("Got PID Gains.");
    UpdatePIDControllerGains();
    debugln("Update PID Gains.");
    /* ********** GET PARAMETERS ********** */
}

void loop()
{
    // for (size_t i = 0; i < 4; i++) {
    //     PublishPingSonarMeasurements();
    // }
    // PublishPingSonarMeasurements();
    // for (size_t i = 0; i < 6; i++)
    // {
    //     for (size_t j = 0; j < 8; j++)
    //     {
    //         debug(thruster_allocation[i][j]);
    //         debug(" ");
    //     }
    //     debugln("");
    // }
    // debugln("");

    TimeoutDetector();
    PublishMotorCurrents(2);

    nh.spinOnce();
    PerformHaltModeCheck();
}

// End of file. Copyright (c) 2019 ITU AUV Team / Electronics
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
    for (size_t i = 0; i < 6; i++)
    {
        Serial.print(controller_output[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void cmd_vel_callback(const geometry_msgs::Twist& data)
{
    float cmd_vector[6] = {data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z};
    
    // PID CONTROLLER
    
    float thruster_vector[8];
    float sum = 0;

    for (size_t i = 0; i < 8; i++)
    {
        sum = 0;
        for (size_t j = 0; j < 6; j++)
        {
            sum += cmd_vector[j] * thruster_allocation[j][i];
        }
        thruster_vector[i] = sum;
    }

    #ifdef defined(DEBUG_PRINT)
    for (size_t i = 0; i < 8; i++)
    {
        Serial.print(thruster_vector[i]);
        Serial.print(" ");
    }
    Serial.println();
    #endif

    for (size_t i = 0; i < 8; i++)
    {
        int motor_pulse_us = (int)thruster_vector[i];
        motor_pulse_us = constrain(motor_pulse_us, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
        motors[i].writeMicroseconds(motor_pulse_us);
    }
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
            motors[i].writeMicroseconds(MOTOR_PULSE_DEFAULT);
        }
    }
    last_motor_update = millis();
    for (size_t i = 0; i < 8; i++)
    {
        int motor_pulse_us = (int)data.data[i];
        motor_pulse_us = constrain(motor_pulse_us, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
        motors[i].writeMicroseconds(motor_pulse_us);
    }
}

void command_callback(const mainboard_firmware::Signal& data)
{
    EvaluateCommand(data.type, data.content);
}

static void indicator_callback(stimer_t *htim)
{
    UNUSED(htim);
    digitalToggle(LED_BUILTIN);
}

/**
 * @brief Application rntry point.
 */
void setup()
{
    Serial.begin(DEBUG_BAUDRATE);
    InitializeHardwareSerials();
    InitNode();

    nh.advertise(diagnose_error);
    nh.advertise(diagnose_info);
    nh.advertise(motor_currents);
    nh.advertise(ping_1_pub);

    nh.subscribe(motor_subs);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(command_sub);
    nh.subscribe(odom_sub);

    analogReadResolution(ADC_READ_RESOLUTION_BIT);

    InitMotors();
    InitializePeripheralPins();
    InitializeIndicatorTimer(1);
    InitializeCurrentsMessage();
    // InitializePingSonarDevices();
    pinMode(USER_BTN, INPUT);

    /* ********** HALT OPERATION ********** */
    while (!nh.connected()) { nh.spinOnce(); }
    /* ********** HALT OPERATION ********** */


    /* ********** GET PARAMETERS ********** */
    GetThrusterAllocationMatrix();
    GetPIDControllerParameters();
    UpdatePIDControllerGains();
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
    //         Serial.print(thruster_allocation[i][j]);
    //         Serial.print(" ");
    //     }
    //     Serial.println();
    // }
    // Serial.println();

    SpinIndicatorTimer();
    PublishMotorCurrents(2);
    delay(5);
    nh.spinOnce();
}

// End of file. Copyright (c) 2019 ITU AUV Team / Electronics
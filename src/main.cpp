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
// Todo: add state machine class, to control arming state.
// perform all arming failure calculations in there.
// put systemwatchdog in there.

double n[6];
double v[6];

void odometry_callback(const mainboard_firmware::Odometry& data)
{
    n[0] = data.pose.position.x;
    n[1] = data.pose.position.y;
    // n[2] = bottom_sonar.distance() / 1000;
    n[2] = -pressure_sensor.depth();
    // debugln(-pressure_sensor.depth());
    // n[2] = 1.0;/
    // The conversion must be done in main computer / host.
    // define custom message type.
    geometry_msgs::Vector3 euler = EulerFromQuaternion(data.pose.orientation);
    n[3] = euler.x;
    n[4] = euler.y;
    n[5] = data.twist.angular.z;
    // n[5] = euler.z;

    v[0] = data.twist.linear.x;
    v[1] = data.twist.linear.y;
    v[2] = data.twist.linear.z;
    v[3] = data.twist.angular.x;
    v[4] = data.twist.angular.y;
    v[5] = data.twist.angular.z;
    
    controller.set_velocity(v);
    controller.set_position(n);
}

void aux_callback(const std_msgs::Int16MultiArray& data)
{
    for (size_t i = 0; i < AUX_LEN; i++)
    {
        int aux_pulse_us = (int)data.data[i];
        aux_pulse_us = constrain(aux_pulse_us, MIN_AUX_PULSE_WIDTH, MAX_AUX_PULSE_WIDTH);
        aux[i].writeMicroseconds(aux_pulse_us);
    }
}


void cmd_vel_callback(const geometry_msgs::Twist& data)
{   
    last_motor_update = millis();

    controller.set_velocity_reference_by_index(data.linear.x, 0);
    controller.set_velocity_reference_by_index(data.linear.y, 1);
    controller.set_velocity_reference_by_index(data.angular.z, 5);
    // indexes of 2 3 4 are for linearz, angularx, angulary which are not set from this callback

}

void cmd_depth_callback(const std_msgs::Float32& data)
{
    controller.set_velocity_reference_by_index(data.data, 2);
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

void arming_service_callback(const std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& resp)
{
    if (!req.data)
    {
        // Disarming Request, Disarm immidately.
        debugln("[DISARM_REQUEST]");
        debugln("[DISARMING]");
        motor_armed = false;
        resp.success = true;
    }
    else
    {
        debugln("[ARM_REQUEST]");

        // Arming Request, first check conditions !!
        bool is_safe2arm = false;
        /*
        * TODO: PERFORM NECESSARY CHECKS
        * - Does battery has normal voltage ?
        * - Timeout exits ? 
        * etc...
        */
        is_safe2arm = true;
        if (!is_safe2arm)
        {
            // NOT safe for arming !
            // TODO: Implement the reason
            resp.message = "REASON_NOT_IMPLEMENTED_YET";
            resp.success = false;
            debugln("[ARM] FAILED !");
        }
        else
        {
            motor_armed = true;
            resp.success = true;
            debugln("[ARM] SUCCESS !");

        }

    }
}

static void HardwareTimer_Callback(HardwareTimer* htim)
{
    if (htim == IndicatorTimer)
    {
        digitalToggle(LED_BUILTIN);
    }
    else if (htim == CurrentCounterTimer)
    {
        bms->setVoltage((double)analogRead(VOLTAGE_SENS_PIN)*(double)ADC_TO_VOLTAGE_RATIO);

        bms->setCurrent((double)analogRead(CURRENT_SENS_PIN)*(double)ADC_TO_CURRENT_RATIO, (double)CURRENT_COUNT_INTERVAL);
    }
    else if (htim == PingSonarTimer)
    {
        if (bottom_sonar.isRequestPending())
        {
            bottom_sonar.checkMessage(Ping1DNamespace::Distance_simple);
        }
    }
    else if (htim == PublishTimer)
    {
        bms->raisePublishFlag();
        bottom_sonar.PublishFlag = true;
        pressure_sensor.setPublishFlag(true);
        armed_publish_flag = true;
    }
    else if (htim == PressureTimer)
    {
        pressure_sensor.incrementTime(htim->getOverflow(MICROSEC_FORMAT));
    }
    else if (htim == PIDTimer)
    {
        controller.set_dt(1000.0 / PID_LOOP_RATE);
        controller.run();

        if (motor_armed)
        {
            double *thrust = controller.get_thrust_vector();
            for (size_t i = 0; i < 8; i++)
            {
                int motor_pulse_us = get_pwm(thrust[i], thruster_direction[i]);
                motor_pulse_us = constrain(motor_pulse_us, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
                motors[i].writeMicroseconds(motor_pulse_us);
            }
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
    InitAux();
    InitializePeripheralPins();
    InitializeCurrentsMessage();
    InitializeBatteryMonitor();
    InitializePingSonarDevices();
    InitPressureSensor();
    InitController();
    pinMode(USER_BTN, INPUT);
    
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
    debugln("[UPDATE_PID_GAINS]");
    /* ********** GET PARAMETERS ********** */

    LogStartUpInfo();
    debugln("[MAIN_LOOP_START]");
}

void loop()
{
    bms->publish(nh.now());
    PerformUARTControl();
    HandlePingSonarRequests();
    HandlePressureSensorRoutine();
    HandleArmedPublish();

    // PublishMotorCurrents(1);
    nh.spinOnce();
    SystemWatchdog();
}

// End of file. Copyright (c) 2019 ITU AUV Team / Electronics
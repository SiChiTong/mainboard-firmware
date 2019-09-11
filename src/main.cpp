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

// #define USE_ETHERNET
#include "main.h"

// region MotorCallbacks
void motor_callback(const std_msgs::Int16MultiArray& data)
{
    last_motor_update = millis();
    // Write Data to Motor.
    for (size_t i = 0; i < 8; i++)
    {
        int motor_pulse_us = (int)data.data[i];
        motor_pulse_us = constrain(motor_pulse_us, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
        motors[i].writeMicroseconds(motor_pulse_us);
        if ((int)data.data[i] != 1500) { Serial.print((int)data.data[i]); Serial.println(" " + String(i)); }
    }
}
// endregion MotorCallbacks

// region callbacks
void command_callback(const mainboard_firmware::Signal& data)
{
    EvaluateCommand(data.type, data.content);
}

static void indicator_callback(stimer_t *htim)
{
    UNUSED(htim);
    digitalToggle(LED_BUILTIN);
}
// endregion callbacks

// region Methods
void InitMotors()
{
    for (size_t i = 0; i < 8; i++)
    {
        motors[i].attach(motor_pinmap[i]);
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
    motor_currents.publish(&current_msg);
}
// endregion Methods

void setup()
{
    Serial.begin(DEBUG_BAUDRATE); // Debug port
    InitializeHardwareSerials();
    InitNode();

    nh.advertise(diagnose_error);
    nh.advertise(diagnose_info);

    nh.advertise(motor_currents);
    nh.advertise(ping_1_pub);

    nh.subscribe(motor_subs);
    nh.subscribe(command_sub);

    analogReadResolution(ADC_READ_RESOLUTION_BIT);

    InitMotors();
    InitializePeripheralPins();
    InitializeIndicatorTimer(1);
    InitializeCurrentsMessage();
    // InitializePingSonarDevices();
    pinMode(USER_BTN, INPUT);
}

void loop()
{
    // for (size_t i = 0; i < 4; i++) {
    //     PublishPingSonarMeasurements();
    // }
    // PublishPingSonarMeasurements();
    SpinIndicatorTimer();
    PublishMotorCurrents(2);
    nh.spinOnce();
}

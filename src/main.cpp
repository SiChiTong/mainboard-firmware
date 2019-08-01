// #define USE_ETHERNET
#include "main.h"

// region MotorCallbacks
void motor_callback(const std_msgs::Int16MultiArray& data)
{
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
void command_callback(const signal_msgs::Signal& data)
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
    Serial.begin(9600); // Debug port
    InitializeHardwareSerials();
    nh.getHardware()->setBaud(115200);
    InitNode();

    // region Publishers
    nh.advertise(diagnose_error);
    nh.advertise(diagnose_info);

    nh.advertise(motor_currents);
    nh.advertise(ping_1_pub);
    // endregion Publishers

    // region Subscribers
    nh.subscribe(motor_subs);
    nh.subscribe(command_sub);
    // endregion Subscibers

    // region Initialize Modules

    analogReadResolution(ADC_READ_RESOLUTION_BIT);

    InitMotors();
    InitializePeripheralPins();
    InitializeIndicatorTimer(1);
    InitializeCurrentsMessage();
    // InitializePingSonarDevices();
    // endregion Initialize Modules

}

void loop()
{
    // for (size_t i = 0; i < 4; i++) {
    //     PublishPingSonarMeasurements();
    // }
    // PublishPingSonarMeasurements();
    PublishMotorCurrents(2);
    nh.spinOnce();
}

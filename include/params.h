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

#include <Arduino.h>

#define MOTOR_TIMEOUT                    400                       // the max time (in ms) between each motor cmd.

/* ******************* Communication Settings ******************* */
#define ROS_BAUDRATE                     921600                    // Ros Communication Baud Rate
#define DEBUG_BAUDRATE                   57600                     // Default Serial port baud rate. (For DEBUG use.)
/* ******************* Communication Settings ******************* */

/* ************************ ADC Settings ************************ */
#define ADC_READ_RESOLUTION_BIT          12                        // 12 bits for analog read resolution
#define ADC_READ_MAX_VALUE               4096.0                    // 2^ADC_READ_RESOLUTION_BIT = 4096
#define ADC_READ_MAX_VOLTAGE             3300.0                    // 12 bits for analog read resolution max 3.3V
/* ************************ ADC Settings ************************ */

/* ********************** ACS 712 Settings ********************** */
#define ACS712_30A_SENS_MV_PER_AMP       66.0                      // 66 mV per 1.A
#define ACS712_20A_SENS_MV_PER_AMP       100.0                     // 100 mV per 1.A
#define ADC_OFFSET_CURRENT_ERROR         800.0                     // 800 mA offset error.
/* ********************** ACS 712 Settings ********************** */

#define INDICATOR_TIMER                  TIM8                      // Timer instance of indicator Led. (LED_GREEN)

/* **************** APM Common Power Module (CPM) **************** */
/**
 * Refer Link:
 * http://ardupilot.org/copter/docs/common-3dr-power-module.html
 */
#define CURRENT_COUNT_TIMER              TIM9                      // Timer instance of current count. Reads current
#define CURRENT_COUNT_INTERVAL           0.1                       // time in seconds between each measurement. (100 ms)
#define CURRENT_SENS_PIN                 PC2                       // ADC Pin.
#define CURRENT_SENS_MAX_AMPS            60.0                      // Max measurable amps for sensor.
#define VOLTAGE_SENS_PIN                 PB1                       // ADC Pin.
#define VOLTAGE_SENS_MAX_VOLTS           18.0                      // 4S Max.
#define ADC_TO_VOLTAGE_RATIO             0.0085                    // Module parameter
#define ADC_TO_CURRENT_RATIO             0.0137134308511           // Module parameter
#define MIN_BATT_VOLTAGE                 9.0                       // 3S Batt Min voltage
#define LOW_BATT_VOLTAGE                 9.5                       // Low Voltage warning.
/* **************** APM Common Power Module (CPM) **************** */

/* ******************** Ping Sonar Settings ********************* */
#define PING_TIMEOUT                     1000                      // Ping Sonar, request timeout
#define PING_TIMER                       TIM11                     // Ping sonar response waiting timer
/* ******************** Ping Sonar Settings ********************* */


/* ***************** Pressure Sensor Settings ******************* */
#define PRESSURE_TIMER                   TIM12
#define PRESSURE_INTERVAL                10000
#define FLUID_DENSITY                    997                       // (freshwater, 1029 for seawater)
/* ***************** Pressure Sensor Settings ******************* */

/* *********************** PID Settings ************************* */
#define PID_TIMER                        TIM14                     // Dont use Timer 13.
#define PID_LOOP_RATE                    80.0
/* *********************** PID Settings ************************* */

/* ********************* Publish Settings *********************** */
#define PUBLISH_TIMER                    TIM10                     // Used for Publish timer
#define GLOBAL_PUBLISH_RATE              2                         // Used for batterystate / currents etc.
/* ********************* Publish Settings *********************** */

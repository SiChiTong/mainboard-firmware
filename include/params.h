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

#define INFO_STREAM_ENABLE               true                      // Set Info Steaming enabled to topic.

#define ACS712_30A_SENS_MV_PER_AMP       66.0                      // 66 mV per 1.A
#define ACS712_20A_SENS_MV_PER_AMP       100.0                     // 100 mV per 1.A

#define ADC_READ_RESOLUTION_BIT          12                        // 12 bits for analog read resolution
#define ADC_READ_MAX_VALUE               4096.0                    // 2^ADC_READ_RESOLUTION_BIT = 4096
#define ADC_READ_MAX_VOLTAGE             3300.0                    // 12 bits for analog read resolution max 3.3V
#define ADC_OFFSET_CURRENT_ERROR         800.0                     // 800 mA offset error.

#define INDICATOR_TIMER                  TIM7                      // Timer instance of indicator Led. (LED_GREEN)

#define PING_PULSE_TIME                  5
#define PING_TIMEOUT                     50
#define ROS_BAUDRATE                     57600                     // Ros Communication Baud Rate
#define DEBUG_BAUDRATE                   9600                      // Default Serial port baud rate. (For DEBUG use.)
#define MOTOR_TIMEOUT                    400                       // the max time (in ms) between each motor cmd.
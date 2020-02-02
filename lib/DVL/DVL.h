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

#define ENABLE_CUSTOM_SERVO

#include <ros.h>
#include <Arduino.h>
#include <std_msgs/String.h>
#include <CustomServo.h>

#if defined(ENABLE_CUSTOM_SERVO)
    #include <CustomServo.h>
#else
    #include <Servo.h>
#endif

class DVL
{
private:
    Stream *dvl_serial_;
    Servo dvl_power_switch_;
    ros::Publisher *publisher_;
    String received_data_ = "";
    char last_char_ = '\0';
    char current_char_ = '\0';
    bool dvl_state_ = false;

public:
    std_msgs::String msg;
    DVL(ros::Publisher *publisher);
    String getReceivedData();
    char getLastChar();
    char getCurrentChar();
    void setDVLStream(Stream *stream);
    void send(char *data);
    void resetReceivedData();
    void updateReceivedData();
    void setLastChar(char last_char);
    void setCurrentChar(char current_char);
    bool getPowerState();
    void setPowerState(bool dvl_state);
    void setPowerPin(int dvl_pin);
    void servoWrite(int value);
    void HandleDVLDataRoutine();
    void publish();

    ~DVL();
};
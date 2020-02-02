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

#include <DVL.h>

DVL::DVL(ros::Publisher *publisher)
{
    publisher_ = publisher;
}

void DVL::setDVLStream(HardwareSerial *stream)
{
    dvl_serial_ = stream;
    dvl_serial_->begin(DVL_DEFAULT_BAUD);
}

bool DVL::getPowerState()
{
    return dvl_state_;
}

void DVL::setPowerState(bool dvl_state)
{
    dvl_state_ = dvl_state;

    if (dvl_state_) dvl_power_switch_.writeMicroseconds(DVL_POWERON);
    else dvl_power_switch_.writeMicroseconds(DVL_POWEROFF);
}

void DVL::setPowerPin(int dvl_pin)
{
    while (!dvl_power_switch_.attached()) { dvl_power_switch_.attach(dvl_pin); }
    setPowerState(false);
}

void DVL::send(char *data)
{
    dvl_serial_->write(data);
}

void DVL::HandleDVLDataRoutine()
{
    while (dvl_serial_->available())
    {
        this->setCurrentChar((char)dvl_serial_->read());
        this->updateReceivedData();

        if(this->getLastChar() == '\r' && this->getCurrentChar() == '\n')
        {
            this->publish();
            this->resetReceivedData();
        }

        this->setLastChar(this->getCurrentChar());
    }
}

void DVL::resetReceivedData()
{
    received_data_ = "";
}

void DVL::updateReceivedData()
{
    received_data_ = received_data_ + current_char_;
}

void DVL::setCurrentChar(char current_char)
{
    current_char_ = current_char;
}

void DVL::setLastChar(char last_char)
{
    last_char_ = last_char;
}

String DVL::getReceivedData()
{
    return received_data_;
}

char DVL::getCurrentChar()
{
    return current_char_;
}

char DVL::getLastChar()
{
    return last_char_;
}

void DVL::publish()
{
    msg.data = received_data_.c_str();
    publisher_->publish(&msg);
}
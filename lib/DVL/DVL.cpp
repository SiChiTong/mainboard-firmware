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
        char incoming_char = (char)dvl_serial_->read();

        if (incoming_char == '\n')
        {
            if (received_data_[recv_index - 1] == '\r')
            {
                received_data_[recv_index - 1] = 0;
                if (received_data_[0] == '#')
                    publish();
            }

            resetReceivedData();
            recv_index = 0;
            break;
        }
        else
        {
            if (recv_index <= DVL_MAX_BUFFER)
                received_data_[recv_index++] = incoming_char;
        }
    }
}

void DVL::resetReceivedData()
{
    memcpy(received_data_, 0, sizeof received_data_);
}

void DVL::publish()
{
    msg.data = received_data_;
    publisher_->publish(&msg);
}
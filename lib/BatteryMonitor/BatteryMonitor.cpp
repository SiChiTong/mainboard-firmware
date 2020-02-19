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

#include <BatteryMonitor.h>

BatteryMonitor::BatteryMonitor(ros::Publisher *publisher)
{
    publisher_ = publisher;
}

void BatteryMonitor::setVoltage(double voltage)
{
    voltage_ = voltage;
}

void BatteryMonitor::setCurrent(double current, double dt)
{
    current_ = current;
    current_consumption_ -= current_ * dt;
}

double BatteryMonitor::getVoltage()
{
    return voltage_;
}
double BatteryMonitor::getCurrent()
{
    return current_;
}
int BatteryMonitor::getCurrentConsumption()
{
    return (int)current_consumption_;
}

void BatteryMonitor::publish(ros::Time now)
{
    if (pending_publish)
    {
        msg.header.stamp = now;
        msg.voltage = voltage_;
        msg.current = -current_;
        msg.charge = current_consumption_;
        publisher_->publish(&msg);
        pending_publish = false;
    }
}
void BatteryMonitor::publish()
{
    if (pending_publish)
    {
        msg.voltage = voltage_;
        msg.current = -current_;
        publisher_->publish(&msg);
        pending_publish = false;
    }
}

void BatteryMonitor::raisePublishFlag()
{
    pending_publish = true;
}

void BatteryMonitor::initBattery(BatteryModel model)
{
    if (model == TURNIGY_5000)
    {
        msg.header.frame_id = "/base_link";
        msg.location = "back_tube";
        msg.design_capacity = 5000;
        msg.serial_number = "TURNIGY_5000MAH";
        msg.present = true;
        msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    }
    else if (model == CUSTOM_LIION)
    {
        msg.header.frame_id = "/base_link";
        msg.location = "back_tube";
        msg.design_capacity = 27000;
        msg.serial_number = "ITU-AUV/US18650VTC6-4S9P-LIION-V1-0";
        msg.present = true;
        msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    }
    else
    {
        msg.header.frame_id = "/base_link";
        msg.location = "back_tube";
        msg.design_capacity = 0;
        msg.serial_number = "UNKNOWN";
        msg.present = true;
        msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    }
    
}
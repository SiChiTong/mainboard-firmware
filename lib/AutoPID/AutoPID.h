// MIT License

// Copyright (c) 2017 rdownin4

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

#ifndef AUTOPID_H
#define AUTOPID_H
#include <Arduino.h>

class AutoPID {

  public:
    // Constructor - takes pointer inputs for control variales, so they are updated automatically
    AutoPID(double outputMin, double outputMax,
            double Kp, double Ki, double Kd);
    // Allows manual adjustment of gains
    void setGains(double Kp, double Ki, double Kd);
    // Sets bang-bang control ranges, separate upper and lower offsets, zero for off
    void setBangBang(double bangOn, double bangOff);
    // Sets bang-bang control range +-single offset
    void setBangBang(double bangRange);
    // Allows manual readjustment of output range
    void setOutputRange(double outputMin, double outputMax);
    // Allows manual adjustment of time step (default 1000ms)
    void setTimeStep(unsigned long timeStep);
    // Returns true when at set point (+-threshold)
    bool atSetPoint(double threshold);
    // Runs PID calculations when needed. Should be called repeatedly in loop.
    // Automatically reads input and sets output via pointers
    double run(double setpoint, double input, unsigned long _dT);

    // Stops PID functionality, output sets to 
    void stop();
    void reset();
    bool isStopped();

    double getIntegral();
    void setIntegral(double integral);

  private:
    double _Kp, _Ki, _Kd;
    double _integral, _previousError;
    double _bangOn, _bangOff;
    double _input, _setpoint, _output;
    double _outputMin, _outputMax;
    unsigned long _timeStep, _lastStep;
    bool _stopped;

};//class AutoPID

#endif

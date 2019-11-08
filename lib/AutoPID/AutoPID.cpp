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


#include "AutoPID.h"

AutoPID::AutoPID(double outputMin, double outputMax,
                 double Kp, double Ki, double Kd) {
  _outputMin = outputMin;
  _outputMax = outputMax;
  setGains(Kp, Ki, Kd);
  _timeStep = 1000;
  _lastStep = millis();
}//AutoPID::AutoPID

void AutoPID::setGains(double Kp, double Ki, double Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}//AutoPID::setControllerParams

void AutoPID::setBangBang(double bangOn, double bangOff) {
  _bangOn = bangOn;
  _bangOff = bangOff;
}//void AutoPID::setBangBang

void AutoPID::setBangBang(double bangRange) {
  setBangBang(bangRange, bangRange);
}//void AutoPID::setBangBang

void AutoPID::setOutputRange(double outputMin, double outputMax) {
  _outputMin = outputMin;
  _outputMax = outputMax;
}//void AutoPID::setOutputRange

void AutoPID::setTimeStep(unsigned long timeStep){
  _timeStep = timeStep;
}


bool AutoPID::atSetPoint(double threshold) {
  return abs(_setpoint - _input) <= threshold;
}//bool AutoPID::atSetPoint

double AutoPID::run(double setpoint, double input, unsigned long _dT) {
    _input = input;
    _setpoint = setpoint;

    if (_stopped) {
        _stopped = false;
        reset();
    }
    
    _lastStep = millis();
    double _error = _setpoint - _input;
    _integral += (_error + _previousError) / 2 * _dT / 1000.0;   //Riemann sum integral
    //_integral = constrain(_integral, _outputMin/_Ki, _outputMax/_Ki);
    double _dError = (_error - _previousError) / _dT / 1000.0;   //derivative
    _previousError = _error;
    double PID = (_Kp * _error) + (_Ki * _integral) + (_Kd * _dError);
    //*_output = _outputMin + (constrain(PID, 0, 1) * (_outputMax - _outputMin));
    _output = constrain(PID, _outputMin, _outputMax);
    return _output;
}//void AutoPID::run

void AutoPID::stop() {
  _stopped = true;
  reset();
}
void AutoPID::reset() {
  _lastStep = millis();
  _integral = 0;
  _previousError = 0;
}

bool AutoPID::isStopped(){
  return _stopped;
}

double AutoPID::getIntegral(){
  return _integral;
}

void AutoPID::setIntegral(double integral){
  _integral = integral;
}
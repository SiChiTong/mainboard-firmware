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

template<typename T>
void debug(T msg)
{
    #ifndef RELEASE_MODE
    Serial.print(String(msg));
    #endif

    // #if defined(DEBUG_LOG)
    // nh.logdebug(String(msg).c_str());
    // #endif
}
template<typename T>
void debugln(T msg)
{
    #ifndef RELEASE_MODE
    Serial.println(String(msg));
    #endif

    // #if defined(DEBUG_LOG)
    // nh.logdebug(String(msg).c_str());
    // #endif
}
template void debug(int);
template void debug(float);
template void debug(double);
template void debug(char*);
template void debug(String);

template void debugln(int);
template void debugln(float);
template void debugln(double);
template void debugln(char*);
template void debugln(String);
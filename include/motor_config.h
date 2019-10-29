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

#define MOTOR_PULSE_RANGE                400 

/* *** FOLLOWING 4 MACROS ARE DEFINED FOR BUILT-IN SERVO LIBRARY, 
        AND IT OVERWRITES THE ONES DEFINED IN SERVO LIBRARY. *** */
#define DEFAULT_PULSE_WIDTH              1500
#define MIN_PULSE_WIDTH                  DEFAULT_PULSE_WIDTH - MOTOR_PULSE_RANGE   // Min uS pulse time
#define MAX_PULSE_WIDTH                  DEFAULT_PULSE_WIDTH + MOTOR_PULSE_RANGE   // Max uS pulse time
#define REFRESH_INTERVAL                 2040                                      // 1000000 / 490 = 2040 (default=20000)

// const int motor_pinmap[8] = {PF8, PF0, PE5, PE3, PE2, PG3, PD6, PD3};

// Temporary pinmap
const int motor_pinmap[8] = {PF8, PD6, PE5, PE3, PE2, PG3, PF0, PD3};
const int thruster_direction[8] = {1, 1, 1, 1, 1, 1, 1, 1};
#define POS_FIT_P1 -0.1717
#define POS_FIT_P2 18.2
#define POS_FIT_P3 1543

#define NEG_FIT_P1 0.6732
#define NEG_FIT_P2 32.1
#define NEG_FIT_P3 1460

int positive_fit(double force)
{
    return pow(force, 2) * POS_FIT_P1 + force * POS_FIT_P2 + POS_FIT_P3;
}

int negative_fit(double force)
{
    return pow(force, 2) * NEG_FIT_P1 + force * NEG_FIT_P2 + NEG_FIT_P3;
}

int get_pwm(double force, int direction)
{
    // TODO: Add voltage scaling, 
    // go for thruster test, use 12V 13V 14V 15V 16V 16.8V as Vcc
    // measure thrust for pwm=1750, -1750
    // check if linear.
    if (force > 0.1)
    {
        if (direction == -1)
        {
            return negative_fit(-force);
        }
        else
        {
            return positive_fit(force);
        }
    }
    else if (force < -0.1)
    {
        if (direction == -1)
        {
            return positive_fit(-force);
        }
        else
        {
            return negative_fit(force);
        }
    }
    else
    {
        return DEFAULT_PULSE_WIDTH;
    }
    
}

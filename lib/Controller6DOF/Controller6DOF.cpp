#include <Controller6DOF.h>

Controller6DOF::Controller6DOF(/* args */)
{
    last_odom_update = millis();
}

double Controller6DOF::get_dt()
{
    return dt;
}

double Controller6DOF::set_dt(double _dt)
{
    dt = _dt;
}

double *Controller6DOF::get_position()
{
    return n;
}

double Controller6DOF::set_position(double *pos)
{
    for (size_t i = 0; i < 6; i++)
    {
        n[i] = pos[i];
    }
    last_odom_update = millis();
}

double *Controller6DOF::get_velocity()
{
    return v;
}

double Controller6DOF::set_velocity(double *vel)
{
    for (size_t i = 0; i < 6; i++)
    {
        v[i] = vel[i];
    }
    last_odom_update = millis();
}

double Controller6DOF::set_velocity_reference(double *vel_ref)
{
    for (size_t i = 0; i < 6; i++)
    {
        velocity_setpoint[i] = vel_ref[i];
    }
}

void Controller6DOF::run()
{
    for (size_t i = 0; i < 6; i++)
    {
        controller_output[i] = controllers[i].run(velocity_setpoint[i], n[i], dt) + offset[i];
    }

    for (size_t i = 0; i < 8; i++)
    {
        sum = 0;
        for (size_t j = 0; j < 6; j++)
        {
            sum += controller_output[j] * thruster_allocation[j][i];
        }
        thrust_vector[i] = sum;
    }
}

double *Controller6DOF::get_thrust_vector()
{
    return thrust_vector;
}

double *Controller6DOF::get_output()
{
    return controller_output;
}

void Controller6DOF::update_gains()
{
    for (size_t i = 0; i < 6; i++)
    {
            controllers[i].setGains((double)pid_gains[i][0], 
            (double)pid_gains[i][1],
            (double)pid_gains[i][2]);
    }
}

void Controller6DOF::set_range(int range)
{
    motor_pulse_range = range;

    for (size_t i = 0; i < 6; i++)
    {
        controllers[i].setOutputRange(-motor_pulse_range, motor_pulse_range);
    }
}

double Controller6DOF::get_range()
{
    return motor_pulse_range;
}


Controller6DOF::~Controller6DOF()
{
}
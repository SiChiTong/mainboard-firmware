// The code below is taken from Wikipedia, 
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#define M_PI 3.14159265358979323846264338327950288

geometry_msgs::Vector3 EulerFromQuaternion(geometry_msgs::Quaternion q)
{
    geometry_msgs::Vector3 euler;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler.x = atan2(sinr_cosp, cosr_cosp);


    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        euler.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    euler.z = atan2(siny_cosp, cosy_cosp);

    return euler;
}

geometry_msgs::Quaternion QuaternionFromEuler(double roll, double pitch, double yaw)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

geometry_msgs::Quaternion QuaternionFromEuler(geometry_msgs::Vector3 euler)
{
    // Abbreviations for the various angular functions
    double cy = cos(euler.z * 0.5);
    double sy = sin(euler.z * 0.5);
    double cp = cos(euler.y * 0.5);
    double sp = sin(euler.y * 0.5);
    double cr = cos(euler.x * 0.5);
    double sr = sin(euler.x * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}
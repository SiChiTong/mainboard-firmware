#ifndef CONTROLLER6DOF_H
#define CONTROLLER6DOF_H
#include <AutoPID.h>
// #include <motor_config.h>

class Controller6DOF
{
private:

    /** @brief Velocity & Ang. Velocity in 6 DOF
     * {vx, vy, vz, ax, ay, az}
     */
    double v[6] = {0, 0, 0, 0, 0, 0};

    /** @brief Position & Orientation in 6 DOF
     * {lx, ly, lz, ax, ay, az}
     */
    double n[6] = {0, 0, 0, 0, 0, 0};

    /** @brief Setpoint for PID controllers
     */
    double velocity_setpoint[6] = {0, 0, 0, 0, 0, 0};

    /** @brief Holds the latest millis() update,
     * Todo: use this to control if the feedback is timeout
     */
    unsigned long last_odom_update = 0;

    /** @brief The sample time, aka. loop time 
     * in milliseconds
     */ 
    double dt = 80; // milliseconds

    /** @brief PID controller output signal
     */
    double controller_output[6] = {0, 0, 0, 0, 0, 0};

    /** @brief The thrust vector which is obtained from
     * thrust_vector = controller_output * TAM 
     * TAM: Thruster allocation
     */
    double thrust_vector[8];
    
    /** @brief used in matrix multiplication
     * ignore it.
     */
    float sum = 0;

    int motor_pulse_range = 400;
    
public:
    float thruster_allocation[6][8];    // Vehicle Spesific parameters
    float pid_gains[6][3];              // Vehicle Spesific parameters
    AutoPID controllers[6] = {
        AutoPID(-motor_pulse_range, motor_pulse_range, 1.0, 0.0, 0.0), // Pos_X
        AutoPID(-motor_pulse_range, motor_pulse_range, 1.0, 0.0, 0.0), // Pos_Y
        AutoPID(-motor_pulse_range, motor_pulse_range, 1.0, 0.0, 0.0), // Pos_Z
        AutoPID(-motor_pulse_range, motor_pulse_range, 1.0, 0.0, 0.0), // Rot_X
        AutoPID(-motor_pulse_range, motor_pulse_range, 1.0, 0.0, 0.0), // Rot_Y
        AutoPID(-motor_pulse_range, motor_pulse_range, 1.0, 0.0, 0.0)  // Rot_Z
        };
    Controller6DOF(/* args */);
    ~Controller6DOF();
    double *get_position();
    double *get_velocity();
    double *get_velocity_reference();
    double get_dt();
    double set_dt(double _dt);
    double set_velocity_reference(double *vel_ref);
    double set_velocity(double *vel);
    double set_position(double *pos);
    double *get_thrust_vector();
    double *get_output();
    double get_range();
    void set_range(int range);
    void run();
    void update_gains();
};
#endif

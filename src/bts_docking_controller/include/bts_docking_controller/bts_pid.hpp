#ifndef BTS_PID_HPP
#define BTS_PID_HPP

#include <iostream>
#include <cmath>

class pid {

    public:
        pid() = default;
        pid(double kp, double ki, double kd, double dt, double integral_max, double integral_min);

        // Interaction
        double run(double setpoint, double measurement, double dt);
        void reset();
        void reset_integral();

        // Set parameters
        void set_kp(double kp);
        void set_ki(double ki);
        void set_kd(double kd);
        void set_integral_max(double max);
        void set_integral_min(double min);
        void set_integral_limits(double integral_min, double integral_max);
        void set_ssa(bool ssa); 
        void set_feedforward(double feedforward);

    private:
        // PID parameters
        double kp_ = 0.0; // Proportional gain
        double ki_ = 0.0; // Integral gain
        double kd_ = 0.0; // Derivative gain
        double integral_ = 0.0; // Integral term
        double integral_max_ = 0.0; // Maximum value for integral term
        double integral_min_ = 0.0; // Minimum value for integral term
        double previous_error_ = 0.0; // Previous error for derivative calculation

        bool ssa_ = false;
        double feedforward_ = 0.0;

};

#endif
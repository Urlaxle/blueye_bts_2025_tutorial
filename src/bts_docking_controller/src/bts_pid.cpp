#include<bts_docking_controller/bts_pid.hpp>

pid::pid(double kp, double ki, double kd, double dt, double integral_max, double integral_min)
    : kp_(kp), ki_(ki), kd_(kd), integral_max_(integral_max), integral_min_(integral_min) {
    // Initialize the previous error to zero
    previous_error_ = 0.0;
    // Reset the integral term
    integral_ = 0.0;
}

double pid::run(double setpoint, double measurement, double dt) {
    // Calculate the error
    double error = 0.0;
    if (!ssa_) {
        error = setpoint - measurement;
    } else {
        // Small-signal approximation: wrap the error to [-pi, pi]
        error = setpoint - measurement;
        if (error > M_PI) {
            error -= 2.0 * M_PI;
        } else if (error < -M_PI) {
            error += 2.0 * M_PI;
        }
    }

    // Proportional term
    double proportional = kp_ * error;

    // Integral term
    integral_ += ki_ * error * dt;
    // Clamp the integral term to prevent windup
    if (integral_ > integral_max_) {
        integral_ = integral_max_;
    } else if (integral_ < integral_min_) {
        integral_ = integral_min_;
    }

    // Derivative term
    double derivative = kd_ * (error - previous_error_) / dt;
    previous_error_ = error; // Update the previous error

    // Calculate the PID output
    return proportional + integral_ + derivative + feedforward_;

}

void pid::reset() {
    // Reset all PID parameters to their initial values
    previous_error_ = 0.0; // Reset the previous error
    integral_ = 0.0; // Reset the integral term
}

void pid::reset_integral() {
    integral_ = 0.0; // Reset the integral term to zero
}

void pid::set_kp(double kp) {
    kp_ = kp; // Set the proportional gain
}

void pid::set_ki(double ki) {
    ki_ = ki; // Set the integral gain
}

void pid::set_kd(double kd) {
    kd_ = kd; // Set the derivative gain
}

void pid::set_integral_max(double max) {
    integral_max_ = max; // Set the maximum limit for the integral term
}

void pid::set_integral_min(double min) {
    integral_min_ = min; // Set the minimum limit for the integral term
}

void pid::set_integral_limits(double integral_min, double integral_max) {
    integral_min_ = integral_min; // Set the minimum limit for the integral term
    integral_max_ = integral_max; // Set the maximum limit for the integral term
}

void pid::set_ssa(bool ssa) {
    ssa_ = ssa; // Set the small-signal approximation flag
}

void pid::set_feedforward(double feedforward) {
    feedforward_ = feedforward; // Set the feedforward term
}
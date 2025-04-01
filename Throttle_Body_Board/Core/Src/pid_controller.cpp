/*
 * pid.cpp
 *
 *  Created on: Mar 27, 2025
 *      Author: kohlmanz
 */
#include <pid_controller.h>

PIDController::PIDController(double p, double i, double d, double min, double max) {
    kp = p;
    ki = i;
    kd = d;
    output_min = min;
    output_max = max;

    integral = 0.0f;
    last_error = 0.0f;
    last_time = 0;
    setpoint = 0.0f;
}

void PIDController::setSetpoint(double sp) {
    setpoint = sp;
}

double PIDController::calculate(double input, uint32_t current_time) {
    // Calculate time delta in seconds
	double dt = (current_time - last_time) / 1000.0f;  // Assuming current_time is in milliseconds
    if (last_time == 0) dt = 0.001f;  // First run protection

    // Calculate error
    double error = setpoint - input;

    // Proportional term
    double p_term = kp * error;

    // Integral term with anti-windup
    integral += error * dt;
    double i_term = ki * integral;
    if (i_term > output_max) {
        i_term = output_max;
        integral = i_term / ki;
    } else if (i_term < output_min) {
        i_term = output_min;
        integral = i_term / ki;
    }

    // Derivative term
    double derivative = (error - last_error) / dt;
    double d_term = kd * derivative;

    // Calculate total output
    double output = p_term + i_term + d_term;

    // Limit output
    if (output > output_max) output = output_max;
    else if (output < output_min) output = output_min;

    // Store values for next iteration
    last_error = error;
    last_time = current_time;

    return output;
}

void PIDController::reset() {
    integral = 0.0f;
    last_error = 0.0f;
    last_time = 0;
}

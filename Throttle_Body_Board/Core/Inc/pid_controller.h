/*
 * pid.h
 *
 *  Created on: Mar 27, 2025
 *      Author: kohlmanz
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_
#include <stdint.h>

class PIDController {
private:
    double kp;           // Proportional gain
    double ki;           // Integral gain
    double kd;           // Derivative gain
    double setpoint;     // Desired value
    double integral;     // Running sum of error
    double last_error;   // Previous error
    double output_min;   // Minimum output limit
    double output_max;   // Maximum output limit
    uint32_t last_time; // Last calculation time

public:
    PIDController(double p, double i, double d, double min, double max);
    void setSetpoint(double sp);
    double calculate(double input, uint32_t current_time);
    void reset();
};



#endif /* INC_PID_CONTROLLER_H_ */

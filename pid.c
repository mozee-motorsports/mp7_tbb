/* https://geekeeceebee.com/DCMotor%20Position%20Tracking.html */
/* PID control for Bosch Throttle Body */

#define gas_pedal PLACEHOLDER
#define tps_sensor PLACEHOLDER



void main(void) {

    int t_prev;
    while(1) {

        tps_cur = analogRead(tps_sensor);
        desired = transfer_function(analogRead(gas_pedal));

        t = current_time_ms();

        dt = (t - t_prev);

        Theta = tps_cur;
        Theta_desired = desired;  // this is our set point: trying to get here

        /* as the error increases, we increase the output */
        error = Theta_desired - Theta;              // error is the difference between the current angle and desired angle
        /* as errors accumulate over time, the integral term has more of a weight in the output */
        inte = inte_prev + ((e + e_prev) / 2)*dt;   // integral controller: trapezoidal integration
        /* as errors start to increase faster, the derivative term has more of a weight on the output*/
        der = (e - e_prev)/dt;                      // derivative controller;

        V = kp*e + ki*inte + kd*der;                // constant control = output voltage
        //
        if (V > Vmax) {                             // cieling for voltage
            V = Vmax;               
            inte = inte_prev;
        }

        if (V < Vmin) {                             // floor for voltage
            V = Vmin;
            inte = inte_prev;
            tps_prev = tsp_cur; // what does this do
        }


        t_prev = t;                                 // advance the time step


    }

}


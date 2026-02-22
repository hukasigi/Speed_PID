#pragma once
#include <Arduino.h>

class SpeedPID {
    private:
        double kp, ki, kd;
        double out_min, out_max;
        double dt;
        double prev_error, prev_delta_error;
        double output;

    public:
        SpeedPID(double kp, double ki, double kd, double out_min, double out_max, double dt)
            : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max), dt(dt), prev_error(0.0), prev_delta_error(0.0),
              output(0.0) {}

        double update(double target_rpm, double now_rpm) {
            // 目標値 - 実測値でエラーを出す
            const double error = target_rpm - now_rpm;

            // 増分型 PID
            const double delta_error = error - prev_error;
            const double deriv       = delta_error - prev_delta_error;
            const double du          = kp * delta_error + ki * error * dt + kd * deriv;
            output += du;
            output = constrain(output, out_min, out_max);

            prev_error       = error;
            prev_delta_error = delta_error;

            return output;
        }

        void reset() {
            prev_error       = 0.;
            prev_delta_error = 0.;
            output           = 0.;
        }
};

#pragma once
#include <Arduino.h>

class SpeedPID {
    private:
        double kp, ki, kd;
        double out_min, out_max;
        double dt;
        double pre_error, pre_prop;
        double output;

    public:
        SpeedPID(double kp, double ki, double kd, double out_min, double out_max, double dt)
            : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max), dt(dt), pre_error(0.0), pre_prop(0.0), output(0.0) {}

        double update(double target_rpm, double now_rpm) {
            // 目標値 - 実測値でエラーを出す
            double error = target_rpm - now_rpm;

            double prop  = error - pre_error;
            double deriv = prop - pre_prop;
            double du    = kp * prop + ki * error * dt + kd * deriv;
            output += du;
            output = constrain(output, out_min, out_max);

            pre_error = error;
            pre_prop  = prop;

            return output;
        }

        void reset() {
            pre_error = 0.;
            pre_prop  = 0.;
            output    = 0.;
        }
};
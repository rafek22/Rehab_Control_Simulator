#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController
{
public:
    PIDController(double kp, double ki, double kd);

    double compute(double target, double current, double dt);

    void reset();

private:
    double kp;
    double ki;
    double kd;

    double integral;

    double previousMeasurement;
    bool initialized;

    double minOutput;
    double maxOutput;
};

#endif
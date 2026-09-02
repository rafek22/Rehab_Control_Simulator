#include "PIDController.hpp"

PIDController::PIDController(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    integral = 0.0;

    previousMeasurement = 0.0;
    initialized = false;

    minOutput = -1.0;
    maxOutput = 1.0;
}

double PIDController::compute(double target, double current, double dt)
{
    double error = target - current;

    double derivative = 0.0;

    if (initialized)
    {
        derivative = -(current - previousMeasurement) / dt;
    }
    else
    {
        initialized = true;
    }

    double candidateIntegral = integral + error * dt;

    double candidateOutput = kp * error + ki * candidateIntegral + kd * derivative;

    bool upperSaturation = candidateOutput > maxOutput && error > 0.0;

    bool lowerSaturation = candidateOutput < minOutput && error < 0.0;

    if (!upperSaturation && !lowerSaturation)
    {
        integral = candidateIntegral;
    }

    double output = kp * error + ki * integral + kd * derivative;

    if (output > maxOutput)
        output = maxOutput;

    else if (output < minOutput)
        output = minOutput;

    previousMeasurement = current;

    return output;
}

void PIDController::reset()
{
    integral = 0.0;
    previousMeasurement = 0.0;
    initialized = false;
}
#ifndef SAFETY_MONITOR_HPP
#define SAFETY_MONITOR_HPP

enum class FaultCode
{
    NONE,
    LIMIT,
    STALL,
    ENCODER,
    RESISTANCE
};

class SafetyMonitor
{
public:
    SafetyMonitor(
        double minAngle,
        double maxAngle,
        double CommandThreshold,
        double VelocityThreshold,
        double TimeThreshold,
        double maxTorque
    );

    FaultCode check(
        double angle,
        double velocity,
        double motorCommand,
        double Person_Torque,
        bool movementExpected,
        double dt
    );

    void reset();

private:
    double minAngle;
    double maxAngle;

    double CommandThreshold;
    double VelocityThreshold;
    double TimeThreshold;
    double stallTimer;

    double maxTorque;
};

#endif
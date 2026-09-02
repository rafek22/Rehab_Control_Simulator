#include <cmath>

#include "SafetyMonitor.hpp"

SafetyMonitor::SafetyMonitor(
    double minAngle,
    double maxAngle,
    double CommandThreshold,
    double VelocityThreshold,
    double TimeThreshold,
    double maxTorque
)
{
    this->minAngle = minAngle;
    this->maxAngle = maxAngle;

    this->CommandThreshold = CommandThreshold;
    this->VelocityThreshold = VelocityThreshold;
    this->TimeThreshold = TimeThreshold;

    this->maxTorque = maxTorque;

    stallTimer = 0.0;
}

FaultCode SafetyMonitor::check(
    double angle,
    double velocity,
    double motorCommand,
    double Person_Torque,
    bool movementExpected,
    double dt
)
{
    if (!std::isfinite(angle) || !std::isfinite(velocity))
    {
        return FaultCode::ENCODER;
    }

    if (angle < minAngle || angle > maxAngle)
    {
        return FaultCode::LIMIT;
    }

    if (std::abs(Person_Torque) > maxTorque)
    {
        return FaultCode::RESISTANCE;
    }

    if (
        movementExpected &&
        std::abs(motorCommand) >= CommandThreshold &&
        std::abs(velocity) <= VelocityThreshold
    )
    {
        stallTimer += dt;
        if (stallTimer >= TimeThreshold)
        {
            return FaultCode::STALL;
        }
    }
    else
    {
        stallTimer = 0.0;
    }
    return FaultCode::NONE;
}

void SafetyMonitor::reset()
{
    stallTimer = 0.0;
}

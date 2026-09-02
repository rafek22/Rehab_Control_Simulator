#include <cmath>
#include <stdexcept>

#include "SafetyMonitor.hpp"

SafetyMonitor::SafetyMonitor(
    double minAngle,
    double maxAngle,
    double commandThreshold,
    double velocityThreshold,
    double timeThreshold,
    double maxTorque
)
{
    // Rechaza configuraciones no numericas o infinitas.
    if (!std::isfinite(minAngle) || !std::isfinite(maxAngle) ||
        !std::isfinite(commandThreshold) || !std::isfinite(velocityThreshold) ||
        !std::isfinite(timeThreshold) || !std::isfinite(maxTorque))
    {
        throw std::invalid_argument("Safety thresholds must be finite");
    }

    // Comprueba que los limites y umbrales tengan sentido fisico.
    if (minAngle >= maxAngle || commandThreshold < 0.0 ||
        velocityThreshold < 0.0 || timeThreshold < 0.0 || maxTorque < 0.0)
    {
        throw std::invalid_argument("Safety thresholds are invalid");
    }

    // Guarda la configuracion validada.
    this->minAngle = minAngle;
    this->maxAngle = maxAngle;

    this->commandThreshold = commandThreshold;
    this->velocityThreshold = velocityThreshold;
    this->timeThreshold = timeThreshold;

    this->maxTorque = maxTorque;

    stallTimer = 0.0;
}

FaultCode SafetyMonitor::check(
    double angle,
    double velocity,
    double motorCommand,
    double personTorque,
    bool movementExpected,
    double dt
)
{
    // Un angulo o una velocidad no finitos representan un fallo del encoder.
    if (!std::isfinite(angle) || !std::isfinite(velocity))
        return FaultCode::ENCODER;

    // El resto de entradas tambien debe ser finito y usar un dt positivo.
    if (!std::isfinite(motorCommand) || !std::isfinite(personTorque) ||
        !std::isfinite(dt) || dt <= 0.0)
    {
        return FaultCode::INVALID_DATA;
    }

    // Comprueba que la articulacion permanezca dentro del recorrido permitido.
    if (angle < minAngle || angle > maxAngle)
        return FaultCode::LIMIT;

    // Detecta una resistencia excesiva en cualquiera de las dos direcciones.
    if (std::abs(personTorque) > maxTorque)
        return FaultCode::RESISTANCE;

    // Acumula tiempo si se ordena movimiento pero el codo casi no se mueve.
    if (
        movementExpected && std::abs(motorCommand) >= commandThreshold &&
        std::abs(velocity) <= velocityThreshold
    )
    {
        stallTimer += dt;
        if (stallTimer >= timeThreshold)
        {
            return FaultCode::STALL;
        }
    }
    else
    {
        // Al desaparecer la condicion de bloqueo, comienza a contar desde cero.
        stallTimer = 0.0;
    }
    return FaultCode::NONE;
}

void SafetyMonitor::reset()
{
    // Permite iniciar una comprobacion de bloqueo nueva.
    stallTimer = 0.0;
}

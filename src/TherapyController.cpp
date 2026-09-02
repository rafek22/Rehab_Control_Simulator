#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "TherapyController.hpp"

namespace
{
    // Acerca una referencia al objetivo sin superar el cambio maximo permitido.
    double moveTowards(
        double current,
        double target,
        double maxStep
    )
    {
        if (current < target)
            return std::min(current + maxStep, target);

        if (current > target)
            return std::max(current - maxStep, target);

        return current;
    }
}

TherapyController::TherapyController(
    double initialAngle,
    double targetAngle,
    int repetitions,
    double holdDuration,
    double angleTolerance,
    double velocityTolerance,
    double movementSpeed
)
{
    // Rechaza valores infinitos o indefinidos antes de iniciar la terapia.
    if (!std::isfinite(initialAngle) || !std::isfinite(targetAngle) ||
        !std::isfinite(holdDuration) || !std::isfinite(angleTolerance) ||
        !std::isfinite(velocityTolerance) || !std::isfinite(movementSpeed))
    {
        throw std::invalid_argument("Therapy parameters must be finite");
    }

    // Una sesion debe tener al menos una repeticion.
    if (repetitions <= 0)
        throw std::invalid_argument("Repetitions must be greater than zero");

    // Las duraciones y tolerancias no pueden ser negativas.
    if (holdDuration < 0.0 || angleTolerance < 0.0 ||
        velocityTolerance < 0.0 || movementSpeed <= 0.0)
        throw std::invalid_argument("Therapy durations and tolerances must be non-negative, and movement speed positive");

    // Guarda la configuracion validada de la sesion.
    this->initialAngle = initialAngle;
    this->targetAngle = targetAngle;

    this->repetitions = repetitions;
    this->holdDuration = holdDuration;

    this->angleTolerance = angleTolerance;
    this->velocityTolerance = velocityTolerance;

    this->movementSpeed = movementSpeed;

    // La referencia comienza en la posicion inicial y el sistema queda esperando.
    referenceAngle = initialAngle;

    completedRepetitions = 0;
    holdElapsed = 0.0;

    state = TherapyState::WAITING;
}

void TherapyController::start()
{
    // Reinicia la sesion y activa el primer movimiento.
    completedRepetitions = 0;
    holdElapsed = 0.0;

    referenceAngle = initialAngle;

    state = TherapyState::MOVE;
}

void TherapyController::update(
    double currentAngle,
    double currentVelocity,
    double dt
)
{
    // Evita actualizar la maquina de estados con medidas o tiempos invalidos.
    if (!std::isfinite(currentAngle) || !std::isfinite(currentVelocity) ||
        !std::isfinite(dt) || dt <= 0.0)
    {
        throw std::invalid_argument("Therapy inputs must be finite and dt must be positive");
    }

    switch (state)
    {
        case TherapyState::WAITING:
            // No hace nada hasta que se llame a start().
            break;

        case TherapyState::MOVE:
        {
            // Avanza suavemente la referencia hacia el angulo objetivo.
            referenceAngle = moveTowards(
                referenceAngle,
                targetAngle,
                movementSpeed * dt
            );

            double error =
                targetAngle - currentAngle;

            // No entra en HOLD hasta que referencia, posicion y velocidad sean estables.
            bool referenceReached =
                std::abs(referenceAngle - targetAngle) < 1e-9;

            if (
                referenceReached &&
                std::abs(error) <= angleTolerance &&
                std::abs(currentVelocity) <= velocityTolerance
            )
            {
                referenceAngle = targetAngle;
                holdElapsed = 0.0;

                state = TherapyState::HOLD;
            }

            break;
        }

        case TherapyState::HOLD:
        {
            // Mantiene fija la referencia en el angulo objetivo.
            referenceAngle = targetAngle;

            double error =
                targetAngle - currentAngle;

            // Si sale de las tolerancias, vuelve a la fase de movimiento.
            if (
                std::abs(error) > angleTolerance ||
                std::abs(currentVelocity) > velocityTolerance
            )
            {
                holdElapsed = 0.0;

                state = TherapyState::MOVE;
            }
            else
            {
                // Acumula solo el tiempo permanecido correctamente en el objetivo.
                holdElapsed += dt;

                if (holdElapsed >= holdDuration)
                {
                    referenceAngle = targetAngle;

                    state = TherapyState::RETURN;
                }
            }

            break;
        }

        case TherapyState::RETURN:
        {
            // Reduce suavemente la referencia hasta el angulo inicial.
            referenceAngle = moveTowards(
                referenceAngle,
                initialAngle,
                movementSpeed * dt
            );

            double error =
                initialAngle - currentAngle;

            bool referenceReached =
                std::abs(referenceAngle - initialAngle) < 1e-9;

            // Cuenta la repeticion cuando referencia y codo han regresado y estan estables.
            if (
                referenceReached &&
                std::abs(error) <= angleTolerance &&
                std::abs(currentVelocity) <= velocityTolerance
            )
            {
                referenceAngle = initialAngle;

                completedRepetitions++;

                if (completedRepetitions >= repetitions)
                    state = TherapyState::WAITING;
                else
                    state = TherapyState::MOVE;
            }

            break;
        }

        case TherapyState::FAULT:
            // Un fallo mantiene detenida la maquina de estados.
            break;
    }
}

double TherapyController::getReferenceAngle() const
{
    // El PID utiliza esta referencia progresiva en lugar de un salto instantaneo.
    return referenceAngle;
}

TherapyState TherapyController::getState() const
{
    return state;
}

int TherapyController::getCompletedRepetitions() const
{
    return completedRepetitions;
}

void TherapyController::setFault()
{
    // Cualquier fallo de seguridad detiene la secuencia de terapia.
    state = TherapyState::FAULT;
}

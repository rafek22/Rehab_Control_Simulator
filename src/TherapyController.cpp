#include <algorithm>
#include <cmath>

#include "TherapyController.hpp"

namespace
{
    double moveTowards(
        double current,
        double target,
        double maxStep
    )
    {
        if (current < target)
        {
            return std::min(current + maxStep, target);
        }

        if (current > target)
        {
            return std::max(current - maxStep, target);
        }

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
    this->initialAngle = initialAngle;
    this->targetAngle = targetAngle;

    this->repetitions = repetitions;
    this->holdDuration = holdDuration;

    this->angleTolerance = angleTolerance;
    this->velocityTolerance = velocityTolerance;

    this->movementSpeed = movementSpeed;

    referenceAngle = initialAngle;

    completedRepetitions = 0;
    holdElapsed = 0.0;

    state = TherapyState::WAITING;
}

void TherapyController::start()
{
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
    switch (state)
    {
        case TherapyState::WAITING:
            break;

        case TherapyState::MOVE:
        {
            referenceAngle = moveTowards(
                referenceAngle,
                targetAngle,
                movementSpeed * dt
            );

            double error =
                targetAngle - currentAngle;

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
            referenceAngle = targetAngle;

            double error =
                targetAngle - currentAngle;

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
            referenceAngle = moveTowards(
                referenceAngle,
                initialAngle,
                movementSpeed * dt
            );

            double error =
                initialAngle - currentAngle;

            bool referenceReached =
                std::abs(referenceAngle - initialAngle) < 1e-9;

            if (
                referenceReached &&
                std::abs(error) <= angleTolerance &&
                std::abs(currentVelocity) <= velocityTolerance
            )
            {
                referenceAngle = initialAngle;

                completedRepetitions++;

                if (completedRepetitions >= repetitions)
                {
                    state = TherapyState::WAITING;
                }
                else
                {
                    state = TherapyState::MOVE;
                }
            }

            break;
        }

        case TherapyState::FAULT:
            break;
    }
}

double TherapyController::getReferenceAngle() const
{
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
    state = TherapyState::FAULT;
}

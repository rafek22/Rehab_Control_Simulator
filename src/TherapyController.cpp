#include <cmath>

#include "TherapyController.hpp"

TherapyController::TherapyController(
    double initialAngle,
    double targetAngle,
    int repetitions,
    double holdDuration,
    double angleTolerance
)
{
    this->initialAngle = initialAngle;
    this->targetAngle = targetAngle;

    this->repetitions = repetitions;
    this->holdDuration = holdDuration;
    this->angleTolerance = angleTolerance;

    completedRepetitions = 0;
    holdElapsed = 0.0;

    state = TherapyState::WAITING;
}

void TherapyController::start()
{
    completedRepetitions = 0;
    holdElapsed = 0.0;

    state = TherapyState::MOVE;
}

void TherapyController::update(double currentAngle, double dt)
{
    switch (state)
    {
        case TherapyState::WAITING:
            break;

        case TherapyState::MOVE:
        {
            double error = targetAngle - currentAngle;

            if (std::abs(error) <= angleTolerance)
            {
                holdElapsed = 0.0;
                state = TherapyState::HOLD;
            }

            break;
        }

        case TherapyState::HOLD:
        {   
            double error = targetAngle - currentAngle;
            holdElapsed += dt;
            if (std::abs(error) > angleTolerance)
            {
                holdElapsed = 0.0;
                state = TherapyState::MOVE;
            }
            else
            {
                holdElapsed += dt;
                if (holdElapsed >= holdDuration)
                    state = TherapyState::RETURN;
            }
            break;
        }

        case TherapyState::RETURN:
        {
            double error = initialAngle - currentAngle;

            if (std::abs(error) <= angleTolerance)
            {
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

double TherapyController::getTargetAngle() const
{
    switch (state)
    {
        case TherapyState::MOVE:
        case TherapyState::HOLD:
            return targetAngle;

        case TherapyState::RETURN:
        case TherapyState::WAITING:
        case TherapyState::FAULT:
        default:
            return initialAngle;
    }
}

TherapyState TherapyController::getState() const
{
    return state;
}

int TherapyController::getCompletedRepetitions() const
{
    return completedRepetitions;
}

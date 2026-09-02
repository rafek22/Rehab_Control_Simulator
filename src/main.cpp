#include <iostream>
#include <iomanip>

#include "ElbowModel.hpp"
#include "PIDController.hpp"
#include "TherapyController.hpp"

constexpr double PI = 3.14159265358979323846;

double degToRad(double degrees)
{
    return degrees * PI / 180.0;
}

double radToDeg(double radians)
{
    return radians * 180.0 / PI;
}

const char* stateToString(TherapyState state)
{
    switch (state)
    {
        case TherapyState::WAITING:
            return "WAITING";

        case TherapyState::MOVE:
            return "MOVE";

        case TherapyState::HOLD:
            return "HOLD";

        case TherapyState::RETURN:
            return "RETURN";

        case TherapyState::FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

int main()
{
    ElbowModel elbow;

    PIDController pid(
        0.574,
        1.344,
        0.154
    );

    TherapyController therapy(
        degToRad(30.0),   // initial angle
        degToRad(110.0),  // target angle
        3,                // repetitions
        2.0,              // hold time
        degToRad(1.0)     // tolerance
    );

    double dt = 0.01;
    double time = 0.0;

    therapy.start();

    TherapyState previousState = therapy.getState();

    while (true)
    {
        therapy.update(elbow.getAngle(), dt);

        TherapyState currentState = therapy.getState();

        if (currentState != previousState)
        {
            if (currentState == TherapyState::RETURN ||
                currentState == TherapyState::MOVE)
            {
                pid.reset();
            }

            previousState = currentState;
        }

        if (currentState == TherapyState::WAITING ||
            currentState == TherapyState::FAULT)
        {
            break;
        }

        double targetAngle = therapy.getTargetAngle();

        double motorCommand = pid.compute(
            targetAngle,
            elbow.getAngle(),
            dt
        );

        elbow.update(motorCommand, dt);

        if (static_cast<int>(time * 100) % 50 == 0)
        {
            double error =
                targetAngle - elbow.getAngle();

            std::cout
                << std::fixed
                << std::setprecision(2)
                << "Time: " << time << " s"
                << " | State: " << stateToString(currentState)
                << " | Rep: " << therapy.getCompletedRepetitions()
                << "/3"
                << " | Target: " << radToDeg(targetAngle) << " deg"
                << " | Angle: " << radToDeg(elbow.getAngle()) << " deg"
                << " | Error: " << radToDeg(error) << " deg"
                << " | Motor: " << motorCommand
                << '\n';
        }

        time += dt;
    }

    std::cout
        << "\nTherapy session completed."
        << "\nCompleted repetitions: "
        << therapy.getCompletedRepetitions()
        << '\n';

    return 0;
}
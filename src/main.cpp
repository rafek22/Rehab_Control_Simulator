#include <iostream>
#include <iomanip>

#include "ElbowModel.hpp"
#include "PIDController.hpp"
#include "TherapyController.hpp"
#include "SafetyMonitor.hpp"
#include "Logger.hpp"

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

const char* faultToString(FaultCode fault)
{
    switch (fault)
    {
        case FaultCode::NONE:
            return "NONE";

        case FaultCode::LIMIT:
            return "LIMIT";

        case FaultCode::STALL:
            return "STALL";

        case FaultCode::ENCODER:
            return "ENCODER";

        case FaultCode::RESISTANCE:
            return "RESISTANCE";

        default:
            return "UNKNOWN";
    }
}

int main()
{
    ElbowModel elbow;

    PIDController pid(
        0.574,  // Kp
        1.344,  // Ki
        0.154   // Kd
    );

    TherapyController therapy(
    degToRad(30.0),   // initial angle
    degToRad(110.0),  // target angle
    3,                // repetitions
    2.0,              // hold time
    degToRad(1.0),    // angle tolerance
    0.15,             // velocity tolerance [rad/s]
    degToRad(40.0)    // movement speed [rad/s]
    );

    SafetyMonitor safety(
        degToRad(0.0),    // minimum joint angle
        degToRad(140.0),  // maximum joint angle

        0.70,             // stall command threshold
        0.02,             // stall velocity threshold [rad/s]
        0.50,             // stall time threshold [s]

        3.0               // maximum patient resistance torque [Nm]
    );

    Logger logger("../../output/session.csv");

    double dt = 0.01;
    double time = 0.0;

    therapy.start();

    TherapyState previousState = therapy.getState();

    while (true)
    {
        // Update therapy state machine
        therapy.update(
            elbow.getAngle(),
            elbow.getVelocity(),
            dt
        );

        TherapyState currentState = therapy.getState();

        // Reset PID when the target changes
        if (currentState != previousState)
        {
            if (currentState == TherapyState::RETURN ||
                currentState == TherapyState::MOVE)
            {
                pid.reset();
            }

            previousState = currentState;
        }

        // Stop if therapy has finished
        if (currentState == TherapyState::WAITING)
        {
            break;
        }

        // Stop if a fault was already active
        if (currentState == TherapyState::FAULT)
        {
            break;
        }

        // Get target according to current therapy state
        double referenceAngle = therapy.getReferenceAngle();

        // Closed-loop PID control
        double motorCommand = pid.compute(
            referenceAngle,
            elbow.getAngle(),
            dt
        );

        // Simulate motor + elbow + patient
        elbow.update(motorCommand, dt);

        double error = referenceAngle - elbow.getAngle();

        // Stall should only be evaluated when movement is expected
        bool movementExpected =
            currentState == TherapyState::MOVE ||
            currentState == TherapyState::RETURN;

        // Safety checks
        FaultCode fault = safety.check(
            elbow.getAngle(),
            elbow.getVelocity(),
            motorCommand,
            elbow.getPersonTorque(),
            movementExpected,
            dt
        );

        if (fault != FaultCode::NONE)
        {
            therapy.setFault();

            // Log the sample that caused the fault
            logger.log(
                time,
                stateToString(currentState),
                radToDeg(referenceAngle),
                radToDeg(elbow.getAngle()),
                elbow.getVelocity(),
                motorCommand,
                radToDeg(error)
            );

            std::cout
                << "\nFAULT detected: "
                << faultToString(fault)
                << '\n';

            break;
        }

        // Log every simulation step (100 Hz)
        logger.log(
            time,
            stateToString(currentState),
            radToDeg(referenceAngle),
            radToDeg(elbow.getAngle()),
            elbow.getVelocity(),
            motorCommand,
            radToDeg(error)
        );

        // Console output every 0.5 seconds
        if (static_cast<int>(time * 100) % 50 == 0)
        {
            std::cout
                << std::fixed
                << std::setprecision(2)
                << "Time: " << time << " s"
                << " | State: " << stateToString(currentState)
                << " | Rep: " << therapy.getCompletedRepetitions()
                << "/3"
                << " | Target: " << radToDeg(referenceAngle) << " deg"
                << " | Angle: " << radToDeg(elbow.getAngle()) << " deg"
                << " | Error: " << radToDeg(error) << " deg"
                << " | Motor: " << motorCommand
                << '\n';
        }

        time += dt;
    }

    if (therapy.getState() == TherapyState::FAULT)
    {
        std::cout
            << "\nTherapy session aborted."
            << "\nCompleted repetitions: "
            << therapy.getCompletedRepetitions()
            << '\n';
    }
    else
    {
        std::cout
            << "\nTherapy session completed."
            << "\nCompleted repetitions: "
            << therapy.getCompletedRepetitions()
            << '\n';
    }

    return 0;
}

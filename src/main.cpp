#include <iostream>
#include <iomanip>

#include "ElbowModel.hpp"
#include "PIDController.hpp"

constexpr double PI = 3.14159265358979323846;

double degToRad(double degrees)
{
    return degrees * PI / 180.0;
}

double radToDeg(double radians)
{
    return radians * 180.0 / PI;
}

int main()
{
    ElbowModel elbow;

    PIDController pid(
        0.574,   // Kp
        1.344,   // Ki
        0.154   // Kd
    );

    double dt = 0.01;
    double simulationTime = 5.0;

    double targetAngle = degToRad(110.0);

    for (double time = 0.0; time <= simulationTime; time += dt)
    {
        double motorCommand = pid.compute(targetAngle, elbow.getAngle(), dt);

        elbow.update(motorCommand, dt);

        if (static_cast<int>(time * 100) % 50 == 0)
        {
            double error =
                targetAngle - elbow.getAngle();

            std::cout
                << std::fixed
                << std::setprecision(2)
                << "Time: " << time << " s"
                << " | Target: " << radToDeg(targetAngle) << " deg"
                << " | Angle: " << radToDeg(elbow.getAngle()) << " deg"
                << " | Error: " << radToDeg(error) << " deg"
                << " | Motor: " << motorCommand
                << '\n';
        }
    }

    return 0;
}
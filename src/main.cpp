#include <iostream>
#include <iomanip>

#include "ElbowModel.hpp"

constexpr double PI = 3.14159265358979323846;

double RadToDeg(double radians)
{
    return radians * 180.0 / PI;
}

int main()
{
    ElbowModel elbow;

    double dt = 0.01;
    double simulationTime = 5.0;

    double motorCommand = 0.4;

    for (double time = 0.0; time <= simulationTime; time += dt)
    {
        elbow.update(motorCommand, dt);

        if (static_cast<int>(time * 100) % 50 == 0)
        {
            std::cout
                << std::fixed
                << std::setprecision(2)
                << "Time: " << time << " s"
                << " | Angle: " << RadToDeg(elbow.getAngle()) << " deg"
                << " | Velocity: " << elbow.getVelocity() << " rad/s"
                << '\n';
        }
    }

    return 0;
}
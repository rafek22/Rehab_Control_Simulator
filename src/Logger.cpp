#include "Logger.hpp"

#include <iomanip>

Logger::Logger(const std::string& filename)
{
    file.open(filename);

    if (file.is_open())
    {
        file
            << "time,"
            << "state,"
            << "target_angle,"
            << "angle,"
            << "velocity,"
            << "motor_command,"
            << "error\n";
    }
}

Logger::~Logger()
{
    if (file.is_open())
    {
        file.close();
    }
}

void Logger::log(
    double time,
    const std::string& state,
    double targetAngle,
    double angle,
    double velocity,
    double motorCommand,
    double error
)
{
    if (!file.is_open())
    {
        return;
    }

    file
        << "time,"
        << "state,"
        << "reference_angle,"
        << "angle,"
        << "velocity,"
        << "motor_command,"
        << "error\n";
}
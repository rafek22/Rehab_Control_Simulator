#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <string>

class Logger
{
public:
    Logger(const std::string& filename);
    ~Logger();

    void log(
        double time,
        const std::string& state,
        double targetAngle,
        double angle,
        double velocity,
        double motorCommand,
        double error
    );

private:
    std::ofstream file;
};

#endif
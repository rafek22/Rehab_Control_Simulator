#include "Logger.hpp"

#include <iomanip>

Logger::Logger(const std::string& filename)
{
    // Abre o reemplaza el CSV de la sesion.
    file.open(filename);

    // La cabecera se escribe una sola vez al crear el logger.
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
    // Asegura que los datos pendientes se escriban y el archivo quede cerrado.
    if (file.is_open())
        file.close();
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
    // Si el archivo no pudo abrirse, no intenta escribir la muestra.
    if (!file.is_open())
        return;

    // Guarda una fila con seis decimales y columnas separadas por comas.
    file
        << std::fixed
        << std::setprecision(6)
        << time << ","
        << state << ","
        << targetAngle << ","
        << angle << ","
        << velocity << ","
        << motorCommand << ","
        << error << "\n";
}

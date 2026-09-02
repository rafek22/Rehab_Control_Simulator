#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>
#include <string>

class Logger
{
public:
    // Abre el CSV indicado y escribe su fila de cabecera.
    Logger(const std::string& filename);

    // Cierra el archivo al destruir el objeto.
    ~Logger();

    // Guarda una muestra completa de la simulacion en una fila del CSV.
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
    std::ofstream file; // Archivo CSV de la sesion.
};

#endif

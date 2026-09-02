#ifndef PLOTTER_HPP
#define PLOTTER_HPP

#include <string>
#include <vector>

struct Plot
{
    double time;           // Instante de la muestra, en segundos.
    double referenceAngle; // Referencia del PID, en grados.
    double angle;          // Angulo real del codo, en grados.
    double velocity;       // Velocidad angular, en rad/s.
    double motorCommand;   // Orden normalizada del motor entre -1 y 1.
};

class Plotter
{
public:
    // Guarda una muestra en memoria para representarla posteriormente.
    void addSample(
        double time,
        double referenceAngle,
        double angle,
        double velocity,
        double motorCommand
    );

    // Genera las tres graficas SVG dentro del directorio indicado.
    void saveAll(const std::string& directory) const;

private:
    std::vector<Plot> samples; // Muestras acumuladas durante la sesion.

    // Genera la grafica de referencia y posicion real.
    void PositionPlot(const std::string& filename) const;

    // Genera la grafica de velocidad angular.
    void VelocityPlot(const std::string& filename) const;

    // Genera la grafica de la orden enviada al motor.
    void MotorPlot(const std::string& filename) const;
};

#endif

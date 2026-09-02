#ifndef METRICS_HPP
#define METRICS_HPP

#include <string>

class Metrics
{
public:
    // Inicializa todos los acumuladores de la sesion.
    Metrics();

    // Incorpora una muestra al calculo de maximos, RMSE y error durante HOLD.
    void update(
        double time,
        double angle,
        double velocity,
        double motorCommand,
        double error,
        bool isHold
    );

    // Guarda las metricas calculadas en un archivo de texto.
    void save(const std::string& filename) const;

private:
    double lastTime; // Ultimo instante registrado; representa la duracion de la sesion.

    double maxAngle; // Mayor angulo observado, en grados.
    double minAngle; // Menor angulo observado, en grados.

    double maxAbsVelocity;     // Mayor velocidad absoluta observada, en rad/s.
    double maxAbsMotorCommand; // Mayor orden absoluta del motor.

    double squaredErrorSum; // Suma de errores al cuadrado para calcular el RMSE.
    int sampleCount;        // Numero total de muestras.

    double holdAbsoluteErrorSum; // Suma del error absoluto durante HOLD.
    int holdSampleCount;          // Numero de muestras tomadas durante HOLD.

    bool initialized; // Indica si ya existe una primera muestra para los maximos.
};

#endif

#include "Metrics.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>

Metrics::Metrics()
{
    // Inicializa tiempos, extremos y acumuladores antes de recibir muestras.
    lastTime = 0.0;

    maxAngle = 0.0;
    minAngle = 0.0;

    maxAbsVelocity = 0.0;
    maxAbsMotorCommand = 0.0;

    squaredErrorSum = 0.0;
    sampleCount = 0;

    holdAbsoluteErrorSum = 0.0;
    holdSampleCount = 0;

    initialized = false;
}

void Metrics::update(
    double time,
    double angle,
    double velocity,
    double motorCommand,
    double error,
    bool isHold
)
{
    // La ultima marca temporal representa la duracion alcanzada.
    lastTime = time;

    // La primera muestra establece los maximos y minimos iniciales.
    if (!initialized)
    {
        maxAngle = angle;
        minAngle = angle;

        initialized = true;
    }
    else
    {
        if (angle > maxAngle)
            maxAngle = angle;

        if (angle < minAngle)
            minAngle = angle;
    }

    // Registra la mayor magnitud de velocidad, sin importar su direccion.
    double absVelocity = std::abs(velocity);

    if (absVelocity > maxAbsVelocity)
        maxAbsVelocity = absVelocity;

    // Registra el mayor esfuerzo normalizado solicitado al motor.
    double absMotorCommand = std::abs(motorCommand);

    if (absMotorCommand > maxAbsMotorCommand)
        maxAbsMotorCommand = absMotorCommand;

    // Acumula el error cuadratico de todas las muestras para calcular el RMSE.
    squaredErrorSum += error * error;
    sampleCount++;

    // Durante HOLD tambien acumula el error absoluto de mantenimiento.
    if (isHold)
    {
        holdAbsoluteErrorSum += std::abs(error);
        holdSampleCount++;
    }
}

void Metrics::save(const std::string& filename) const
{
    // Crea o reemplaza el informe de metricas.
    std::ofstream file(filename);

    if (!file.is_open())
        return;

    double rmse = 0.0;
    double holdMae = 0.0;

    // Evita dividir entre cero si no se recibieron muestras.
    if (sampleCount > 0)
        rmse = std::sqrt(squaredErrorSum / sampleCount);

    // Calcula el error medio solo si la terapia alcanzo HOLD.
    if (holdSampleCount > 0)
        holdMae = holdAbsoluteErrorSum / holdSampleCount;

    // Presenta todos los resultados con tres cifras decimales.
    file
        << std::fixed
        << std::setprecision(3);

    file
        << "REHAB CONTROL SIMULATOR - SESSION METRICS\n"
        << "=========================================\n\n"

        << "Session duration:            "
        << lastTime
        << " s\n"

        << "Maximum angle:               "
        << maxAngle
        << " deg\n"

        << "Minimum angle:               "
        << minAngle
        << " deg\n"

        << "Maximum absolute velocity:   "
        << maxAbsVelocity
        << " rad/s\n"

        << "Maximum absolute command:    "
        << maxAbsMotorCommand
        << "\n"

        << "Tracking RMSE:               "
        << rmse
        << " deg\n"

        << "HOLD mean absolute error:    "
        << holdMae
        << " deg\n";
}

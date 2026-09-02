#ifndef SIMULATION_CONFIG_HPP
#define SIMULATION_CONFIG_HPP

#include <string>

// Agrupa en un solo lugar todos los parametros configurables del simulador.
// Los angulos destinados al usuario se expresan en grados para facilitar su lectura.
struct SimulationConfig
{
    // Ganancias del controlador PID.
    double kp = 0.574;
    double ki = 1.344;
    double kd = 0.154;

    // Configuracion de la sesion de terapia.
    double initialAngleDegrees = 30.0;
    double targetAngleDegrees = 110.0;
    int repetitions = 3;
    double holdDuration = 2.0;
    double angleToleranceDegrees = 1.0;
    double velocityTolerance = 0.15;
    double movementSpeedDegreesPerSecond = 40.0;

    // Limites utilizados por el monitor de seguridad.
    double minimumAngleDegrees = 0.0;
    double maximumAngleDegrees = 140.0;
    double stallCommandThreshold = 0.70;
    double stallVelocityThreshold = 0.02;
    double stallTimeThreshold = 0.50;
    double maximumPersonTorque = 3.0;

    // Configuracion temporal de la simulacion.
    double timeStep = 0.01;
    double maximumSimulationTime = 120.0;

    // Archivos generados al finalizar la sesion.
    std::string sessionCsvPath = "../../output/session.csv";
    std::string metricsPath = "../../output/metrics.txt";
    std::string figuresDirectory = "../../output/figures";
};

#endif

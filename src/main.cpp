#include <iostream>
#include <iomanip>

#include "ElbowModel.hpp"
#include "PIDController.hpp"
#include "TherapyController.hpp"
#include "SafetyMonitor.hpp"
#include "Logger.hpp"
#include "Metrics.hpp"
#include "Plotter.hpp"
#include "SimulationConfig.hpp"

constexpr double PI = 3.14159265358979323846;

double degToRad(double degrees) {return degrees * PI / 180.0;}

double radToDeg(double radians) {return radians * 180.0 / PI;}

const char* stateToString(TherapyState state)
{
    switch (state)
    {
        case TherapyState::WAITING:
            return "WAITING";

        case TherapyState::MOVE:
            return "MOVE";

        case TherapyState::HOLD:
            return "HOLD";

        case TherapyState::RETURN:
            return "RETURN";

        case TherapyState::FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

const char* faultToString(FaultCode fault)
{
    switch (fault)
    {
        case FaultCode::NONE:
            return "NONE";

        case FaultCode::LIMIT:
            return "LIMIT";

        case FaultCode::STALL:
            return "STALL";

        case FaultCode::ENCODER:
            return "ENCODER";

        case FaultCode::RESISTANCE:
            return "RESISTANCE";

        case FaultCode::INVALID_DATA:
            return "INVALID_DATA";

        default:
            return "UNKNOWN";
    }
}

int main()
{
    // Todos los parametros de la ejecucion se obtienen de una sola configuracion.
    const SimulationConfig config;

    ElbowModel elbow;

    PIDController pid(
        config.kp,
        config.ki,
        config.kd
    );

    TherapyController therapy(
        degToRad(config.initialAngleDegrees),
        degToRad(config.targetAngleDegrees),
        config.repetitions,
        config.holdDuration,
        degToRad(config.angleToleranceDegrees),
        config.velocityTolerance,
        degToRad(config.movementSpeedDegreesPerSecond)
    );

    SafetyMonitor safety(
        degToRad(config.minimumAngleDegrees),
        degToRad(config.maximumAngleDegrees),
        config.stallCommandThreshold,
        config.stallVelocityThreshold,
        config.stallTimeThreshold,
        config.maximumPersonTorque
    );

    Logger logger(config.sessionCsvPath);

    Metrics metrics;
    Plotter plotter;

    const double dt = config.timeStep;
    double time = 0.0;

    therapy.start();

    TherapyState previousState = therapy.getState();

    while (time <= config.maximumSimulationTime)
    {
        // Actualiza la maquina de estados de la terapia.
        therapy.update(
            elbow.getAngle(),
            elbow.getVelocity(),
            dt
        );

        TherapyState currentState = therapy.getState();

        // Reinicia el PID cuando cambia la direccion o el objetivo.
        if (currentState != previousState)
        {
            if (currentState == TherapyState::RETURN || currentState == TherapyState::MOVE)
                pid.reset();

            previousState = currentState;
        }

        // Sale del bucle cuando termina la terapia.
        if (currentState == TherapyState::WAITING)
            break;

        // Sale del bucle si ya existe un fallo.
        if (currentState == TherapyState::FAULT)
            break;

        // Obtiene la referencia progresiva generada por la terapia.
        double referenceAngle = therapy.getReferenceAngle();

        // Calcula la orden del motor mediante el control PID en lazo cerrado.
        double motorCommand = pid.compute(
            referenceAngle,
            elbow.getAngle(),
            dt
        );

        // Simula el motor, el codo y la resistencia de la persona.
        elbow.update( motorCommand, dt);

        // Calcula el error entre la referencia y el angulo real.
        double error = referenceAngle - elbow.getAngle();

        bool isHold = currentState == TherapyState::HOLD;

        // Solo comprueba bloqueo durante las fases que esperan movimiento.
        bool movementExpected =
            currentState == TherapyState::MOVE ||
            currentState == TherapyState::RETURN;

        // Ejecuta todas las comprobaciones de seguridad.
        FaultCode fault = safety.check(
            elbow.getAngle(),
            elbow.getVelocity(),
            motorCommand,
            elbow.getPersonTorque(),
            movementExpected,
            dt
        );

        if (fault != FaultCode::NONE)
        {
            therapy.setFault();
            // Guarda la muestra exacta que produjo el fallo.
            logger.log(
                time,
                "FAULT",
                radToDeg(referenceAngle),
                radToDeg(elbow.getAngle()),
                elbow.getVelocity(),
                motorCommand,
                radToDeg(error)
            );
            // Incluye tambien la muestra del fallo en las metricas.
            metrics.update(
                time,
                radToDeg(elbow.getAngle()),
                elbow.getVelocity(),
                motorCommand,
                radToDeg(error),
                isHold
            );

            std::cout
                << "\nFAULT detected: "
                << faultToString(fault)
                << '\n';

            break;
        }

        // Guarda cada paso de la simulacion en el CSV.
        logger.log(
            time,
            stateToString(currentState),
            radToDeg(referenceAngle),
            radToDeg(elbow.getAngle()),
            elbow.getVelocity(),
            motorCommand,
            radToDeg(error)
        );

        // Actualiza las metricas generales de la sesion.
        metrics.update(
            time,
            radToDeg(elbow.getAngle()),
            elbow.getVelocity(),
            motorCommand,
            radToDeg(error),
            isHold
        );

        plotter.addSample(
            time,
            radToDeg(referenceAngle),
            radToDeg(elbow.getAngle()),
            elbow.getVelocity(),
            motorCommand
        );

        // Muestra un resumen en la consola cada 0.5 segundos.
        if (static_cast<int>(time * 100) % 50 == 0)
        {
            std::cout
                << std::fixed
                << std::setprecision(2)

                << "Time: " << time << " s"

                << " | State: "
                << stateToString(currentState)

                << " | Rep: "
                << therapy.getCompletedRepetitions()
                << "/" << config.repetitions

                << " | Reference: "
                << radToDeg(referenceAngle)
                << " deg"

                << " | Angle: "
                << radToDeg(elbow.getAngle())
                << " deg"

                << " | Error: "
                << radToDeg(error)
                << " deg"

                << " | Motor: "
                << motorCommand

                << '\n';
        }
        time += dt;
    }

    if (therapy.getState() != TherapyState::WAITING && therapy.getState() != TherapyState::FAULT)
    {
        therapy.setFault();
        std::cout << "\nFAULT detected: TIMEOUT\n";
    }

    // Guarda las metricas y las graficas al terminar la sesion.
    metrics.save(config.metricsPath);
    plotter.saveAll(config.figuresDirectory);

    if (therapy.getState() == TherapyState::FAULT)
    {
        std::cout
            << "\nTherapy session aborted."

            << "\nCompleted repetitions: "
            << therapy.getCompletedRepetitions()

            << '\n';
    }
    else
    {
        std::cout
            << "\nTherapy session completed."

            << "\nCompleted repetitions: "
            << therapy.getCompletedRepetitions()

            << '\n';
    }
    return 0;
}

#ifndef SAFETY_MONITOR_HPP
#define SAFETY_MONITOR_HPP

enum class FaultCode
{
    NONE,         // No se ha detectado ningun problema.
    LIMIT,        // El angulo ha salido de los limites permitidos.
    STALL,        // El motor ordena movimiento, pero el codo permanece bloqueado.
    ENCODER,      // El angulo o la velocidad recibidos no son validos.
    RESISTANCE,   // La resistencia de la persona supera el limite.
    INVALID_DATA  // Otro dato de entrada o el paso temporal no es valido.
};

class SafetyMonitor
{
public:
    // Configura los limites articulares y los umbrales de seguridad.
    SafetyMonitor(
        double minAngle,
        double maxAngle,
        double commandThreshold,
        double velocityThreshold,
        double timeThreshold,
        double maxTorque
    );

    // Comprueba una muestra y devuelve el primer fallo detectado.
    FaultCode check(
        double angle,
        double velocity,
        double motorCommand,
        double personTorque,
        bool movementExpected,
        double dt
    );

    // Reinicia el tiempo acumulado de bloqueo.
    void reset();

private:
    double minAngle; // Angulo minimo permitido, en radianes.
    double maxAngle; // Angulo maximo permitido, en radianes.

    double commandThreshold;  // Orden minima para comprobar si existe bloqueo.
    double velocityThreshold; // Velocidad por debajo de la cual se considera inmovil.
    double timeThreshold;     // Tiempo necesario para confirmar un bloqueo.
    double stallTimer;        // Tiempo acumulado cumpliendo la condicion de bloqueo.

    double maxTorque; // Resistencia maxima permitida de la persona, en N*m.
};

#endif

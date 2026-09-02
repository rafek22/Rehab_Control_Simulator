#ifndef THERAPY_CONTROLLER_HPP
#define THERAPY_CONTROLLER_HPP

enum class TherapyState
{
    WAITING, // Esperando el inicio de una sesion.
    MOVE,    // Moviendo el codo hacia el angulo objetivo.
    HOLD,    // Manteniendo el codo en el angulo objetivo.
    RETURN,  // Regresando al angulo inicial.
    FAULT    // Terapia detenida por un fallo de seguridad.
};

class TherapyController
{
public:
    // Configura el recorrido, las repeticiones y las tolerancias de la terapia.
    TherapyController(
        double initialAngle,
        double targetAngle,
        int repetitions,
        double holdDuration,
        double angleTolerance,
        double velocityTolerance,
        double movementSpeed
    );

    // Reinicia los contadores y comienza el movimiento hacia el objetivo.
    void start();

    // Actualiza la maquina de estados usando la posicion y velocidad actuales.
    void update(
        double currentAngle,
        double currentVelocity,
        double dt
    );

    // Devuelve el angulo progresivo que debe seguir el PID, en radianes.
    double getReferenceAngle() const;

    // Devuelve el estado actual sin modificar la terapia.
    TherapyState getState() const;

    // Devuelve el numero de repeticiones terminadas.
    int getCompletedRepetitions() const;

    // Detiene la terapia y activa el estado de fallo.
    void setFault();

private:
    TherapyState state; // Fase actual de la sesion.

    double initialAngle; // Posicion desde la que comienza y a la que regresa el codo.
    double targetAngle;  // Posicion final que debe alcanzar el codo.

    double referenceAngle; // Objetivo progresivo enviado al PID.
    double movementSpeed;  // Velocidad maxima de cambio de la referencia, en rad/s.

    int repetitions;          // Numero total de repeticiones solicitadas.
    int completedRepetitions; // Numero de repeticiones ya terminadas.

    double holdDuration; // Tiempo durante el que debe mantenerse el objetivo, en segundos.
    double holdElapsed;  // Tiempo ya mantenido dentro de las tolerancias.

    double angleTolerance;    // Error angular maximo aceptado, en radianes.
    double velocityTolerance; // Velocidad maxima para considerar estable el codo, en rad/s.
};

#endif

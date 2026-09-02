#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController
{
public:
    // Crea el controlador con las ganancias proporcional, integral y derivativa.
    PIDController(double kp, double ki, double kd);

    // Calcula una orden de motor entre -1 y 1 a partir del objetivo y la medida actual.
    double compute(double target, double current, double dt);

    // Borra la memoria integral y la medida anterior del controlador.
    void reset();

private:
    double kp; // Ganancia proporcional: responde al error actual.
    double ki; // Ganancia integral: responde al error acumulado.
    double kd; // Ganancia derivativa: amortigua cambios rapidos de la medida.

    double integral; // Suma del error acumulado a lo largo del tiempo.

    double previousMeasurement; // Medida del ciclo anterior para calcular la derivada.
    bool initialized;           // Indica si ya existe una medida anterior valida.

    double minOutput; // Limite inferior de la orden del motor.
    double maxOutput; // Limite superior de la orden del motor.
};

#endif

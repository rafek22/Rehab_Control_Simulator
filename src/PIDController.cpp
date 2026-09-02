#include "PIDController.hpp"

#include <cmath>
#include <stdexcept>

PIDController::PIDController(double kp, double ki, double kd)
{
    // Evita crear un controlador con ganancias infinitas o indefinidas.
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd))
    {
        throw std::invalid_argument("PID gains must be finite");
    }

    // Guarda las ganancias recibidas en los atributos del controlador.
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    // El controlador comienza sin error acumulado ni medida anterior.
    integral = 0.0;

    previousMeasurement = 0.0;
    initialized = false;

    // La orden del motor esta normalizada entre -1 y 1.
    minOutput = -1.0;
    maxOutput = 1.0;
}

double PIDController::compute(double target, double current, double dt)
{
    // Comprueba que el calculo puede realizarse de forma segura.
    if (!std::isfinite(target) || 
        !std::isfinite(current) ||
        !std::isfinite(dt) || dt <= 0.0)
    {
        throw std::invalid_argument("PID inputs must be finite and dt must be positive");
    }

    // Diferencia entre la posicion deseada y la posicion medida.
    double error = target - current;

    double derivative = 0.0;

    // Derivada sobre la medida: evita un salto derivativo al cambiar el objetivo.
    if (initialized)
        derivative = -(current - previousMeasurement) / dt;
    else
        initialized = true;

    // Calcula una integral candidata antes de decidir si conviene acumularla.
    double candidateIntegral = integral + error * dt;

    double candidateOutput = kp * error + ki * candidateIntegral + kd * derivative;

    // Detecta si integrar empujaria aun mas la salida fuera de sus limites.
    bool upperSaturation = candidateOutput > maxOutput && error > 0.0;

    bool lowerSaturation = candidateOutput < minOutput && error < 0.0;

    // Anti-windup: solo guarda la integral si no empeora la saturacion.
    if (!upperSaturation && !lowerSaturation)
        integral = candidateIntegral;

    double output = kp * error + ki * integral + kd * derivative;

    // Limita definitivamente la orden enviada al motor.
    if (output > maxOutput)
        output = maxOutput;

    else if (output < minOutput)
        output = minOutput;

    // Conserva la medida para calcular la derivada en el siguiente ciclo.
    previousMeasurement = current;
    
    return output;
}

void PIDController::reset()
{
    // Elimina la memoria del PID cuando cambia la fase del movimiento.
    integral = 0.0;
    previousMeasurement = 0.0;
    initialized = false;
}

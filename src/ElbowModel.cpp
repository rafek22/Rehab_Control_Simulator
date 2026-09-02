#include "ElbowModel.hpp"

#include <cmath>
#include <stdexcept>

ElbowModel::ElbowModel()
{
    // Estado inicial del codo
    angle = 0.523599; // Angulo inicial: 30 grados expresados en radianes
    velocity = 0.0;   // Velocidad angular inicial en rad/s

    // Propiedades fisicas del modelo
    inertia = 0.08;  // Resistencia a cambiar su velocidad angular
    damping = 0.15;  // Frena el movimiento en proporcion a la velocidad

    maxMotorTorque = 5.0; // Par maximo en N*m

    // Propiedades que simulan la resistencia ejercida por la persona
    personStiffness = 1.0;  // Rigidez: fuerza que intenta devolver el codo al angulo de reposo
    personDamping = 0.2;    // Resistencia debida a la velocidad
    restAngle = 0.523599;    // Angulo de reposo de la persona: 30 grados en radianes

    personTorque = 0.0;
}

void ElbowModel::update(double motorCommand, double dt)
{
    if (!std::isfinite(motorCommand) || motorCommand < -1.0 || motorCommand > 1.0)
        throw std::invalid_argument("Motor command must be finite and between -1 and 1");

    if (!std::isfinite(dt) || dt <= 0.0)
        throw std::invalid_argument("Simulation dt must be finite and positive");

    // Convierte la orden normalizada del motor en un par real.
    double motorTorque = motorCommand * maxMotorTorque;

    // Resistencia total de la persona: rigidez por desplazamiento mas amortiguamiento.
    const double resistanceTorque = personStiffness * (angle - restAngle) + personDamping * velocity;

    // Par que queda despues de restar las resistencias al par producido por el motor.
    double netTorque = motorTorque - resistanceTorque - damping * velocity;

    // Segunda ley de Newton para rotacion: aceleracion angular = par / inercia.
    double acceleration = netTorque / inertia;

    // Actualiza la velocidad y el angulo durante el intervalo de tiempo dt.
    velocity += acceleration * dt;
    angle += velocity * dt;

    // Mantiene el par publicado sincronizado con el angulo y la velocidad actuales.
    personTorque = personStiffness * (angle - restAngle) + personDamping * velocity;
}

double ElbowModel::getAngle() const {return angle;}

double ElbowModel::getVelocity() const {return velocity;}

double ElbowModel::getPersonTorque() const {return personTorque;}

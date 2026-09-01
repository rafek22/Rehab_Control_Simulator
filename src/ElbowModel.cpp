#include "ElbowModel.hpp"

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
    person_Stiffness = 1.0;  // Rigidez: fuerza que intenta devolver el codo al angulo de reposo
    person_Damping = 0.2;    // Resistencia debida a la velocidad
    restAngle = 0.523599;    // Angulo de reposo de la persona: 30 grados en radianes
}

void ElbowModel::update(double motor_command, double dt)
{
    // Convierte la orden normalizada del motor en un par real.
    double motorTorque = motor_command * maxMotorTorque;

    // Resistencia total de la persona: rigidez por desplazamiento mas amortiguamiento.
    double person_Resistance = person_Stiffness * (angle - restAngle)+ person_Damping * velocity;

    // Par que queda despues de restar las resistencias al par producido por el motor.
    double netTorque = motorTorque - person_Resistance - damping * velocity;

    // Segunda ley de Newton para rotacion: aceleracion angular = par / inercia.
    double acceleration = netTorque / inertia;

    // Actualiza la velocidad y el angulo durante el intervalo de tiempo dt.
    velocity += acceleration * dt;
    angle += velocity * dt;
}

double ElbowModel::getAngle() const
{
    return angle;
}

double ElbowModel::getVelocity() const
{
    return velocity;
}

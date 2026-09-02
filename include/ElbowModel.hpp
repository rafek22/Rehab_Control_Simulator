#ifndef ELBOW_MODEL_HPP
#define ELBOW_MODEL_HPP

class ElbowModel
{
public:
    // Crea el modelo con sus valores iniciales.
    ElbowModel();

    // Actualiza el movimiento del codo usando la orden del motor y el tiempo transcurrido
    void update(double motor_command, double dt);

    // Devuelve el angulo actual del codo
    double getAngle() const;

    // Devuelve la velocidad angular actual
    double getVelocity() const;

    double getPersonTorque() const;

private:
    double angle;     // Angulo actual del codo, en radianes
    double velocity;  // Velocidad angular actual del codo, en rad/s

    double inertia;   // Resistencia del codo a cambiar su velocidad angular
    double damping;   // Amortiguamiento que frena el movimiento del codo

    double maxMotorTorque; // Par maximo que puede ejercer el motor, en N*m

    double person_Stiffness; // Rigidez de la persona que devuelve el codo al reposo
    double person_Damping;   // Resistencia de la persona debida a la velocidad
    double restAngle;        // Angulo de reposo de la persona, en radianes

    double PersonTorque;

};

#endif

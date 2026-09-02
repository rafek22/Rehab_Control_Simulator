#ifndef THERAPY_CONTROLLER_HPP
#define THERAPY_CONTROLLER_HPP

enum class TherapyState
{
    WAITING,
    MOVE,
    HOLD,
    RETURN,
    FAULT
};

class TherapyController
{
public:
    TherapyController(
        double initialAngle,
        double targetAngle,
        int repetitions,
        double holdDuration,
        double angleTolerance,
        double velocityTolerance,
        double movementSpeed
    );

    void start();

    void update(
        double currentAngle,
        double currentVelocity,
        double dt
    );

    double getReferenceAngle() const;

    TherapyState getState() const;

    int getCompletedRepetitions() const;

    void setFault();

private:
    TherapyState state;

    double initialAngle;
    double targetAngle;

    double referenceAngle;
    double movementSpeed;

    int repetitions;
    int completedRepetitions;

    double holdDuration;
    double holdElapsed;

    double angleTolerance;
    double velocityTolerance;
};

#endif

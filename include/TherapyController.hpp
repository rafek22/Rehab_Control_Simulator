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
        double angleTolerance
    );

    void start();
    void update(double currentAngle, double dt);

    double getTargetAngle() const;
    TherapyState getState() const;

    int getCompletedRepetitions() const;

private:
    TherapyState state;

    double initialAngle;
    double targetAngle;

    int repetitions;
    int completedRepetitions;

    double holdDuration;
    double holdElapsed;

    double angleTolerance;
};

#endif

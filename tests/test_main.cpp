#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "ElbowModel.hpp"
#include "Logger.hpp"
#include "PIDController.hpp"
#include "SafetyMonitor.hpp"
#include "TherapyController.hpp"

namespace
{
int failures = 0;

void expect(bool condition, const std::string& message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        failures++;
    }
}

template <typename Function>
void expectInvalidArgument(Function function, const std::string& message)
{
    try
    {
        function();
        expect(false, message);
    }
    catch (const std::invalid_argument&)
    {
    }
}

void testPid()
{
    PIDController pid(1.0, 0.1, 0.05);
    const double output = pid.compute(10.0, 0.0, 0.01);
    expect(output >= -1.0 && output <= 1.0, "PID output must remain saturated");

    expectInvalidArgument(
        [&pid]() { pid.compute(1.0, 0.0, 0.0); },
        "PID must reject dt equal to zero"
    );
}

void testTherapyStateMachine()
{
    TherapyController therapy(0.0, 1.0, 1, 0.02, 0.01, 0.01, 1.0);
    expect(therapy.getState() == TherapyState::WAITING, "Therapy must start in WAITING");

    therapy.start();
    expect(therapy.getState() == TherapyState::MOVE, "start() must enter MOVE");

    for (int step = 0; step < 100; step++)
    {
        therapy.update(1.0, 0.0, 0.01);
    }

    expect(therapy.getState() == TherapyState::HOLD, "Therapy must enter HOLD at target");
    therapy.update(1.0, 0.0, 0.01);
    therapy.update(1.0, 0.0, 0.01);
    expect(therapy.getState() == TherapyState::RETURN, "Therapy must enter RETURN after hold duration");

    for (int step = 0; step < 100; step++)
    {
        therapy.update(0.0, 0.0, 0.01);
    }

    expect(therapy.getState() == TherapyState::WAITING, "Therapy must finish in WAITING");
    expect(therapy.getCompletedRepetitions() == 1, "Therapy must count one repetition");

    expectInvalidArgument(
        []() { TherapyController invalid(0.0, 1.0, 0, 1.0, 0.1, 0.1, 1.0); },
        "Therapy must reject zero repetitions"
    );
}

void testSafetyMonitor()
{
    SafetyMonitor safety(0.0, 2.0, 0.7, 0.02, 0.5, 3.0);
    expect(safety.check(3.0, 0.0, 0.0, 0.0, false, 0.01) == FaultCode::LIMIT,
        "Safety monitor must detect joint limits");
    expect(safety.check(1.0, 0.0, 0.0, 4.0, false, 0.01) == FaultCode::RESISTANCE,
        "Safety monitor must detect excessive resistance");
    expect(safety.check(1.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, false, 0.01)
            == FaultCode::INVALID_DATA,
        "Safety monitor must detect invalid motor data");
}

void testElbowTorqueSynchronization()
{
    ElbowModel elbow;
    elbow.update(0.2, 0.01);

    constexpr double restAngle = 0.523599;
    const double expectedTorque =
        (elbow.getAngle() - restAngle) + 0.2 * elbow.getVelocity();

    expect(std::abs(elbow.getPersonTorque() - expectedTorque) < 1e-12,
        "Published patient torque must match the current elbow state");
}

void testLogger()
{
    const std::string filename = "logger_test.csv";

    {
        Logger logger(filename);
        logger.log(1.25, "MOVE", 30.0, 29.0, 0.5, 0.2, 1.0);
    }

    std::ifstream file(filename);
    std::string header;
    std::string sample;
    std::string extra;
    std::getline(file, header);
    std::getline(file, sample);
    std::getline(file, extra);

    expect(header == "time,state,target_angle,angle,velocity,motor_command,error",
        "Logger must write the CSV header once");
    expect(sample.rfind("1.250000,MOVE,", 0) == 0,
        "Logger must write sample values");
    expect(extra.empty(), "Logger must not repeat the CSV header");

    std::remove(filename.c_str());
}
}

int main()
{
    testPid();
    testTherapyStateMachine();
    testSafetyMonitor();
    testElbowTorqueSynchronization();
    testLogger();

    if (failures != 0)
    {
        std::cerr << failures << " test(s) failed\n";
        return 1;
    }

    std::cout << "All tests passed\n";
    return 0;
}

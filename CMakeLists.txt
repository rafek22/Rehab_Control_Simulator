cmake_minimum_required(VERSION 3.16)

project(Rehab_Control_Simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Rehab_Control_Simulator
    src/main.cpp
    src/ElbowModel.cpp
    src/PIDController.cpp
    src/TherapyController.cpp
    src/SafetyMonitor.cpp
    src/Logger.cpp
    src/Metrics.cpp
    src/Plotter.cpp
)

target_include_directories(Rehab_Control_Simulator PRIVATE include)

if(MSVC)
    target_compile_options(Rehab_Control_Simulator PRIVATE /W4 /permissive-)
else()
    target_compile_options(Rehab_Control_Simulator PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)

if(BUILD_TESTING)
    add_executable(Rehab_Control_Simulator_Tests
        tests/test_main.cpp
        src/ElbowModel.cpp
        src/PIDController.cpp
        src/TherapyController.cpp
        src/SafetyMonitor.cpp
        src/Logger.cpp
    )

    target_include_directories(Rehab_Control_Simulator_Tests PRIVATE include)

    if(MSVC)
        target_compile_options(Rehab_Control_Simulator_Tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(Rehab_Control_Simulator_Tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME Rehab_Control_Simulator_Tests COMMAND Rehab_Control_Simulator_Tests)
endif()

cmake_minimum_required(VERSION 3.16)

project(Rehab_Control_Simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Rehab_Control_Simulator
    src/main.cpp
    src/ElbowModel.cpp
    src/PIDController.cpp
)

target_include_directories(Rehab_Control_Simulator PRIVATE include)

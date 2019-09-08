cmake_minimum_required (VERSION 3.1)

project(ExtendedKF LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(sources
    src/fusion_ekf.cpp
    src/kalman_filter.cpp
    src/main.cpp
    src/utilities.cpp)

add_executable(ekf ${sources})

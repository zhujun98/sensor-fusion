cmake_minimum_required (VERSION 3.1)

project(ExtendedKF LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED CONFIG)
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${Eigen3_VERSION}, ${Eigen3_INCLUDE_DIRS}")
endif()

set(sources
    src/fusion_ekf.cpp
    src/kalman_filter.cpp
    src/main.cpp
    src/utilities.cpp)

add_executable(ekf ${sources})

target_include_directories(ekf PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ekf PRIVATE Eigen3::Eigen)
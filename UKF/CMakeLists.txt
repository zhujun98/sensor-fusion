cmake_minimum_required (VERSION 3.1)

project(UnscentedKF LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED CONFIG)
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${Eigen3_VERSION}, ${Eigen3_INCLUDE_DIRS}")
endif()

set(sources
    src/ukf.cpp
    src/main.cpp
    src/utilities.cpp)

add_executable(ukf ${sources})

target_include_directories(ukf PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ukf PRIVATE Eigen3::Eigen)
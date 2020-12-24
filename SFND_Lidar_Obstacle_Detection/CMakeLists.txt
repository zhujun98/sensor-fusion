cmake_minimum_required(VERSION 3.1)

project(lidar_obstacle_detection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

OPTION(USE_PCL_SEG "Use RANSAC segmentation algorithm implemented in the PCL library" OFF)
OPTION(USE_PCL_CLUSTER "Use kd-tree clustering algorithm implemented in the PCL library" OFF)

OPTION(BUILD_TEST "BUILD unittest" OFF)

find_package(PCL 1.9.1 REQUIRED)
if(PCL_FOUND)
    message(STATUS "Found PCL: ${PCL_VERSION}")
endif()

add_definitions(${PCL_DEFINITIONS})

if(USE_PCL_SEG)
    add_compile_definitions(USE_PCL_SEG)
endif()

if(USE_PCL_CLUSTER)
    add_compile_definitions(USE_PCL_CLUSTER)
endif()

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

if(BUILD_TEST)
    enable_testing()
    add_subdirectory(tests)
endif()

add_executable(environment
    src/main.cpp
    src/streamer.cpp)

target_include_directories(environment
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS})

target_link_libraries(environment
    PRIVATE
        ${PCL_LIBRARIES})

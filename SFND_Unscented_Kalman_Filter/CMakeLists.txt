cmake_minimum_required(VERSION 3.1)

project(sfnd_ukf_highway LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

OPTION(BUILD_TEST "BUILD unittest" OFF)

find_package(PCL REQUIRED)
if(PCL_FOUND)
    message(STATUS "Found PCL: ${PCL_VERSION}")
endif()

add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

if(BUILD_TEST)
    enable_testing()
    add_subdirectory(tests)
endif()

add_executable(ukf_highway
    src/highway.cpp
    src/car.cpp
    src/meter.cpp
    src/ukf.cpp
    src/main.cpp)

target_include_directories(ukf_highway
    PRIVATE
        ${PCL_INCLUDE_DIRS}
        include)

target_link_libraries(ukf_highway
    PRIVATE
        ${PCL_LIBRARIES})

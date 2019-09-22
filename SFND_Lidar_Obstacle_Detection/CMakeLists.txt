cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_FLAGS "-Wall")

project(lidar_obstacle_detection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.9.1 REQUIRED)
if(PCL_FOUND)
    message(STATUS "Found PCL: ${PCL_VERSION}")
endif()

add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(environment
    src/main.cpp
    src/render.cpp
    )

target_include_directories(environment
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(environment
    PRIVATE
        ${PCL_LIBRARIES}
    )

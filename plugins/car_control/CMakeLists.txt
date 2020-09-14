cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(car_control_plugin)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(car_control_plugin SHARED car_control.cc)
target_link_libraries(car_control_plugin ${GAZEBO_LIBRARIES})

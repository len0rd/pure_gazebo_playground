cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gimbal_motor_plugin SHARED gimbal_motor_plugin.cpp)
target_link_libraries(gimbal_motor_plugin ${GAZEBO_LIBRARIES})

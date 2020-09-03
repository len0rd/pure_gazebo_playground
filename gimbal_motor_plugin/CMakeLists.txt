cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gimbal_motor_plugin SHARED gimbal_motor_plugin.cpp)
target_link_libraries(gimbal_motor_plugin ${GAZEBO_LIBRARIES} gimbal_motor_plugin_msgs)

add_executable(info_listener gimbal_motor_info_listener.cpp)
target_link_libraries(info_listener gimbal_motor_plugin_msgs ${GAZEBO_LIBRARIES})

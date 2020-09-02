find_package(Protobuf REQUIRED)

# use this if your message relies on a pre-defined gazebo message
# this finds the directory of (usually /usr/include/gazebo-<YOUR_GAZEBO_VERSION>/gazebo/msgs/proto)
set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(msgs
    gimbal_motor_info.proto
    ${PROTOBUF_IMPORT_DIRS}/time.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gimbal_motor_plugin_msgs SHARED ${PROTO_SRCS})
target_link_libraries(gimbal_motor_plugin_msgs ${PROTOBUF_LIBRARY})

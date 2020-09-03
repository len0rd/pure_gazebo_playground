#include <gimbal_motor_info.pb.h>

#include <boost/shared_ptr.hpp>

#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

typedef const boost::shared_ptr<
  const gimbal_motor_plugin::msgs::GimbalMotorInfo>
    ConstGimbalMotorInfoPtr;

void newData(ConstGimbalMotorInfoPtr &msg) {
    std::cout << msg->DebugString();
}

int main(int argc, char **argv) {
      // Load gazebo
  gazebo::client::setup(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo imu topic
  // I found this topic name using gazebo's "Topic Visualization" window: Window -> Topic Visualization
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/gimbal_joints/info", newData);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}

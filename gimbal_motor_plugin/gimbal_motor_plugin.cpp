#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/common.hh>

// custom messages
#include <gimbal_motor_info.pb.h>
#include <gimbal_motor_cmd.pb.h>

#include <iostream>

// boost
#include <boost/bind.hpp>

namespace gazebo {

typedef const boost::shared_ptr<
  const gimbal_motor_plugin::msgs::GimbalMotorCmd>
    ConstGimbalMotorCmdPtr;

class GimbalMotorPlugin : public ModelPlugin {
  private:
    // how often to publish the joint info topic in seconds
    static constexpr float JOINT_INFO_PUB_RATE_S = 0.01;
    common::Time m_lastTopicPubTime;
    physics::ModelPtr m_model;
    gazebo::transport::PublisherPtr m_pub;
    event::ConnectionPtr m_updateConnection;

    // for subscribing to the cmd topic
    transport::NodePtr m_node;
    transport::SubscriberPtr m_gimbalMotorCmdSub;
  public:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

        if (model->GetJointCount() < 2) {
            std::cerr << "Invalid joint count, plugin not loaded\n";
            return;
        }
        this->m_model = model;
        m_node = transport::NodePtr(new transport::Node());
        m_node->Init();
        m_pub = m_node->Advertise<gimbal_motor_plugin::msgs::GimbalMotorInfo>("~/gimbal_joints/info");
        m_gimbalMotorCmdSub = m_node->Subscribe("~/gimbal_joints/cmd", &GimbalMotorPlugin::handleCmd, this);
        m_lastTopicPubTime = m_model->GetWorld()->SimTime();
        m_updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GimbalMotorPlugin::onUpdate, this));
    }

    void onUpdate() {
        common::Time currentTime = m_model->GetWorld()->SimTime();
        double secSinceLastPub = (currentTime - m_lastTopicPubTime).Double();
        if (secSinceLastPub > JOINT_INFO_PUB_RATE_S) {
            publishJointInfo(currentTime);
            m_lastTopicPubTime += common::Time(JOINT_INFO_PUB_RATE_S);
        }
    }

    void handleCmd(ConstGimbalMotorCmdPtr &cmd) {
        std::cout << "Received message" << std::endl;
        // TODO: handle cmd
    }

    void publishJointInfo(common::Time &currentTime) {
        for (size_t ii = 0; ii < m_model->GetJointCount(); ii++) {
            physics::JointPtr gimJoint = m_model->GetJoints()[ii];
            if (gimJoint != nullptr) {
                gimbal_motor_plugin::msgs::GimbalMotorInfo info;
                // set type. TODO: better way todo this
                std::string jName = gimJoint->GetName();

                if (jName.find("pitch_joint") != std::string::npos) {
                    info.set_jointtype(gimbal_motor_plugin::msgs::GimbalMotorInfo_GimbalJointType_PITCH);
                }
                else if (jName.find("yaw_joint") != std::string::npos) {
                    info.set_jointtype(gimbal_motor_plugin::msgs::GimbalMotorInfo_GimbalJointType_YAW);
                }
                else if (jName.find("roll_joint") != std::string::npos) {
                    info.set_jointtype(gimbal_motor_plugin::msgs::GimbalMotorInfo_GimbalJointType_ROLL);
                }
                else {
                    // doesnt appear to be a gimbal joint
                    continue;
                }
                info.set_jointname(gimJoint->GetName());
                info.set_pose_rad(gimJoint->Position());
                msgs::Time* t = info.mutable_time();
                t->set_sec(currentTime.sec);
                t->set_nsec(currentTime.nsec);
                m_pub->Publish(info);
            }
        }
    }
};


GZ_REGISTER_MODEL_PLUGIN(GimbalMotorPlugin)
} // namespace gazebo

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/common.hh>
#include <gimbal_motor_info.pb.h>

// boost
#include <boost/bind.hpp>

namespace gazebo {

class GimbalMotorPlugin : public ModelPlugin {
  private:
    common::Time m_lastUpdateTime;
    physics::ModelPtr m_model;
    gazebo::transport::PublisherPtr m_pub;
    event::ConnectionPtr m_updateConnection;
  public:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

        if (model->GetJointCount() < 2) {
            std::cerr << "Invalid joint count, plugin not loaded\n";
            return;
        }
        this->m_model = model;
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();
        m_pub = node->Advertise<gimbal_motor_plugin::msgs::GimbalMotorInfo>("~/gimbal_joints/test_pos");
        m_lastUpdateTime = m_model->GetWorld()->SimTime();
        m_updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GimbalMotorPlugin::onUpdate, this));
    }

    void onUpdate() {
        common::Time current_time = m_model->GetWorld()->SimTime();
        double seconds_since_last_update = ( current_time - m_lastUpdateTime ).Double();
        if (seconds_since_last_update > 0.25) {
            // std::string pyJointName = m_model->GetName() + "::" + "pitch_yaw_joint";
            // physics::JointPtr pyJoint = m_model->GetJoint(pyJointName);
            physics::JointPtr pyJoint = m_model->GetJoints()[0];// m_model->GetJoint("pitch_yaw_joint");
            if (pyJoint != nullptr) {
                gimbal_motor_plugin::msgs::GimbalMotorInfo info;
                msgs::Time* t = info.mutable_time();
                info.set_jointname(pyJoint->GetName());
                info.set_pose_rad(pyJoint->Position());
                t->set_sec(current_time.sec);
                t->set_nsec(current_time.nsec);
                m_pub->Publish(info);
            }

            double yPayPos = 0.0;
            // std::string yPayJointName = m_model->GetName() + "::" + "yaw_payload_joint";
            // physics::JointPtr yPayJoint = m_model->GetJoint(yPayJointName);
            physics::JointPtr yPayJoint = m_model->GetJoints()[1]; // m_model->GetJoint("yaw_payload_joint");
            if (yPayJoint != nullptr) {
                gimbal_motor_plugin::msgs::GimbalMotorInfo info;
                msgs::Time* t = info.mutable_time();
                info.set_jointname(yPayJoint->GetName());
                info.set_pose_rad(yPayJoint->Position());
                t->set_sec(current_time.sec);
                t->set_nsec(current_time.nsec);
                m_pub->Publish(info);
            }
            m_lastUpdateTime += common::Time(0.25);
        }
    }
};


GZ_REGISTER_MODEL_PLUGIN(GimbalMotorPlugin)
} // namespace gazebo

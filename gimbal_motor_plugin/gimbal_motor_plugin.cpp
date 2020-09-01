#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/common.hh>

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
        this->m_model = model;
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();
        m_pub = node->Advertise<gazebo::msgs::Vector3d>("~/gimbal_joints/test_pos");
        m_lastUpdateTime = m_model->GetWorld()->SimTime();
        m_updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GimbalMotorPlugin::onUpdate, this));
    }

    void onUpdate() {
        common::Time current_time = m_model->GetWorld()->SimTime();
        double seconds_since_last_update = ( current_time - m_lastUpdateTime ).Double();
        if (seconds_since_last_update > 1.0) {
            msgs::Vector3d msg;
            double pyPos = 0.0;
            physics::JointPtr pyJoint = m_model->GetJoint("pitch_yaw_joint");
            if (pyJoint != nullptr) {
                pyPos = pyJoint->Position();
            }

            double yPayPos = 0.0;
            physics::JointPtr yPayJoint = m_model->GetJoint("yaw_payload_joint");
            if (yPayJoint != nullptr) {
                yPayPos = yPayJoint->Position();
            }
            msgs::Set(&msg, ignition::math::Vector3d(pyPos, yPayPos, 0));
            m_pub->Publish(msg);
            m_lastUpdateTime += common::Time(1.0);
        }
    }
};


GZ_REGISTER_MODEL_PLUGIN(GimbalMotorPlugin)
} // namespace gazebo

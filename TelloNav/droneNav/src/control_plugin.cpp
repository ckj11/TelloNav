#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>

namespace gazebo {

    class TelloControl : public ModelPlugin {
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                this->model = _parent;

                this->update_connection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&TelloControl::update, this));

                this->currentPosition = this->model->WorldPose();

                this->currentSpeed = ignition::math::Vector3d(0, 0, 0);

                this->targetPosition = ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
            }

            void update() {
                
            }

        private:
            physics::ModelPtr model;
            event::ConnectionPtr update_connection;

            ignition::math::Pose3d targetPosition;
            ignition::math::Pose3d currentPosition;
            ignition::math::Vector3d currentSpeed;

            




    };

    GZ_REGISTER_MODEL_PLUGIN(TelloControl);
}
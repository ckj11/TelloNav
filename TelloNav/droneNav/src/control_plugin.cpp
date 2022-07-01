#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>


namespace gazebo {

    class TelloControl : public ModelPlugin {
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                this->model = _parent;

                this->update_connection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&TelloControl::update, this));


                this->currentSpeed = ignition::math::Vector3d(0, 0, 0);

                this->targetPosition = ignition::math::Pose3d(5, 5, 5, 0, 0, 0);

                this->pub_pos = n.advertise<std_msgs::String>("location", 10);
            }

            void update() {
                this->currentPosition = this->model->WorldPose();
                ss << currentPosition.Pos().X() << " " << currentPosition.Pos().Y() << " " << currentPosition.Pos().Z();
                msg.data = ss.str();
                pub_pos.publish(msg);
                ss.str("");
                ss.clear();

            if(this->targetPosition.Pos().X() > this->model->WorldPose().Pos().X()) {
                this->currentSpeed.X(0.3);
            }
            else {
                this->currentSpeed.X(0);
            }

            if(this->targetPosition.Pos().Y() > this->model->WorldPose().Pos().Y()) {
                this->currentSpeed.Y(0.3);
            }
            else {
                this->currentSpeed.Y(0);
            }
            
            if(this->targetPosition.Pos().Z() > this->model->WorldPose().Pos().Z()) {
                this->currentSpeed.Z(0.3);
            }
            else {
                this->currentSpeed.Z(0);
            }
            
            this->model->SetLinearVel(this->currentSpeed);
            }

        private:
            physics::ModelPtr model;
            event::ConnectionPtr update_connection;

            ignition::math::Pose3d targetPosition;
            ignition::math::Pose3d currentPosition;
            ignition::math::Vector3d currentSpeed;

            ros::Publisher pub_pos;
            ros::NodeHandle n;
            std_msgs::String msg;

            std::stringstream ss;





    };

    GZ_REGISTER_MODEL_PLUGIN(TelloControl);
}
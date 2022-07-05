#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <string>


namespace gazebo {

    class TelloControl : public ModelPlugin {
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                if (!ros::isInitialized()) {
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, "Tello",
                    ros::init_options::NoSigintHandler);
                }
                
                this->model = _parent;

                this->body = this->model->GetLink((std::string) "Tello::link_0_clone");

                this->rosNode.reset(new ros::NodeHandle("/Gazebo_Tello"));

                //Set subscriber options for sub topic
                ros::SubscribeOptions so = 
                ros::SubscribeOptions::create<geometry_msgs::Twist>(
                "/" + this->model->GetName() + "/location",
                1,
                boost::bind(&TelloControl::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);

                //Subscribe to topic using above options
                this->sub_pos = this->rosNode->subscribe(so);

                //Set up queue helper thread
                this->rosQueueThread = std::thread(std::bind(&TelloControl::QueueThread, this));

                this->update_connection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&TelloControl::update, this));

                this->targetPosition = ignition::math::Pose3d(1, 1, 1, 0, 0, 0);

                this->currentSpeed = 0.5;

                this->pub_pos = rosNode->advertise<std_msgs::String>("location", 10);

                
            }

            void update() {
                this->currentPosition = this->model->WorldPose();
                ss << currentPosition.Pos().X() << " " << currentPosition.Pos().Y() << " " << currentPosition.Pos().Z();
                msg.data = ss.str();
                pub_pos.publish(msg);
                ss.str("");
                ss.clear();

                //Calculate heading to next point
                ignition::math::Vector3d heading = (targetPosition - currentPosition).Pos();

                //Get unit vector
                heading.Normalize();

                //Multiply speed to get velocity vector to the target
                heading *= currentSpeed;
                
                //give model that set speed
                this->model->SetLinearVel(heading);



            // if(this->targetPosition.Pos().X() > this->model->WorldPose().Pos().X()) {
            //     this->currentSpeed.X(1);
            // }
            // else {
            //     this->currentSpeed.X(0);
            // }

            // if(this->targetPosition.Pos().Y() > this->model->WorldPose().Pos().Y()) {
            //     this->currentSpeed.Y(1);
            // }
            // else {
            //     this->currentSpeed.Y(0);
            // }
            
            // if(this->targetPosition.Pos().Z() > this->model->WorldPose().Pos().Z()) {
            //     this->currentSpeed.Z(1);
            // }
            // else {
            //     this->currentSpeed.Z(0);
            // }
            
            // this->model->SetLinearVel(this->currentSpeed);
            }

        void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg) {
             targetPosition.Pos().X(_msg->linear.x);
             targetPosition.Pos().Y(_msg->linear.y);
             targetPosition.Pos().Z(_msg->linear.z);
            
        }

        void QueueThread() {
            static const double timeout = 0.01;
            while(this->rosNode->ok()) {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        private:
            //Ptr for model object
            physics::ModelPtr model;

            //Link ptr for main body
            physics::LinkPtr body;
            
            event::ConnectionPtr update_connection;

            ignition::math::Pose3d targetPosition;
            ignition::math::Pose3d currentPosition;
            double currentSpeed;

            //Publisher object
            ros::Publisher pub_pos;

            //Subscriber object
            ros::Subscriber sub_pos;
            
            //Node handle for publishing and subscribing
            std::unique_ptr<ros::NodeHandle> rosNode;

            //Callback queue for messages
            ros::CallbackQueue rosQueue;

            //Thread for callback queue
            std::thread rosQueueThread;

            std_msgs::String msg;

            std::stringstream ss;





    };

    GZ_REGISTER_MODEL_PLUGIN(TelloControl);
}
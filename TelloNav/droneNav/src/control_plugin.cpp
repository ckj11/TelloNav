#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "rosgraph_msgs/Clock.h"

#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include "../include/droneNav/pid.hpp"


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
                if (_sdf->HasElement("center_of_mass")) {
                    centerOfMass = _sdf->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
                }

                this->model = _parent;
                this->gravity = this->model->GetWorld()->Gravity();

                //Simulation tuning for gravity cancelation
                //This happens because this update functions runs faster than the simulation so the gravity vector is applied twice
                double factor = 0.5004;
                gravity *= factor;
                
                std::cout << "gravity vector: " << gravity.X() << " " << gravity.Y() << " " << gravity.Z() << std::endl;
                //Fetch prop links
                this->body = this->model->GetLink((std::string)"Tello::body");
                this->prop1 = this->model->GetLink((std::string)"Tello::prop_1");
                this->prop2 = this->model->GetLink((std::string)"Tello::prop_2");
                this->prop3 = this->model->GetLink((std::string)"Tello::prop_3");
                this->prop4 = this->model->GetLink((std::string)"Tello::prop_4");

                this->rosNode.reset(new ros::NodeHandle("/Tello"));

                //Set subscriber options for sub topic
                ros::SubscribeOptions so = 
                ros::SubscribeOptions::create<geometry_msgs::Twist>(
                "/" + this->model->GetName() + "/target",
                10,
                boost::bind(&TelloControl::OnRosTargetMsg, this, _1),
                ros::VoidPtr(), &this->rosTargetQueue);

                //Subscribe to topic using above options
                this->sub_target = this->rosNode->subscribe(so);

                //Set up target queue helper thread
                this->rosTargetQueueThread = std::thread(std::bind(&TelloControl::QueueTargetThread, this));
                
                //Hook into gazebo world update event
                this->update_connection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&TelloControl::update, this));

                this->targetPosition = ignition::math::Pose3d(0, 0, 1, 0, 0, 0);

                set_target_position(0, 0, 1, 0);

                this->rotationSpeed = 1;

                this->pub_pos = rosNode->advertise<geometry_msgs::Twist>("location", 10);
            }

            void update() {
                this->currentPosition = this->body->WorldPose();
                geometry_msgs::Twist msg;
                msg.linear.x = currentPosition.Pos().X();
                msg.linear.y = currentPosition.Pos().Y();
                msg.linear.z = currentPosition.Pos().Z();

                msg.angular.x = currentPosition.Rot().X();
                msg.angular.y = currentPosition.Rot().Y();
                msg.angular.z = currentPosition.Rot().Z();

                pub_pos.publish(msg);

                //Calculate the dt
                double dt = (ros::Time::now() - prevTime).toSec();
                if(dt == 0) {
                    dt = 0.001;
                }
                //double dt = 0.001;
                //Get velocities
                ignition::math::Vector3d linear_velocity = body->RelativeLinearVel();
                ignition::math::Vector3d angular_velocity = body->RelativeAngularVel();
                
                //Calc desired accelerations(ubar)
                ignition::math::Vector3d lin_ubar, ang_ubar;
                lin_ubar.X(xController.calc(this->currentPosition.Pos().X(), dt, 0));
                lin_ubar.Y(yController.calc(this->currentPosition.Pos().Y(), dt, 0));
                lin_ubar.Z(zController.calc(this->currentPosition.Pos().Z(), dt, 0));

                // Compensate for gravity
                lin_ubar -= gravity;
                
                //Calc force and torque
                ignition::math::Vector3d force = lin_ubar * body->GetInertial()->Mass();
                ignition::math::Vector3d torque = ang_ubar * body->GetInertial()->MOI();

                //Set roll and pitch to zero
                ignition::math::Pose3d pose = body->WorldPose();
                pose.Rot().X(0);
                pose.Rot().Y(0);
                body->SetWorldPose(pose);

                body->AddForce(force);
                body->AddRelativeTorque(torque);

                //Calculate vector for rotation
                ignition::math::Vector3d rotation = (targetPosition - currentPosition).Rot().Euler();

                rotation.Normalize();

                //Multiply speed to get velocity vector to the target
                rotation *= rotationSpeed;
                
                //give model that set speed
                this->model->SetAngularVel(rotation);

                //give props angular velocity to spin
                SpinUpProps();

                //Update time for dt
                prevTime = ros::Time::now();
            }

        void OnRosTargetMsg(const geometry_msgs::TwistConstPtr &_msg) {
            ignition::math::Vector3d pos(_msg->linear.x, _msg->linear.y, _msg->linear.z);
            ignition::math::Vector3d rot(_msg->angular.x, _msg->angular.y, _msg->angular.z);
            targetPosition.Set(pos,rot);

            set_target_position(_msg->linear.x, _msg->linear.y, _msg->linear.z, _msg->angular.z);
            
            while(NotClose(&currentPosition, &targetPosition)) {
                
            }
        }

        void QueueTargetThread() {
            static const double timeout = 5;
            while(this->rosNode->ok()) {
                this->rosTargetQueue.callOne();
            }
        }

        void SpinUpProps() {
            ignition::math::Vector3d tor(0, 0, 8);
            this->prop1->SetAngularVel(tor);
            this->prop2->SetAngularVel(tor);
            this->prop3->SetAngularVel(tor);
            this->prop4->SetAngularVel(tor);
        }

        void StopProps() {
            ignition::math::Vector3d tor(0, 0, 0);
            this->prop1->SetTorque(tor);
            this->prop2->SetTorque(tor);
            this->prop3->SetTorque(tor);
            this->prop4->SetTorque(tor);
        }

         void set_target_position(double x, double y, double z, double yaw)
        {
            xController.set_target(x);
            yController.set_target(y);
            zController.set_target(z);
        }

        inline double clamp(const double v, const double max)
        {
            return v > max ? max : (v < -max ? -max : v);
        }

        bool NotClose(ignition::math::Pose3d *p1, ignition::math::Pose3d *p2) {
            if(std::abs(p1->Pos().X() - p2->Pos().X()) < 0.05) {
                if(std::abs(p1->Pos().Y() - p2->Pos().Y()) < 0.05) {
                    if(std::abs(p1->Pos().Z() - p2->Pos().Z()) < 0.05) {
                        return false;
                    }
                }
            }
            return true;
        }

        private:
            //Ptr for model object
            physics::ModelPtr model;

            //Link ptr for main body
            physics::LinkPtr body;
            physics::LinkPtr prop1;
            physics::LinkPtr prop2;
            physics::LinkPtr prop3;
            physics::LinkPtr prop4;
            
            event::ConnectionPtr update_connection;

            ignition::math::Pose3d targetPosition;
            ignition::math::Pose3d currentPosition;
            ignition::math::Vector3d gravity;
            ignition::math::Vector3d centerOfMass{0, 0, 0};
            float rotationSpeed;
            float currentClock;

            //Publisher object
            ros::Publisher pub_pos;

            //Subscriber objects
            ros::Subscriber sub_target;
            
            //Node handle for publishing and subscribing
            std::unique_ptr<ros::NodeHandle> rosNode;

            //Callback queues for messages
            ros::CallbackQueue rosTargetQueue;

            //Threads for callback queue
            std::thread rosTargetQueueThread;

            std_msgs::String msg;

            std::stringstream ss;
            
            //PID controllers for axis for Tello
            pid::Controller xController{false, 0.1, 0, 0.5};
            pid::Controller yController{false, 0.1, 0, 0.5};
            pid::Controller zController{false, 0.1, 0.02, 1};

            ros::Time prevTime;

            const double MAX_XY_V = 8.0;
            const double MAX_Z_V = 4.0;
            const double MAX_ANG_V = M_PI;

            const double MAX_XY_A = 8.0;
            const double MAX_Z_A = 4.0;
            const double MAX_ANG_A = M_PI;





    };

    GZ_REGISTER_MODEL_PLUGIN(TelloControl);
}
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

#include <tf/transform_broadcaster.h>


namespace gazebo {

    class LidarWorldScanner : public ModelPlugin {
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {           
                this->model = _parent;

                //Hook into gazebo world update event
                this->update_connection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&LidarWorldScanner::update, this));

                this->rosNode.reset(new ros::NodeHandle("/lidar_world_scanner"));

                //Make subscribe options for custom subscriber
                ros::SubscribeOptions so = 
                ros::SubscribeOptions::create<geometry_msgs::Twist>(
                "/" + this->model->GetName() + "/target",
                10,
                boost::bind(&LidarWorldScanner::OnRosTargetMsg, this, _1),
                ros::VoidPtr(), &this->rosTargetQueue);

                //Construct custom subscriber
                this->sub_target = this->rosNode->subscribe(so);

                //Bind callback queue thread to the callback function
                this->rosTargetQueueThread = std::thread(std::bind(&LidarWorldScanner::QueueTargetThread, this));

                this->oscillation = 2.5;
                this->speed = 1.0;
                this->targetPosition = ignition::math::Pose3d(0, 0, 1, 0, 0, 0);
            }

            void update() {
                tf::Transform transform;

                ignition::math::Pose3d currentPosition = this->model->WorldPose();

                transform.setOrigin(tf::Vector3(currentPosition.Pos().X(), currentPosition.Pos().Y(), 
                currentPosition.Pos().Z()));
                transform.setRotation(tf::Quaternion(currentPosition.Rot().X(), currentPosition.Rot().Y(), 
                currentPosition.Rot().Z(), currentPosition.Rot().W()));

                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

                //Calculate linear speed heading for the next iteration and apply it
                ignition::math::Vector3d heading = (targetPosition - currentPosition).Pos();
                heading.Normalize();
                heading *= speed;
                this->model->SetLinearVel(heading);
            }

            void OnRosTargetMsg(const geometry_msgs::TwistConstPtr &_msg) {
                //ignore z only care about x and y
                this->targetPosition.Pos().X(_msg->linear.x);
                this->targetPosition.Pos().Y(_msg->linear.y);
                this->targetPosition.Pos().Z(_msg->linear.z);
            }

            void QueueTargetThread() {
                static const double timeout = 5;
                while(this->rosNode->ok()) {
                    this->rosTargetQueue.callOne();
                }
            }

        private:
            //Gazebo stuff
            physics::ModelPtr model;
            event::ConnectionPtr update_connection;
            ignition::math::Pose3d targetPosition;
            double oscillation;
            double speed;

            //Ros stuff
            std::unique_ptr<ros::NodeHandle> rosNode;
            ros::Publisher pos_pub;
            ros::Subscriber sub_target;

            //Ros callback queues
            ros::CallbackQueue rosTargetQueue;

            //Threads for ROS
            std::thread rosTargetQueueThread;

            //TF stuff
            tf::TransformBroadcaster br;

    };

    GZ_REGISTER_MODEL_PLUGIN(LidarWorldScanner);
}
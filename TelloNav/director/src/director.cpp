#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "../include/director/aStar.hpp"
#include <iostream>
#include "../include/director/node.hpp"
int main(int argc, char** argv) {
    
    // ros::init(argc, argv, "director");

    // ros::NodeHandle n;
    // ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/Tello/target", 1000);

    // //set up waypoint msgs to send to tello
    // geometry_msgs::Twist msgs[7];
    // int size = 7;
    // int count = 0;
    // msgs[0].linear.x = 0;
    // msgs[0].linear.y = 0;
    // msgs[0].linear.z = 3;
    // msgs[0].angular.x = 0;
    // msgs[0].angular.y = 0;
    // msgs[0].angular.z = 0;

    // msgs[1].linear.x = 0;
    // msgs[1].linear.y = 0;
    // msgs[1].linear.z = 3;
    // msgs[1].angular.x = 0;
    // msgs[1].angular.y = 0;
    // msgs[1].angular.z = 0;

    // msgs[2].linear.x = 0;
    // msgs[2].linear.y = 0;
    // msgs[2].linear.z = 3;
    // msgs[2].angular.x = 0;
    // msgs[2].angular.y = 0;
    // msgs[2].angular.z = 0;

    // msgs[3].linear.x = 3;
    // msgs[3].linear.y = 0;
    // msgs[3].linear.z = 3;
    // msgs[3].angular.x = 0;
    // msgs[3].angular.y = 0;
    // msgs[3].angular.z = 1.57;

    // msgs[4].linear.x = 3;
    // msgs[4].linear.y = 5;
    // msgs[4].linear.z = 3;
    // msgs[4].angular.x = 0;
    // msgs[4].angular.y = 0;
    // msgs[4].angular.z = 3.14;

    // msgs[5].linear.x = 0;
    // msgs[5].linear.y = 5;
    // msgs[5].linear.z = 3;
    // msgs[5].angular.x = 0;
    // msgs[5].angular.y = 0;
    // msgs[5].angular.z = -1.57;

    // msgs[6].linear.x = 0;
    // msgs[6].linear.y = 0;
    // msgs[6].linear.z = 3;
    // msgs[6].angular.x = 0;
    // msgs[6].angular.y = 0;
    // msgs[6].angular.z = 0;

    // ros::Rate loop_rate(0.5);

    // while(ros::ok()) {
    //     if(count < size) {
    //         pub.publish(msgs[count]);
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     count++;
    // }

    
    return 0;
}
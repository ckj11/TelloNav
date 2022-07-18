#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "../include/director/aStar.hpp"
#include <iostream>

int main(int argc, char** argv) {
    
    //ros::init(argc, argv, "director");

    //ros::NodeHandle n;
    //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/Tello/target", 1000);

    //set up waypoint msgs to send to tello
   
    //ros::Rate loop_rate(0.5);
    int count = 0;
    int size = 5;

    while(ros::ok()) {
        if(count < size) {
            //pub.publish(msgs[count]);
        }
        //ros::spinOnce();
        //loop_rate.sleep();
        count++;
    }

    
    return 0;
}
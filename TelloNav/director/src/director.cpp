#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "../include/director/aStar.hpp"
#include <iostream>
#include "../include/director/grid.hpp"

typedef std::tuple<int, int, int> Point;

int main(int argc, char** argv) {
    
    //ros::init(argc, argv, "director");

    //ros::NodeHandle n;
    //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/Tello/target", 1000);

    //set up waypoint msgs to send to tello
   
    //ros::Rate loop_rate(0.5);
    int count = 0;
    int size = 5;

    const int xSize = 11;
    const int ySize = 11;
    const int zSize = 11;

    // Cell*** grid;
    // grid = new Cell**[zSize];
    // for(int i = 0; i < zSize; ++i) {
    //     grid[i] = Cell*[ySize];
    // }
    // for(int i = 0; i < zSize; ++i) {
    //     for(int j = 0; j < ySize; ++j) {
    //         grid[i][j] = Cell[xSize];
    //     }
    // }
    
    //Gross and weird array definition, but should work none the less
    std::array<std::array<std::array<Cell, xSize>, ySize>, zSize> array;
    Grid<xSize, ySize, zSize> grid(array);

    AStar<xSize, ySize, zSize> aStar(Point(0, 0, 0), Point(-5, -5, -5), grid);
    aStar.aStarSearch();

    // while(ros::ok()) {
    //     if(count < size) {
    //         //pub.publish(msgs[count]);
    //     }
    //     //ros::spinOnce();
    //     //loop_rate.sleep();
    //     count++;
    // }
    
    
    return 0;
}
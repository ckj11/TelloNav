#pragma once
#include "cell.hpp"
#include <queue>
#include <stack>
#include <cmath>
#include "grid.hpp"
#include <iostream>

typedef std::tuple<int, int, int> Point;

template<size_t xSize, size_t ySize, size_t zSize>
class AStar {
    private:
        Grid<xSize, ySize, zSize>* grid;
        Point src;
        Point dest;
        
    public:

        //template<size_t ROW, size_t COL, size_t DEP>
        AStar(Point src, Point dest, Grid<xSize, ySize, zSize> &grid) {
            this->grid = &grid;
            this->src = src;
            this->dest = dest;
        }

        //Checks to see if the node is blocked or not
        //template<size_t ROW, size_t COL, size_t DEP>
        bool isUnBlocked(Point p) {
            if(grid->at(std::get<0>(p), std::get<1>(p), std::get<2>(p))->blocked) {
                return false;
            }
            else {
                return true;
            }
        }

        //Checks to see if the current node is the destination node or not
        //template<size_t ROW, size_t COL, size_t DEP>
        bool isDestination(Point p) {
            if(p == dest) {
                return true;
            }
            else {
                return false;
            }
        }

        //template<size_t ROW, size_t COL, size_t DEP>
        void tracePath() {
            std::stack<Point> path;
            int row;
            int col;
            int depth;

        }

        //Calculate the H value between current node and the destination node
        //template<size_t ROW, size_t COL, size_t DEP>
        double calculateHValue(Point p) {
            return std::sqrt(std::pow((std::get<0>(dest) - std::get<0>(p)) , 2) +
                             std::pow((std::get<1>(dest) - std::get<1>(p)) , 2) +
                             std::pow((std::get<2>(dest) - std::get<2>(p)) , 2));
        }
        //template<size_t ROW, size_t COL, size_t DEP>
        void aStarSearch() {
            //Preliminary checks to make sure points are correctly set up
            if(!grid->isValid(std::get<0>(src), std::get<1>(src), std::get<2>(src))) {
                std::cout << "src point is out of bounds!" << std::endl;
                return;
            }
            if(!grid->isValid(std::get<0>(dest), std::get<1>(dest), std::get<2>(dest))) {
                std::cout << "dest point is out of bounds!" << std::endl;
                return;
            }
            if(!isUnBlocked(src)) {
                std::cout << "src point is blocked and cannot be started at" << std::endl;
                return;
            }
            if(!isUnBlocked(dest)) {
                std::cout << "dest point is blocked and cannot be reached" << std::endl;
                return;
            }

            //Initialize the closed list as a 3D array for easy O(1) access
            bool closedList[zSize][ySize][xSize];
            memset(closedList, false, sizeof(closedList));

            int i, j, k;

            i = std::get<0>(src);
            j = std::get<1>(src);
            k = std::get<2>(src);
        }
};
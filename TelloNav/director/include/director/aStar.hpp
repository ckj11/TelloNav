#pragma once
#include "cell.hpp"
#include <queue>
#include <stack>
#include <cmath>

typedef std::tuple<int, int, int> Point;

class AStar {
    private:
        Cell*** grid;
        Point src;
        Point dest;
        int rSize;
        int cSize;
        int dSize;
    public:

        AStar(Point src, Point dest, Cell*** grid, int rSize, int cSize, int dSize) {
            this->grid = grid;
            this->src = src;
            this->dest = dest;
        }

        //Checks to make sure that the point is a valid point
        bool isValid(Point p) {
            if(std::get<0>(p) >= 0 && std::get<1>(p) >= 0 && std::get<0>(p) >= 0 &&
               std::get<0>(p) < dSize && std::get<1>(p) < cSize && std::get<0>(p) < rSize) {
                return true;
               }
        }

        //Checks to see if the node is blocked or not
        bool isUnBlocked(Point p) {
            if(grid[std::get<2>(p)][std::get<1>(p)][std::get<0>(p)].blocked) {
                return false;
            }
            else {
                return true;
            }
        }

        //Checks to see if the current node is the destination node or not
        bool isDestination(Point p) {
            if(p == dest) {
                return true;
            }
            else {
                return false;
            }
        }

        //Calculate the H value between current node and the destination node
        double calculateHValue(Point p) {
            return std::sqrt(std::pow((std::get<0>(dest) - std::get<0>(p)) , 2) +
                             std::pow((std::get<1>(dest) - std::get<1>(p)) , 2) +
                             std::pow((std::get<2>(dest) - std::get<2>(p)) , 2));
        }

        void AStarSearch() {

        }
};
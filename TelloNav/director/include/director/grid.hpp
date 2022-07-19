#pragma once
#include "cell.hpp"

//This class is a wrapped array data structure aimed to make it easier to match
//the coordinate system in the algorithm with the coordinate system in gazebo
//The sizes for each dimension must be odd, so that there are equal spaces on each side and a middle point
template<size_t xSize, size_t ySize, size_t zSize>
class Grid {
    private:
        std::array<std::array<std::array<Cell, xSize>, ySize>, zSize> grid;

    public:
        Grid() {
            std::array<std::array<std::array<Cell, xSize>, ySize>, zSize> s;
            grid = s;
        }

        Grid(std::array<std::array<std::array<Cell, xSize>, ySize>, zSize> &grid) {
            this->grid = grid;
        }
        
        //Calculates the index value for a given coordinate and dimension value
        //Assumes that it is given good numbers
        int calculateArrIndex(int num, char dimension) {
            switch(dimension) {
                case 'x':
                case 'X':
                    return ((xSize / 2) + num);
                    break;
                case 'y':
                case 'Y':
                    return ((xSize / 2) + num);
                    break;
                case 'z':
                case 'Z':
                    return ((xSize / 2) + num);
                    break;
                default:
                    return -1;
            }
        }



        bool isValid(int x, int y, int z) {
            if(calculateArrIndex(x, 'x') < xSize &&
               calculateArrIndex(y, 'y') < ySize &&
               calculateArrIndex(z, 'z') < zSize &&
               calculateArrIndex(x, 'x') >= 0 &&
               calculateArrIndex(y, 'y') >= 0 &&
               calculateArrIndex(z, 'z') >= 0) {
                return true;
            }
            else {
                return false;
            }
            
        }

        //function to return a mutable reference to a specific cell
        //given descrete cartesian coordinates
        Cell* at(int x, int y, int z) {
            if(isValid(x, y, z))
                return &grid[calculateArrIndex(z, 'z')][calculateArrIndex(y, 'y')][calculateArrIndex(x, 'x')];
            else
                return nullptr;
        }
        
};
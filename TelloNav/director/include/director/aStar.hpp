#pragma once
#include "cell.hpp"
#include "priorityQueue.hpp"
class AStar {
    private:
        Cell*** cells;
        int xSize;
        int ySize;
        int zSize;
        //This is the "voxel" resolution for mapping onto the real world
        int resolution;
    public:
        AStar() {
            this->cells = nullptr;
            this->xSize = 0;
            this->ySize = 0;
            this->zSize = 0;
        }

        AStar(int x, int y, int z) {
            //Initialize 3D dynamic array

            //Make rows
            cells = new Cell**[z];

            //Make columns
            for(int i = 0; i < z; ++i) {
                cells[i] = new Cell*[x];
            }

            //Make the 3rd dimension
            for(int i = 0; i < z; ++i) {
                for(int j = 0; j < x; ++j) {
                    cells[i][j] = new Cell[y];
                }
            }

        }

        AStar(Cell*** cells) {
            this->cells = cells;
        }

        ~AStar() {
            
        }
};
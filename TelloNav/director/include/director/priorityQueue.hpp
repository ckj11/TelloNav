#pragma once
#include "node.hpp"
#include <iostream>
class PriorityQueue {
    private:
        Node* queue;
        int maxSize;
        int size;
    public:
        PriorityQueue(int maxSize) {
            //intialize sizes
            this->maxSize = maxSize;
            this->size = 0;

            //Allocate heap array
            queue = new Node[maxSize];
        }

        int parent(int i) {
            return (i - 1) / 2;
        }

        int leftChild(int i) {
            return ((2 * i) + 1);
        }

        int rightChild(int i) {
            return ((2 * i) + 2);
        }

        void swap(int i1, int i2) {
            Node temp(-1, -1, -1, 1);
            temp = queue[i1];
            queue[i1] = queue[i2];
            queue[i2] = temp;
        }

        void insert(int x, int y, int z, int order) {
            Node newNode(x, y, z, order);
            queue[size] = newNode;
            shiftUp(size);
            size++;
        }

        void shiftUp(int loc) {
            while(loc > 0 && queue[parent(loc)].getOrder() > queue[loc].getOrder()) {
                swap(parent(loc), loc);
                loc = parent(loc);
            }
        }

        void shiftDown(int loc) {
            int maxIndex = loc;

            int l = leftChild(loc);

            if(l <= size && queue[l].getOrder() < queue[maxIndex].getOrder()) {
                maxIndex = l;
            }

            int r = rightChild(loc);

            if(r <= size && queue[r].getOrder() < queue[maxIndex].getOrder()) {
                maxIndex = r;
            }

            if(loc != maxIndex) {
                swap(loc, maxIndex);
                shiftDown(maxIndex);
            }


        }

        Node extractMin() {
            Node result = queue[0];
            queue[0] = queue[size];
            size = size - 1;

            shiftDown(0);
            return result;
        }

        void showQueue() {
            for(int i = 0; i < size; ++i) {
                std::cout << queue[i].getOrder() << " ";
            }
            std::cout <<    std::endl;
        }
};
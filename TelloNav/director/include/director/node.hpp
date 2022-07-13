#pragma once

class Node {
    private:
        int x;
        int y;
        int z;
        int order;
    public:
        Node() {
            this->x = -1;
            this->y = -1;
            this->z = -1;
            this->order = 100000000;
        }
        Node(int x, int y, int z, int order) {
            this->x = x;
            this->y = y;
            this->z = z;
            this->order = order;
        }
        int getX() {return x;}
        int getY() {return y;}
        int getZ() {return z;}
        int getOrder() {return order;}
    
};

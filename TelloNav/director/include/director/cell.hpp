#pragma once 

class Cell {
    private:
        bool visited;
        bool blocked;
    public:
    Cell() {   
        this->visited = false;
        this->blocked = false;
    }

    Cell(bool blocked) {
        this->blocked = blocked;
        this->visited = false;
    }

    void setVisted(bool visited) {
        this->visited = visited;
    }


    void setBlocked(bool blocked) {
        this->blocked = blocked; 
    }

};
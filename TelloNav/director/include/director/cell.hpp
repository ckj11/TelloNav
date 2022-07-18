#pragma once 
#include <tuple>
struct Cell {
    std::tuple<int, int, int> parent;
    double f, g, h;
	bool blocked;
    Cell()
		: parent(-1, -1, -1)
		, f(-1)
		, g(-1)
		, h(-1)
		, blocked(false)
	{
	}
};
// #include <Arduino.h> 
#include <list>
#include <algorithm>
#include <math.h>
// #include "PriorityQueue-master/PriorityQueue.h"
#include <queue>

// debugging: ====================
#ifndef Serial
#include <iostream>
class SerialT {
public:
    template<class T>
    void print(T a) const {
	std::cout << a;
    }

    template<class T>
    void println(T a) const {
	std::cout << a << std::endl;
    }    
};
const SerialT Serial;
#endif
// debugging ^^^ =================

class Vec {
public:
    Vec() : x(0), y(0) {}
    Vec(int i, int j) : x(i), y(j) {}
    bool operator ==(const Vec& o) { return o.x == x && o.y == y; }
    Vec operator+(const Vec& o) { return Vec(o.x + x, o.y + y); }
    int x, y;
};

class Grid {
    // Square Grid of odd size centered at Vec (0, 0) (the size / 2)
 public:
	// size should by an odd number
    Grid() : size(27) {
	w = size;
	h = size;
	grid = new bool *[size];
	for (size_t x = 0; x < size; ++x) {
	    grid[x] = new bool[size];
	    for (size_t y = 0; y < size; ++y) {
		grid[x][y] = false;
	    }
	}
    }
    ~Grid() {
	for (size_t x = 0; x < size; ++x) {
	    delete[] grid[x];
	}
	delete[] grid;
    }

    Grid(Grid& o) {
	w = o.w; h = o.h; size = o.size;
	grid = new bool *[size];
	for (size_t x = 0; x < size; ++x) {
	    grid[x] = new bool[size];
	    for (size_t y = 0; y < size; ++y) {
		grid[x][y] = o[x][y];
	    }
	}
    }
    
    Grid& operator=(Grid& o) {       	
	for (size_t x = 0; x < size; ++x) {
	    delete[] grid[x];
	}
	delete[] grid;
	
	w = o.w; h = o.h; size = o.size;
	grid = new bool *[size];
	for (size_t x = 0; x < size; ++x) {
	    grid[x] = new bool[size];
	    for (size_t y = 0; y < size; ++y) {
		grid[x][y] = o[x][y];
	    }
	}
	
	return *this;
    }
    void debug();
    bool operator()(int x, int y) const;
    void print() const;
    int w, h;
protected:
    bool* operator[](int x) {
	return grid[x];
    }
 private:
    size_t size;
    bool** grid;
};

void Grid::debug() {
    /* (4,6), (5,6), (5,5), (5,4), (5,3) */
    int o = size / 2; // origin
    grid[o - 1][o + 1] = true;
    grid[o + 0][o + 1] = true;
    grid[o + 0][o + 0] = true;
    grid[o + 0][o - 1] = true;
    grid[o + 0][o - 2] = true;
}

bool Grid::operator() (int x, int y) const {
    int originIndex = size / 2;
    return grid[x + originIndex][y + originIndex];
}

void Grid::print() const {
    for (int y = size - 1; y >= 0; --y) {
	for (size_t x = 0; x < size; ++x) {
	    Serial.print(grid[x][y] ? 'X' : '_');
	    Serial.print(" ");
	}
	Serial.print("\n");
    }
}

class Node {
public:
    bool operator ==(const Node& o) { return pos == o.pos; }
    bool operator ==(const Vec& o) { return pos == o; }
    bool operator <(const Node& o) { return dist + cost < o.dist + o.cost; }
    Vec pos, parent;
    int dist, cost;
};

// Need to implement linked list class with push_back and pop_front methods. Or use priority queue instead!

// pretty print lists (of Vecs)
void pp(const std::list<Vec>& v) {
    std::cout << "[";
    for (std::list<Vec>::const_iterator i = v.begin(); i != v.end(); ++i) {
	std::cout << "〈" << i->x << "," << i->y << "〉";
    }
    std::cout << "]" << std::endl;
}

// pretty print lists
void pp(const std::list<Node>& v) {
    std::cout << "{";
    for (std::list<Node>::const_iterator i = v.begin(); i != v.end(); ++i) {
	std::cout << "〈" << i->pos.x << "," << i->pos.y << "〉";
    }
    std::cout << "}" << std::endl;
}

void pp(const Node& n) {
    std::cout << "〈" << n.pos.x << "," << n.pos.y << "〉" << std::endl;
}

void pp(const Vec& v) {
    std::cout << "〈" << v.x << "," << v.y << "〉" << std::endl;
}

class Astar {
public:
    Astar() {
	neighbours[0] = Vec(-1, -1); neighbours[1] = Vec( 1, -1);
        neighbours[2] = Vec(-1,  1); neighbours[3] = Vec( 1,  1);
        neighbours[4] = Vec( 0, -1); neighbours[5] = Vec(-1,  0);
        neighbours[6] = Vec( 0,  1); neighbours[7] = Vec( 1,  0);
    }

    int calcDist(Vec& p){
        // need a better heuristic
        // int x = end.x - p.x, y = end.y - p.y;
        // return(x * x + y * y);
	return round(sqrt(
			  powf((float) abs(end.x - p.x), 2.0) + powf((float) abs(end.y - p.y), 2.0)
			  ) * 10.0);
    }
    
    bool isValid(Vec& p) {
        return (p.x > 0-(grid.w/2) && p.y > 0-(grid.h/2) && p.x < (grid.w / 2) && p.y < (grid.h / 2) );
    }

    bool existVec(Vec& p, int cost) {
        std::list<Node>::iterator i;
        i = std::find(closed.begin(), closed.end(), p);
        if(i != closed.end()) {
            if(i->cost + i->dist < cost ) {
		return true;
	    }
            else {
		closed.erase(i);
		return false;
	    }
        }
        i = std::find(open.begin(), open.end(), p);
        if(i != open.end()) {
            if(i->cost + i->dist < cost ) return true;
            else { open.erase( i ); return false; }
        }
        return false;
    }

    // TODO
    // step cost should be scaled up by 10 to avoid using floats with arduino
    // NESW = 10, Diagonals are 14 (approximately sqrt(2) * 10)
    
    bool fillOpen(Node& n) {
        int stepCost, nc, dist;
        Vec neighbour;
	// iterate through the 8 neighbors of n
        for (int i = 0; i < 8; ++i) { 
            stepCost = i < 4 ? 14 : 10; // DIAGONALS ARE i < 4
            neighbour = n.pos + neighbours[i];
	    pp(open);
	    pp(neighbour);	    
	    
            if (neighbour == end) return true;
	    // I don't understand isValid
	    if (isValid(neighbour)) {
		std::cout << "is valid!" << std::endl;
	    }
	    else {
		std::cout << "is not valid" << std::endl;
	    }
            if (isValid(neighbour) && grid(neighbour.x, neighbour.y) != true) {
		std::cout << "valid" << std::endl;
                nc = stepCost + n.cost;
                dist = calcDist(neighbour);
                if (!existVec(neighbour, nc + dist)) {
		    std::cout << "if block" << std::endl;
                    Node m;
                    m.cost = nc;
		    m.dist = dist;
                    m.pos = neighbour;
                    m.parent = n.pos;
                    open.push(m);
                }
            }
        }
	std::cout << "THIS SPOT" << std::endl;
        return false;
    }

    bool search(Vec& s, Vec& e, Grid& g) { // The algorithm is here!
	Node n;
	end = e;
	start = s;
	grid = g;
	n.cost = 0; n.pos = s; n.parent = s; n.dist = calcDist(s); // what should n.parent be?
	open.push(n);
	while(!open.empty()) {
	    //open.sort();
	    Node n = open.front();
	    open.pop();
	    closed.push_back(n);
	    std::cout << "HERE" << std::endl;
	    // pp(open);
	    if (fillOpen(n)) return true;
	}
	return false;
    }

    int path(std::list<Vec>& path) {
	path.push_front(end);
	int cost = 1 + closed.back().cost;
	path.push_front(closed.back().pos);
	Vec parent = closed.back().parent;

	for (std::list<Node>::reverse_iterator i = closed.rbegin(); i != closed.rend(); ++i) {
	    if(i->pos == parent && !(i->pos == start)) {
		path.push_front(i->pos);
		parent = i->parent;
	    }
	}
	path.push_front(start);
	return cost;
    }

    Grid grid;
    Vec end, start;
    Vec neighbours[8];
    // std::list<Node> open;
    std::list<Node> closed;
    std::priority_queue<Node, std::list<Node>, std::less<Node>> open;
};

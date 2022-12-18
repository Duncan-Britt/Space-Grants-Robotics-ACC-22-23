#include <Arduino.h> 
#include <math.h>
#include "PriorityQueue-master/PriorityQueue.h"

// GRID LENGTH IS SMALL NOW FOR DEBUGGING
#define GRID_LENGTH 7

// debugging: ====================
#ifndef ARDUINO
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
   
    void print() {
	Serial.print("(");
	Serial.print(x);
	Serial.print(",");
	Serial.print(y);
	Serial.println(")");
    }
};

class Grid {
    // Square Grid of odd size centered at Vec (0, 0) (the length / 2)
    // Represents obstacles on terrain. Truthy values represent obstacles.
 public:
	// size should by an odd number
    Grid() {
	for (size_t i = 0; i < area; ++i) {
	    grid[i] = false;
	}
    }
    
    void debug();
    bool operator()(int x, int y) const;
    bool operator()(const Vec&) const;
    void print() const;
    void print(const Vec[], const size_t) const;
protected:
    bool operator[](int i) {
	return grid[i];
    }
 private:
    const size_t length = GRID_LENGTH;
    const size_t area = GRID_LENGTH * GRID_LENGTH;
    bool grid[169];
    size_t indexFromCartesian(int, int);
    size_t indexFromCartesian(const Vec&);
    bool inBounds(int, int);
    bool inBounds(const Vec&);
};

bool Grid::inBounds(int x, int y) {
    return (length/2) >= abs(x) &&
	   (length/2) >= abs(y);
}

bool Grid::inBounds(const Vec& v) {
    return inBounds(v.x, v.y);
}

size_t Grid::indexFromCartesian(int x, int y) {
    return length / 2 + x + length * (length / 2 + y);
}

size_t Grid::indexFromCartesian(const Vec& v) {
    return length / 2 + v.x + length * (length / 2 + v.y);
}

void Grid::debug() {
    // grid[indexFromCartesian(-1,1)] = true;
    grid[indexFromCartesian(0,1)] = true;
    grid[indexFromCartesian(0,0)] = true;
    grid[indexFromCartesian(0,-1)] = true;
}

bool Grid::operator()(int x, int y) const {
    // maps 2D coordinates to the index of the internal 1D Array, returns value at index.
    // (0, 0) is at the center
    // returns true if there is an obstacle or vector is off grid.    
				 return inBounds(x,y) ? grid[indexFromCartesian(x,y)] : true;
}

bool Grid::operator()(const Vec& v) const {
    // maps 2D coordinates to the index of the internal 1D Array, returns value at index.
    // (0, 0) is at the center
    // returns true if there is an obstacle or vector is off grid.
    return inBounds(v) ? grid[indexFromCartesian(v)] : true;
}

void Grid::print() const {
    // (i,j) are centered at the bottom left of the grid
    int j = length - 1;
    int i = 0;

    for (size_t j = length - 1; true; --j) {
	for (size_t i = 0; i < length; ++i) {
	    Serial.print(grid[i + j*length] ? '#' : '.');
	    Serial.print(" ");
	}
	Serial.print("\n");
	if (j == 0) break;
    }
}

void Grid::print(const Vec path[], const size_t pathSize) const {
    // (i,j) are centered at the bottom left of the grid

    // convert path Vectors to indeces
    size_t pathIndeces[pathSize];
    for (int k = 0; k < pathSize; ++k) {
	pathIndeces[k] = indexFromCartesian(path[k]);
    }
    
    int j = length - 1;
    int i = 0;
    size_t index;
    for (size_t j = length - 1; true; --j) {
	for (size_t i = 0; i < length; ++i) {
	    index = i + j*length;
	    // if index matches any element of the path
	    bool matched = false;
	    for (int k = 0; k < pathSize; ++k) {
		if (pathIndeces[k] == index) {
		    matched = true;
		    break;
		}
	    }

	    if (matched) {
		Serial.print("@ ");
	    }
	    else {
		Serial.print(grid[i + j*length] ? '#' : '.');
		Serial.print(" ");		
	    }	  
	}
	Serial.print("\n");
	if (j == 0) break;
    }
}

class Node {
public:
    bool operator ==(const Node& o) { return pos == o.pos; }
    bool operator ==(const Vec& o) { return pos == o; }
    bool operator <(const Node& o) {
	size_t cost = gCost + hCost;
	size_t oCost = o.gCost + o.hCost;
	if (cost == oCost) {
	    return hCost < o.hCost;
	}
	else {
	    return cost < oCost;
	}
    }
    void print() {
	Serial.print("Node[\n  ");
	pos.print();
	Serial.println("]");
    }
    Node() {
	Serial.println("Constructor Called");
    }
    ~Node() {
	Serial.println("Destructor Called");
    }
    Vec pos;
    int parent;
    size_t gCost, hCost;
    bool isNull = false;
};

// Need to implement linked list class with push_back and pop_front methods. Or use priority queue instead!

// pretty print lists (of Vecs)
// void pp(const std::list<Vec>& v) {
//     std::cout << "[";
//     for (std::list<Vec>::const_iterator i = v.begin(); i != v.end(); ++i) {
// 	std::cout << "〈" << i->x << "," << i->y << "〉";
//     }
//     std::cout << "]" << std::endl;
// }

// pretty print lists
// void pp(const std::list<Node>& v) {
//     std::cout << "{";
//     for (std::list<Node>::const_iterator i = v.begin(); i != v.end(); ++i) {
// 	std::cout << "〈" << i->pos.x << "," << i->pos.y << "〉";
//     }
//     std::cout << "}" << std::endl;
// }

// void pp(const Node& n) {
//     std::cout << "〈" << n.pos.x << "," << n.pos.y << "〉" << std::endl;
// }

// void pp(const Vec& v) {
//     std::cout << "〈" << v.x << "," << v.y << "〉" << std::endl;
// }

class Astar {
public:
    Astar() {
	neighbours[0] = Vec(-1, -1); neighbours[1] = Vec( 1, -1);
        neighbours[2] = Vec(-1,  1); neighbours[3] = Vec( 1,  1);
        neighbours[4] = Vec( 0, -1); neighbours[5] = Vec(-1,  0);
        neighbours[6] = Vec( 0,  1); neighbours[7] = Vec( 1,  0);
    }

    int calcDist(const Vec& p){
	return round(sqrt(
			  powf((float) abs(end.x - p.x), 2.0) + powf((float) abs(end.y - p.y), 2.0)
			  ) * 10.0);
    }

    void search(const Vec&, const Vec&, const Grid&, Vec path[], size_t&, size_t);

    Vec end, start;
    Vec neighbours[8];
};

Node nodes[80];

// Mutates path and pathSize argument. Given start and end positions, and a grid, finds the optimal path
// Issue: Currently returns the path in reverse order, which might be problematic if the path exceeds the maxPathSize
void Astar::search(const Vec& s, const Vec& e, const Grid& obstacles, Vec path[], size_t& pathSize, size_t maxPathSize) {   
    start = s;     end = e;
   
    PriorityQueue<size_t> queue = PriorityQueue<size_t>([](size_t a, size_t b)->bool{ return nodes[a] < nodes[b]; });
    
    // Node* nodes = (Node*) malloc(sizeof(*nodes) * 50);
   
    nodes[0].pos = s;
    nodes[0].parent = -1;
    nodes[0].gCost = 0;
    nodes[0].hCost = calcDist(nodes[0].pos);
    
    size_t index = 1;
    size_t parentIdx;
    queue.push(0);

    while (!queue.isEmpty()) {
	parentIdx = queue.pop();
	// add neighbors to queue
	// Serial.print("P: ");
	// nodes[parentIdx].pos.print();
	for (int i = 0; i != 8; ++i) {
	    Vec neighbourVec = (nodes+parentIdx)->pos + neighbours[i];
	    if (obstacles(neighbourVec)) continue;

	    (nodes+index)->pos = neighbourVec;
	    (nodes+index)->parent = parentIdx;
	    (nodes+index)->gCost = (nodes+parentIdx)->gCost + (i < 4 ? 14 : 10); // diagonally pieces are 14 <- sqrt(2) * 10
	    (nodes+index)->hCost = calcDist(neighbourVec);

	    // Serial.print("  N: ");
	    // (nodes+index)->pos.print();
	    if (neighbourVec == end) {
		// finished!
		int i = index;
		int j = 0;
		while (i != -1 && j < maxPathSize) {
		    path[j++] = (nodes+i)->pos;
		    i = (nodes+i)->parent;
		}	    
		
		pathSize = j;
		Serial.println("we made it!");
		return;
	    }

	    queue.push(index++);
	}
    }

    Serial.println("Done!");
}

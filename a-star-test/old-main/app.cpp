#include "grid.h"
#include <iostream>

int main( int argc, char* argv[] ) {
    Grid m;
    Vec s, e(2, 0);
    Astar as;

    if (as.search(s, e, m)) {
        std::list<Vec> path;
        int c = as.path(path); // what is the c for ?

	for (std::list<Vec>::iterator i = path.begin(); i != path.end(); ++i) {
	    std::cout << "x: " << i->x << " y: " << i->y << std::endl;
	}
	
        for (int y = 13; y >= -13; --y) {
            for (int x = -13; x <= 13; ++x) {          
		if (std::find(path.begin(), path.end(), Vec(x, y))!= path.end()) {
		    std::cout << ". ";
		}
		else {
		    std::cout << "_ ";
		}
            }
            std::cout << "\n";
        }

        std::cout << "\nPath cost " << c << ": ";
        for(std::list<Vec>::iterator i = path.begin(); i != path.end(); ++i) {
            std::cout<< "(" << i->x << ", " << i->y << ") ";
        }
    }
    
    std::cout << "\n\n";
    return 0;
}

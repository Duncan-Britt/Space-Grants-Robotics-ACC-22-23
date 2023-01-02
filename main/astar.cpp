#include "astar.h"

typedef struct Node {
    unsigned int grid_idx;
    Node* parent;
    unsigned gCost;
    unsigned hCost; // NOTE: Once max cols and rows are determined, the bits used for this can be restricted
    bool operator<(Node o) {
        return (gCost + hCost) < (o.gCost + o.hCost);
    }
    bool operator>(Node o) {
        return (gCost + hCost) > (o.gCost + o.hCost);
    }
} Node;

//  _   _  _          _      ___ ___  _        __   
// | \ |_ /  |   /\  |_)  /\  |   |  / \ |\ | (_  o 
// |_/ |_ \_ |_ /--\ | \ /--\ |  _|_ \_/ | \| __) o

// ___     ___ _  _                  _           _ ___ ___  _        __ 
//  |  |\ | | |_ |_) |\ |  /\  |    |_ | | |\ | /   |   |  / \ |\ | (_  
// _|_ | \| | |_ | \ | \| /--\ |_   |  |_| | \| \_  |  _|_ \_/ | \| __) 

// void printBits(size_t const size, void const * const ptr);
int pq_parent(int i);
int pq_left_child(int i);
int pq_right_child(int i);
void pq_shift_down(Node* pq, const size_t size, const unsigned int i);
void node_copy(Node* orig, Node* cpy);
void pq_dequeue(Node* pq, size_t* size, Node* result);
void pq_shift_up(Node* pq, int i);
void node_print(Node* node);
void pq_print(Node* pq, size_t size);
    
//  _   _  _ ___      ___ ___ ___  _        __   
// | \ |_ |_  |  |\ |  |   |   |  / \ |\ | (_  o 
// |_/ |_ |  _|_ | \| _|_  |  _|_ \_/ | \| __) o

// ___     ___ _  _                  _           _ ___ ___  _        __ 
//  |  |\ | | |_ |_) |\ |  /\  |    |_ | | |\ | /   |   |  / \ |\ | (_  
// _|_ | \| | |_ | \ | \| /--\ |_   |  |_| | \| \_  |  _|_ \_/ | \| __)  


void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");
}

int pq_parent(int i)
{
    return (i - 1) / 2;
}

int pq_left_child(int i)
{
    return (2 * i) + 1;
}

int pq_right_child(int i)
{
    return (2 * i) + 2;
}

void pq_shift_down(Node* pq, const size_t size, const unsigned int i)
{
    unsigned int minIndex = i;
    unsigned int l = pq_left_child(i);
    
    if (l < size && pq[l] < pq[minIndex]) {
        minIndex = l;
    }

    unsigned int r = pq_right_child(i);

    if (r < size && pq[r] < pq[minIndex]) {
        minIndex = r;
    }

    if (i != minIndex) {
        Node temp = pq[i];
        pq[i] = pq[minIndex];
        pq[minIndex] = temp;

        pq_shift_down(pq, size, minIndex);
    }
}

void node_copy(Node* orig, Node* cpy)
{
    cpy->grid_idx = orig->grid_idx;
    cpy->parent = orig->parent;
    cpy->gCost = orig->gCost;
    cpy->hCost = orig->hCost;
}

void pq_dequeue(Node* pq, size_t* size, Node* result)
{
    node_copy(pq, result);    
    pq[0] = pq[(*size)-1];
    (*size)--;
    pq_shift_down(pq, *size, 0);
}

void pq_shift_up(Node* pq, int i)
{
    while (i > 0 && pq[pq_parent(i)] > pq[i]) {
        int parent_i = pq_parent(i);
        Node temp = pq[i];
        pq[i] = pq[parent_i];
        pq[parent_i] = temp;

        i = parent_i;
    }
}

void node_print(Node* node)
{
    // printf("[gC:%u gI:%u]", node->gCost, node->grid_idx);
    Serial.println("node_print needs to be reimplemented");
}

void pq_print(Node* pq, size_t size)
{
    // printf("pq: ");
    // for (size_t i = 0; i < size; ++i) {
    //     node_print(pq+i);
    // }
    // printf("\n");
    Serial.println("pq_print needs to be reimplemented");
}


//  _       _     ___  _         _ ___ 
// |_) | | |_) |   |  /     /\  |_) |  
// |   |_| |_) |_ _|_ \_   /--\ |  _|_ 

bool grid_obstacle_at(const Grid* grid, size_t idx)
{
    const uint8_t bit_idx = (idx % 8);
    const size_t byte_idx = idx / 8;
    return (grid->obstacles[byte_idx] & (1 << bit_idx)) >> bit_idx;
}

void grid_print(const Grid* grid)
{    
    for (size_t i = 0; i < (grid->cols * grid->rows); ++i) {     
        Serial.print(grid_obstacle_at(grid, i) ? "# " : ". ");
        if ((i+1) % grid->cols == 0) {
            Serial.println("");
        }
    }
}

void grid_print_mark(const Grid* grid, const size_t marked)
{    
    for (size_t i = 0; i < (grid->cols * grid->rows); ++i) {
        if (i == marked) {            
            Serial.print("@ ");
        }
        else {
            Serial.print(grid_obstacle_at(grid, i) ? "# " : ". ");
        }
        if ((i+1) % grid->cols == 0) {
            Serial.println("");
        }
    }
}

void grid_print_path(const Grid* grid, const unsigned int* path, const unsigned char path_size)
{
    for (size_t i = 0; i < (grid->cols * grid->rows); ++i) {
        bool in_path = false;
        for (size_t j = 0; j < path_size; ++j) {
            if (path[j] == i) {
                in_path = true;
                break;
            }
        }
        
        if (in_path) {
            Serial.print("@ ");
        }
        else {
            Serial.print(grid_obstacle_at(grid, i) ? "# " : ". ");
        }
        if ((i+1) % grid->cols == 0) {
            Serial.println("");
        }
    }
}

Err grid_init_str(char* s, Grid* grid)
{
    grid->cols = 0;
    grid->rows = 0;
    char end_row_found = 0;
    for (size_t i = 0; i < strlen(s); ++i) {
        if (s[i] == '\n') {
            end_row_found = 1;
            grid->rows++;
        }
        if (!end_row_found) {
            if (s[i] != ' ') {
                grid->cols++;
            }
        }
    }
    grid->rows++;
    size_t grid_size = (grid->cols * grid->rows * sizeof(char)) / 8 + 1;
    grid->obstacles = (char*) malloc(grid_size);
    for (size_t i = 0; i < grid_size; ++i) {
        grid->obstacles[i] = 0;
    }
    
    char* delim = "\n";    
    char* row = strtok(s, delim);

    int j = 0;  // byte index
    char k = 0; // bit index
    while (row != NULL) {
        for (size_t i = 0; i < strlen(row); ++i) {
            switch (row[i]) {
            case ' ':
                continue;
                break;
            case '.':
                if (++k == 8) {
                    k = 0;
                    ++j;
                }
                break;
            case '#':
                grid->obstacles[j] = grid->obstacles[j] | (1<<k);
                if (++k == 8) {
                    k = 0;
                    ++j;
                }
                break;
            default:
                Serial.print("Unexpected char: ");
                Serial.print(row[i]);
                Serial.println("");
                return 1;
            }
        }
        
        row = strtok(NULL, delim);
    }

    return 0;
}

void grid_idx_to_cartesian(const Grid* grid, const unsigned int i, int* x, int* y)
{
    *x = i % grid->cols;
    *y = i / grid->cols;    
}

unsigned grid_distance(const Grid* grid, const unsigned int i, const unsigned int j)
{
    int ix;
    int iy;
    grid_idx_to_cartesian(grid, i, &ix, &iy);
    int jx;
    int jy;
    grid_idx_to_cartesian(grid, j, &jx, &jy);

    return round(10 * sqrt(pow((float)abs(jy - iy), 2) + pow((float)abs(jx - ix), 2)));    
}


Err grid_find_path(const Grid* grid, const unsigned int start, const unsigned int dest, unsigned int* path, unsigned char* path_size, const unsigned char max_path_size)
{
    if (grid_obstacle_at(grid, start)) {
        return -2;
    }
    if (grid_obstacle_at(grid, dest)) {
        return -3;
    }
    // 5 6 7
    // 4 X 0
    // 3 2 1
    const int neighbors[8] = {1, grid->cols+1, grid->cols, grid->cols-1, -1, 0-grid->cols-1, 0-grid->cols, 0-grid->cols+1};

    const size_t pq_max_size = 80; // EXAMINE THIS
    const size_t explored_max_size = pq_max_size; // AND THIS!

    Node pq[pq_max_size];
    pq[0].grid_idx = dest;
    pq[0].parent = NULL;
    pq[0].gCost = 0;
    pq[0].hCost = grid_distance(grid, dest, start);
    size_t pq_size = 1;
    
    Node explored[explored_max_size];
    size_t explored_size = 0;

    while (pq_size != 0) {
        pq_dequeue(pq, &pq_size, explored + explored_size);
        explored_size++;
        
        for (char i = 0; i < 8; ++i) {
            if (explored[explored_size-1].grid_idx < grid->cols && i >= 5) continue; // first row
            if (explored[explored_size-1].grid_idx % grid->cols == 0 && i >= 3 && i <= 5) continue; // first column
            if (explored[explored_size-1].grid_idx % grid->cols == grid->rows - 1 && (i <= 1 || i == 7)) continue; // last column
            if (explored[explored_size-1].grid_idx / grid->cols == grid->rows - 1 && (i <= 3 && i >= 1)) continue; // last row
        
            size_t neighbor_idx = explored[explored_size - 1].grid_idx + neighbors[i];            
            if (grid_obstacle_at(grid, neighbor_idx)) continue;            

            if (neighbor_idx == start) { // this isn't necessarily the shortest, but good enough                
                path[0] = neighbor_idx;                
                (*path_size)++;
                Node* node_ptr = explored + (explored_size - 1);
                for (int i = 1; node_ptr != NULL && *path_size < max_path_size; ++i) {
                    path[i] = node_ptr->grid_idx;                    
                    node_ptr = node_ptr->parent;
                    (*path_size)++;
                    if (*path_size == max_path_size && node_ptr != NULL) {
                        return -1;
                    }
                }                

                return 0;
            }

            // ask whether the neighbor_idx exists in the pq already
            bool in_pq = false;
            size_t pq_idx = 0;
            for (size_t i = 0; i < pq_size; ++i) {
                if (pq[i].grid_idx == neighbor_idx) {
                    in_pq = true;
                    pq_idx = i;
                    break;
                }
            }                        
            // If so, the gCost and parent may need to be updated to reflect a lower cost path
            if (in_pq) { // when switching to using fCost, this needs to be updated
                unsigned new_gCost = explored[explored_size-1].gCost + grid_distance(grid, explored[explored_size-1].grid_idx, neighbor_idx);
                if (new_gCost < pq[pq_idx].gCost) {
                    pq[pq_idx].gCost = new_gCost;
                    pq[pq_idx].parent = explored+(explored_size-1);
                    /* printf("modifying parent => &pq[pq_idx]: %x, &explored[explored_size-1]: %x\n", &(pq[pq_idx]), &explored[explored_size-1]); */                    
                    pq_shift_up(pq, pq_idx);                    
                }
            }
            else { // otherwise, add a new node to the queue as below
                pq[pq_size].grid_idx = neighbor_idx;
                pq[pq_size].parent = explored+(explored_size-1);
                /* printf("adding parent => &pq[pq_size]: %x, &explored[explored_size-1]: %x\n", &(pq[pq_size]), &explored[explored_size-1]); */
                pq[pq_size].gCost = explored[explored_size-1].gCost + grid_distance(grid, explored[explored_size-1].grid_idx, neighbor_idx);
                pq[pq_size].hCost = grid_distance(grid, neighbor_idx, start);
                pq_shift_up(pq, pq_size);
                ++pq_size;
            }
        }
    }
    
    return 1;
}

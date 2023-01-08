#include "pose.h"

#ifdef DEBUG
void Vec2D_print(const Vec2D* v)
{
    DEBUG_PRINT(F("("));
    DEBUG_PRINT(v->x);
    DEBUG_PRINT(F(","));
    DEBUG_PRINT(v->y);
    DEBUG_PRINT(F(")"));
}

void Vec2D_println(const Vec2D* v)
{
    Vec2D_print(v);
    DEBUG_PRINTLN(F(""));
}

void Pose_print(const Pose* p)
{
    DEBUG_PRINT(F("[ "));
    Vec2D_print(&(p->translation));
    DEBUG_PRINT(F(" "));
    DEBUG_PRINT(p->rotation);
    DEBUG_PRINT(F(" ]"));
}

void Pose_println(const Pose* p)
{
    Pose_print(p);
    DEBUG_PRINTLN(F(""));
}

void Vec2DQueue::print()
{
    DEBUG_PRINT(F(":: "));
    for (uint8_t i = b, j = 0; j < size; i = (i + 1) % capacity) {
        Vec2D_print(queue + i);
        DEBUG_PRINT(F(" :: "));
        ++j;
    }
}

void Vec2DQueue::println()
{
    print();
    DEBUG_PRINTLN(F(""));
}
#endif//DEBUG

bool operator==(const Pose& a, const Pose& b)
{
    return a.translation == b.translation && a.rotation == b.rotation;
}

bool operator==(const Vec2D& a, const Vec2D& b)
{
    return a.x == b.x && a.y == b.y;
}

// Deteremines the quadrant of a cartesian coordinate.
// Returns numbers in range [1, 4].
uint8_t quadrant(const Vec2D* A);

void Vec2D_sub(const Vec2D* a, const Vec2D* b, Vec2D* result)
{
    result->x = a->x - b->x;
    result->y = a->y - b->y;
}

uint8_t quadrant(const Vec2D* A)
{    
    return A->x >= 0 && A->y >= 0 ? 1 :
          (A->x  < 0 && A->y >= 0 ? 2 :
          (A->x  < 0 && A->y  < 0 ? 3 : 4));
}

void Pose_enqueue_transition(const Pose* current_pose, const Vec2D* desired_vec2d, Pose* steps)
{
    // if (current_pose->translation == *desired_vec2d) return;
    
    // To find the first pose, subtract the current pose translation vector from the desired pose translation vector.
    // Take the angle from the positive x axis of this new vector.
    Vec2D diff;
    Vec2D_sub(desired_vec2d, &(current_pose->translation), &diff);
 
    steps->translation = current_pose->translation;
    
    switch (quadrant(&diff)) {
    case 1: // QUADRANT I
        steps->rotation = atan((double)diff.y / (double)diff.x);
        break;
    case 2: // QUADRANT II
        steps->rotation = atan((double)diff.y / (double)diff.x) + 3.14;
        break;
    case 3: // QUADRANT III
        steps->rotation = atan((double)diff.y / (double)diff.x) + 3.14;        
        break;
    case 4: // QUADRANT IV
        steps->rotation = 6.28 + atan((double)diff.y / (double)diff.x);        
        break;        
    }
           
    // Second pose:
    (steps + 1)->translation = *desired_vec2d;
    (steps + 1)->rotation = steps->rotation;   
}

Vec2DQueue& Vec2DQueue::enqueue(Vec2D* position)
{
    if (size == capacity) return *this;

    *(queue + e) = *position;
    e = (e + 1) % capacity;
    ++size;
        
    return *this;
}

Vec2DQueue& Vec2DQueue::dequeue()
{
    if (size == 0) {
        return *this;
    }
        
    b = (b + 1) % capacity;
    --size;
        
    return *this;
}

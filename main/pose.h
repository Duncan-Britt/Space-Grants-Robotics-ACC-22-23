#ifndef GUARD_pose_h
#define GUARD_pose_h

#include "debug.h"

typedef struct Vec2D {
    int x;
    int y;
} Vec2D;

bool operator==(const Vec2D& a, const Vec2D& b);
void Vec2D_sub(const Vec2D* a, const Vec2D* b, Vec2D* result);
void Vec2D_print(const Vec2D* v);
void Vec2D_println(const Vec2D* v);

// DO NOT CHANGE THE ORDER OF THE STRUCT MEMBERS
typedef struct Pose {
    Vec2D translation;
    double rotation; // radians in range [0, 2π]
} Pose;


bool operator==(const Pose& a, const Pose& b);
void Pose_print(const Pose* p);
void Pose_println(const Pose* p);

// Given the robots current pose and a desired pose, the transformation can be broken down into a series of 
// intermediary poses. Supposing that there is no obstacle between the current and desired poses
// - the first pose should be in the current spot but facing the desired spot
// - the second pose should be in the desired spot
// - the third pose should be in the desired spot and facing the desired direction
   // steps must be an array of length 3;
void Pose_enqueue_transition(const Pose* current_pose, const Pose* desired_pose, Pose* steps);

class PoseQueue {
/* Example use: */
/*     Adding a pose to the back of the queue: */
/*     Pose a = { .translation = { .x = 42, .y = (42 / 2) }, .rotation = ((double)42 / 10.0) }; */
/*     pose_queue.enqueue(&a); */

/*     Removing a pose from the front of the queue: */
/*     pose_queue.dequeue(); */

/*     Accessing the pose at the front of the queue: */
/*     pose_queue.front(); */
/*     Note that ^^^ this returns a const pointer to the pose at the front of the
       queue. */
/*     ... pose_queue.front()->rotation; */
 public:
    // ALWAYS check that the pose_queue is not empty before calling front() or last()!
    const Pose* front() { return queue + b; }
    Pose* last() { return queue + ((e - 1) % capacity); }
    PoseQueue& enqueue(Pose* pose);
    PoseQueue& dequeue();
    PoseQueue& clear() { b = 0; e = 0; size = 0; return *this; }
    bool full() { return size == capacity; }
    bool empty() { return size == 0; }
    void print();   //
    void println(); // For testing/debugging
    uint8_t size = 0; 
 private:
    static const uint8_t capacity = 10; // Should this be greater? Lesser?
    Pose queue[capacity];
    uint8_t b = 0;
    uint8_t e = 0;

};

#endif//GUARD_pose_h

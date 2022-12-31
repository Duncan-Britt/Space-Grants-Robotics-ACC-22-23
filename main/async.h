#ifndef GUARD_async_h
#define GUARD_async_h
#include <Arduino.h>

// There is no multi-threading/multi-process support on the Arduino, so this code implementes an interface 
// for defining distinct loops that will run asynchronously.

//  _                                       
// |_     _. ._ _  ._  |  _    | |  _  _  o 
// |_ >< (_| | | | |_) | (/_   |_| _> (/_ o 
//                 |                        

// AsyncLoop bread_loop;
// AsyncLoop laundry_loop;
//
// void setup() {
//    ...
//    bread_loop
//        .when(the_bread_is_ready)
//        .then(take_it_out_of_the_oven)
//        .when(it_has_cooled_for_2_min)
//        .then(enjoy)
//        .when(done_eating)
//        .then(put_more_bread_in_the_oven);
//    laundry_loop
//        .when(too_much_dirty_laundry)
//        .then(put_laundry_in_washer)
//        .when(finished_in_washer)
//        .then(put_laundry_in_dryer)
//        .when(dryer_finished)
//        .then(fold);
//    ...
// }
//
// void loop() {
//     ...
//     bread_loop();
//     laundry_loop();
//     ...
// }
//
// bool the_bread_is_ready()
// {
//     ... 
// }
//
//
// void take_it out of the oven()
// {
//     ...
// }
//
// etc...

class AsyncLoop {
 public:
    AsyncLoop& when(void* predicate) { listeners[e] = predicate; return *this; }    
    AsyncLoop& then(void* handler) { handlers[e++] = handler; return *this; }
    void operator()() {
        if (b == e) {
            return;
        }
        // cast the listener at the front of the queue to a function pointer -> bool.
        // Invoke the function.
        if (  (  (bool (*)())(listeners[b]) )()  ) {
            // cast the associated event handler to a function pointer -> void.
            // Invoke the function.
            (  (void (*)())(handlers[b])  )();
            
            b++;
            if (b == e) {
                b = 0;
            }
        }
    }
    
 private:
    static const uint8_t capacity = 10; // so far the max used is 2, not 10
    uint8_t b = 0;
    uint8_t e = 0;
    void* listeners[capacity];
    void* handlers[capacity];
};

#endif//GUARD_async_h

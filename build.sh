#!/bin/sh

set -xe

# -Wall -Wextra
g++ -x c++ -fsanitize=address -std=c++11 main/main.ino main/grid.cpp main/motors.cpp main/pose.cpp main/SharpIR.cpp main/debug.cpp -o app

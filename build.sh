#!/bin/sh

set -xe

g++ -x c++ -fsanitize=address -std=c++11 -Wall -Wextra main/main.ino main/grid.cpp main/motors.cpp main/pose.cpp main/SharpIR.cpp main/debug.cpp -o app

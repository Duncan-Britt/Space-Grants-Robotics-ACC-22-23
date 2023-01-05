#!/bin/sh

set -xe

g++ -x c++ -fsanitize=address -std=c++11 `Magick++-config --cxxflags --cppflags` -Wall -Wextra main/main.ino main/astar.cpp main/motors.cpp main/pose.cpp main/SharpIR.cpp main/debug.cpp -o app `Magick++-config --ldflags --libs`

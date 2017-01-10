#!/bin/sh
rm -rf build
mkdir build
echo "Compiling Main Camera Code"
g++ -ggdb `pkg-config opencv --cflags --libs` -o build/camera camera.cpp `pkg-config --libs opencv` -Wall
./build/camera

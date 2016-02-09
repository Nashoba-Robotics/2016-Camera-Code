#!/bin/sh
rm -rf build
mkdir build
echo "Compiling Main Camera Code"
g++ -ggdb `pkg-config opencv --cflags --libs` camera.cpp -o build/camera `pkg-config --libs opencv`
echo "Compiling Camera Calibration Code"
g++ -ggdb `pkg-config opencv --cflags --libs` camera_calibration.cpp -o build/camera_calib `pkg-config --libs opencv`
./build/camera

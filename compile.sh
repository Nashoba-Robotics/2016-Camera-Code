#!/bin/sh
rm -rf build
mkdir build
echo "Compiling Main Camera Code"
g++ -L /usr/local/cuda-6.5/lib -ggdb `pkg-config opencv --cflags` camera.cpp -o build/camera `pkg-config --libs opencv` -Wall
echo "Compiling Camera Calibration Code"
g++ -L /usr/local/cuda-6.5/lib -ggdb `pkg-config opencv --cflags` camera_calibration.cpp -o build/camera_calib `pkg-config --libs opencv` -Wall
echo "Compiling Network Testing Code"
gcc network.c -o build/network -Wall
if [ $1 = "run" ] ; then
  ./build/camera
fi

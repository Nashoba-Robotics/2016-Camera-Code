#!/bin/sh

echo "Compiling Main Camera Code"
g++ -ggdb `pkg-config opencv --cflags --libs` camera.cpp -o camera `pkg-config --libs opencv`
echo "Compiling Camera Calibration Code"
g++ -ggdb `pkg-config opencv --cflags --libs` camera_calibration.cpp -o camera_calib `pkg-config --libs opencv`


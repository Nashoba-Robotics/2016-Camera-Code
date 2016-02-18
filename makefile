all: camera camera_calib camera_test

camera: clean camera.cpp GetImage.h network.h GetImage.cpp network.cpp
	-mkdir build
	g++ -o build/camera GetImage.cpp network.cpp camera.cpp -ggdb `pkg-config opencv --cflags --libs` `pkg-config --libs opencv` -Wall

camera_calib: clean camera_calibration.cpp GetImage.h GetImage.cpp
	-mkdir build
	g++ -o build/camera_calib -ggdb `pkg-config opencv --cflags --libs` camera_calibration.cpp GetImage.cpp `pkg-config --libs opencv`

camera_test: clean camera_test.cpp GetImage.h GetImage.cpp
	-mkdir build
	g++ -ggdb `pkg-config opencv --cflags --libs` -o build/camera_test GetImage.cpp camera_test.cpp `pkg-config --libs opencv` -Wall

clean:
	rm -rf build

run: run_camera

run_camera: camera
	./build/camera

run_calib: camera_calib
	./build/camera_calib

run_test: camera_test
	./build/camera_test

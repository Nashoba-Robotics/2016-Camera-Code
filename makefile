build/camera: camera.cpp GetImage.h network.h
	g++ -o build/camera GetImage.cpp network.cpp camera.cpp -ggdb `pkg-config opencv --cflags --libs` `pkg-config --libs opencv` -Wall

build/camera_calib: camera_calibration.cpp GetImage.h
	g++ -o build/camera_calib -ggdb `pkg-config opencv --cflags --libs` camera_calibration.cpp GetImage.cpp `pkg-config --libs opencv`

build/camera_test: camera_test.cpp GetImage.h
	g++ -ggdb `pkg-config opencv --cflags --libs` -o build/camera_test GetImage.cpp camera_test.cpp `pkg-config --libs opencv` -Wall

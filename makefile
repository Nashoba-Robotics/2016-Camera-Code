camera: camera.cpp
	rm -rf build
	mkdir build
	g++ -ggdb `pkg-config opencv --cflags --libs` -o build/camera camera.cpp tcp_client.cpp `pkg-config --libs opencv` -Wall -I/usr/local/include/ -lraspicam -lraspicam_cv

run: 
	./build/camera

clean:
	rm -rf build

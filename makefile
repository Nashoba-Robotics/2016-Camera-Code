camera: clean camera.cpp
	-mkdir build
	g++ -ggdb `pkg-config opencv --cflags --libs` -o build/camera camera.cpp `pkg-config --libs opencv` -Wall

run:
	./build/camera

clean:
	rm -rf build

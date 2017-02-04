The code for the 2016 robot.

It runs on a Jetson TK1 or Raspberry Pi.

It uses OpenCV to identify the target.

**Compilation**
Install opencv 2.4.x 
Install raspicam (www.uco.es/investiga/grupos/ava/node/40) and enable USE_RASPICAM in camera.cpp if you want to use the raspicam 

Run `make camera` to compile.

**Running**

Run `make run` or `./build/camera`.

**Clean up build**

Run `make clean`

This shouldn't ever need to be done. 

**Overclocking Raspberry Pi**
In /boot/config.txt, add the lines

arm_freq=1400
over_voltage=5
adram_freq=500

arm_freq refers to the MHz of the cpu, the other settings allow it to interface with the rest of the Pi successfully.


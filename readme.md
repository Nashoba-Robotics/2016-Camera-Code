The code for the 2016 robot.

It runs on a Jetson TK1.

It uses OpenCV to identify the target.

camera_calibration.cpp uses a chessboard pattern to find a calibration matrix that is used to calibrate the camera.

**Compilation**
install opencv 2.4.* and raspicam (www.uco.es/investiga/grupos/ava/node/40 

Run ./compile.sh

**Overclocking Raspberry Pi**
In /boot/config.txt, add the lines

arm_freq=1400
over_voltage=5
adram_freq=500

arm_freq refers to the MHz of the cpu, the other settings allow it to interface with the rest of the Pi successfully.


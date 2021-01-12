# IMU-visualizer
Small tool using OpenGL for visualizing IMU orientation.

So far it supports MPU-6050 and MPU-9250.

As of now, it is broke down into 2 scripts, one running on a Raspberry Pi which fetches the IMU data and calculates angles and another script running on the pc fetching the angle data from the RPi by serial to do the OpenGL visualization.

Thanks to Fredrik Andersson for the initial code.

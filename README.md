4-axis drone controller for Parrot AR Drone symulation in Robot Operating System (ROS). Uses 4 independent Active Disturbance Rejection Control (ADRC) based controllers. Code was written for purposes of my PhD research.


Scripts:

reg.py - is the main programm, that gathers sensor data and control the drone.

ADRC.py - script with Active Disturbance Rejection Control based controller.

PID.py - script with PID controller (simplest than ADRC).

...plotter.py - additional files, that plots regulator data.

This project consists of a 4-axis drone controller for the Parrot AR Drone simulation in the Robot Operating System (ROS). It makes use of 4 independent and advanced Active Disturbance Rejection Control (ADRC) based controllers. The code has been developed specifically for use in my PhD research project.

This repository contains the following scripts:

1. reg.py: This is the primary program. It is responsible for collecting sensor data and controlling the drone based on the data received.

2. ADRC.py: This script implements the Active Disturbance Rejection Control-based controller algorithm, which is an advanced method for drone control and provides better performance than traditional PID control.

3. PID.py: This script implements a Proportional-Integral-Derivative (PID) controller, which is a simpler drone control method compared to ADRC. The PID regulator is included for testing purposes and facilitates performance comparison between the PID and ADRC drone control approaches, helping to showcase the advantages and improvements offered by the ADRC-based controllers.

4. Plotter.py: This is an additional script that creates regulation curve plots to help visualize and analyze the performance of the drone.

The sophisticated 4-axis drone controller has been carefully designed and implemented to offer a modern and efficient solution for controlling Parrot AR Drone simulations in the Robot Operating System (ROS). The use of ADRC-based controllers enhances the drone's stability and ability to handle disturbances, making it an ideal candidate for research and development purposes.

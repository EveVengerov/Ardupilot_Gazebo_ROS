# Robotics Software Engineer Assignment

## Tasks:
### 1. Simulation environment setup:
For my task I have chosen Ardupilot to get started with, the reason being easy to follow documentation on integrating it with ros using the Mavros package, also having some experience with ROS is easy to get started with communicating with Ardupilot and Gazebo at the same time . Below is a diagram showing the relationship among them.
![ROS_ArduPilot_Gazebo](https://github.com/EveVengerov/Ardupilot_Gazebo_ROS/assets/70894557/b73222fb-f6d2-426e-b007-0bfaf2255682)

Video demmo of the setup environment: 
The drone takes off at an altitude of 10 mtrs, mob=ves forward in y axis at a speed of 10 mtrs/s towards its destination at 30 mtrs from launch, then finally lands.
The script for this maneuver is given in src\set_wp.cpp

https://github.com/EveVengerov/Ardupilot_Gazebo_ROS/assets/70894557/8f9749da-73f8-4b84-b58f-34588bfe4651


### 2. Spiral path implementation:

The script src\spiral_trajectory.cpp shows the implementation of a spiral path using auto mode. auto mode accepts a list of waypoints in global coordinates (latitude, longitude and altitude) as ‘missions’ and automatically traces through each waypoint as a spline curve. Note that in guided mode the velocity reaches 0 before heading to the next waypoint, in auto mode the velocity slows down but does not tend to 0, resulting in a smoother trajectory. 

Spiral Path parallel to y-axis:


https://github.com/EveVengerov/Ardupilot_Gazebo_ROS/assets/70894557/9c91c5ae-e173-447e-92fa-dded3a60184f


### 3. AruCo landing:

This is the demon of the script src\precision_landing.cpp. The drone takes off at an attitude of 3 meters, simultaneously the camera looks for an aruco marker, if the marker is detected it hovers over the marker at an attitude of 0.1 meters while waiting for land command. 


https://github.com/EveVengerov/Ardupilot_Gazebo_ROS/assets/70894557/62e4b154-d522-4b57-a93c-c557365cd296




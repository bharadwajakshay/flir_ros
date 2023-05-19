# ROS2 drivers for FLIR Cameras
## Repository Name: flir_ros2
### Currently tested only on Blackfly cameras with ROS2 Hubmble

### Introduction
The inspiration for writing this code is to use SFOTWARE trigger to get images at the frequency you need to.
Additionally, there is a ros subscriber that can subscribe to another topic to initiate a image capture. The use case is for
precise synchronization between other sensors. 

This code is heavily inspired by https://github.com/berndpfrommer/flir_spinnaker_ros2.

### Current status
Curerntly the program only supports adjusting exposure settings at the time of launch. The image encoding is is always RGB8.

###  TODO
* Add adjusting the gain settings
* Add destructor to close all the initialized cameras 
* Add code to trigger the camera when subscribed to a triggering topic
* Add code to keep waiting for images when hardware trigger is enabled

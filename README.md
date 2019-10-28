# ROS Tutorial Publisher / Subscriber

## Overview
This code was developed for ENPM808X Software Development for Robotics. The assignment was to get familiar with ROS by running several tutorials located at their webiste http://wiki.ros.org/ROS/Tutorials
The main purpose of this code is to understand publishers and subscribers. The two pieces of code talker.cpp and listener.cpp are a publisher and subscriber respectively.

## Build
In order to build this package it is necessary to use catkin_make. The beginning steps of the tutorial are centered around learneing catkin workspaces and how to make packages from those workspaces.
The code to create a catkin worspace is below:
```
catkin_make
```
After the catkin workspace is created it was time to create the package we would be building in. In order to do that the following code must be followed:
```
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```
This will then create the package "beginner_tutorials" with the dependancies std_msgs, rospy, and roscpp.
After making the package there are two important files that must be updated. Those files are package.xml and CMakeLists.txt. The correct format for those two files are inside this repository.

## Run
In order to run the Publisher and Subscriber code the first step is to have the master of ROS running in its own terminal:
```
roscore
```
Once it is running, the code back in your workspace can be run:
```
source ./devel/setup.bash
rosrun beginner_tutorials talker
```
and for the subscriber:
```
rosrun beginner_tutorials listener
```

## Dependencies
The Publisher/Subscriber code is dependent of ROS's standard messages, ROS's C++ libraries, and ROS's python libraries. In the above section about building this code those libraries should naturally be include if all steps were followed correctly.

## Assumptions
It is assumed that in order to run this repository your computer has already downloaded and installed ROS through the standard Ubuntu downloading format to get the standard libraries that come with ROS.
For more help on installing ROS on your system visit: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

It is also an assumption that this code is programmed and run on C++11, to run this code make sure your system can run C++11 code.

## Aditional Help
Additional help can be found on the ROS tutorial webiste located at: http://wiki.ros.org/ROS/Tutorials


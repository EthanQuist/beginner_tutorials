# ROS Tutorial ROS Services, Logging and Launch Files

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


If you already have a running catkin worspace then you can copy this respository as a package for your catkin workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/EthanQuist/beginner_tutorials.git
```


Further instructions on the build of this project can be found at:
http://wiki.ros.org/ROS/Tutorials/CreatingPackage
http://wiki.ros.org/catkin/Tutorials/create_a_workspace


## Run
The below steps are to run the codes individually. After this is the example to run from the launcher file.
In order to run the Publisher and Subscriber code the first step is to have the master of ROS running in its own terminal:
```
roscore
```
The talker uses a service to add two integers together that it is counting. The service must be running before the talker code is executed. In a separate terminal run
```
source ./devel/setup.bash
rosrun beginner_tutorials add_two_ints_server
```

Once it is running, the code back in your workspace can be run. The Talker executable must be run with a command arguement for the Hertz it runs at. Putting an integer with the command line will run it correctly:
```
source ./devel/setup.bash
rosrun beginner_tutorials talker 10
```
and for the subscriber (note: I will omit the source line which needs to be run in each new terminal):
```
rosrun beginner_tutorials listener
```

To run with the launcher file:
```
cd src/beginner_tutorials/launch
roslaunch beginner_tutorials week10HW.launch hertz:=10
```

This will start the nodes from the launcher file. You must open up separate terminals in order to individually run the nodes to see their outputs. Make sure to have the correct source before running on a terminal.



A core aspect of this assignment is the use of tf within ros. The talker.cpp code has been updated to add a frame called "/talk" to the "/world" frame. To see the new frame use the above code to run the service server and talker code and then in a separate terminal run:
```
rosrun rqt_tf_tree rqt_tf_tree
``` 

Another aspect of this assignment was demonstrating a level 2 test within ros to test out either a publisher, subscriber or a service. The test implemented in the test folder will test out the service that runs in talker.cpp which adds two ints together. The tests runs when catkin makes the package by flagging the run test option. This can be implemented from the catkin workspace my making the package with the following line of code:
```
catkin_make run_tests
```

The last aspect of this assignment was understanding and using bagfiles to record and use recorded data. A new folder was added to the package called /bagfiles which is where the bags are recorded and saved. While the nodes are running, in a separate terminal the information can be recorded by the following code:
```
cd bagfiles
rosbag record -a
```
This will run until cancelled by ctl+c and will then store the recorded bagfile in the current folder /bagfile and will be named by the timestamp of the recording. To examine the bagfile you just created run the following line of code:
```
rosbag info "your_bagfile" 
```

With the saved bagfile the information recorded can be played back and in fact the listener node created can relisten to the data recently recorded. With the listener node running use the following line of code to play back the recording:
```
rosbag play "your_bagfile"
```

## In your catkin workspace
The following source line needs to be run in every new terminal for the ros code to be executed correclty. 
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```


## Dependencies
This entire code is depenent on the correct installation of ROS KINETIC - it is the Kinetic version of ROS and may cause errors if another version of ROS is installed on your system.

The talker node REQUIRES a command line argument of Hertz. This is an integer appended after the run command. This is why in the two examples above for either running the node itself or by the launcher file hertz is passsed in as 10.

The Publisher/Subscriber code is dependent of ROS's standard messages, ROS's C++ libraries, and ROS's python libraries. In the above section about building this code those libraries should naturally be include if all steps were followed correctly.

## Assumptions
It is assumed that in order to run this repository your computer has already downloaded and installed ROS through the standard Ubuntu downloading format to get the standard libraries that come with ROS.
For more help on installing ROS on your system visit: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

It is also an assumption that this code is programmed and run on C++11, to run this code make sure your system can run C++11 code.

## Aditional Help
Additional help can be found on the ROS tutorial webiste located at: http://wiki.ros.org/ROS/Tutorials


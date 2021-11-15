# beginner_tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
-----

## Overview

A beginners tutorial for ROS Publisher and Subscriber using C++.
This repository contains a simple tutorial to work with ROS Publisher and Subscriber to understand their roles in ROS.

## Dependencies

* Ubuntu 18.04 LTS
* ROS Melodic
* Modern C++ Programming Language
* Roscpp Package
* Std_msgs Package
* Message_generation Package
* Catkin_Make Build System

## Build

Create a Catkin Workspace and cd into it

```
cd ~/catkin_ws
catkin_make
```

Clone the repository 

```
cd src
git clone https://github.com/iamjadhav/beginner_tutorials.git
```

## Run

In a New Terminal, run the code using launch file

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch beginner_tutorials begin_tutorials.launch
```

Running with a frequency different than the default: just change the last command to,

```
roslaunch beginner_tutorials begin_tutorials.launch frequency:="<desired frequency>"
```

## ROS Service

In a New Terminal (while the nodes are running), type

```
cd ~/catkin_ws
source devel/setup.bash
rosservice call /AddTwoFloats "a: <float of your choice> b: <float of your choice>"
```

## Inspecting ROS TF Frames

To view the transform between the world and talk frames: in one terminal,

- launch the nodes from the catkin_ws using the launch file

and in another terminal,

- type,

```
cd ~/catkin_ws
rosrun tf tf_echo world talk
```

And for creating a diagram of the frames: in the same terminal after tf_echo , type,

```
rosrun tf view_frames
evince frames.pdf
rosrun rqt_tf_tree rqt_tf_tree
```

frames.pdf will be saved in the package which contains the view_frames result.

## Tests

To run the tests: In a new terminal, type,

```
cd ~/catkin_ws
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
```

OR to run the tests with the launch file: type,

```
cd ~/catkin_ws
source devel/setup.bash
rostest beginner_tutorials AddTwoFloatsTest.launch
```

## ROSBAG 

# Recording a bag

The rosbagEnable argument is disabled by default. To record a rosbag: launch the nodes with the following commands,

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch beginner_tutorials begin_tutorials.launch rosbagEnable:=true
```

# Playing the bag

To play a rosbag: while the nodes are launched OR a separate roscore is running,

- in a different terminal, cd to the directory where rosbags are stored and type,

```
cd cakin_ws/src/beginner_tutorials/results
rosbag play recordedbag.bag
```

To play a rosbag with the listener demo: in one terminal, type,

```
cd cakin_ws/src/beginner_tutorials/results
rosbag play recordedbag.bag
```
- and in a different terminal, type,

```
cd ~/catkin_ws
rostopic echo /chatter
```

# Inspecting the bag 

To inspect a recorded bag file: cd into the directory where rosbags are stored and type,

```
rosbag info recordedbag.bag
```



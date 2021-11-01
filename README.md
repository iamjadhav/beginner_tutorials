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
* Catkin_Make Build System

## Build

1) Create a Catkin Workspace and cd into it
```
cd ~/catkin_ws
catkin_make
```
2) Clone the repository 
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

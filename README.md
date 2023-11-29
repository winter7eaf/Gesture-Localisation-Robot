# GestureCommand: A Camera-Based Gesture Recognition System for Autonomous Table-Specific Delivery Robotics

`Gesture-Localisaton-Robot` is a package of camera based hand gesture robot control system. This is a official repository of Final assignment of Intelligent Robotics Module at University of Birmingham. <br />

## Installation

Git clone this repo to your `<catkin_ws>/src` <br />
**NOTE**: *We assume catkin environment is set up already. Run `catkin_make` after all the installation.*

### Ideal Working Environment

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
(desktop-full)
- Python 3.8

### Install dependencies

Before install everything, run the upgrade command so your system stays update to date.
```
sudo apt upgrade && sudo apt update
```
- `sudo apt install ros-$ROS_DISTRO-pr2-teleop ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-map-server`.

This package mainly relies on two libraries: [Mediapipe Machine
Learning library](https://github.com/google/mediapipe) developed by Google and [OpenCV](https://github.com/opencv/opencv) Open
source Computer Vision library (for real time hand detection). <br />
- `pip install Madiapipe`
- `pip install opencv`

## Testing Simulation and Installation

If everything installed correctly, the following steps should provide
a very simplistic simulation of a robot in a provided world map.

1. In one terminal, run roscore.
2. In another, run `rosrun stage_ros stageros <catkin_ws>/src/socspioneer/data/meeting.world`.
This should start a simple simulated world with a robot and a map.
3. In a third terminal, run `roslaunch socspioneer keyboard_teleop.launch`.

This would allow you to move the robot using keyboard commands. Note that
when controlling using the keyboard control, the terminal where the
keyboard control node is running should be in focus (click on the terminal
before using the keys to control the robot).


Joint recog.:<br />


https://developers.google.com/mediapipe/solutions/vision/gesture_recognizer/python#live-stream

IMPORTANT IMPORTS:<br />
import cv2 <br />
import tensorflow as tf  # or import torch

https://github.com/ahmetgunduz/Real-time-GesRec
https://github.com/MahmudulAlam/Unified-Gesture-and-Fingertip-Detection
https://github.com/ErickWendel/live-recognizing-multiple-gestures-tensorflowjs <br />


https://github.com/kinivi/hand-gesture-recognition-mediapipe

### Nav Stack with Stage. 
Git clone the following link to your `<catkin_ws>/src` <br />
https://github.com/ros-planning/navigation_tutorial  <br />
Then run the following. Use 2D goal arrow to set the goal.
```commandline
roscore

//change in to new terminal

roslaunch navigation_stage move_base_amcl_2.5cm.launch
```
Change the last part respectively 

Install:
mediapipe
opencv

# GestureCommand: A Camera-Based Gesture Recognition System for Autonomous Table-Specific Delivery Robot

`Gesture-Localisaton-Robot` is a package of camera based hand gesture robot control system. This is the official repository of the Final assignment of Intelligent Robotics Module at University of Birmingham. <br />

![](Table5_Robot_Simulation.gif)
## Installation

### Ideal Working Environment

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
(desktop-full)
- Python 3.8

### Install ROS Noetic

[Install ROS Noetic](http://wiki.ros.org/ROS/Installation/TwoLineInstall/). <br />
[Set up your catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace). <br />

### Install dependencies

Before install everything, run the upgrade command so your system stays update to date.

- `sudo apt upgrade && sudo apt update`
- `sudo apt install ros-$ROS_DISTRO-pr2-teleop ros-$ROS_DISTRO-map-server`.

This package mainly relies on two libraries: [Mediapipe Machine
Learning library](https://github.com/google/mediapipe) developed by Google and [OpenCV](https://github.com/opencv/opencv) Open
source Computer Vision library (for real time hand detection). <br />
- `pip install mediapipe`
- `pip install opencv-python`

### Clone our repository and build

Git clone this repo to your `<catkin_ws>/src` <br />
Run `catkin_make` <br />

## Testing Simulation and Installation

Run `roslaunch Gesture-Localisation-Robot everything.launch` <br />
This should start
- the map server
- the robot simulator stage_ros
- rviz for visualisation
- pf_localisation for particle filter localisation
- move_to_coords.py for the local planner which moves the robot to the goal coordinates
- hand_track_control.py for the hand tracking and gesture recognition
- initial_pose_publisher.py for publishing the initial pose of the robot

This will allow you to input hand gesture from 0 to 5. 0 is corresponding the Till, and 1 to 5 to Tables respectfully. Hold you hand still about 3 seconds, and the robot should start heading to the ordered table number.

## Contributor

Chit Lee ([Github](https://github.com/chit-uob))<br />
Juni Katsu ([Github](https://github.com/JuniJoo))<br />
Cheuk Yu Lam ([Github](https://github.com/winter7eaf))<br />
Abbas Mandasorwala ([Github](https://github.com/abbas-119)) <br />
Kozerenko Elizaveta ([Github](https://github.com/IBMr))<br />

[//]: # (## Alternative testing with Move_base library)

[//]: # ()
[//]: # (This is testing with Path_finding library `move_base`)

[//]: # (Joint recog.:<br />)

[//]: # ()
[//]: # ()
[//]: # (https://developers.google.com/mediapipe/solutions/vision/gesture_recognizer/python#live-stream)

[//]: # ()
[//]: # (IMPORTANT IMPORTS:<br />)

[//]: # (import cv2 <br />)

[//]: # (import tensorflow as tf  # or import torch)

[//]: # ()
[//]: # (https://github.com/ahmetgunduz/Real-time-GesRec)

[//]: # (https://github.com/MahmudulAlam/Unified-Gesture-and-Fingertip-Detection)

[//]: # (https://github.com/ErickWendel/live-recognizing-multiple-gestures-tensorflowjs <br />)

[//]: # ()
[//]: # ()
[//]: # (https://github.com/kinivi/hand-gesture-recognition-mediapipe)

[//]: # ()
[//]: # (### Nav Stack with Stage. )

[//]: # (Git clone the following link to your `<catkin_ws>/src` <br />)

[//]: # (https://github.com/ros-planning/navigation_tutorial  <br />)

[//]: # (Then run the following. Use 2D goal arrow to set the goal.)

[//]: # (```commandline)

[//]: # (roscore)

[//]: # ()
[//]: # (//change in to new terminal)

[//]: # ()
[//]: # (roslaunch navigation_stage move_base_amcl_2.5cm.launch)

[//]: # (```)

[//]: # (Change the last part respectively )

[//]: # ()
[//]: # (Install:)

[//]: # (mediapipe)

[//]: # (opencv)

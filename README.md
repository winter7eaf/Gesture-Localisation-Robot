# GestureCommand: A Simulated Camera-Based Gesture Recognition System for Autonomous Table-Specific Delivery Robot

`Gesture-Localisaton-Robot` is a package of camera based hand gesture robot control system. This is the official repository of the final team assignment of Intelligent Robotics Module at University of Birmingham. <br />
A demonstration can be viewed on our [Project website](https://winter7eaf.github.io/gesture_localisation_robot/).<br />

|       Robot going Table 5        | Ordering robot to Table 5 using mediapipe |
|:--------------------------------:|:-----------------------------------------:|
| ![](Table5_Robot_Simulation.gif) |        ![](Table5_hand_recog.gif)         |

## Contributor

Chit Lee ([Github](https://github.com/chit-uob))<br />
Juni Katsu ([Github](https://github.com/JuniJoo))<br />
Cheuk Yu Lam ([Github](https://github.com/winter7eaf))<br />
Abbas Mandasorwala ([Github](https://github.com/abbas-119)) <br />
Kozerenko Elizaveta ([Github](https://github.com/Lizzzzzok))<br />

## Motivation

Imagining you are a barista in a coffee shop. You just made a cup of coffee, and you want it delivered to a customer at a certain table. You have a robot, and you put the cup of coffee on top of it. Your hands are not clean, so it will be problematic to press a touchscreen. How can you tell the robot where to go? <br />

Introducing GestureCommand, a camera-based gesture recognition system for autonomous table-specific delivery robot. You can gesture the robot where to go, and it will go there. <br />

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

### Compile laser_trace

* Compile laser_trace.cpp (provides laser ray tracing) as follows **if you are not using arm system(windows, unix...)**:

        cd <catkin_ws>/src/gesture_localisation_robot/src/laser_trace
        ./compile.sh #You may have to '''chmod +x compile.sh'''

* replace `./compile.sh` with `./compilearm.sh`  **if you are using arm system(m1 chip mac)**:

If correctly compiled, you should find `laser_trace.so` in the directory `<catkin_ws>/src/gesture_localisation_robot/src/pf_localisation`.
If the ***code does not compile*** you need to install PythonBoost from https://github.com/boostorg/python. This requires the download and compiling of Boost and installation of Faber.

### Make scripts executable

You may need to make the varies scripts executable by running `chmod +x {filename}` in the directory `<catkin_ws>/src/gesture_localisation_robot/scripts`.

## Running the Code

Run `roslaunch Gesture-Localisation-Robot everything.launch` <br />
This should start
- the map server
- the robot simulator stage_ros
- rviz for visualisation
- pf_localisation for particle filter localisation
- move_to_coords.py for the local planner which moves the robot to the goal coordinates
- hand_track_control.py for the hand tracking and gesture recognition
- initial_pose_publisher.py for publishing the initial pose of the robot

This will allow you to input hand gesture from 0 to 5. 0 is corresponding the Till, and 1 to 5 to Tables respectfully. Hold you hand still about 3 seconds, and the robot should start heading to the ordered table number. <br />
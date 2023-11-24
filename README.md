# Gesture-Localisation-Robot

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

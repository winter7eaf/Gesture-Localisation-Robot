#!/usr/bin/python3

import math
import cv2
import mediapipe as mp
import time

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion


goals = [
    Pose(position=Point(x=2.139872800488411, y=2.4091084324016747, z=0.0),
         orientation=Quaternion(w=0.9999750369921838, x=0.0, y=0.0, z=0.007065790294124714)),
    Pose(position=Point(x=1.7942427343093557, y=10.997075823076422, z=0.0),
         orientation=Quaternion(w=0.6945051411862567, x=0.0, y=0.0, z=0.7194877405945549)),
    Pose(position=Point(x=14.4578006343115, y=10.575440599835618, z=0.0),
         orientation=Quaternion(w=0.9995247774133863, x=0.0, y=0.0, z=0.03082562792094638)),
    Pose(position=Point(x=13.689259857644291, y=21.31942142179531, z=0.0),
         orientation=Quaternion(w=0.6917303017201268, x=0.0, y=0.0, z=0.722155931694937)),
    Pose(position=Point(x=17.286134723080238, y=22.362174253978868, z=0.0),
         orientation=Quaternion(w=0.9942319132574566, x=0.0, y=0.0, z=0.10725158581772617)),
    Pose(position=Point(x=12.420124525361263, y=23.583000051064765, z=0.0),
         orientation=Quaternion(w=0.2607200755719132, x=0.0, y=0.0, z=0.9654144406387217)),
    Pose(position=Point(x=4.485485112377619, y=13.62544721833074, z=0.0),
         orientation=Quaternion(w=0.9426620672762931, x=0.0, y=0.0, z=0.33374874818999023)),
    Pose(position=Point(x=12.377816896015693, y=7.411229829575238, z=0.0),
         orientation=Quaternion(w=0.5067124484646074, x=0.0, y=0.0, z=-0.8621151283738168)),
    Pose(position=Point(x=22.86934957609736, y=8.080257281520984, z=0.0),
         orientation=Quaternion(w=0.9440032004771164, x=0.0, y=0.0, z=-0.3299362930763473)),
]

Table_4 = [goals[0], goals[1], goals[2], goals[7]]
Table_5 = [goals[0], goals[1], goals[2], goals[8]]
Table_3 = [goals[0], goals[1], goals[6]]
Table_2 = [goals[0], goals[1], goals[2], goals[3], goals[5]]
Table_1 = [goals[0], goals[1], goals[2], goals[3], goals[4]]

target_path = None # will be set to one of the above tables


def send_goal(pose):
    global arrived_yet
    arrived_yet = False
    goal = Pose()
    goal.position.x = pose.position.x
    goal.position.y = pose.position.y
    goal_pub.publish(goal)
    print("published goal")

arrived_yet = False

def from_move_to_coords_callback(msg):
    global arrived_yet
    if msg.data == "finished":
        arrived_yet = True
        print("arrived")


def main():
    global goal_pub, One, Two, Three, Four, Five
    rospy.init_node('hand_track_control', anonymous=True)
    goal_pub = rospy.Publisher('/move_to_goal', Pose, queue_size=10)
    rospy.Subscriber('/move_to_coords', String, from_move_to_coords_callback)

    while True:
        answer = input("Destination: ")
        if answer == '1':
            target_path = Table_1
        elif answer == '2':
            target_path = Table_2
        elif answer == '3':
            target_path = Table_3
        elif answer == '4':
            target_path = Table_4
        elif answer == '5':
            target_path = Table_5
        elif answer == '0':
            target_path = reversed(target_path)
        else:
            print("Invalid input")
            continue


        for pose in target_path:
            send_goal(pose)
            while not arrived_yet:
                rospy.sleep(2)
        rospy.sleep(2)

        print("Finished path, now wait for next input to go back")


if __name__ == "__main__":
    main()

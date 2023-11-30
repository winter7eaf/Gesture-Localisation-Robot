#!/usr/bin/python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String


rospy.init_node('navigate_robot')

# Updated list of goals with your coordinates
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

Table_4 = [goals[1], goals[2], goals[7]]
Table_5 = [goals[1], goals[2], goals[8]]
Table_3 = [goals[1], goals[6]]
Table_2 = [goals[1], goals[2], goals[3], goals[5]]
Table_1 = [goals[1], goals[2], goals[3], goals[4]]

goal_pub = rospy.Publisher('/move_to_goal', Pose, queue_size=10)

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

rospy.Subscriber('/move_to_coords', String, from_move_to_coords_callback)


answer = input("Destination: ")
if answer == '1':
    for pose in Table_1:
        result = send_goal(pose)
        while not arrived_yet:
            rospy.sleep(1)
    rospy.sleep(1)
elif answer == '2':
    for pose in Table_2:
        result = send_goal(pose)
        while not arrived_yet:
            rospy.sleep(1)
    rospy.sleep(1)
elif answer == '3':
    for pose in Table_3:
        result = send_goal(pose)
        while not arrived_yet:
            rospy.sleep(1)
    rospy.sleep(1)
elif answer == '4':
    for pose in Table_4:
        result = send_goal(pose)
        while not arrived_yet:
            rospy.sleep(1)
    rospy.sleep(1)
elif answer == '5':
    for pose in Table_5:
        result = send_goal(pose)
        while not arrived_yet:
            rospy.sleep(1)
    rospy.sleep(1)
elif answer == 'home':
    home = send_goal(goals[0])
    while not arrived_yet:
        rospy.sleep(1)
    rospy.sleep(1)

rospy.spin()

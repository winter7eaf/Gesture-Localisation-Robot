#!/usr/bin/python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

rospy.init_node('navigate_robot')

# Updated list of goals with your coordinates
goals = [
    Pose(position=Point(x=3.039054575402686, y=3.739731769616226, z=0.0),
         orientation=Quaternion(w=0.8337602955527833, x=0.0, y=0.0, z=0.5521265883470342)),
    Pose(position=Point(x=1.9327985587137941, y=11.423447682767932, z=0.0),
         orientation=Quaternion(w=0.9201964048699748, x=0.0, y=0.0, z=0.39145699184504723)),
    Pose(position=Point(x=15.11636640422266, y=11.425471330763873, z=0.0),
         orientation=Quaternion(w=0.997910297702185, x=0.0, y=0.0, z=0.06461453195633741)),
    Pose(position=Point(x=12.905579134804732, y=7.9848322620675525, z=0.0),
         orientation=Quaternion(w=0.980740448390913, x=0.0, y=0.0, z=-0.19531557257420862)),
    Pose(position=Point(x=9.504558737679664, y=12.853681794445112, z=0.0),
         orientation=Quaternion(w=0.9478868168720372, x=0.0, y=0.0, z=-0.31860725415501284)),
    Pose(position=Point(x=14.295679704866666, y=21.880189496153275, z=0.0),
         orientation=Quaternion(w=0.9999562584662598, x=0.0, y=0.0, z=0.009353136060097923)),
    Pose(position=Point(x=12.955258052470636, y=23.688073509473423, z=0.0),
         orientation=Quaternion(w=0.9775336959333584, x=0.0, y=0.0, z=0.2107792051291313)),
    Pose(position=Point(x=16.279586618366, y=26.657067088902657, z=0.0),
         orientation=Quaternion(w=0.9268607199649388, x=0.0, y=0.0, z=0.37540538859488315)),
    Pose(position=Point(x=16.958741068978618, y=21.78485755495617, z=0.0),
         orientation=Quaternion(w=0.9578953891383724, x=0.0, y=0.0, z=-0.28711743845932836)),
    Pose(position=Point(x=18.948990828185984, y=17.857989217411323, z=0.0),
         orientation=Quaternion(w=0.9674684317201359, x=0.0, y=0.0, z=-0.25299176592328215)),
    Pose(position=Point(x=26.60789029586225, y=17.77152826708097, z=0.0),
         orientation=Quaternion(w=0.9999915213696485, x=0.0, y=0.0, z=0.004117910734322616)),
    Pose(position=Point(x=27.33880758748402, y=27.158517770018815, z=0.0),
         orientation=Quaternion(w=0.9264104237715857, x=0.0, y=0.0, z=-0.376515241029299))
    # You can add more poses here in the same format
]

Table_4 = [goals[1], goals[2], goals[3]]
Table_5 = [goals[1], goals[2], goals[4]]
Table_3 = [goals[1], goals[2], goals[5]]
Table_2 = [goals[1], goals[2], goals[6], goals[7]]
Table_1 = [goals[1], goals[2], goals[6], goals[9]]

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()


def send_goal(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # or your robot's frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
    else:
        return client.get_result()


# for pose in goals:
#     result = send_goal(pose)
#     if result:
#         rospy.loginfo("Goal achieved!")
#     else:
#         rospy.loginfo("Failed to reach goal.")

answer = input("Destination: ")
if answer == '1':
    for pose in Table_1:
        result = send_goal(pose)
        if result:
            rospy.loginfo("Goal achieved!")
        else:
            rospy.loginfo("Failed to reach goal.")
    rospy.slee(10)
    home = send_goal(goals[0])
    if home:
        rospy.loginfo("home achieved!")
    else:
        rospy.loginfo("Failed to reach home.")
elif answer == '2':
    for pose in Table_2:
        result = send_goal(pose)
        if result:
            rospy.loginfo("Goal achieved!")
        else:
            rospy.loginfo("Failed to reach goal.")
    rospy.sleep(10)
    home = send_goal(goals[0])
    if home:
        rospy.loginfo("home achieved!")
    else:
        rospy.loginfo("Failed to reach home.")
elif answer == '3':
    for pose in Table_3:
        result = send_goal(pose)
        if result:
            rospy.loginfo("Goal achieved!")
        else:
            rospy.loginfo("Failed to reach goal.")
    rospy.sleep(10)
    home = send_goal(goals[0])
    if home:
        rospy.loginfo("home achieved!")
    else:
        rospy.loginfo("Failed to reach home.")
elif answer == '4':
    for pose in Table_4:
        result = send_goal(pose)
        if result:
            rospy.loginfo("Goal achieved!")
        else:
            rospy.loginfo("Failed to reach goal.")
    rospy.sleep(10)
    home = send_goal(goals[0])
    if home:
        rospy.loginfo("home achieved!")
    else:
        rospy.loginfo("Failed to reach home.")
elif answer == '5':
    for pose in Table_5:
        result = send_goal(pose)
        if result:
            rospy.loginfo("Goal achieved!")
        else:
            rospy.loginfo("Failed to reach goal.")
    rospy.sleep(10)
    home = send_goal(goals[0])
    if home:
        rospy.loginfo("home achieved!")
    else:
        rospy.loginfo("Failed to reach home.")

rospy.spin()

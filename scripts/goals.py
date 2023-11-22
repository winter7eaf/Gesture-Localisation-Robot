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
    # You can add more poses here in the same format
]

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

for pose in goals:
    result = send_goal(pose)
    if result:
        rospy.loginfo("Goal achieved!")
    else:
        rospy.loginfo("Failed to reach goal.")
rospy.spin()

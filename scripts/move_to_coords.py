#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import time
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String


COORD_TO_MOVE_TO = None
message = None
TOLERANCE = 0.2
DISTANCE_TOLERANCE = 0.5
told_about_finished = False


def pose_callback(msg):
    global message
    message = msg


def move_to_goal_callback(msg):
    global COORD_TO_MOVE_TO, told_about_finished
    COORD_TO_MOVE_TO = (msg.position.x, msg.position.y)
    told_about_finished = False
    print("got new goal")


def move():
    global message, told_about_finished
    if not message:
        return
    if not COORD_TO_MOVE_TO:
        return
    # coords
    x, y = message.pose.pose.position.x, message.pose.pose.position.y
    yaw = getHeading(message.pose.pose.orientation)

    # calculate the distance to the target
    distance = math.sqrt((COORD_TO_MOVE_TO[0] - x) ** 2 + (COORD_TO_MOVE_TO[1] - y) ** 2)
    if distance < DISTANCE_TOLERANCE:
        if not told_about_finished:
            string_data = String()
            string_data.data = "finished"
            move_to_coords_pub.publish(string_data)
            told_about_finished = True
        return

    # calculate the yaw to the target
    target_yaw = math.atan2(COORD_TO_MOVE_TO[1] - y, COORD_TO_MOVE_TO[0] - x)

    if (target_yaw - yaw) > TOLERANCE:
        # turn left
        base_data = Twist()
        base_data.angular.z = 0.3
        # base_data.linear.x = 0.2
        movement_pub.publish(base_data)
    elif (target_yaw - yaw) < -TOLERANCE:
        # turn right
        base_data = Twist()
        base_data.angular.z = -0.3
        # base_data.linear.x = 0.2
        movement_pub.publish(base_data)
    else:
        # move forward
        base_data = Twist()
        base_data.linear.x = 0.4
        movement_pub.publish(base_data)



def main():
    rospy.init_node('move_to_coords')
    global movement_pub, move_to_coords_pub
    movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    move_to_coords_pub = rospy.Publisher('/move_to_coords', String, queue_size=100)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_to_goal', Pose, move_to_goal_callback)

    while not rospy.is_shutdown():
        move()
        time.sleep(0.1)

    rospy.spin()
def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.
    
    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0
 
    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)
 
    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # ----- Multiply new (heading-only) quaternion by the existing (pitch and bank) 
    # ----- quaternion. Order is important! Original orientation is the second 
    # ----- argument rotation which will be applied to the quaternion is the first 
    # ----- argument. 
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions( qa, qb ):
    """
    Multiplies two quaternions to give the rotation of qb by qa.
    
    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()
    
    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.
    
    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

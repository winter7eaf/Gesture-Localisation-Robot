#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion, Pose
import math
import time
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Constants for obstacle distances
checkf_distance = 0.15
checks_distance = 0.2

# ROS publishers and subscribers
cmd_vel_pub = None
front_sensor_sub = None
left_sensor_sub = None
right_sensor_sub = None
odom_sub = None

# Robot state
x, y, theta = 0.0, 0.0, 0.0
front_distance = -999
left_distance = 0.0
right_distance = 0.0
state = "BEGIN"
pledge_number = 0  # Total rotation done, see Pledge algorithm


# ROS callback functions
def front_sensor_callback(range_msg):
    global front_distance
    front_distance = range_msg.range


def right_sensor_callback(range_msg):
    global right_distance
    right_distance = range_msg.range


def left_sensor_callback(range_msg):
    global left_distance
    left_distance = range_msg.range

def left_front_right_distance(scan_data):
    """
    Analyzes the LIDAR scan data to find the clearest direction.
    """
    # print(scan_data)
    num_ranges = len(scan_data.ranges)
    # print(num_ranges)
    segment_size = int(30 / 360 * num_ranges)
    # print((segment_size))

    right_segment = scan_data.ranges[:segment_size]
    left_segment = scan_data.ranges[-segment_size:]
    front_segment = scan_data.ranges[num_ranges//2 - segment_size//2:num_ranges//2 + segment_size//2]
    # print(left_segment)
    # print(right_segment)
    # print(front_segment)

    avg_distance_left = sum(left_segment) / len(left_segment)
    avg_distance_right = sum(right_segment) / len(right_segment)
    avg_distance_front = sum(front_segment) / len(front_segment)

    print("left", avg_distance_left, "front", avg_distance_front, "right", avg_distance_right)

    return avg_distance_left, avg_distance_right, avg_distance_front
def odom_callback(odom_msg):
    global x, y, theta
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    orientation_q = odom_msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, theta) = euler_from_quaternion(orientation_list)

def scan_data_callback(msg):
    global scan_data
    scan_data = msg

# State functions
def state_begin():
    global state
    msg = Twist()
    if front_distance < checkf_distance:
        state = "TURNRIGHT"
    msg.linear.x = 1.0
    cmd_vel_pub.publish(msg)


def state_following():
    global state
    msg = Twist()

    # State transitions
    if front_distance < checkf_distance:
        state = "TURNRIGHT"
        return
    if left_distance > checks_distance:
        state = "TURNLEFT"
        return

    # Add some correction if too close to the wall on the left
    if left_distance < checks_distance / 2 + 0.02:
        msg.angular.z = -0.4
        msg.linear.x = 0.06
    else:
        msg.linear.x = 1.0

    cmd_vel_pub.publish(msg)

def state_turn_right():
    global state
    msg = Twist()

    # State transition
    if front_distance > 2 * checkf_distance:
        state = "FOLLOWING"

    msg.angular.z = -0.4
    # Uncomment the following line if you want the robot to move forward while turning
    # msg.linear.x = 0.06
    cmd_vel_pub.publish(msg)

def state_turn_left():
    global state
    msg = Twist()

    # State transition
    if left_distance <= checks_distance:
        state = "FOLLOWING"

    msg.angular.z = 0.4
    msg.linear.x = 0.06
    cmd_vel_pub.publish(msg)


# Calculate and send commands based on state
def calculate_command():
    global state
    rospy.loginfo("Current state is %s", state)
    if state == "BEGIN":
        state_begin()
    elif state == "FOLLOWING":
        state_following()
    elif state == "TURNRIGHT":
        state_turn_right()
    elif state == "TURNLEFT":
        state_turn_left()


# Initialize ROS node
def main():
    global cmd_vel_pub, front_sensor_sub, left_sensor_sub, right_sensor_sub, odom_sub
    rospy.init_node('reactive_controller')

    # Initialize publishers and subscribers
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    rospy.Subscriber('/base_scan', LaserScan, scan_data_callback)
    move_to_coords_pub = rospy.Publisher('/move_to_coords', String, queue_size=100)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_to_goal', Pose, move_to_goal_callback)
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        calculate_command()
        rate.sleep()

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

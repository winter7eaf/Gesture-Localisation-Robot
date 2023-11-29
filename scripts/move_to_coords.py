#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import time
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

ANGLE_TOLERANCE = 0.3
DISTANCE_TOLERANCE = 0.5
OBSTACLE_DISTANCE_THRESHOLD = 0.7  # meters for obstacle detection
TURNING_SPEED = 1.0  # Speed at which the robot turns for obstacle avoidance
FORWARD_SPEED = 1.0  # Forward movement speed towards the goal

coord_to_move_to = None
pose_cache = None
scan_cache = None
told_about_finished = False


def pose_callback(msg):
    global pose_cache
    pose_cache = msg


def move_to_goal_callback(msg):
    global coord_to_move_to, told_about_finished
    coord_to_move_to = (msg.position.x, msg.position.y)
    told_about_finished = False
    print("got new goal")


def scan_data_callback(msg):
    global scan_cache
    scan_cache = msg


def find_clear_direction_v2(scan_data):
    # get the distance of in front of the robot
    distance_front = scan_data.ranges[len(scan_data.ranges) // 2]
    # get the distance of the 30 degrees to the left of the robot
    distance_right = scan_data.ranges[len(scan_data.ranges) // 3]
    # get the distance of the 30 degrees to the right of the robot
    distance_left = scan_data.ranges[len(scan_data.ranges) // 3 * 2]

    distance = min(distance_front, distance_left, distance_right)

    if distance > OBSTACLE_DISTANCE_THRESHOLD:
        return 'front'

    # check average distance of each side
    num_ranges = len(scan_data.ranges)
    avg_distance_right = sum(scan_data.ranges[:num_ranges // 2]) / (num_ranges // 2)
    avg_distance_left = sum(scan_data.ranges[num_ranges // 2:]) / (num_ranges // 2)
    if avg_distance_left > avg_distance_right:
        return 'left'
    else:
        return 'right'


def move():
    global pose_cache, told_about_finished, scan_cache
    if not pose_cache:
        return
    if not coord_to_move_to:
        return

    # coords
    x, y = pose_cache.pose.pose.position.x, pose_cache.pose.pose.position.y
    yaw = getHeading(pose_cache.pose.pose.orientation)

    # calculate the distance to the target
    distance = math.sqrt((coord_to_move_to[0] - x) ** 2 + (coord_to_move_to[1] - y) ** 2)
    # calculate the yaw to the target
    target_yaw = math.atan2(coord_to_move_to[1] - y, coord_to_move_to[0] - x)

    if scan_cache:
        clear_direction = find_clear_direction_v2(scan_cache)
    else:
        print("no scan data")
        clear_direction = 'front'
    twist = Twist()

    if distance < DISTANCE_TOLERANCE:
        if not told_about_finished:
            string_data = String()
            string_data.data = "finished"
            move_to_coords_pub.publish(string_data)
            told_about_finished = True
        return

    if clear_direction != 'front':
        # If there's an obstacle, turn towards the clearest direction
        print(f"doing evasive action: {clear_direction}")
        if clear_direction == 'left':
            twist.angular.z = TURNING_SPEED * 3
            twist.linear.x = FORWARD_SPEED / 2
        elif clear_direction == 'right':
            twist.angular.z = -TURNING_SPEED * 3
            twist.linear.x = FORWARD_SPEED / 2
    else:
        # If the path is clear, adjust heading towards the target
        angle_diff = target_yaw - yaw
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        if abs(angle_diff) > ANGLE_TOLERANCE:
            twist.angular.z = TURNING_SPEED if angle_diff > 0 else -TURNING_SPEED
        else:
            # Move forward if facing the target
            twist.linear.x = FORWARD_SPEED

    movement_pub.publish(twist)


def main():
    rospy.init_node('move_to_coords')
    global movement_pub, move_to_coords_pub
    movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    move_to_coords_pub = rospy.Publisher('/move_to_coords', String, queue_size=100)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_to_goal', Pose, move_to_goal_callback)
    rospy.Subscriber('/base_scan', LaserScan, scan_data_callback)

    while not rospy.is_shutdown():
        move()
        time.sleep(0.2)

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


def multiply_quaternions(qa, qb):
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

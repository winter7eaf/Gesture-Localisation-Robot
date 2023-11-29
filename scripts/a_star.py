#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import time
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


COORD_TO_MOVE_TO = None
pose_message = None
TOLERANCE = 0.2
DISTANCE_TOLERANCE = 0.5
told_about_finished = False
scan_data = None

OBSTACLE_DISTANCE_THRESHOLD = 1.0  # meters for obstacle detection
TURNING_SPEED = 0.5  # Speed at which the robot turns for obstacle avoidance
FORWARD_SPEED = 0.6  # Forward movement speed towards the goal
ANGLE_RANGE = 30  # Angle range to consider for each direction (in degrees for obstacle detection)

def pose_callback(msg):
    global pose_message
    message = msg


def move_to_goal_callback(msg):
    global COORD_TO_MOVE_TO, told_about_finished
    COORD_TO_MOVE_TO = (msg.position.x, msg.position.y)
    told_about_finished = False
    print("got new goal")

def scan_data_callback(msg):
    global scan_data
    scan_data = msg

def find_clear_direction(scan_data):
    """
    Analyzes the LIDAR scan data to find the clearest direction.
    """
    # print(scan_data)
    num_ranges = len(scan_data.ranges)
    print(num_ranges)
    segment_size = int(ANGLE_RANGE / 360 * num_ranges)
    print((segment_size))

    left_segment = scan_data.ranges[:segment_size]
    right_segment = scan_data.ranges[-segment_size:]
    front_segment = scan_data.ranges[num_ranges//2 - segment_size//2:num_ranges//2 + segment_size//2]
    print(left_segment)
    print(right_segment)
    print(front_segment)

    min_distance_left = min(left_segment)
    min_distance_right = min(right_segment)
    min_distance_front = min(front_segment)
    print(min_distance_left)
    print(min_distance_right)
    print(min_distance_front)

    if min_distance_front > OBSTACLE_DISTANCE_THRESHOLD:
        print("going front")
        return 'front'
    elif min_distance_left < min_distance_right:
        print("going left")
        return 'left'
    else:
        print("going right")
        return 'right'


import numpy as np
import heapq


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance


def a_star_search(grid, start, goal):
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4-directional movement
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    open_set = []

    heapq.heappush(open_set, (fscore[start], start))

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Reverse path

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in open_set]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (fscore[neighbor], neighbor))

    return False

# ... [Your existing imports and global variables]

def update_grid_based_on_sensor_data():
    # Update your grid map based on sensor data
    # This is an example, implement according to your robot's sensor data
    grid = np.zeros((100, 100))  # Example grid
    # Update grid with obstacles (set cells to 1 where obstacles are detected)
    return grid

def convert_path_to_movement_commands(path):
    # Convert the A* path to robot movement commands
    for point in path:
        # Convert grid point to real-world coordinates
        # Send movement command to the robot
        pass

def move():
    global pose_message, told_about_finished, scan_data
    if not message:
        print("no pose")
        return
    if not COORD_TO_MOVE_TO:
        print("no coords")
        return
    # coords
    x, y = message.pose.pose.position.x, message.pose.pose.position.y
    yaw = getHeading(message.pose.pose.orientation)

    # calculate the distance to the target
    distance = math.sqrt((COORD_TO_MOVE_TO[0] - x) ** 2 + (COORD_TO_MOVE_TO[1] - y) ** 2)
    # calculate the yaw to the target
    target_yaw = math.atan2(COORD_TO_MOVE_TO[1] - y, COORD_TO_MOVE_TO[0] - x)

    if scan_data:
        clear_direction = find_clear_direction(scan_data)
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
        twist.angular.z = TURNING_SPEED*2 if clear_direction == 'left' else -TURNING_SPEED
        twist.linear.x = FORWARD_SPEED
    else:
        # If the path is clear, adjust heading towards the target
        angle_diff = target_yaw - yaw
        if abs(angle_diff) > TOLERANCE:
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

    # while not rospy.is_shutdown():
    #     if message and COORD_TO_MOVE_TO:
    #         move()
    #     time.sleep(0.1)
    #
    # rospy.spin()

    while not rospy.is_shutdown():
        grid = update_grid_based_on_sensor_data()
        start = (0, 0)  # Starting point in grid coordinates
        goal = (99, 99)  # Goal point in grid coordinates
        path = a_star_search(grid, start, goal)
        if path:
            convert_path_to_movement_commands(path)
        # ... [Rest of your loop]
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

#!/usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


DISTANCE_THRESHOLD = 1


def laser_callback(msg):
    # get the distance of in front of the robot
    distance_front = msg.ranges[len(msg.ranges) // 2]
    # get the distance of the 30 degrees to the left of the robot
    distance_left = msg.ranges[len(msg.ranges) // 3]
    # get the distance of the 30 degrees to the right of the robot
    distance_right = msg.ranges[len(msg.ranges) // 3 * 2]

    distance = min(distance_front, distance_left, distance_right)

    # if distance is greater than 1 meter move forward
    if distance > DISTANCE_THRESHOLD:
        base_data = Twist()
        base_data.linear.x = 0.5
        pub.publish(base_data)

    # if distance is less than 1 meter turn right
    if distance < DISTANCE_THRESHOLD:
        base_data = Twist()
        base_data.angular.z = 0.5
        pub.publish(base_data)


def main():
    rospy.init_node('move')
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    rospy.Subscriber('/base_scan', LaserScan, laser_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

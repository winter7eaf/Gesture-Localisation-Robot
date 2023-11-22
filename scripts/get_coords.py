#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def pose_callback(msg):
    # print the coordinates of the robot
    print("coords: ")
    print("x: ", msg.pose.pose.position.x)
    print("y: ", msg.pose.pose.position.y)
    print("z: ", msg.pose.pose.position.z)
    print("orientation: ")
    print("ow: ", msg.pose.pose.orientation.w)
    print("ox: ", msg.pose.pose.orientation.x)
    print("oy: ", msg.pose.pose.orientation.y)
    print("oz: ", msg.pose.pose.orientation.z)



def main():
    rospy.init_node('get_coords')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

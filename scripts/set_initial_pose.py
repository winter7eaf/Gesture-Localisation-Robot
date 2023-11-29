#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose_estimate():
    rospy.init_node('initial_pose_publisher')
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    pose = PoseWithCovarianceStamped()

    # target pose
    # header:
    #   seq: 1
    #   stamp:
    #     secs: 38
    #     nsecs: 100000000
    #   frame_id: "map"
    # pose:
    #   pose:
    #     position:
    #       x: 2.2427940368652344
    #       y: 2.572723388671875
    #       z: 0.0
    #     orientation:
    #       x: 0.0
    #       y: 0.0
    #       z: -0.007397551138444635
    #       w: 0.9999726377442305
    #   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

    pose.header.seq = 1
    pose.header.stamp.secs = 38
    pose.header.stamp.nsecs = 100000000
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = 2.2427940368652344
    pose.pose.pose.position.y = 2.572723388671875
    pose.pose.pose.position.z = 0.0
    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y = 0.0
    pose.pose.pose.orientation.z = -0.007397551138444635
    pose.pose.pose.orientation.w = 0.9999726377442305
    pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

    time.sleep(1)
    pub.publish(pose)

if __name__ == '__main__':
    try:
        publish_initial_pose_estimate()
    except rospy.ROSInterruptException:
        pass
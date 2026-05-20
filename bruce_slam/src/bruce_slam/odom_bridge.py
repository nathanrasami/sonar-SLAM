#!/usr/bin/env python
"""Bridge node: converts /odom_pose (PoseStamped) from aracati2017 to nav_msgs/Odometry
on LOCALIZATION_ODOM_TOPIC so bruce_slam/slam.py can consume it without DVL/IMU."""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from bruce_slam.utils.topics import LOCALIZATION_ODOM_TOPIC, ODOM_BRIDGE_INPUT_TOPIC


class OdomBridge:
    def __init__(self):
        self.pub = rospy.Publisher(LOCALIZATION_ODOM_TOPIC, Odometry, queue_size=10)
        rospy.Subscriber(ODOM_BRIDGE_INPUT_TOPIC, PoseStamped, self.callback, queue_size=10)

    def callback(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose = msg.pose
        # Fixed covariance — larger than DVL/IMU since this is wheel odometry
        cov = [0.5, 0, 0, 0, 0, 0,
               0, 0.5, 0, 0, 0, 0,
               0, 0, 0.5, 0, 0, 0,
               0, 0, 0, 0.1, 0, 0,
               0, 0, 0, 0, 0.1, 0,
               0, 0, 0, 0, 0, 0.1]
        odom.pose.covariance = cov
        self.pub.publish(odom)


def main():
    rospy.init_node("odom_bridge")
    OdomBridge()
    rospy.spin()


if __name__ == "__main__":
    main()

#!/usr/bin/env python
"""Pont d'odométrie minimal pour Aracati2017.

Aracati n'a ni IMU ni DVL : on substitue l'odométrie en intégrant /cmd_vel (vitesses
commandées, modèle unicycle) en pose absolue, publiée en nav_msgs/Odometry sur
LOCALIZATION_ODOM_TOPIC — le topic d'odométrie attendu par le SLAM de Bruce.

/cmd_vel fournit linear.x (vx) et angular.z (wz, dérivé du compas du véhicule).
Intégration Euler avant (REP-103), base de temps = header.stamp du bag :
    x += vx·cos(θ)·dt ; y += vx·sin(θ)·dt ; θ += wz·dt
Seed (0,0,0) : l'évaluation ATE aligne par Umeyama, le point de départ absolu n'importe pas.
"""
import math
import threading
import rospy
import tf
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from bruce_slam.utils.topics import LOCALIZATION_ODOM_TOPIC

CMD_VEL_TOPIC = "/cmd_vel"


class CmdVelOdom:
    """Intègre /cmd_vel → pose et publie une Odometry pour le SLAM."""

    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.last_t = None
        self.lock = threading.Lock()

        self.pub = rospy.Publisher(LOCALIZATION_ODOM_TOPIC, Odometry, queue_size=10)
        rospy.Subscriber(CMD_VEL_TOPIC, TwistStamped, self._cmd_cb, queue_size=50)
        rospy.loginfo("cmd_vel_odom : %s -> %s (Odometry)",
                      CMD_VEL_TOPIC, LOCALIZATION_ODOM_TOPIC)

    def _cmd_cb(self, msg: TwistStamped) -> None:
        t = msg.header.stamp.to_sec()
        vx = msg.twist.linear.x
        wz = msg.twist.angular.z
        with self.lock:
            if self.last_t is not None:
                dt = t - self.last_t
                if dt > 0:
                    self.x += vx * math.cos(self.theta) * dt
                    self.y += vx * math.sin(self.theta) * dt
                    self.theta += wz * dt
            self.last_t = t
            self._publish(msg.header.stamp, vx, wz)

    def _publish(self, stamp, vx, wz) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        # twist (Bruce lit frame.twist) : vitesses corps
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.pub.publish(odom)

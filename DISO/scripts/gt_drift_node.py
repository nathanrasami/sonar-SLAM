#!/usr/bin/env python
"""Ablation GT pour DISO : republie /pose_gt en y injectant une DÉRIVE connue,
sur /pose_gt_drift, que DISO consomme comme OdomTopic.

But : DISO sur Aracati utilise /pose_gt (la vérité terrain) comme prior d'odométrie
(config OdomTopic, System.cpp frameLoad). On teste donc si DISO est une vraie
odométrie SONAR ou s'il dépend de l'exactitude de la GT :
  - si la trajectoire DISO reste proche de la GT malgré la dérive injectée
    => le sonar corrige => odométrie sonar réelle (GT n'aide qu'à l'init/scale).
  - si la trajectoire DISO suit la dérive injectée
    => DISO recopie le prior => le "2 m" vient de la GT, pas du sonar.

Republie avec le MÊME header.stamp (synchro ApproximateTime parfaite) et dans le
MÊME repère que /pose_gt (GT ± offset) → aucun risque de convention de repère.

Paramètres (rosparam ~) :
  lateral_drift_rate : dérive latérale en m/s (défaut 0.003 → ~8 m sur 2640 s)
  yaw_bias_deg       : biais de cap constant en degrés (défaut 3.0)
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply


class GTDrift:
    def __init__(self):
        self.rate = rospy.get_param("~lateral_drift_rate", 0.003)   # m/s
        self.yaw_bias = math.radians(rospy.get_param("~yaw_bias_deg", 3.0))
        self.t0 = None
        self.pub = rospy.Publisher("/pose_gt_drift", PoseStamped, queue_size=50)
        rospy.Subscriber("/pose_gt", PoseStamped, self.cb, queue_size=50)
        rospy.loginfo("gt_drift: lateral=%.4f m/s, yaw_bias=%.1f deg",
                      self.rate, math.degrees(self.yaw_bias))

    def cb(self, msg):
        t = msg.header.stamp.to_sec()
        if self.t0 is None:
            self.t0 = t
        dt = t - self.t0

        out = PoseStamped()
        out.header = msg.header  # même stamp + frame → synchro et repère préservés
        out.pose.position.x = msg.pose.position.x
        out.pose.position.y = msg.pose.position.y + self.rate * dt  # dérive latérale
        out.pose.position.z = msg.pose.position.z

        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        dq = quaternion_from_euler(0.0, 0.0, self.yaw_bias)         # biais de cap
        nq = quaternion_multiply(dq, q)
        out.pose.orientation.x, out.pose.orientation.y, \
            out.pose.orientation.z, out.pose.orientation.w = nq
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("gt_drift")
    GTDrift()
    rospy.spin()

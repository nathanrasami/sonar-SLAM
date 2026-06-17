#!/usr/bin/env python
"""Intègre /cmd_vel (vitesses commandées, unicycle) en pose absolue et la publie
sur ODOM_BRIDGE_INPUT_TOPIC (PoseStamped) — odométrie INDÉPENDANTE de la GT.

Drop-in du relay GT : OdomBridge lit /direct_sonar/pose à l'identique. La GT
(/pose_gt) ne sert qu'à seeder position + cap à t=0 ; ensuite on intègre /cmd_vel
seul → dérive réaliste (≈15 m ATE Umeyama mesuré). C'est au SSM + loop closures
de rattraper cette dérive.

Modèle unicycle 2D : sur Aracati seules linear.x (vx) et angular.z (wz) de
/cmd_vel sont non nulles (vérifié sur le bag), donc x+=vx·cosθ·dt, y+=vx·sinθ·dt,
θ+=wz·dt (Euler avant, base de temps = header.stamp du bag).
"""
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped

from bruce_slam.utils.topics import ODOM_BRIDGE_INPUT_TOPIC

CMD_VEL_TOPIC = "/cmd_vel"
GT_TOPIC = "/pose_gt"


class CmdVelOdom:
    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.seeded = False
        self.last_t = None
        self.pub = rospy.Publisher(ODOM_BRIDGE_INPUT_TOPIC, PoseStamped, queue_size=10)
        # GT uniquement pour la 1ère pose (cf. _seed_cb), puis ignorée
        rospy.Subscriber(GT_TOPIC, PoseStamped, self._seed_cb, queue_size=1)
        rospy.Subscriber(CMD_VEL_TOPIC, TwistStamped, self._cmd_cb, queue_size=50)

    def _seed_cb(self, msg: PoseStamped) -> None:
        """Initialise position + cap depuis la 1ère GT reçue, puis ne fait plus rien."""
        if self.seeded:
            return
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.seeded = True
        rospy.loginfo("cmd_vel_odom seedé sur GT : x=%.2f y=%.2f yaw=%.3f rad",
                      self.x, self.y, self.theta)

    def _cmd_cb(self, msg: TwistStamped) -> None:
        if not self.seeded:
            return  # on attend le seed GT (cmd_vel démarre ~0.5 s avant /pose_gt)
        t = msg.header.stamp.to_sec()
        if self.last_t is None:
            self.last_t = t
            self._publish(msg.header.stamp)  # publie la pose seedée
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0:
            return
        vx = msg.twist.linear.x
        wz = msg.twist.angular.z
        # Euler avant : position au cap courant, puis mise à jour du cap
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt
        self.theta += wz * dt
        self._publish(msg.header.stamp)

    def _publish(self, stamp) -> None:
        out = PoseStamped()
        out.header.stamp = stamp  # base de temps du bag → ATE associable
        out.header.frame_id = "map"
        out.pose.position.x = self.x
        out.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        out.pose.orientation.x = q[0]
        out.pose.orientation.y = q[1]
        out.pose.orientation.z = q[2]
        out.pose.orientation.w = q[3]
        self.pub.publish(out)

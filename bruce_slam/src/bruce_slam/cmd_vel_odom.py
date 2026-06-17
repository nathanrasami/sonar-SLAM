#!/usr/bin/env python
"""Intègre /cmd_vel (vitesses commandées, unicycle) en pose absolue et la publie
sur ODOM_BRIDGE_INPUT_TOPIC (PoseStamped) — odométrie INDÉPENDANTE de la GT.

Drop-in du relay GT : OdomBridge lit /direct_sonar/pose à l'identique. La GT
(/pose_gt) ne sert qu'à seeder position + cap à t=0 ; ensuite on intègre /cmd_vel
seul → dérive réaliste (≈11.5 m ATE Umeyama mesuré). C'est aux loop closures de
rattraper cette dérive.

Modèle unicycle 2D : sur Aracati seules linear.x (vx) et angular.z (wz) de
/cmd_vel sont non nulles (vérifié sur le bag), donc x+=vx·cosθ·dt, y+=vx·sinθ·dt,
θ+=wz·dt (Euler avant, base de temps = header.stamp du bag).

OPTION USBL (~use_usbl=True) : fusionne les fixes acoustiques /usbl_point (≈1.4 m
médian vs GT, INDÉPENDANTS de la GT) par filtre complémentaire en position. Ancre
la dérive sans recourir à la GT. Rejet d'outliers (glitches ≈73 m) par gate de
vitesse vs le dernier fix accepté — indépendant de la dérive de cmd_vel.
"""
import math
import threading
import rospy
import tf
from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped

from bruce_slam.utils.topics import ODOM_BRIDGE_INPUT_TOPIC

CMD_VEL_TOPIC = "/cmd_vel"
GT_TOPIC = "/pose_gt"
USBL_TOPIC = "/usbl_point"


class CmdVelOdom:
    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.seeded = False
        self.last_t = None
        self.lock = threading.Lock()
        self.pub = rospy.Publisher(ODOM_BRIDGE_INPUT_TOPIC, PoseStamped, queue_size=10)
        # GT uniquement pour la 1ère pose (cf. _seed_cb), puis ignorée
        rospy.Subscriber(GT_TOPIC, PoseStamped, self._seed_cb, queue_size=1)
        rospy.Subscriber(CMD_VEL_TOPIC, TwistStamped, self._cmd_cb, queue_size=50)

        # Fusion USBL optionnelle (ancrage absolu acoustique, indépendant de GT)
        self.use_usbl = rospy.get_param("~use_usbl", False)
        # K du filtre complémentaire. τ≈1.6/K s : K=0.1 moyenne le bruit USBL
        # (~1.4 m) sur ~10 fixes sans zigzag, tout en suivant la dérive lente de
        # cmd_vel. K=0.5 testé → trajet 2.57× trop long (dents-de-scie).
        self.usbl_gain = rospy.get_param("~usbl_gain", 0.1)
        self.usbl_max_speed = rospy.get_param("~usbl_max_speed", 3.0)  # m/s, gate outliers
        self.last_usbl = None  # (t, x, y) du dernier fix ACCEPTÉ
        self.usbl_kept = self.usbl_rejected = 0
        if self.use_usbl:
            rospy.Subscriber(USBL_TOPIC, PointStamped, self._usbl_cb, queue_size=10)
            rospy.loginfo("cmd_vel_odom : fusion USBL ON (gain=%.2f, max_speed=%.1f m/s)",
                          self.usbl_gain, self.usbl_max_speed)

    def _seed_cb(self, msg: PoseStamped) -> None:
        """Initialise position + cap depuis la 1ère GT reçue, puis ne fait plus rien."""
        if self.seeded:
            return
        with self.lock:
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
        with self.lock:
            # Euler avant : position au cap courant, puis mise à jour du cap
            self.x += vx * math.cos(self.theta) * dt
            self.y += vx * math.sin(self.theta) * dt
            self.theta += wz * dt
        self._publish(msg.header.stamp)

    def _usbl_cb(self, msg: PointStamped) -> None:
        """Fusionne un fix USBL en position (filtre complémentaire), après gate
        d'outlier par vitesse vs le dernier fix accepté (indépendant de la dérive)."""
        if not self.seeded:
            return
        t = msg.header.stamp.to_sec()
        ux, uy = msg.point.x, msg.point.y
        if self.last_usbl is not None:
            lt, lx, ly = self.last_usbl
            dt = t - lt
            if dt > 0 and math.hypot(ux - lx, uy - ly) / dt > self.usbl_max_speed:
                self.usbl_rejected += 1
                return  # saut physiquement impossible → glitch acoustique
        self.last_usbl = (t, ux, uy)
        self.usbl_kept += 1
        # correction complémentaire de position (le cap reste géré par cmd_vel)
        with self.lock:
            self.x += self.usbl_gain * (ux - self.x)
            self.y += self.usbl_gain * (uy - self.y)

    def _publish(self, stamp) -> None:
        out = PoseStamped()
        out.header.stamp = stamp  # base de temps du bag → ATE associable
        out.header.frame_id = "map"
        with self.lock:
            out.pose.position.x = self.x
            out.pose.position.y = self.y
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        out.pose.orientation.x = q[0]
        out.pose.orientation.y = q[1]
        out.pose.orientation.z = q[2]
        out.pose.orientation.w = q[3]
        self.pub.publish(out)

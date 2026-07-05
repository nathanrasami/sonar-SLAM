#!/usr/bin/env python3
"""Bridge MSIS (sonar à balayage mécanique) → OculusPingUncompressed.

Dataset CIRS caves : le Tritech Micron DST publie UN FAISCEAU PAR MESSAGE sur
/sonar_micron_ros (sensor_msgs/LaserScan : l'angle du faisceau dans angle_min,
le profil d'écho dans intensities[], la portée dans range_max). Ce nœud
assemble les faisceaux en un TOUR complet → image polaire (bins × beams) →
OculusPingUncompressed, que la chaîne feature_extraction polaire consomme
telle quelle (même chemin que holoocean).

⚠ v1 SANS compensation de mouvement : un tour Micron prend plusieurs secondes,
le véhicule bouge pendant le balayage → distorsion si vitesse élevée. Dans la
grotte (vitesse ~0.2 m/s) c'est tolérable en premier jet ; la compensation DVL
(à la ULCDfMS, cf. Paper/Loop) est le TODO n°1 si les scans sortent smeared.
⚠ À VALIDER au premier bag réel : lancer d'abord
   python3 analysis/inspect_bag.py caves.bag
et vérifier le format exact des LaserScan (cf. HOLOOCEAN_GARDE_FOU.md §3, les
invariants s'appliquent ici aussi — stamps recopiés, chiralité à tester).

Params : ~scan_topic (/sonar_micron_ros), ~range_m (0 = lire msg.range_max),
~beam_tol_deg (regroupement des angles), ~min_coverage_deg (déclenche la
publication d'un tour, défaut 300°), ~intensity_scale (échelle → uint8).
"""
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from sonar_oculus.msg import OculusPingUncompressed

from bruce_slam.utils.topics import SONAR_TOPIC_UNCOMPRESSED


class MsisScanBridge:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/sonar_micron_ros")
        self.range_m = float(rospy.get_param("~range_m", 0.0))  # 0 → msg.range_max
        self.beam_tol = np.deg2rad(float(rospy.get_param("~beam_tol_deg", 0.5)))
        self.min_cov = np.deg2rad(float(rospy.get_param("~min_coverage_deg", 300.0)))
        self.iscale = float(rospy.get_param("~intensity_scale", 1.0))
        self.beams = {}          # angle (rad, wrap [0,2π)) -> (stamp, intensities)
        self.first_angle = None
        self.n_bins = None
        self.pub = rospy.Publisher(SONAR_TOPIC_UNCOMPRESSED, OculusPingUncompressed,
                                   queue_size=5)
        rospy.Subscriber(self.scan_topic, LaserScan, self.cb, queue_size=200)
        rospy.loginfo("[msis_bridge] %s (faisceaux LaserScan) -> %s",
                      self.scan_topic, SONAR_TOPIC_UNCOMPRESSED)

    def cb(self, msg: LaserScan):
        inten = np.asarray(msg.intensities, dtype=np.float32)
        if inten.size == 0:      # certains exports mettent le profil dans ranges
            inten = np.asarray(msg.ranges, dtype=np.float32)
        if inten.size == 0:
            return
        if self.n_bins is None:
            self.n_bins = inten.size
            self.rng = self.range_m if self.range_m > 0 else float(msg.range_max)
            rospy.loginfo("[msis_bridge] %d bins/faisceau, portée %.1f m",
                          self.n_bins, self.rng)
        a = float(msg.angle_min) % (2 * np.pi)
        self.beams[round(a / self.beam_tol)] = (msg.header.stamp, a, inten)
        # un tour est complet quand la couverture angulaire dépasse min_coverage
        angles = sorted(v[1] for v in self.beams.values())
        if len(angles) >= 8:
            gaps = np.diff(angles + [angles[0] + 2 * np.pi])
            coverage = 2 * np.pi - float(np.max(gaps))
            if coverage >= self.min_cov:
                self.publish_sweep()

    def publish_sweep(self):
        items = sorted(self.beams.values(), key=lambda v: v[1])
        self.beams = {}
        stamps = [it[0] for it in items]
        angles = np.array([it[1] for it in items])
        img = np.stack([it[2] for it in items], axis=1)   # (bins, beams)
        img = np.clip(img * self.iscale, 0, 255).astype(np.uint8)
        n_bins, n_beams = img.shape

        im = Image()
        im.header.stamp = stamps[len(stamps) // 2]        # stamp du faisceau médian
        im.header.frame_id = "sonar"
        im.height, im.width = n_bins, n_beams
        im.encoding, im.step = "mono8", n_beams
        im.data = img.tobytes()

        ping = OculusPingUncompressed()
        ping.header = im.header
        ping.ping_id = 0
        # convention holoocean_sonar_bridge : bearings en centi-degrés, [-180,180)
        deg = (np.degrees(angles) + 180.0) % 360.0 - 180.0
        order = np.argsort(deg)                            # colonnes triées par angle
        im.data = img[:, order].tobytes()
        ping.bearings = (deg[order] * 100).astype(np.int16).tolist()
        ping.range_resolution = self.rng / n_bins
        ping.num_ranges = n_bins
        ping.num_beams = n_beams
        ping.ping = im
        self.pub.publish(ping)


if __name__ == "__main__":
    rospy.init_node("msis_scan_bridge")
    MsisScanBridge()
    rospy.spin()

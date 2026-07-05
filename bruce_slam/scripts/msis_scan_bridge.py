#!/usr/bin/env python3
"""Bridge MSIS (sonar à balayage mécanique 360°) → FEATURES Bruce-SLAM directes.

Dataset CIRS caves : le Tritech Micron publie UN FAISCEAU PAR MESSAGE sur
/sonar_micron_ros (LaserScan : angle du faisceau dans angle_min, profil d'écho
dans intensities[], portée dans range_max). Ce nœud assemble les faisceaux en
TOURS complets, fait le CFAR + les filtres EN POLAIRE, convertit (r,θ) → (x,y)
métriques EXACTS et publie directement les features du SLAM.

POURQUOI pas la chaîne image de feature_extraction (leçon du 07-05, 2 runs) :
sa conversion cartésienne suppose un SECTEUR FLS (<180°) — avec un tour 360°,
sin(FOV/2)=sin(180°)≈0 → toutes les coordonnées s'effondrent (nuages
bit-identiques de 672 pts quels que soient les seuils). En polaire pur, la
conversion est exacte pour n'importe quelle couverture angulaire.

Calibré sur le bag réel (dry-run conteneur, 1 tour 397×200) :
CFAR SOCA(20,4,0.1,10) → 11 811 pics ; seuil 60 → 456 ; downsample 0.5 → 180 ;
outliers (1.5 m, 3 voisins) → ~160 pts/tour (l'ancien 1.0/10 n'en laissait 26).

⚠ v1 sans compensation de mouvement pendant le tour (8.6 s, ~1.7 m à 0.2 m/s) —
TODO n°1 si les scans sortent smeared (interpolation /odometry par faisceau,
façon ULCDfMS). ⚠ chiralité à vérifier au 1er run (checklist GARDE_FOU §6.1) :
param ~flip_y.

Méthode bruce_sonar_usbl : ~sonar_context/enable publie aussi le descripteur
SONAR Context calculé sur le tour polaire (même format que feature_extraction).
"""
import numpy as np
import rospy
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32MultiArray

from bruce_slam import pcl
from bruce_slam.CFAR import CFAR
from bruce_slam.sonar_context import build_sonar_context, build_polar_key
from bruce_slam.utils.conversions import n2r
from bruce_slam.utils.topics import SONAR_FEATURE_TOPIC, SONAR_DESCRIPTOR_TOPIC


class MsisScanBridge:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/sonar_micron_ros")
        self.range_m = float(rospy.get_param("~range_m", 0.0))  # 0 → msg.range_max
        self.beam_tol = np.deg2rad(float(rospy.get_param("~beam_tol_deg", 0.5)))
        self.min_cov = np.deg2rad(float(rospy.get_param("~min_coverage_deg", 350.0)))
        # extraction (défauts = calibration bag réel 07-05)
        self.threshold = float(rospy.get_param("~threshold", 60.0))
        self.resolution = float(rospy.get_param("~resolution", 0.5))
        self.out_radius = float(rospy.get_param("~outlier_radius", 1.5))
        self.out_min = int(rospy.get_param("~outlier_min_points", 3))
        self.flip_y = bool(rospy.get_param("~flip_y", False))  # chiralité (PIEGES §1)
        self.detector = CFAR(int(rospy.get_param("~cfar/Ntc", 20)),
                             int(rospy.get_param("~cfar/Ngc", 4)),
                             float(rospy.get_param("~cfar/Pfa", 0.1)),
                             int(rospy.get_param("~cfar/rank", 10)))
        self.alg = rospy.get_param("~cfar/alg", "SOCA")
        self.sc_enable = bool(rospy.get_param("~sonar_context/enable", False))
        self.sc_A = int(rospy.get_param("~sonar_context/num_azimuth", 40))
        self.sc_R = int(rospy.get_param("~sonar_context/num_range", 40))
        self.sc_thr = float(rospy.get_param("~sonar_context/intensity_threshold", 60))

        self.beams = {}          # clé angle → (stamp, angle, intensités)
        self.n_bins, self.rng = None, None
        self.pub = rospy.Publisher(SONAR_FEATURE_TOPIC, PointCloud2, queue_size=5)
        self.pub_desc = rospy.Publisher(SONAR_DESCRIPTOR_TOPIC, Float32MultiArray,
                                        queue_size=5) if self.sc_enable else None
        rospy.Subscriber(self.scan_topic, LaserScan, self.cb, queue_size=300)
        rospy.loginfo("[msis_bridge] %s -> FEATURES %s (CFAR polaire, seuil %.0f, "
                      "outliers %.1f m/%d, SC=%s)", self.scan_topic,
                      SONAR_FEATURE_TOPIC, self.threshold, self.out_radius,
                      self.out_min, self.sc_enable)

    def cb(self, msg: LaserScan):
        inten = np.asarray(msg.intensities, dtype=np.float32)
        if inten.size == 0:
            inten = np.asarray(msg.ranges, dtype=np.float32)
        if inten.size == 0:
            return
        if self.n_bins is None:
            self.n_bins = inten.size
            self.rng = self.range_m if self.range_m > 0 else float(msg.range_max)
            rospy.loginfo("[msis_bridge] %d bins/faisceau, portée %.1f m "
                          "(%.3f m/bin)", self.n_bins, self.rng, self.rng / self.n_bins)
        a = float(msg.angle_min) % (2 * np.pi)
        self.beams[round(a / self.beam_tol)] = (msg.header.stamp, a, inten)
        angles = sorted(v[1] for v in self.beams.values())
        if len(angles) >= 8:
            gaps = np.diff(angles + [angles[0] + 2 * np.pi])
            if 2 * np.pi - float(np.max(gaps)) >= self.min_cov:
                self.publish_sweep()

    def publish_sweep(self):
        items = sorted(self.beams.values(), key=lambda v: v[1])
        self.beams = {}
        stamp = items[len(items) // 2][0]                 # stamp du faisceau médian
        angles = np.array([it[1] for it in items])
        img = np.clip(np.stack([it[2] for it in items], axis=1), 0, 255).astype(np.uint8)

        # CFAR + seuil, EN POLAIRE (lignes = bins de range, colonnes = faisceaux)
        peaks = self.detector.detect(img, self.alg)
        peaks &= img > self.threshold
        rows, cols = np.nonzero(peaks)
        if rows.size == 0:
            return
        # conversion polaire → métrique EXACTE (valide en 360°) :
        # x = avant, y = latéral (signe testable via ~flip_y)
        r = (rows.astype(np.float32) + 0.5) * (self.rng / self.n_bins)
        th = angles[cols].astype(np.float32)
        x, y = r * np.cos(th), r * np.sin(th)
        inten = img[rows, cols].astype(np.float32)

        pts = np.column_stack([x, y]).astype(np.float32)
        if len(pts) and self.resolution > 0:
            pts = pcl.downsample(pts, self.resolution)
        if self.out_min > 1 and len(pts) > 0:
            pts = pcl.remove_outlier(pts.astype(np.float32),
                                     self.out_radius, self.out_min)
        if len(pts) == 0:
            return
        # ré-associe l'intensité de chaque survivant (plus proche détection brute)
        _, idx = cKDTree(np.column_stack([x, y])).query(pts, k=1)
        inten = inten[idx]

        yy = -pts[:, 1] if self.flip_y else pts[:, 1]
        packed = np.c_[pts[:, 0], inten, -yy]   # [x, I, -y] convention aracati
        out = n2r(packed.astype(np.float32), "PointCloudXYZ")
        out.header.stamp = stamp
        out.header.frame_id = "base_link"
        self.pub.publish(out)

        if self.pub_desc is not None:
            ctx = build_sonar_context(img, self.sc_A, self.sc_R, self.sc_thr)
            pkey = build_polar_key(ctx)
            # format feature_extraction._publish_descriptor : timestamp scindé en
            # moitiés 16 bits (float32 corromprait sec/nsec) —
            # [sec_hi, sec_lo, nsec_hi, nsec_lo, A, R, context(A*R), key(R)]
            sec, nsec = int(stamp.secs), int(stamp.nsecs)
            data = [float(sec >> 16), float(sec & 0xFFFF),
                    float(nsec >> 16), float(nsec & 0xFFFF),
                    float(ctx.shape[0]), float(ctx.shape[1])]
            data += ctx.astype(np.float32).ravel().tolist()
            data += np.asarray(pkey, dtype=np.float32).tolist()
            m = Float32MultiArray()
            m.data = data
            self.pub_desc.publish(m)


if __name__ == "__main__":
    rospy.init_node("msis_scan_bridge")
    MsisScanBridge()
    rospy.spin()

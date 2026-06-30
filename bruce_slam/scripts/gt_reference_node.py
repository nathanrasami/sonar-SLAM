#!/usr/bin/env python3
"""
Référence GT — SANS SLAM.

Construit la vérité-terrain pour caractériser cap / trajectoire / carte :
  trajectory.csv  : pose GT par frame sonar   (keyframe_id, time, x, y, theta, ...)  <- /pose_gt direct
  groundtruth.csv : toutes les poses GT       (time, x, y, theta)                     <- /pose_gt direct
  pointcloud.csv  : features sonar (CFAR) PROJETÉES sur les poses GT (keyframe_id, x, y)

Aucun DISO, aucun iSAM2, aucune loop closure : le nuage est le plus propre possible
car les poses SONT la vérité-terrain. ATE = 0 par construction. On ne "corrige" pas
la GT vers la GT (ce serait circulaire) — on place juste chaque retour sonar là où
la GT dit que le véhicule était.

Topics :
  /pose_gt                                geometry_msgs/PoseStamped  (vérité terrain)
  /bruce/slam/feature_extraction/feature  sensor_msgs/PointCloud2    (features CFAR,
                                          repère body, packées [x_avant, 0, -y_latéral])
"""
import bisect
import csv
import math
import os

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header

gt_buf   = []   # (time, x, y, yaw) — trié par temps (le bag publie dans l'ordre)
gt_times = []   # liste parallèle des temps, pour bisect
frames   = []   # par frame sonar : (time, gx, gy, gyaw, world_points Nx2)

# --- visualisation live (RViz video.rviz, fixed frame=map, on publie en world) ---
VIZ_FRAME = "world"          # le launch publie le TF world->map (flip 180°)
traj_pub  = None
cloud_pub = None
pose_pub  = None
_traj_xy  = []               # trajectoire GT accumulée (x, y) pour l'affichage
_cloud_xy = [np.zeros((0, 2), np.float32)]  # points monde accumulés pour l'affichage


def _yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def gt_cb(msg):
    p, q = msg.pose.position, msg.pose.orientation
    t = msg.header.stamp.to_sec()
    gt_buf.append((t, p.x, p.y, _yaw(q.x, q.y, q.z, q.w)))
    gt_times.append(t)


def _nearest_gt(t):
    """Pose GT dont l'horodatage est le plus proche de t."""
    if not gt_buf:
        return None
    i = bisect.bisect_left(gt_times, t)
    if i == 0:
        return gt_buf[0]
    if i >= len(gt_buf):
        return gt_buf[-1]
    before, after = gt_buf[i - 1], gt_buf[i]
    return before if (t - before[0]) <= (after[0] - t) else after


def feature_cb(msg):
    g = _nearest_gt(msg.header.stamp.to_sec())
    if g is None:
        return
    _, gx, gy, gyaw = g

    n = msg.width * msg.height
    if n == 0:
        return
    # parse PointCloud2 : x,y,z = 3 premiers float32 ; packé [x_avant, 0, -y_latéral]
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    xyz = raw[:, :12].copy().view(np.float32).reshape(n, 3)
    fx = xyz[:, 0]          # avant (forward) — même repère que la reconstruction Bruce
    fy = -xyz[:, 2]         # latéral (Bruce parse aussi (x, -z))

    ok = ~(np.isnan(fx) | np.isnan(fy))   # retire les placeholders NaN de synchro
    fx, fy = fx[ok], fy[ok]
    if len(fx) == 0:
        return

    # projection body -> monde via la pose GT (rotation yaw + translation)
    c, s = math.cos(gyaw), math.sin(gyaw)
    wx = gx + fx * c - fy * s
    wy = gy + fx * s + fy * c
    world = np.column_stack((wx, wy))
    frames.append((msg.header.stamp.to_sec(), gx, gy, gyaw, world))

    # --- affichage live ---
    _traj_xy.append((gx, gy))
    _cloud_xy.append(world.astype(np.float32))
    _publish_viz(gx, gy, len(frames))


def _xy_cloud_msg(xy):
    """(N,2) -> PointCloud2 XYZ (z=0) dans le repère world."""
    header = Header(stamp=rospy.Time.now(), frame_id=VIZ_FRAME)
    pts = [(float(x), float(y), 0.0) for x, y in xy]
    return pc2.create_cloud_xyz32(header, pts)


def _publish_viz(gx, gy, n):
    # pose courante (le véhicule bouge dans RViz) — à chaque frame
    if pose_pub is not None:
        pm = PoseWithCovarianceStamped()
        pm.header.stamp = rospy.Time.now()
        pm.header.frame_id = VIZ_FRAME
        pm.pose.pose.position.x = gx
        pm.pose.pose.position.y = gy
        pm.pose.pose.orientation.w = 1.0
        pose_pub.publish(pm)
    # trajectoire (légère) — à chaque frame
    if traj_pub is not None:
        traj_pub.publish(_xy_cloud_msg(_traj_xy))
    # nuage (lourd) — throttlé tous les 25 frames (~5 s)
    if cloud_pub is not None and n % 25 == 0:
        allpts = np.concatenate(_cloud_xy)
        if len(allpts) > 80000:                       # sous-échantillonne l'affichage
            idx = np.random.choice(len(allpts), 80000, replace=False)
            allpts = allpts[idx]
        cloud_pub.publish(_xy_cloud_msg(allpts))


def export():
    out = os.environ.get("SLAM_RESULTS_DIR", "results")
    os.makedirs(out, exist_ok=True)

    # trajectory.csv : pose GT par frame sonar (x,y et dr_* identiques = GT)
    try:
        with open(os.path.join(out, "trajectory.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["keyframe_id", "time", "x", "y", "theta",
                        "dr_x", "dr_y", "dr_theta",
                        "cov_xx", "cov_yy", "cov_tt", "nssm_constraints"])
            for i, (t, gx, gy, gyaw, _) in enumerate(frames):
                w.writerow([i, t, gx, gy, gyaw, gx, gy, gyaw, 0.0, 0.0, 0.0, 0])
        rospy.loginfo("trajectory.csv: %d frames", len(frames))
    except Exception as e:
        rospy.logerr("trajectory.csv FAILED: %s", e)

    # pointcloud.csv : features projetées sur les poses GT
    try:
        with open(os.path.join(out, "pointcloud.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["keyframe_id", "x", "y"])
            for i, (_, _, _, _, pts) in enumerate(frames):
                for x, y in pts:
                    w.writerow([i, x, y])
        rospy.loginfo("pointcloud.csv: %d frames projetées", len(frames))
    except Exception as e:
        rospy.logerr("pointcloud.csv FAILED: %s", e)

    # groundtruth.csv : toutes les poses GT
    try:
        with open(os.path.join(out, "groundtruth.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time", "x", "y", "theta"])
            for t, gx, gy, gyaw in gt_buf:
                w.writerow([t, gx, gy, gyaw])
        rospy.loginfo("groundtruth.csv: %d poses", len(gt_buf))
    except Exception as e:
        rospy.logerr("groundtruth.csv FAILED: %s", e)


def main():
    global traj_pub, cloud_pub, pose_pub
    rospy.init_node("gt_reference")
    # publie sur les MÊMES topics que video.rviz affiche déjà → rien à régler dans RViz
    traj_pub  = rospy.Publisher("/bruce/slam/slam/traj",  PointCloud2, queue_size=1, latch=True)
    cloud_pub = rospy.Publisher("/bruce/slam/slam/cloud", PointCloud2, queue_size=1, latch=True)
    pose_pub  = rospy.Publisher("/bruce/slam/slam/pose",  PoseWithCovarianceStamped, queue_size=1)
    rospy.Subscriber("/pose_gt", PoseStamped, gt_cb, queue_size=5000)
    rospy.Subscriber("/bruce/slam/feature_extraction/feature",
                     PointCloud2, feature_cb, queue_size=200)
    rospy.on_shutdown(export)
    rospy.loginfo("gt_reference prêt — features projetées sur poses GT, AUCUN SLAM")
    rospy.spin()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Save DISO trajectory, odometry, point cloud and ground truth to CSV on shutdown.
Topics:
  /direct_sonar/pose        (geometry_msgs/PoseStamped) — DISO estimated pose
  /direct_sonar/odom        (geometry_msgs/PoseStamped) — pure odometry
  /direct_sonar/point_cloud (sensor_msgs/PointCloud2)   — landmarks map
  /pose_gt                  (geometry_msgs/PoseStamped) — ground truth
"""
import csv
import os
import struct
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

poses     = []
odom      = []
gt_poses  = []
cloud_pts = []  # accumulated (x, y, z) from all point cloud messages


def pose_cb(msg):
    p, q = msg.pose.position, msg.pose.orientation
    poses.append((msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))


def odom_cb(msg):
    p, q = msg.pose.position, msg.pose.orientation
    odom.append((msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))


def gt_cb(msg):
    p = msg.pose.position
    gt_poses.append((msg.header.stamp.to_sec(), p.x, p.y))


def cloud_cb(msg):
    # Parse PointCloud2 (x,y,z floats, 4 bytes each)
    point_step = msg.point_step
    data = msg.data
    for i in range(msg.width * msg.height):
        offset = i * point_step
        x, y, z = struct.unpack_from('fff', data, offset)
        cloud_pts.append((x, y, z))


def export():
    output_dir = os.environ.get("SLAM_RESULTS_DIR",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results"))
    os.makedirs(output_dir, exist_ok=True)

    traj_path = os.path.join(output_dir, "diso_trajectory.csv")
    with open(traj_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "x", "y", "z", "qx", "qy", "qz", "qw"])
        w.writerows(poses)
    rospy.loginfo("DISO trajectory saved: %d poses -> %s", len(poses), traj_path)

    odom_path = os.path.join(output_dir, "diso_odom.csv")
    with open(odom_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "x", "y", "z", "qx", "qy", "qz", "qw"])
        w.writerows(odom)
    rospy.loginfo("DISO odom saved: %d poses -> %s", len(odom), odom_path)

    cloud_path = os.path.join(output_dir, "diso_pointcloud.csv")
    with open(cloud_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z"])
        w.writerows(cloud_pts)
    rospy.loginfo("Point cloud saved: %d points -> %s", len(cloud_pts), cloud_path)

    if gt_poses:
        gt_path = os.path.join(output_dir, "groundtruth.csv")
        with open(gt_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time", "x", "y"])
            w.writerows(gt_poses)
        rospy.loginfo("Ground truth saved: %d poses -> %s", len(gt_poses), gt_path)


def main():
    rospy.init_node("bruce_save")
    rospy.Subscriber("/direct_sonar/pose",        PoseStamped, pose_cb,  queue_size=2000)
    rospy.Subscriber("/direct_sonar/odom",        PoseStamped, odom_cb,  queue_size=2000)
    rospy.Subscriber("/direct_sonar/point_cloud", PointCloud2, cloud_cb, queue_size=100)
    rospy.Subscriber("/pose_gt",                  PoseStamped, gt_cb,    queue_size=2000)
    rospy.on_shutdown(export)
    rospy.loginfo("bruce_save ready — will export CSV on shutdown")
    rospy.spin()


if __name__ == "__main__":
    main()

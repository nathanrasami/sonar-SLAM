#!/usr/bin/env python3
"""
Save DISO trajectory and ground truth to CSV on shutdown.
Topics:
  /direct_sonar/pose  (geometry_msgs/PoseStamped) — DISO estimated pose
  /pose_gt            (geometry_msgs/PoseStamped) — ground truth
"""
import csv
import os
import rospy
from geometry_msgs.msg import PoseStamped

poses = []   # [(time, x, y, z, qx, qy, qz, qw), ...]
gt_poses = []


def pose_cb(msg):
    p = msg.pose.position
    q = msg.pose.orientation
    poses.append((msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))


def gt_cb(msg):
    p = msg.pose.position
    gt_poses.append((msg.header.stamp.to_sec(), p.x, p.y))


def export():
    output_dir = os.environ.get("SLAM_RESULTS_DIR",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results"))
    os.makedirs(output_dir, exist_ok=True)

    traj_path = os.path.join(output_dir, "diso_trajectory.csv")
    with open(traj_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(poses)
    rospy.loginfo("DISO trajectory saved to %s (%d poses)", traj_path, len(poses))

    if gt_poses:
        gt_path = os.path.join(output_dir, "groundtruth.csv")
        with open(gt_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "x", "y"])
            writer.writerows(gt_poses)
        rospy.loginfo("Ground truth saved to %s (%d poses)", gt_path, len(gt_poses))


def main():
    rospy.init_node("bruce_save")
    rospy.Subscriber("/direct_sonar/pose", PoseStamped, pose_cb, queue_size=1000)
    rospy.Subscriber("/pose_gt", PoseStamped, gt_cb, queue_size=1000)
    rospy.on_shutdown(export)
    rospy.loginfo("bruce_save ready — will export CSV on shutdown")
    rospy.spin()


if __name__ == "__main__":
    main()

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

results_dir = os.environ.get("SLAM_RESULTS_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))

traj = pd.read_csv(os.path.join(results_dir, "trajectory.csv"))
gt_path = os.path.join(results_dir, "groundtruth.csv")

fig, ax = plt.subplots(figsize=(10, 8))

tx, ty = traj["x"].to_numpy(), traj["y"].to_numpy()
dx, dy = traj["dr_x"].to_numpy(), traj["dr_y"].to_numpy()

if os.path.exists(gt_path):
    gt = pd.read_csv(gt_path)
    gx, gy = gt["x"].to_numpy(), gt["y"].to_numpy()
    # Align all trajectories to GT start point
    offset_x = gx[0] - tx[0]
    offset_y = gy[0] - (-ty[0])
    tx, ty = tx + offset_x, -ty + offset_y
    dx, dy = dx + offset_x, -dy + offset_y
    # ATE: interpolate GT to SLAM keyframe count
    idx = np.round(np.linspace(0, len(gx)-1, len(tx))).astype(int)
    ate = np.sqrt(np.mean((tx - gx[idx])**2 + (ty - gy[idx])**2))
    ax.plot(gx, gy, label="Ground truth (GPS)", color="red", linestyle="--")
    ax.plot(gx[0], gy[0], marker="*", color="red", markersize=14)
    ax.plot(gx[-1], gy[-1], marker="X", color="red", markersize=12)
else:
    print("groundtruth.csv not found — run simulation with /pose_gt topic active")
    ate = None

slam_label = f"Bruce-SLAM (DISO odom, iSAM2, ATE={ate:.1f} m)" if ate is not None else "Bruce-SLAM (DISO odom, iSAM2)"
ax.plot(tx, ty, label=slam_label, color="black", linewidth=1.5)
ax.plot(tx[0], ty[0], marker="*", color="black", markersize=14, label="Start")
ax.plot(tx[-1], ty[-1], marker="X", color="black", markersize=12, label="End")
# drawn after SLAM so it's visible on top when trajectories overlap
ax.plot(dx, dy, label="Odometry (dead reckoning)", color="steelblue", linestyle=":", linewidth=2.5)
ax.plot(dx[0], dy[0], marker="*", color="steelblue", markersize=14)
ax.plot(dx[-1], dy[-1], marker="X", color="steelblue", markersize=12)

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Trajectory: SLAM vs Ground truth")
ax.legend()
ax.axis("equal")
ax.grid(True)
plt.tight_layout()

out = os.path.join(results_dir, "trajectory_plot.png")
plt.savefig(out, dpi=150)
plt.show()
print(f"Saved to {out}")

# --- Point cloud map ---
cloud_path = os.path.join(results_dir, "pointcloud.csv")
if os.path.exists(cloud_path):
    cloud = pd.read_csv(cloud_path)
    fig2, ax2 = plt.subplots(figsize=(10, 8))
    ax2.scatter(cloud["x"].to_numpy(), cloud["y"].to_numpy(),
                s=0.5, c="navy", alpha=0.3)
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.set_title("Point cloud map (accumulated sonar points)")
    ax2.axis("equal")
    ax2.grid(True)
    plt.tight_layout()
    out2 = os.path.join(results_dir, "pointcloud_map.png")
    plt.savefig(out2, dpi=150)
    plt.show()
    print(f"Saved to {out2}")
else:
    print("pointcloud.csv not found")

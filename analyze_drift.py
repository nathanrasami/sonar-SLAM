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
    offset_y = gy[0] - ty[0]
    tx, ty = tx + offset_x, ty + offset_y
    dx, dy = dx + offset_x, dy + offset_y
    ax.plot(gx, gy, label="Ground truth (GPS)", color="green", linestyle="--")
    ax.plot(gx[0], gy[0], marker="*", color="green", markersize=14)
    ax.plot(gx[-1], gy[-1], marker="X", color="green", markersize=12)
else:
    print("groundtruth.csv not found — run simulation with /pose_gt topic active")

ax.plot(dx, dy, label="Odometry (dead reckoning)", color="orange", linestyle=":")
ax.plot(dx[0], dy[0], marker="*", color="orange", markersize=14)
ax.plot(dx[-1], dy[-1], marker="X", color="orange", markersize=12)

ax.plot(tx, ty, label="Bruce-SLAM (SSM+NSSM)", color="steelblue")
ax.plot(tx[0], ty[0], marker="*", color="steelblue", markersize=14, label="Start")
ax.plot(tx[-1], ty[-1], marker="X", color="steelblue", markersize=12, label="End")

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

#!/usr/bin/env python3
"""
Plot DISO trajectory vs ground truth.
Reads diso_trajectory.csv and groundtruth.csv from SLAM_RESULTS_DIR.
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

results_dir = os.environ.get("SLAM_RESULTS_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "DISO", "results"))

traj_path = os.path.join(results_dir, "diso_trajectory.csv")
gt_path = os.path.join(results_dir, "groundtruth.csv")

if not os.path.exists(traj_path):
    print(f"ERROR: {traj_path} not found — run DISO then CTRL+C to generate it")
    exit(1)

traj = pd.read_csv(traj_path)
tx, ty = traj["x"].to_numpy(), traj["y"].to_numpy()

fig, ax = plt.subplots(figsize=(10, 8))

if os.path.exists(gt_path):
    gt = pd.read_csv(gt_path)
    gx, gy = gt["x"].to_numpy(), gt["y"].to_numpy()
    # Align DISO start to GT start
    tx = tx + (gx[0] - tx[0])
    ty = ty + (gy[0] - ty[0])
    ax.plot(gx, gy, label="Ground truth (GPS)", color="red", linestyle="--")
    ax.plot(gx[0], gy[0], marker="*", color="red", markersize=14)
    ax.plot(gx[-1], gy[-1], marker="X", color="red", markersize=12)

    # ATE (2D)
    n = min(len(tx), len(gx))
    ate = np.sqrt(np.mean((tx[:n] - gx[:n])**2 + (ty[:n] - gy[:n])**2))
    ax.set_title(f"DISO trajectory vs Ground truth — ATE={ate:.2f} m")
else:
    print("groundtruth.csv not found — plotting DISO only")
    ax.set_title("DISO trajectory")

ax.plot(tx, ty, label="DISO odometry", color="green")
ax.plot(tx[0], ty[0], marker="*", color="green", markersize=14, label="Start")
ax.plot(tx[-1], ty[-1], marker="X", color="green", markersize=12, label="End")

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.legend()
ax.axis("equal")
ax.grid(True)
plt.tight_layout()

out = os.path.join(results_dir, "diso_trajectory_plot.png")
plt.savefig(out, dpi=150)
plt.show()
print(f"Saved to {out}")

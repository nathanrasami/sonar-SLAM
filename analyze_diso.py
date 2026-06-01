#!/usr/bin/env python3
"""
Plot DISO trajectory vs ground truth.
Aligns by proportional index resampling then Umeyama SE2 (no scale).
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

results_dir = os.environ.get("SLAM_RESULTS_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))

traj_path = os.path.join(results_dir, "diso_trajectory.csv")
gt_path   = os.path.join(results_dir, "groundtruth.csv")

if not os.path.exists(traj_path):
    print(f"ERROR: {traj_path} not found")
    exit(1)

traj = pd.read_csv(traj_path)
gt   = pd.read_csv(gt_path)

tx, ty = traj["x"].to_numpy(), traj["y"].to_numpy()
gx, gy = gt["x"].to_numpy(),   gt["y"].to_numpy()

# Resample DISO to same number of points as GT (proportional index)
idx = np.round(np.linspace(0, len(tx)-1, len(gx))).astype(int)
tx_r, ty_r = tx[idx], ty[idx]

# Umeyama SE2: find R, t mapping DISO → GT (no scale)
p = np.stack([tx_r, ty_r], axis=1)
q = np.stack([gx,   gy  ], axis=1)
mu_p, mu_q = p.mean(0), q.mean(0)
H = (p - mu_p).T @ (q - mu_q)
U, _, Vt = np.linalg.svd(H)
R = Vt.T @ U.T
# Force proper rotation (no reflection)
if np.linalg.det(R) < 0:
    Vt[1, :] *= -1
    R = Vt.T @ U.T
t_vec = mu_q - R @ mu_p

# Apply to full DISO trajectory
aligned = (R @ np.stack([tx, ty])).T + t_vec
tx_a, ty_a = aligned[:, 0], aligned[:, 1]

# ATE on resampled points
tx_a_r = (R @ np.stack([tx_r, ty_r])).T + t_vec
ate = np.sqrt(np.mean((tx_a_r[:,0] - gx)**2 + (tx_a_r[:,1] - gy)**2))

fig, ax = plt.subplots(figsize=(11, 9))
ax.plot(gx,   gy,   label="Ground truth (GPS)", color="red",   linestyle="--", linewidth=1.5)
ax.plot(gx[0], gy[0], marker="*", color="red",   markersize=14)
ax.plot(gx[-1],gy[-1],marker="X", color="red",   markersize=12)
ax.plot(tx_a, ty_a, label="DISO odometry",      color="green", linewidth=1.5)
ax.plot(tx_a[0],  ty_a[0],  marker="*", color="green", markersize=14, label="Start")
ax.plot(tx_a[-1], ty_a[-1], marker="X", color="green", markersize=12, label="End")

ax.set_title(f"DISO trajectory vs Ground truth — ATE = {ate:.2f} m")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.legend()
ax.axis("equal")
ax.grid(True)
plt.tight_layout()

out = os.path.join(results_dir, "diso_trajectory_plot.png")
plt.savefig(out, dpi=150)
plt.show()
print(f"Saved to {out} — ATE = {ate:.2f} m")

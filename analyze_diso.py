#!/usr/bin/env python3
"""
Generate 2 plots from DISO results:
  1. Trajectories: GT + DISO
  2. Point cloud map
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

results_dir = os.environ.get("SLAM_RESULTS_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))

def load(name):
    path = os.path.join(results_dir, name)
    if not os.path.exists(path):
        print(f"WARNING: {path} not found")
        return None
    return pd.read_csv(path)

traj  = load("diso_trajectory.csv")
gt    = load("groundtruth.csv")
cloud = load("diso_pointcloud.csv")

if traj is None:
    print("ERROR: diso_trajectory.csv missing — run DISO and CTRL+C first")
    exit(1)

tx, ty = traj["x"].to_numpy(), traj["y"].to_numpy()
gx, gy = gt["x"].to_numpy(),   gt["y"].to_numpy()

# ── Alignment DISO → GT: flip Y, rotate by centroid angle, translate ─────────
ty_f = -ty

diso_angle = np.arctan2(ty_f.mean(), tx.mean())
gt_angle   = np.arctan2(gy.mean(),   gx.mean())
theta = gt_angle - diso_angle
c, s = np.cos(theta), np.sin(theta)
R = np.array([[c, -s], [s, c]])

rotated = (R @ np.stack([tx, ty_f])).T
t_vec = np.array([gx.mean(), gy.mean()]) - rotated.mean(axis=0)
tx_a = rotated[:, 0] + t_vec[0]
ty_a = rotated[:, 1] + t_vec[1]

# Rotate everything +90° CCW for display
_a = np.pi / 4
Rd = np.array([[np.cos(_a), -np.sin(_a)], [np.sin(_a), np.cos(_a)]])
def rot90(x, y):
    pts = (Rd @ np.stack([x, y])).T
    return pts[:, 0], pts[:, 1]

tx_a, ty_a = rot90(tx_a, ty_a)
gx, gy     = rot90(gx, gy)
if cloud is not None:
    cx, cy = cloud["x"].to_numpy(), cloud["y"].to_numpy()
    cx, cy = rot90(cx, cy)

# ATE
idx = np.round(np.linspace(0, len(gx)-1, len(tx_a))).astype(int)
ate = np.sqrt(np.mean((tx_a - gx[idx])**2 + (ty_a - gy[idx])**2))

# ── Figure 1: Trajectories ────────────────────────────────────────────────────
fig1, ax1 = plt.subplots(figsize=(11, 9))
ax1.plot(gx, gy, label="Ground truth (GPS)", color="red", linestyle="--", linewidth=1.5)
ax1.plot(gx[0], gy[0], marker="*", color="red", markersize=14)
ax1.plot(gx[-1], gy[-1], marker="X", color="red", markersize=12)
ax1.plot(tx_a, ty_a, label=f"DISO (ATE={ate:.1f} m)", color="green", linewidth=1.5)
ax1.plot(tx_a[0],  ty_a[0],  marker="*", color="green", markersize=14, label="Start")
ax1.plot(tx_a[-1], ty_a[-1], marker="X", color="green", markersize=12, label="End")
ax1.set_title("Trajectories: DISO vs Ground truth")
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
ax1.legend(); ax1.axis("equal"); ax1.grid(True)
plt.tight_layout()
out1 = os.path.join(results_dir, "plot1_trajectories.png")
fig1.savefig(out1, dpi=150)
print(f"Saved: {out1}")

# ── Figure 2: Point cloud ─────────────────────────────────────────────────────
if cloud is not None:
    fig2, ax2 = plt.subplots(figsize=(11, 9))
    ax2.scatter(cx, cy, s=0.5, c="green", alpha=0.4, label="Sonar landmarks")
    ax2.set_title("DISO Point cloud map")
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
    ax2.legend(); ax2.axis("equal"); ax2.grid(True)
    plt.tight_layout()
    out2 = os.path.join(results_dir, "plot2_pointcloud.png")
    fig2.savefig(out2, dpi=150)
    print(f"Saved: {out2}")

plt.show()

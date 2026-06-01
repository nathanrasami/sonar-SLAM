#!/usr/bin/env python3
"""
Generate 3 plots from DISO results:
  1. Trajectories: GT + pure odom + DISO
  2. Point cloud map
  3. Point cloud + trajectories overlay
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
odom  = load("diso_odom.csv")
gt    = load("groundtruth.csv")
cloud = load("diso_pointcloud.csv")

if traj is None:
    print("ERROR: diso_trajectory.csv missing — run DISO and CTRL+C first")
    exit(1)

# ── Alignment: flip Y then rotate by centroid angle ──────────────────────────
def align_to_gt(x, y, gx, gy):
    """Flip Y then rotate so DISO centroid matches GT centroid direction."""
    y = -y
    diso_angle = np.arctan2(y.mean(), x.mean())
    gt_angle   = np.arctan2(gy.mean(), gx.mean())
    theta = gt_angle - diso_angle
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    aligned = (R @ np.stack([x, y])).T
    return aligned[:, 0], aligned[:, 1], R

tx, ty = traj["x"].to_numpy(), traj["y"].to_numpy()
gx, gy = gt["x"].to_numpy(),   gt["y"].to_numpy()

tx_a, ty_a, R = align_to_gt(tx, ty, gx, gy)

# ATE
idx = np.round(np.linspace(0, len(gx)-1, len(tx_a))).astype(int)
ate = np.sqrt(np.mean((tx_a - gx[idx])**2 + (ty_a - gy[idx])**2))

# ── Figure 1: Trajectories ────────────────────────────────────────────────────
fig1, ax1 = plt.subplots(figsize=(11, 9))
ax1.plot(gx, gy, label="Ground truth (GPS)", color="red", linestyle="--", linewidth=1.5)
ax1.plot(gx[0], gy[0], marker="*", color="red", markersize=14)
ax1.plot(gx[-1], gy[-1], marker="X", color="red", markersize=12)

if odom is not None:
    ox, oy = odom["x"].to_numpy(), odom["y"].to_numpy()
    ox_a, oy_a, _ = align_to_gt(ox, oy, gx, gy)
    ax1.plot(ox_a, oy_a, label="Pure odometry (dead reckoning)", color="steelblue",
             linestyle=":", linewidth=1.2)
    ax1.plot(ox_a[0], oy_a[0], marker="*", color="steelblue", markersize=12)
    ax1.plot(ox_a[-1], oy_a[-1], marker="X", color="steelblue", markersize=10)

ax1.plot(tx_a, ty_a, label=f"DISO odometry (ATE={ate:.1f} m)", color="green", linewidth=1.5)
ax1.plot(tx_a[0],  ty_a[0],  marker="*", color="green", markersize=14, label="Start")
ax1.plot(tx_a[-1], ty_a[-1], marker="X", color="green", markersize=12, label="End")

ax1.set_title("Trajectories: DISO vs Ground truth vs Odometry")
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
ax1.legend(); ax1.axis("equal"); ax1.grid(True)
plt.tight_layout()
out1 = os.path.join(results_dir, "plot1_trajectories.png")
fig1.savefig(out1, dpi=150)
print(f"Saved: {out1}")

# ── Figure 2: Point cloud ─────────────────────────────────────────────────────
if cloud is not None:
    cx, cy = cloud["x"].to_numpy(), cloud["y"].to_numpy()
    # Apply same alignment (flip Y then rotate)
    cy_f = -cy
    aligned_c = (R @ np.stack([cx, cy_f])).T
    cx_a, cy_a = aligned_c[:, 0], aligned_c[:, 1]

    fig2, ax2 = plt.subplots(figsize=(11, 9))
    ax2.scatter(cx_a, cy_a, s=0.5, c="green", alpha=0.4, label="Sonar landmarks")
    ax2.set_title("DISO Point cloud map")
    ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
    ax2.legend(); ax2.axis("equal"); ax2.grid(True)
    plt.tight_layout()
    out2 = os.path.join(results_dir, "plot2_pointcloud.png")
    fig2.savefig(out2, dpi=150)
    print(f"Saved: {out2}")

    # ── Figure 3: Point cloud + trajectories ─────────────────────────────────
    fig3, ax3 = plt.subplots(figsize=(11, 9))
    ax3.scatter(cx_a, cy_a, s=0.5, c="lightgreen", alpha=0.3, label="Sonar landmarks")
    ax3.plot(gx, gy, label="Ground truth", color="red", linestyle="--", linewidth=1.5)
    if odom is not None:
        ax3.plot(ox_a, oy_a, label="Pure odometry", color="steelblue", linestyle=":", linewidth=1.2)
    ax3.plot(tx_a, ty_a, label=f"DISO (ATE={ate:.1f} m)", color="green", linewidth=1.5)
    ax3.plot(tx_a[0], ty_a[0], marker="*", color="green", markersize=14, label="Start")
    ax3.plot(tx_a[-1], ty_a[-1], marker="X", color="green", markersize=12, label="End")
    ax3.set_title("DISO Point cloud + Trajectories")
    ax3.set_xlabel("x (m)"); ax3.set_ylabel("y (m)")
    ax3.legend(); ax3.axis("equal"); ax3.grid(True)
    plt.tight_layout()
    out3 = os.path.join(results_dir, "plot3_map_trajectories.png")
    fig3.savefig(out3, dpi=150)
    print(f"Saved: {out3}")

plt.show()

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
from traj_eval import associer_par_temps, umeyama, appliquer, calculer_ate

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

gx, gy = gt["x"].to_numpy(), gt["y"].to_numpy()  # GT dans son repère natif

# ── Alignement DISO → GT : association temporelle + Umeyama (rigide, sans flip)
src = traj[["x", "y"]].to_numpy()
gt_xy = associer_par_temps(traj["time"], gt["time"], gt["x"], gt["y"])
s, R, t = umeyama(src, gt_xy)               # absorbe la réflexion Y automatiquement
est = appliquer(s, R, t, src)
tx_a, ty_a = est[:, 0], est[:, 1]
ate = calculer_ate(est, gt_xy)

# Le point cloud reçoit la MÊME transfo que la trajectoire (cohérence visuelle)
if cloud is not None:
    cloud_a = appliquer(s, R, t, cloud[["x", "y"]].to_numpy())
    cx, cy = cloud_a[:, 0], cloud_a[:, 1]

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

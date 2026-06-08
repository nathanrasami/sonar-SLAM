#!/usr/bin/env python3
"""
Analyse HoloOcean simulation results:
  1. Extract GT from test.bag
  2. Load Bruce-SLAM trajectory from results/
  3. Compute ATE + plot
"""
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from traj_eval import associer_par_temps, umeyama, appliquer, calculer_ate

BAG_PATH     = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test.bag")
RESULTS_DIR  = os.environ.get("SLAM_RESULTS_DIR",
               os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))

# ── 1. Extract GT from bag ────────────────────────────────────────────────────
def extract_gt(bag_path):
    import rosbag
    times, xs, ys, yaws = [], [], [], []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=["/ground_truth"]):
            t = msg.header.stamp.to_sec()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            o = msg.pose.pose.orientation
            yaw = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_euler("xyz")[2]
            times.append(t); xs.append(x); ys.append(y); yaws.append(yaw)
    return pd.DataFrame({"time": times, "x": xs, "y": ys, "yaw": yaws})

print("Extracting GT from bag...")
gt = extract_gt(BAG_PATH)
print(f"  GT poses: {len(gt)}, duration: {gt['time'].iloc[-1]-gt['time'].iloc[0]:.1f}s")

# ── 2. Load Bruce-SLAM trajectory ─────────────────────────────────────────────
traj_path = os.path.join(RESULTS_DIR, "trajectory.csv")
if not os.path.exists(traj_path):
    print(f"ERROR: {traj_path} not found — run Bruce-SLAM first")
    exit(1)

traj = pd.read_csv(traj_path)
print(f"  Bruce-SLAM KFs: {len(traj)}")

# ── 3. Align + ATE ────────────────────────────────────────────────────────────
src    = traj[["x", "y"]].to_numpy()
gt_xy  = associer_par_temps(traj["time"], gt["time"], gt["x"], gt["y"])
s, R, t = umeyama(src, gt_xy)
est    = appliquer(s, R, t, src)
ate    = calculer_ate(est, gt_xy)
print(f"  ATE Bruce-SLAM: {ate:.2f} m")

# ── 4. Plot ───────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 8))

gx, gy = gt["x"].to_numpy(), gt["y"].to_numpy()
ax.plot(gx, gy, label="Ground truth", color="red", linestyle="--", linewidth=1.5)
ax.plot(gx[0], gy[0], marker="*", color="red", markersize=14)
ax.plot(gx[-1], gy[-1], marker="X", color="red", markersize=12)

ax.plot(est[:, 0], est[:, 1], label=f"Bruce-SLAM (ATE={ate:.2f} m)",
        color="black", linewidth=1.5)
ax.plot(est[0, 0], est[0, 1], marker="*", color="black", markersize=14, label="Start")
ax.plot(est[-1, 0], est[-1, 1], marker="X", color="black", markersize=12, label="End")

ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.set_title("HoloOcean — Bruce-SLAM vs Ground truth")
ax.legend(); ax.axis("equal"); ax.grid(True)
plt.tight_layout()

out = os.path.join(RESULTS_DIR, "holoocean_trajectory_plot.png")
fig.savefig(out, dpi=150)
print(f"Saved: {out}")
plt.show()

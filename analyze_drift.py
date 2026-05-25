import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

traj = pd.read_csv(os.path.join(results_dir, "trajectory.csv"))
gt_path = os.path.join(results_dir, "groundtruth.csv")

fig, ax = plt.subplots(figsize=(10, 8))

ax.plot(traj["x"], traj["y"], label="SLAM / Dead reckoning", color="steelblue")

if os.path.exists(gt_path):
    gt = pd.read_csv(gt_path)
    ax.plot(gt["x"], gt["y"], label="Ground truth (GPS)", color="green", linestyle="--")
else:
    print("groundtruth.csv not found — run simulation with /pose_gt topic active")

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

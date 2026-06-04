import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from traj_eval import associer_par_temps, umeyama, appliquer, calculer_ate

results_dir = os.environ.get("SLAM_RESULTS_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))

traj = pd.read_csv(os.path.join(results_dir, "trajectory.csv"))
gt_path = os.path.join(results_dir, "groundtruth.csv")
diso_path = os.path.join(results_dir, "diso_trajectory.csv")

fig, ax = plt.subplots(figsize=(10, 8))

s_b = R_b = t_b = None  # transfo Bruce, réutilisée pour le point cloud

if os.path.exists(gt_path):
    gt = pd.read_csv(gt_path)
    gx, gy = gt["x"].to_numpy(), gt["y"].to_numpy()  # GT natif
    # Alignement Bruce → GT : association temporelle + Umeyama (sans flip)
    src_b = traj[["x", "y"]].to_numpy()
    gt_b = associer_par_temps(traj["time"], gt["time"], gt["x"], gt["y"])
    s_b, R_b, t_b = umeyama(src_b, gt_b)
    est_b = appliquer(s_b, R_b, t_b, src_b)
    tx, ty = est_b[:, 0], est_b[:, 1]
    ate = calculer_ate(est_b, gt_b)
    ax.plot(gx, gy, label="Ground truth (GPS)", color="red", linestyle="--")
    ax.plot(gx[0], gy[0], marker="*", color="red", markersize=14)
    ax.plot(gx[-1], gy[-1], marker="X", color="red", markersize=12)
else:
    print("groundtruth.csv not found — run simulation with /pose_gt topic active")
    tx, ty = traj["x"].to_numpy(), traj["y"].to_numpy()
    ate = None

# DISO standalone as odometry reference
if os.path.exists(diso_path):
    diso = pd.read_csv(diso_path)
    if os.path.exists(gt_path):
        src_d = diso[["x", "y"]].to_numpy()
        gt_d = associer_par_temps(diso["time"], gt["time"], gt["x"], gt["y"])
        s_d, R_d, t_d = umeyama(src_d, gt_d)
        est_d = appliquer(s_d, R_d, t_d, src_d)
        ox, oy = est_d[:, 0], est_d[:, 1]
        ate_diso = calculer_ate(est_d, gt_d)
    else:
        ox, oy = diso["x"].to_numpy(), diso["y"].to_numpy()
        ate_diso = None
    diso_label = f"DISO standalone (ATE={ate_diso:.1f} m)" if ate_diso is not None else "DISO standalone"
    ax.plot(ox, oy, label=diso_label, color="steelblue", linestyle=":", linewidth=2.0)
    ax.plot(ox[0], oy[0], marker="*", color="steelblue", markersize=14)
    ax.plot(ox[-1], oy[-1], marker="X", color="steelblue", markersize=12)
else:
    print("diso_trajectory.csv not found")

slam_label = f"Bruce-SLAM (DISO odom, iSAM2, ATE={ate:.1f} m)" if ate is not None else "Bruce-SLAM (DISO odom, iSAM2)"
ax.plot(tx, ty, label=slam_label, color="black", linewidth=1.5)
ax.plot(tx[0], ty[0], marker="*", color="black", markersize=14, label="Start")
ax.plot(tx[-1], ty[-1], marker="X", color="black", markersize=12, label="End")

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
    cxy = cloud[["x", "y"]].to_numpy()
    if R_b is not None:  # même transfo que la trajectoire Bruce
        cxy = appliquer(s_b, R_b, t_b, cxy)
    fig2, ax2 = plt.subplots(figsize=(10, 8))
    ax2.scatter(cxy[:, 0], cxy[:, 1], s=0.5, c="navy", alpha=0.3)
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

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from traj_eval import associer_par_temps, umeyama, appliquer, calculer_ate

results_dir = os.environ.get("SLAM_RESULTS_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))

traj = pd.read_csv(os.path.join(results_dir, "trajectory.csv"))
gt_path   = os.path.join(results_dir, "groundtruth.csv")
diso_path = os.path.join(results_dir, "diso_trajectory.csv")
odom_path = os.path.join(results_dir, "odometry.csv")


def aligner(src_xy, t_src, gt, label):
    """Aligne une trajectoire sur le GT par interpolation temporelle + Umeyama.
    Retourne (xy_aligné, ATE, (s,R,t)). Lève une erreur claire si bases de temps
    incompatibles (CSV de runs différents)."""
    try:
        gt_xy = associer_par_temps(t_src, gt["time"], gt["x"], gt["y"])
    except ValueError as e:
        print(f"ERREUR d'association temporelle ({label}) :", e)
        print("=> les CSV ne viennent pas du même run. Vérifie results/.")
        exit(1)
    s, R, t = umeyama(src_xy, gt_xy)
    est = appliquer(s, R, t, src_xy)
    return est, calculer_ate(est, gt_xy), (s, R, t)


fig, ax = plt.subplots(figsize=(10, 8))
s_b = R_b = t_b = None  # transfo Bruce, réutilisée pour le point cloud

# ── Alignement de chaque trajectoire sur le GT ────────────────────────────────
if os.path.exists(gt_path):
    gt = pd.read_csv(gt_path)
    gxy = gt[["x", "y"]].to_numpy()

    est_b, ate, (s_b, R_b, t_b) = aligner(traj[["x", "y"]].to_numpy(), traj["time"], gt, "Bruce")

    est_d = ate_diso = None
    if os.path.exists(diso_path):
        diso = pd.read_csv(diso_path)
        est_d, ate_diso, _ = aligner(diso[["x", "y"]].to_numpy(), diso["time"], gt, "DISO")

    est_o = ate_odom = None
    if os.path.exists(odom_path):
        odom = pd.read_csv(odom_path)
        est_o, ate_odom, _ = aligner(odom[["x", "y"]].to_numpy(), odom["time"], gt, "Odometry")
else:
    print("groundtruth.csv not found — affichage en repère natif (pas d'ATE)")
    gt = None
    gxy = None
    est_b = traj[["x", "y"]].to_numpy(); ate = None
    est_d = ate_diso = est_o = ate_odom = None

# ── Translation cosmétique : tout ramener au départ (0,0) pour l'affichage ─────
# IMPORTANT : appliquée APRÈS le calcul des ATE → ne fausse PAS l'ATE.
# L'ATE reste calculé sur les coordonnées alignées par Umeyama.
def au_depart(xy):
    return xy - xy[0] if xy is not None and len(xy) else xy

gxy_p  = au_depart(gxy)
est_b_p = au_depart(est_b)
est_d_p = au_depart(est_d)
est_o_p = au_depart(est_o)

# ── Tracé ─────────────────────────────────────────────────────────────────────
if gxy_p is not None:
    ax.plot(gxy_p[:, 0], gxy_p[:, 1], label="Ground truth (GPS)", color="red", linestyle="--")
    ax.plot(gxy_p[-1, 0], gxy_p[-1, 1], marker="X", color="red", markersize=12)

if est_o_p is not None:
    lbl = f"Odometry (DISO, ATE={ate_odom:.1f} m)" if ate_odom is not None else "Odometry"
    ax.plot(est_o_p[:, 0], est_o_p[:, 1], label=lbl, color="orange", linestyle="-.", linewidth=1.5)
    ax.plot(est_o_p[-1, 0], est_o_p[-1, 1], marker="X", color="orange", markersize=12)

if est_d_p is not None:
    lbl = f"DISO standalone (ATE={ate_diso:.1f} m)" if ate_diso is not None else "DISO standalone"
    ax.plot(est_d_p[:, 0], est_d_p[:, 1], label=lbl, color="steelblue", linestyle=":", linewidth=2.0)
    ax.plot(est_d_p[-1, 0], est_d_p[-1, 1], marker="X", color="steelblue", markersize=12)

slam_label = f"Bruce-SLAM (DISO odom, iSAM2, ATE={ate:.1f} m)" if ate is not None else "Bruce-SLAM (DISO odom, iSAM2)"
ax.plot(est_b_p[:, 0], est_b_p[:, 1], label=slam_label, color="black", linewidth=1.5)
ax.plot(est_b_p[-1, 0], est_b_p[-1, 1], marker="X", color="black", markersize=12, label="End")

# toutes les trajectoires partent de (0,0) → un seul marqueur Start
ax.plot(0, 0, marker="*", color="green", markersize=16, label="Start (0,0)")

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Trajectory: SLAM vs Odometry vs Ground truth")
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
        cxy = cxy - est_b[0]  # même translation cosmétique que la trajectoire
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

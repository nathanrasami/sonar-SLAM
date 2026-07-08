#!/usr/bin/env python3
"""LA carte 3D unique d'un run : <run>/carte_3d.html (+ .png, .npy).

Règle de Nathan (08-07) : UNE carte par run, VRAIE 3D uniquement.
- Une source (topic de points) n'entre dans la carte QUE si son std(z)
  INTRA-message médian > 0.5 m (= le capteur voit du volume). Les tranches
  planes (pseudo-3D « 2D plaqué au z du robot ») sont EXCLUES ; s'il n'y a
  aucune vraie source, le script REFUSE (pas de carte mensongère).
- Carte 100 % GT-free : points composés avec la trajectoire SLAM.
  * bags v4 (frame_id=auv0) : composition directe pose SLAM × points véhicule ;
  * bags v3 (frame_id=map) : dé-projection via la pose GT du même stamp
    (décodage du format, aucune info GT dans le résultat) puis re-composition.
- Métrique : même carte composée avec les poses GT, alignement **Umeyama SE(2)**
  des trajectoires (remplace l'alignement première-pose : sur un grand site,
  le yaw à t0 × bras de levier dominait le NN).

Remplace comme livrable : view3d/carte_3d.png (pseudo), profiler_3d,
profiler_slam_3d, traj3_map_3d (gardés comme outils, plus des livrables).

Usage (conteneur ros1) :
  python3 analysis/carte_3d.py <run_dir> [--bag <bag>]
  (--bag omis : lit <run_dir>/bag_source.txt écrit par run_slam.sh)
"""
import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import rosbag
import sensor_msgs.point_cloud2 as pc2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_eval import umeyama, associer_par_temps

TOPICS = ["/profiler_points", "/sonar_points"]
SEUIL_VRAIE_3D = 0.5   # m, std(z) intra-message médian
N_SONDAGE = 60         # messages par topic pour le verdict vraie/pseudo 3D


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def rot_z(pts, th):
    c, s = np.cos(th), np.sin(th)
    return np.c_[c * pts[:, 0] - s * pts[:, 1],
                 s * pts[:, 0] + c * pts[:, 1], pts[:, 2]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--bag", default=None)
    a = ap.parse_args()
    bag_path = a.bag
    if bag_path is None:
        src = os.path.join(a.run, "bag_source.txt")
        if not os.path.isfile(src):
            sys.exit("carte_3d : pas de --bag ni de bag_source.txt — abandon")
        bag_path = open(src).read().strip()
    if not os.path.isfile(bag_path):
        sys.exit(f"carte_3d : bag introuvable : {bag_path}")

    traj = np.genfromtxt(os.path.join(a.run, "trajectory.csv"),
                         delimiter=",", names=True)
    ts, th_slam = traj["time"], np.unwrap(traj["theta"])
    rng = np.random.default_rng(0)

    # ── passe 1 : sondage — quelles sources sont de la VRAIE 3D ? ──────────
    stds = {t: [] for t in TOPICS}
    frames = {}
    seen = 0
    bag = rosbag.Bag(bag_path)
    for topic, m, _ in bag.read_messages(topics=TOPICS):
        seen += 1
        if seen > 4 * N_SONDAGE:   # borne dure : un topic absent ne bloque pas
            break
        if len(stds[topic]) >= N_SONDAGE:
            continue
        pts = np.array(list(pc2.read_points(m, field_names=("x", "y", "z"),
                                            skip_nans=True)))
        if len(pts) > 10:
            stds[topic].append(pts[:, 2].std())
            frames[topic] = m.header.frame_id
    retenus = []
    for t in TOPICS:
        if not stds[t]:
            continue
        med, p90 = float(np.median(stds[t])), float(np.percentile(stds[t], 90))
        # p90 : un capteur oscillant (tilt) alterne pings plats et pings 3D —
        # on retient le topic si une fraction substantielle est volumique,
        # puis le gate PAR MESSAGE (passe 2) ne garde que les pings 3D.
        ok = p90 > SEUIL_VRAIE_3D
        print(f"{t} : std(z) intra méd {med:.2f} / p90 {p90:.2f} m ({frames[t]}) → "
              f"{'vraie 3D (gate par ping en passe 2)' if ok else 'pseudo-3D (tranches), EXCLU'}")
        if ok:
            retenus.append(t)
    if not retenus:
        bag.close()
        sys.exit("carte_3d : AUCUNE source vraie-3D dans ce bag — pas de carte "
                 "(refus : une carte 2D-plaquée serait mensongère).")

    # ── GT (dé-projection v3 si besoin + métrique) ──────────────────────────
    gt_t, gt_x, gt_y, gt_z, gt_th = [], [], [], [], []
    for _, m, _ in bag.read_messages(topics=["/ground_truth"]):
        gt_t.append(m.header.stamp.to_sec())
        p = m.pose.pose.position
        gt_x.append(p.x); gt_y.append(p.y); gt_z.append(p.z)
        gt_th.append(yaw_of(m.pose.pose.orientation))
    gt_t = np.array(gt_t)
    gt_x, gt_y, gt_z = map(np.array, (gt_x, gt_y, gt_z))
    gt_th = np.unwrap(np.array(gt_th)) if gt_th else np.array([])
    has_gt = len(gt_t) > 0

    # ── passe 2 : construction (SLAM = livrable ; GT = référence métrique) ──
    S_all, G_all, counters = [], [], {t: 0 for t in retenus}
    for topic, m, _ in bag.read_messages(topics=retenus):
        counters[topic] += 1
        if counters[topic] % 2:
            continue
        t = m.header.stamp.to_sec()
        if t > ts[-1] + 0.5:
            continue
        pts = np.array(list(pc2.read_points(m, field_names=("x", "y", "z"),
                                            skip_nans=True)))
        if not len(pts):
            continue
        if pts[:, 2].std() <= SEUIL_VRAIE_3D:   # gate par PING : tranche plate
            continue                             # (ex. tilt≈0) → exclue
        if len(pts) > 300:
            pts = pts[rng.choice(len(pts), 300, replace=False)]
        if frames[topic] == "map":                 # v3 : monde → véhicule (GT)
            if not has_gt:
                continue
            gx = np.interp(t, gt_t, gt_x); gy = np.interp(t, gt_t, gt_y)
            gz = np.interp(t, gt_t, gt_z); gth = np.interp(t, gt_t, gt_th)
            pts = rot_z(pts - [gx, gy, gz], -gth)
        # composition pose SLAM (GT-free)
        w = rot_z(pts, np.interp(t, ts, th_slam))
        w += [np.interp(t, ts, traj["x"]), np.interp(t, ts, traj["y"]),
              np.interp(t, ts, traj["z"])]
        S_all.append(w[w[:, 2] < 0])
        if has_gt:                                  # référence carte-GT
            g = rot_z(pts, np.interp(t, gt_t, gt_th))
            g += [np.interp(t, gt_t, gt_x), np.interp(t, gt_t, gt_y),
                  np.interp(t, gt_t, gt_z)]
            G_all.append(g[g[:, 2] < 0])
    bag.close()
    S = np.vstack(S_all)
    titre_nn = ""

    # ── métrique : alignement UMEYAMA des trajectoires (pas première-pose) ──
    if has_gt and G_all:
        from scipy.spatial import cKDTree
        G = np.vstack(G_all)
        gxy = associer_par_temps(ts, gt_t, gt_x, gt_y)
        _, R, tr = umeyama(np.column_stack([traj["x"], traj["y"]]), gxy,
                           with_scale=False)
        z_off = np.median(np.interp(ts, gt_t, gt_z) - traj["z"])
        S_al = np.c_[(S[:, :2] @ R.T) + tr, S[:, 2] + z_off]
        idx = rng.choice(len(S_al), min(20000, len(S_al)), replace=False)
        dnn = cKDTree(G).query(S_al[idx], k=1)[0]
        med, p90 = np.median(dnn), np.percentile(dnn, 90)
        titre_nn = f" — NN vs carte GT {med:.2f}/{p90:.2f} m (Umeyama)"
        print(f"carte GT-free vs carte GT (Umeyama) : NN méd {med:.3f} m | "
              f"p90 {p90:.3f} m")

    # ── sorties : LA carte (repère SLAM, GT-free) ───────────────────────────
    run_name = os.path.basename(a.run.rstrip("/"))
    out = os.path.join(a.run, "carte_3d")
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(S[:, 0], S[:, 1], S[:, 2], s=0.3, c=S[:, 2], cmap="viridis",
               linewidths=0)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title(f"{run_name} — carte 3D GT-free (vraie 3D : "
                 f"{'+'.join(t.strip('/') for t in retenus)}, "
                 f"{len(S):,} pts){titre_nn}")
    fig.tight_layout(); fig.savefig(out + ".png", dpi=140)
    plt.close(fig)
    np.save(out + ".npy", S)
    try:
        import plotly.graph_objects as go
        i2 = rng.choice(len(S), min(150000, len(S)), replace=False)
        f2 = go.Figure(go.Scatter3d(
            x=S[i2, 0], y=S[i2, 1], z=S[i2, 2], mode="markers",
            marker=dict(size=1.2, color=S[i2, 2], colorscale="Viridis",
                        colorbar=dict(title="z (m)"))))
        f2.update_layout(title=f"{run_name} — carte 3D GT-free{titre_nn}",
                         scene=dict(aspectmode="data"),
                         margin=dict(l=0, r=0, t=40, b=0))
        f2.write_html(out + ".html", include_plotlyjs=True)
        print("->", out + ".html")
    except ImportError:
        print("⚠ plotly absent (pip3 install --user plotly) : pas de .html interactif")
    print("->", out + ".png/.npy")


if __name__ == "__main__":
    main()

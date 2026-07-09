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


def voxel_grid(P, vs):
    """Sous-échantillonnage voxel : 1 centroïde par cellule (dédoublonne, allège)."""
    if not len(P):
        return P
    k = np.floor(P / vs).astype(np.int64)
    _, inv, cnt = np.unique(k, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(cnt), 3))
    np.add.at(sums, inv, P)
    return sums / cnt[:, None]


def per_beam_max(pts):
    """Réduction « méthode grottes » (caves_3d.py) pour un profiler TRANSVERSE.

    pts : Nx4 (x,y,z,intensity) en repère véhicule, plan transverse x≡0. Chaque
    faisceau (azimut φ = atan2(-y,-z) dans le plan y-z) a ~2 retours : on garde le
    plus FORT = la paroi/le fond (1 faisceau ≈ 1 distance). C'est ce qui change une
    bouillie dense en sections propres empilées le long de la trajectoire → treillis
    et pilotis lisibles (validé sur traj3 : pilotis −19→−7 m, treillis en X visibles).
    """
    if len(pts) < 3 or pts.shape[1] < 4:
        return pts[:, :3] if len(pts) else pts
    y, z, i = pts[:, 1], pts[:, 2], pts[:, 3]
    key = np.round(np.degrees(np.arctan2(-y, -z)) * 2).astype(int)  # bin 0.5°
    order = np.lexsort((i, key))            # tri par faisceau puis intensité croissante
    ks = key[order]
    last = np.ones(len(ks), bool)
    last[:-1] = ks[1:] != ks[:-1]           # dernier de chaque faisceau = intensité max
    return pts[order][last][:, :3]


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
    stds = {t: [] for t in TOPICS}      # std(z) intra-ping (sonar tilté oscillant)
    stds_x = {t: [] for t in TOPICS}    # std(x) intra-ping (≈0 = fan transverse)
    stds_y = {t: [] for t in TOPICS}    # std(y) intra-ping
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
            stds_x[topic].append(pts[:, 0].std())
            stds_y[topic].append(pts[:, 1].std())
            frames[topic] = m.header.frame_id
    retenus = []
    for t in TOPICS:
        if not stds[t]:
            continue
        med, p90 = float(np.median(stds[t])), float(np.percentile(stds[t], 90))
        mx, my = float(np.median(stds_x[t])), float(np.median(stds_y[t]))
        # Deux géométries de vraie 3D :
        #  1. sonar tilté oscillant → std(z) intra-ping grand sur les pings inclinés
        #     (p90 : le tilt alterne pings plats/3D ; le gate par ping en passe 2 trie).
        #  2. profiler TRANSVERSE (fix §2.3ter) → fan dans le plan y-z : std(x)≈0 et
        #     std(y) grand. Un fond plat donne std(z)≈0 par ping (r=prof/cosφ) → le
        #     test std(z) le rejetterait à tort : on le détecte par sa géométrie x≡0.
        transverse = mx < 0.05 and my > SEUIL_VRAIE_3D
        ok = p90 > SEUIL_VRAIE_3D or transverse
        geo = ("profiler transverse (méthode grottes)" if transverse
               else "vraie 3D (gate par ping en passe 2)" if ok else "pseudo-3D (tranches), EXCLU")
        print(f"{t} : std(z) méd {med:.2f}/p90 {p90:.2f} | std(x) {mx:.2f} std(y) {my:.2f} m "
              f"({frames[t]}) → {geo}")
        if ok:
            retenus.append((t, transverse))
    # Priorité au profiler TRANSVERSE : c'est la source STRUCTURELLE propre (1 paroi
    # /faisceau, sections empilées → pilotis/treillis nets). Le sonar tilté, lui,
    # « pulvérise » des fans radiaux (la bouillie qu'on veut éviter) → on l'exclut
    # de la carte quand un profiler transverse est disponible.
    transverses = [t for t, tr in retenus if tr]
    if transverses:
        retenus = transverses
        print(f"→ carte STRUCTURELLE (méthode grottes) depuis {retenus} ; "
              f"sonar tilté exclu (fans radiaux).")
    else:
        retenus = [t for t, _ in retenus]
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

    # ── passe 2 : construction PAR SOURCE (SLAM = livrable ; GT = réf métrique) ──
    S_by, G_by = {t: [] for t in retenus}, {t: [] for t in retenus}
    counters = {t: 0 for t in retenus}
    for topic, m, _ in bag.read_messages(topics=retenus):
        counters[topic] += 1
        if counters[topic] % 2:
            continue
        t = m.header.stamp.to_sec()
        if t > ts[-1] + 0.5:
            continue
        prof = topic == "/profiler_points" and frames[topic] == "auv0"
        if prof:
            # profiler TRANSVERSE (x≡0) : réduction « grottes » 1 retour fort/faisceau.
            # PAS de gate std(z) par ping : un fond plat donne z≈cst (géométrie
            # r=prof/cos φ), c'est de la vraie 3D par construction (le fan balaye y-z).
            pts = np.array(list(pc2.read_points(
                m, field_names=("x", "y", "z", "intensity"), skip_nans=True)))
            if len(pts) < 3:
                continue
            pts = per_beam_max(pts)
            # ⚠ MIROIR y du profiler traj3 : les points /profiler_points ont l'axe
            # y INVERSÉ vs la convention véhicule (x avant, y GAUCHE) qu'attend
            # rot_z. Preuve (09-09) : sans flip, les murs de quai imagés par le
            # profiler tombent à x≈+2/+38 (au CENTRE) ; avec y négé ils reviennent
            # PILE sur les quais du sonar horizontal (x≈−10/+59). Bug de repère du
            # bag (mount profiler) → à corriger idéalement côté générateur ; en
            # attendant, on le corrige ici pour une carte juste.
            pts[:, 1] = -pts[:, 1]
        else:
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
        S_by[topic].append(w[w[:, 2] < 0])
        if has_gt:                                  # référence carte-GT
            g = rot_z(pts, np.interp(t, gt_t, gt_th))
            g += [np.interp(t, gt_t, gt_x), np.interp(t, gt_t, gt_y),
                  np.interp(t, gt_t, gt_z)]
            G_by[topic].append(g[g[:, 2] < 0])
    bag.close()
    S_full = np.vstack([p for lst in S_by.values() for p in lst])

    # ── CARTE = nuage 3D COMPLET, dédoublonné par voxel léger (densité) ─────
    # On garde TOUT (fond + structures) : c'est la carte honnête. Le voxel ne
    # fait que fusionner les points redondants pour une surface propre — il ne
    # retire PAS de structure (leçon 08-08 : un filtre de verticalité agressif
    # vidait la carte, cf. SLAM_3D_MIGRATION.md).
    S = voxel_grid(S_full, 0.2)
    kind = ("structurel — méthode grottes, 1 paroi/faisceau (voxel 0.2 m)"
            if retenus == ["/profiler_points"]
            else "nuage 3D complet (voxel 0.2 m)")
    G = voxel_grid(np.vstack([p for lst in G_by.values() for p in lst]), 0.2) \
        if has_gt and any(G_by.values()) else None
    print(f"carte : {len(S_full):,} pts bruts → {len(S):,} pts ({kind})")
    titre_nn = ""

    # ── métrique : alignement UMEYAMA des trajectoires (pas première-pose) ──
    if has_gt and G is not None and len(G):
        from scipy.spatial import cKDTree
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

    # ── overlay : nuage horizontal du SLAM (pointcloud.csv) = MURS DE QUAI en
    # plan, dans le même repère SLAM. Le profiler (méthode grottes) donne le
    # VOLUME (pilotis, treillis, fond) mais le fond marin large « dilue » le
    # cadrage ; le sonar horizontal, lui, ne voit que les structures verticales
    # (les 2 quais) → il ENCADRE la trajectoire, comme carte_finale. On superpose
    # les deux (idem caves_3d.py --with-map). GT-free (nuage SLAM natif). ──────
    Q = None
    pc_path = os.path.join(a.run, "pointcloud.csv")
    if os.path.isfile(pc_path):
        pc = np.genfromtxt(pc_path, delimiter=",", names=True)
        qz = pc["z"] if "z" in pc.dtype.names else np.zeros(len(pc))
        Q = np.column_stack([pc["x"], pc["y"], qz])
        if len(Q) > 60000:
            Q = Q[rng.choice(len(Q), 60000, replace=False)]

    # ── sorties : LA carte (repère SLAM, GT-free) ───────────────────────────
    run_name = os.path.basename(a.run.rstrip("/"))
    out = os.path.join(a.run, "carte_3d")
    tx, ty, tz = traj["x"], traj["y"], traj["z"]   # trajectoire SLAM (même repère)
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(S[:, 0], S[:, 1], S[:, 2], s=0.3, c=S[:, 2], cmap="viridis",
               linewidths=0, alpha=0.4)
    if Q is not None:                              # murs de quai (encadrent la traj)
        ax.scatter(Q[:, 0], Q[:, 1], Q[:, 2], s=0.5, c="darkorange",
                   linewidths=0, alpha=0.5, label="murs de quai (sonar horiz.)")
    ax.plot(tx, ty, tz, color="red", lw=1.8, label="trajectoire SLAM")
    ax.scatter(tx[0], ty[0], tz[0], c="lime", s=70, marker="^",
               depthshade=False, label="départ")
    ax.scatter(tx[-1], ty[-1], tz[-1], c="red", s=55, marker="s",
               depthshade=False, label="arrivée")
    ax.legend(loc="lower left", fontsize=8)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title(f"{run_name} — carte 3D GT-free — {kind}\n"
                 f"{len(S):,} pts{titre_nn}", fontsize=9)
    fig.tight_layout(); fig.savefig(out + ".png", dpi=140)
    plt.close(fig)
    np.save(out + ".npy", S)
    try:
        import plotly.graph_objects as go
        i2 = rng.choice(len(S), min(150000, len(S)), replace=False)
        f2 = go.Figure(go.Scatter3d(
            x=S[i2, 0], y=S[i2, 1], z=S[i2, 2], mode="markers",
            name="volume profiler (z)",
            marker=dict(size=1.2, color=S[i2, 2], colorscale="Viridis",
                        colorbar=dict(title="z (m)"))))
        if Q is not None:                          # murs de quai (encadrent la traj)
            f2.add_trace(go.Scatter3d(
                x=Q[:, 0], y=Q[:, 1], z=Q[:, 2], mode="markers",
                name="murs de quai (sonar horiz.)",
                marker=dict(size=1.4, color="darkorange", opacity=0.55)))
        # trajectoire SLAM par-dessus le nuage + marqueurs départ/arrivée
        f2.add_trace(go.Scatter3d(x=tx, y=ty, z=tz, mode="lines",
                                  line=dict(color="red", width=5),
                                  name="trajectoire SLAM"))
        f2.add_trace(go.Scatter3d(x=[tx[0]], y=[ty[0]], z=[tz[0]], mode="markers",
                                  marker=dict(color="lime", size=6, symbol="diamond"),
                                  name="départ"))
        f2.add_trace(go.Scatter3d(x=[tx[-1]], y=[ty[-1]], z=[tz[-1]], mode="markers",
                                  marker=dict(color="red", size=6, symbol="square"),
                                  name="arrivée"))
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

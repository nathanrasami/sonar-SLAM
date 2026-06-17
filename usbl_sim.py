#!/usr/bin/env python3
"""Mini-bac à sable HORS-LIGNE pour pré-évaluer la fusion USBL — SANS le SLAM.

But : décider si l'USBL peut améliorer l'odométrie cmd_vel AVANT de toucher au
back-end. On lit les premières secondes du bag (cmd_vel, usbl_point, pose_gt),
on reconstruit l'odométrie, et on compare 3 fusions sur LES MÊMES données :

  1. Dead-reckoning seul (cmd_vel intégré)        → référence, lisse mais dérive
  2. Filtre complémentaire (correction par à-coups) → ce qui zigzague
  3. Pose-graph (USBL = facteurs + moindres carrés) → ce que ferait gtsam

Métriques : ATE (RMSE brut vs GT, repère commun seedé à t=0) et RATIO DE TRAJET
(longueur estimée / longueur GT) = indicateur de zigzag (1.0 = lisse).

La GT (/pose_gt) ne sert QU'À mesurer l'erreur (jamais à la fusion, sauf seed t=0).

Usage :
    python3 usbl_sim.py [duree_s] [pas_noeud_s]
    DURATION=400 NODE_DT=1.5 python3 usbl_sim.py
"""
import os
import sys
import math
import numpy as np
from scipy.optimize import least_squares

BAG = os.environ.get("BAG_PATH", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                              "ARACATI_2017_8bits_full.bag"))
DURATION = float(os.environ.get("DURATION", sys.argv[1] if len(sys.argv) > 1 else 400))
NODE_DT  = float(os.environ.get("NODE_DT",  sys.argv[2] if len(sys.argv) > 2 else 1.5))
USBL_MAX_SPEED = 3.0   # m/s, gate outliers acoustiques (sauts ~73 m)
SIG_ODOM_T = 0.10      # sigma (m) sur la translation relative cmd_vel par pas
SIG_ODOM_R = 0.05      # sigma (rad) sur la rotation relative
SIG_USBL   = 1.4       # sigma (m) d'un fix USBL (mesuré : médiane 1.4 m vs GT)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def lire_bag():
    """Renvoie cmd[(t,vx,wz)], usbl[(t,x,y)], gt[(t,x,y,yaw)] sur les DURATION 1ères s."""
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    cmd, usbl, gt = [], [], []
    with Reader(BAG) as r:
        t0 = None
        for c, _, raw in r.messages():
            if c.topic not in ("/cmd_vel", "/usbl_point", "/pose_gt"):
                continue
            m = ts.deserialize_ros1(raw, c.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if t0 is None:
                t0 = t
            if t - t0 > DURATION:
                break
            if c.topic == "/cmd_vel":
                cmd.append((t, m.twist.linear.x, m.twist.angular.z))
            elif c.topic == "/usbl_point":
                usbl.append((t, m.point.x, m.point.y))
            else:
                q = m.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                gt.append((t, m.pose.position.x, m.pose.position.y, yaw))
    return sorted(cmd), sorted(usbl), sorted(gt, key=lambda g: g[0])


def dead_reckoning(cmd, seed):
    """Intègre cmd_vel (unicycle) depuis la pose seed (x,y,yaw). Renvoie poses (t,x,y,th)."""
    x, y, th = seed
    last = None
    out = []
    for t, vx, wz in cmd:
        if last is None:
            last = t
            out.append((t, x, y, th)); continue
        dt = t - last; last = t
        if dt <= 0:
            out.append((t, x, y, th)); continue
        x += vx * math.cos(th) * dt
        y += vx * math.sin(th) * dt
        th += wz * dt
        out.append((t, x, y, th))
    return np.array(out)


def filtre_complementaire(cmd, usbl, seed, K):
    """Dead-reckoning + correction position par à-coups à chaque fix USBL (gate vitesse)."""
    x, y, th = seed
    last = None; lu = None; ui = 0
    out = []
    for t, vx, wz in cmd:
        if last is None:
            last = t
        dt = t - last; last = t
        if dt > 0:
            x += vx * math.cos(th) * dt; y += vx * math.sin(th) * dt; th += wz * dt
        while ui < len(usbl) and usbl[ui][0] <= t:
            ut, ux, uy = usbl[ui]; ui += 1
            if lu is not None:
                pt, px, py = lu; ddt = ut - pt
                if ddt > 0 and math.hypot(ux - px, uy - py) / ddt > USBL_MAX_SPEED:
                    continue
            lu = (ut, ux, uy)
            x += K * (ux - x); y += K * (uy - y)
        out.append((t, x, y, th))
    return np.array(out)


def gate_usbl(usbl):
    """Rejette les outliers par vitesse vs dernier fix accepté."""
    keep = []; lu = None
    for ut, ux, uy in usbl:
        if lu is not None:
            pt, px, py = lu; ddt = ut - pt
            if ddt > 0 and math.hypot(ux - px, uy - py) / ddt > USBL_MAX_SPEED:
                continue
        lu = (ut, ux, uy); keep.append((ut, ux, uy))
    return keep


def pose_graph(cmd, usbl, seed, dr):
    """Optimisation moindres carrés (= ce que ferait gtsam) :
      - facteurs ODOMÉTRIE : mouvement relatif cmd_vel entre nœuds consécutifs
      - facteurs USBL      : prior de position absolue (robuste) sur le nœud le plus proche
      - ancre : prior fort sur le nœud 0 (seed GT)
    État optimisé : (x,y,th) par nœud. Init = dead-reckoning. Renvoie poses (t,x,y,th)."""
    # nœuds sous-échantillonnés à NODE_DT
    t_nodes = np.arange(dr[0, 0], dr[-1, 0], NODE_DT)
    idx = np.searchsorted(dr[:, 0], t_nodes)
    idx = np.clip(idx, 0, len(dr) - 1)
    P0 = dr[idx]                      # (N,4) init dead-reckoning
    N = len(P0)
    tn = P0[:, 0]

    # mesures odométriques relatives (depuis le dead-reckoning) en repère corps du nœud i
    meas = []
    for i in range(N - 1):
        xi, yi, thi = P0[i, 1], P0[i, 2], P0[i, 3]
        dxw, dyw = P0[i + 1, 1] - xi, P0[i + 1, 2] - yi
        dxb = math.cos(thi) * dxw + math.sin(thi) * dyw
        dyb = -math.sin(thi) * dxw + math.cos(thi) * dyw
        meas.append((dxb, dyb, wrap(P0[i + 1, 3] - thi)))

    # association USBL → nœud le plus proche en temps
    usbl_k = gate_usbl(usbl)
    usbl_assoc = []
    for ut, ux, uy in usbl_k:
        j = int(np.argmin(np.abs(tn - ut)))
        if abs(tn[j] - ut) < NODE_DT:
            usbl_assoc.append((j, ux, uy))

    def residus(s):
        P = s.reshape(N, 3)
        r = []
        # ancre nœud 0 (seed) — prior fort
        r += [(P[0, 0] - seed[0]) / 0.01, (P[0, 1] - seed[1]) / 0.01, wrap(P[0, 2] - seed[2]) / 0.01]
        # odométrie
        for i, (mdx, mdy, mdth) in enumerate(meas):
            xi, yi, thi = P[i]
            dxw, dyw = P[i + 1, 0] - xi, P[i + 1, 1] - yi
            dxb = math.cos(thi) * dxw + math.sin(thi) * dyw
            dyb = -math.sin(thi) * dxw + math.cos(thi) * dyw
            r += [(dxb - mdx) / SIG_ODOM_T, (dyb - mdy) / SIG_ODOM_T,
                  wrap(P[i + 1, 2] - thi - mdth) / SIG_ODOM_R]
        # USBL (prior position)
        for j, ux, uy in usbl_assoc:
            r += [(P[j, 0] - ux) / SIG_USBL, (P[j, 1] - uy) / SIG_USBL]
        return r

    s0 = P0[:, 1:4].flatten()
    sol = least_squares(residus, s0, loss="soft_l1", f_scale=2.0, max_nfev=200)
    P = sol.x.reshape(N, 3)
    return np.column_stack([tn, P]), len(usbl_assoc)


def metrics(poses, gt):
    gt = np.array(gt)
    gx = np.interp(poses[:, 0], gt[:, 0], gt[:, 1])
    gy = np.interp(poses[:, 0], gt[:, 0], gt[:, 2])
    ate = math.sqrt(((poses[:, 1] - gx) ** 2 + (poses[:, 2] - gy) ** 2).mean())
    plen = np.hypot(np.diff(poses[:, 1]), np.diff(poses[:, 2])).sum()
    glen = np.hypot(np.diff(gx), np.diff(gy)).sum()
    return ate, plen / max(glen, 1e-9)


def main():
    print(f"Lecture du bag (premières {DURATION:.0f} s)…")
    cmd, usbl, gt = lire_bag()
    print(f"  cmd_vel={len(cmd)}  usbl={len(usbl)}  gt={len(gt)}")
    if not cmd or not gt:
        raise SystemExit("Données manquantes dans le bag.")
    seed = (gt[0][1], gt[0][2], gt[0][3])

    dr = dead_reckoning(cmd, seed)
    res = [("1. Dead-reckoning (cmd_vel)", dr)]
    res.append(("2. Filtre complémentaire K=0.1", filtre_complementaire(cmd, usbl, seed, 0.1)))
    res.append(("2. Filtre complémentaire K=0.5", filtre_complementaire(cmd, usbl, seed, 0.5)))
    pg, n_usbl = pose_graph(cmd, usbl, seed, dr)
    res.append((f"3. Pose-graph USBL ({n_usbl} fixes)", pg))

    print(f"\n{'Méthode':<34} {'ATE (m)':>9} {'ratio trajet':>13}")
    print("-" * 58)
    for name, poses in res:
        ate, ratio = metrics(poses, gt)
        flag = "  ← zigzag" if ratio > 1.4 else ("  ← lisse" if ratio < 1.15 else "")
        print(f"{name:<34} {ate:>9.2f} {ratio:>13.2f}{flag}")

    # plot optionnel
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        gt = np.array(gt)
        fig, ax = plt.subplots(figsize=(9, 8))
        ax.plot(gt[:, 1], gt[:, 2], "r--", lw=2, label="GT")
        if len(usbl):
            u = np.array(usbl); ax.scatter(u[:, 1], u[:, 2], s=8, c="green", alpha=0.4, label="USBL (fixes)")
        for name, poses in res:
            ax.plot(poses[:, 1], poses[:, 2], lw=1.2, label=name)
        ax.axis("equal"); ax.grid(True); ax.legend(fontsize=8)
        ax.set_title(f"Sandbox fusion USBL — {DURATION:.0f} s, pas {NODE_DT} s")
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "usbl_sim.png")
        plt.tight_layout(); plt.savefig(out, dpi=140)
        print(f"\nPlot : {out}")
    except ImportError:
        pass


if __name__ == "__main__":
    main()

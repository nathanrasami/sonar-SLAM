#!/usr/bin/env python3
"""Figures « ancré origine SEULEMENT » pour le papier (décision Nathan/tutrice :
comparaison origine, pas d'Umeyama dans les figures).

Convention d'ancrage = CELLE PRÉSENTÉE À LA TUTRICE (analyze_origine.aligner_origine,
2026-07-17) : translation au départ + ORIENTATION INITIALE ajustée par Umeyama
(rot|réflexion, sans échelle) sur les 15 premiers %% de POINTS (la GT DGPS n'a pas
d'orientation ; les publiés ancrent leur 1re pose au magnétomètre). Validation :
notre odométrie ré-ancrée par section reproduit la ligne « Odometry alone » de la
Table I d'ISOPoT (5.05/11.69/7.62 vs 5.8/12.5/6.5). Les sections S1/S2/S3 (tiers
temporels) sont RÉ-ANCRÉES au début de chaque section, même règle.
⚠ la corde 15 m (ancien choix) donnait 5.08 au lieu de 1.83 sur BSU : ancre brûlée
par un début localement bruité — ne pas y revenir sans décision de Nathan.

Produit dans --out :
  <label>_traj_origine.png       GT vs SLAM vs odométrie, ancrés départ
  <label>_err_time_origine.png   |erreur| position dans le temps (ancré), bornes S1/S2/S3
  <label>_sections_origine.png   1x3 : chaque section ré-ancrée + ATE fp par section

Usage :
  python3 analysis/paper_figs_origine.py <run_dir> --label Bruce --out Paper/Image
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from paper_eval import charger  # noqa: E402
from traj_eval import umeyama, calculer_ate  # noqa: E402

C_GT, C_SLAM, C_ODOM = "tab:green", "tab:blue", "tab:orange"


def au_depart(xy):
    return xy - xy[0]


def ate_origine(src_xy, gt_xy, frac=0.15):
    """Convention tutrice : départ (0,0) + orientation initiale fit sur 15 % des points.
    Retourne (ATE, trajectoire alignée EXPRIMÉE DANS LE REPÈRE GT MONDE)."""
    gt_p, src_p = au_depart(gt_xy), au_depart(src_xy)
    n = max(2, int(round(len(src_p) * frac)))
    _, R, _ = umeyama(src_p[:n], gt_p[:n], with_scale=False, allow_reflection=True)
    est_p = au_depart((R @ src_p.T).T)
    return calculer_ate(est_p, gt_p), est_p + gt_xy[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--label", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--sections", type=int, default=3)
    a = ap.parse_args()
    out = a.out or a.run_dir
    os.makedirs(out, exist_ok=True)

    d = charger(a.run_dir)
    est, gxy = d["est"], d["gxy"]
    t_min = (d["t"] - d["t"][0]) / 60.0

    # odométrie : odometry.csv (pleine cadence, comme analyze_origine) si présent
    op = os.path.join(a.run_dir, "odometry.csv")
    if os.path.exists(op):
        od = np.genfromtxt(op, delimiter=",", names=True)
        gt = np.genfromtxt(os.path.join(a.run_dir, "groundtruth.csv"), delimiter=",", names=True)
        dr = np.column_stack([od["x"], od["y"]])
        gxy_o = np.column_stack([np.interp(od["time"], gt["time"], gt["x"]),
                                 np.interp(od["time"], gt["time"], gt["y"])])
    else:
        dr, gxy_o = d["dr"], gxy

    ate_fp, est_fp = ate_origine(est, gxy)
    ate_o_fp, dr_fp = ate_origine(dr, gxy_o)

    # ---- 1) trajectoire ancrée origine
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(gxy[:, 0], gxy[:, 1], color=C_GT, lw=1.6, label="Ground truth (DGPS)")
    ax.plot(est_fp[:, 0], est_fp[:, 1], color=C_SLAM, lw=1.3,
            label=f"SLAM, start-anchored — ATE {ate_fp:.2f} m")
    ax.plot(dr_fp[:, 0], dr_fp[:, 1], color=C_ODOM, lw=1.0, ls="--",
            label=f"Odometry, start-anchored — ATE {ate_o_fp:.2f} m")
    ax.plot(*gxy[0], "k*", ms=12, label="Common start")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(loc="best", fontsize=9)
    ax.set_title(f"{a.label} — first-pose alignment (initial orientation on first 15% of poses)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"{a.label}_traj_origine.png"), dpi=150)
    plt.close(fig)

    # ---- 2) erreur position dans le temps (ancré origine seulement)
    e_slam = np.linalg.norm(est_fp - gxy, axis=1)
    e_odom = np.linalg.norm(dr_fp - gxy_o, axis=1)
    t_odom = (np.linspace(d["t"][0], d["t"][-1], len(e_odom)) - d["t"][0]) / 60.0 \
        if len(e_odom) != len(t_min) else t_min
    if os.path.exists(op):
        od_t = np.genfromtxt(op, delimiter=",", names=True)["time"]
        t_odom = (od_t - d["t"][0]) / 60.0
    tb = np.linspace(d["t"][0], d["t"][-1], a.sections + 1)
    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.plot(t_odom, e_odom, color=C_ODOM, lw=1.0, ls="--",
            label=f"Odometry (end {e_odom[-1]:.1f} m)")
    ax.plot(t_min, e_slam, color=C_SLAM, lw=1.3,
            label=f"SLAM (ATE {ate_fp:.2f} m)")
    for k in range(1, a.sections):
        ax.axvline((tb[k] - d["t"][0]) / 60.0, color="gray", ls=":", lw=0.8)
    for k in range(a.sections):
        xm = ((tb[k] + tb[k + 1]) / 2 - d["t"][0]) / 60.0
        ax.text(xm, ax.get_ylim()[1] * 0.92, f"S{k+1}", ha="center",
                color="gray", fontsize=9)
    ax.set_xlabel("mission time (min)")
    ax.set_ylabel("position error (m)")
    ax.grid(alpha=0.3); ax.legend(loc="upper left", fontsize=9)
    ax.set_title(f"{a.label} — error over time, start-anchored")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"{a.label}_err_time_origine.png"), dpi=150)
    plt.close(fig)

    # ---- 3) sections ré-ancrées (mêmes graphiques que la traj entière, par section)
    fig, axes = plt.subplots(1, a.sections, figsize=(4.2 * a.sections, 4.4))
    ates = []
    for k, ax in enumerate(np.atleast_1d(axes)):
        m = (d["t"] >= tb[k]) & (d["t"] <= tb[k + 1])
        a_fp, e_fp = ate_origine(est[m], gxy[m])
        ates.append(a_fp)
        ax.plot(gxy[m, 0], gxy[m, 1], color=C_GT, lw=1.5, label="GT")
        ax.plot(e_fp[:, 0], e_fp[:, 1], color=C_SLAM, lw=1.2, label="SLAM re-anchored")
        ax.plot(*gxy[m][0], "k*", ms=10)
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title(f"S{k+1} — ATE {a_fp:.2f} m", fontsize=10)
        ax.set_xlabel("x (m)")
        if k == 0:
            ax.set_ylabel("y (m)"); ax.legend(fontsize=8, loc="best")
    fig.suptitle(f"{a.label} — sections re-anchored at their start (DISO/ISOPoT convention)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, f"{a.label}_sections_origine.png"), dpi=150)
    plt.close(fig)

    print(f"{a.label}: ATE fp global {ate_fp:.2f} m | sections "
          + " / ".join(f"{x:.2f}" for x in ates)
          + f" | odom fp {ate_o_fp:.2f} m -> {out}")


if __name__ == "__main__":
    main()

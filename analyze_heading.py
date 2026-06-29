#!/usr/bin/env python3
"""analyze_heading.py — dérive de cap (theta) : comparaison GT vs runs, dans le BON repère.

IMPORTANT (repère) : le theta du SLAM est dans le repère MAP, qui est en ROLL π par rapport
au repère world (cf. static_transform_publisher "world map ... 3.14159"). L'axe Y est donc
inversé → theta_map = -theta_world. La référence vraie = le theta GT (groundtruth.csv, yaw du
quaternion /pose_gt = compas). On ramène chaque run dans le repère GT en détectant le signe
(±1) + l'offset constant qui minimisent l'erreur, et on le REPORTE explicitement (l'ancien
align() le masquait → plots faux).

Usage :
    python3 analyze_heading.py <run_gt> <run_cmdvel> [...]
    (ou SLAM_RESULTS_DIR=<run> python3 analyze_heading.py)

Chaque <run> = dossier avec trajectory.csv (time,x,y,theta,...,nssm_constraints) et
groundtruth.csv (time,x,y,theta). Sorties (dans le 1er run) :
    heading_vs_time.png   heading_error_time.png   heading_error_dist.png
"""
import os, sys
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

def wrap(a): return np.arctan2(np.sin(a), np.cos(a))

runs = sys.argv[1:] or [os.environ.get("SLAM_RESULTS_DIR", ".")]


def load(run):
    t = pd.read_csv(os.path.join(run, "trajectory.csv"))
    g = pd.read_csv(os.path.join(run, "groundtruth.csv")).sort_values("time").reset_index(drop=True)
    if "theta" not in g.columns:
        raise SystemExit(f"{run}/groundtruth.csv n'a pas de colonne 'theta' (run trop ancien)")
    gth = wrap(np.interp(t.time, g.time, np.unwrap(g.theta.values)))  # GT theta aux keyframes
    return t, gth


def to_gt_frame(theta, gth):
    """Ramène theta dans le repère GT : signe ±1 + offset constant (moyenne circulaire) min erreur."""
    best = None
    for s in (1.0, -1.0):
        d = wrap(s * theta - gth)
        off = np.arctan2(np.sin(d).mean(), np.cos(d).mean())
        e = np.abs(wrap(s * theta - off - gth))
        if best is None or e.mean() < best[0]:
            best = (e.mean(), s, off)
    _, s, off = best
    return wrap(s * theta - off), s, off


def dist_along(t):
    d = np.hypot(np.diff(t.x.values), np.diff(t.y.values))
    return np.concatenate([[0.0], np.cumsum(d)])


colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]
out_dir = runs[0]

fig1, ax1 = plt.subplots(figsize=(13, 5))
fig2, ax2 = plt.subplots(figsize=(13, 5))
fig3, ax3 = plt.subplots(figsize=(13, 5))

# Référence GT (du 1er run) tracée une fois
t0, gth0 = load(runs[0]); rel0 = t0.time.values - t0.time.values[0]
ax1.plot(rel0, np.degrees(np.unwrap(gth0)), "k-", lw=2.4, alpha=.9, label="GT theta (vrai cap)")

print("=== dérive de cap (theta) vs GT ===")
for i, run in enumerate(runs):
    t, gth = load(run); rel = t.time.values - t.time.values[0]
    th, s, off = to_gt_frame(t.theta.values, gth)
    err = np.degrees(np.abs(wrap(th - gth)))
    d = dist_along(t)
    name = os.path.basename(run.rstrip("/"))
    refl = "REFLECHI" if s < 0 else "direct  "
    print(f"[{name}]")
    print(f"    repère {refl} (offset {np.degrees(off):+.0f}°)")
    print(f"    erreur cap : moyenne {err.mean():5.1f}°   max {err.max():5.0f}°   std {err.std():5.1f}°")
    c = colors[i % len(colors)]
    ax1.plot(rel, np.degrees(np.unwrap(th)), color=c, lw=1.2, alpha=.85, label=name)
    ax2.plot(rel, err, color=c, lw=1.0, alpha=.85, label=f"{name} (moy {err.mean():.1f}°)")
    ax3.plot(d, err, color=c, lw=1.0, alpha=.85, label=f"{name} (moy {err.mean():.1f}°)")
    if "nssm_constraints" in t.columns:
        lc = t.nssm_constraints.values > 0
        if lc.any():
            ax2.plot(rel[lc], err[lc], "o", color=c, ms=4, mec="k", mew=.4)
            ax3.plot(d[lc], err[lc], "o", color=c, ms=4, mec="k", mew=.4,
                     label="loop closure" if i == 0 else None)

ax1.set_xlabel("Temps (s)"); ax1.set_ylabel("Cap déroulé (°)")
ax1.set_title("Cap theta : GT vs runs (ramenés au repère GT)")
ax2.set_xlabel("Temps (s)"); ax2.set_ylabel("|erreur de cap| (°)")
ax2.set_title("Erreur de cap vs GT au cours du temps (● = loop closure)")
ax3.set_xlabel("Distance parcourue (m)"); ax3.set_ylabel("|erreur de cap| (°)")
ax3.set_title("Erreur de cap vs GT en fonction de la distance")
for ax in (ax1, ax2, ax3):
    ax.legend(fontsize=9); ax.grid(alpha=.3)

for fig, fn in [(fig1, "heading_vs_time"), (fig2, "heading_error_time"), (fig3, "heading_error_dist")]:
    fig.tight_layout(); p = os.path.join(out_dir, fn + ".png"); fig.savefig(p, dpi=140)
    print("PNG :", p)

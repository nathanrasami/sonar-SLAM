#!/usr/bin/env python3
"""plot_heading_comparison.py — caractérisation de l'erreur de cap pour présentation.

3 subplots :
  1. Cap GT reconstruit (gt_course depuis groundtruth.csv)
  2. Cap DISO (run GT-prior) vs GT
  3. Notre cap GT-free (-dr_theta = compas) vs GT

Sortie : heading_characterization.png dans SLAM_RESULTS_DIR (run cmd_vel).

Usage :
    SLAM_RESULTS_DIR=TESTS_image/run_..._150959 python3 plot_heading_comparison.py
"""
import os, bisect
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DISO_DIR = "results/run_aracati_Bruce_DISO_Sonar_2026-06-16_120307"
CV_DIR   = os.environ.get("SLAM_RESULTS_DIR",
           "TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959")

def wrap(a): return np.arctan2(np.sin(a), np.cos(a))

def gt_course_from_gt(gt_csv, t_query, dt=1.5):
    """Cap GT = atan2 du déplacement GT sur ±dt autour de chaque instant."""
    gtp = pd.read_csv(gt_csv).sort_values("time").reset_index(drop=True)
    gtt = gtp.time.to_numpy(); gx = gtp.x.to_numpy(); gy = gtp.y.to_numpy()
    out = np.full(len(t_query), np.nan)
    for k, tq in enumerate(t_query):
        i0 = max(bisect.bisect_left(gtt, tq - dt), 0)
        i1 = min(bisect.bisect_left(gtt, tq + dt), len(gtt) - 1)
        if i1 > i0:
            out[k] = np.arctan2(gy[i1] - gy[i0], gx[i1] - gx[i0])
    return out

def align_to_ref(vals, ref):
    """Retire l'offset de repère (médiane circulaire) pour comparer les variations."""
    ok = ~np.isnan(ref) & ~np.isnan(vals)
    if not ok.any(): return vals
    off = wrap(vals[ok] - ref[ok])
    med = np.arctan2(np.sin(off).mean(), np.cos(off).mean())
    return wrap(vals - med)

def error_stats(aligned, ref):
    ok = ~np.isnan(ref) & ~np.isnan(aligned)
    e = np.degrees(np.abs(wrap(aligned[ok] - ref[ok])))
    return e, e.mean(), np.median(e)

# ── Chargement ─────────────────────────────────────────────────────────────
diso_traj = pd.read_csv(os.path.join(DISO_DIR, "trajectory.csv"))
cv_traj   = pd.read_csv(os.path.join(CV_DIR,   "trajectory.csv"))

gt_diso = gt_course_from_gt(os.path.join(DISO_DIR, "groundtruth.csv"), diso_traj.time.to_numpy())
gt_cv   = gt_course_from_gt(os.path.join(CV_DIR,   "groundtruth.csv"), cv_traj.time.to_numpy())

# DISO theta est en repère réfléchi → on inverse le signe pour comparer à GT course
diso_theta = -diso_traj.theta.to_numpy()    # -theta : undo reflection
cv_theta   = cv_traj.theta.to_numpy()       # theta SLAM cmd_vel (direct)

t_diso = diso_traj.time.to_numpy() - diso_traj.time.iloc[0]
t_cv   = cv_traj.time.to_numpy()   - cv_traj.time.iloc[0]

# Alignement sur GT course (retire offset de repère constant)
diso_aligned = align_to_ref(diso_theta, gt_diso)
cv_aligned   = align_to_ref(cv_theta,   gt_cv)

err_diso, avg_diso, med_diso = error_stats(diso_aligned, gt_diso)
err_cv,   avg_cv,   med_cv   = error_stats(cv_aligned,   gt_cv)

print(f"DISO  theta  : avg={avg_diso:.1f}°  med={med_diso:.1f}°")
print(f"Notre compas : avg={avg_cv:.1f}°   med={med_cv:.1f}°")

# ── Figure ─────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=False)
fig.suptitle("Caractérisation de l'erreur de cap", fontsize=14, fontweight="bold")

GREY   = "#555555"
BLUE   = "#1f77b4"
ORANGE = "#d62728"
GT_COLOR = "#2ca02c"

valid_diso = ~np.isnan(gt_diso)
valid_cv   = ~np.isnan(gt_cv)

# ── Subplot 1 : cap GT reconstruit ─────────────────────────────────────────
ax = axes[0]
ax.plot(t_cv[valid_cv], np.degrees(gt_cv[valid_cv]), color=GT_COLOR, lw=1.8)
ax.set_ylabel("Cap (°)", fontsize=11)
ax.set_title("Cap GT reconstruit (course depuis groundtruth.csv)", fontsize=11)
ax.grid(True, alpha=0.3)
ax.set_xlim(0, max(t_cv[-1], t_diso[-1]))

# ── Subplot 2 : cap DISO vs GT ─────────────────────────────────────────────
ax = axes[1]
ax.plot(t_diso[valid_diso], np.degrees(gt_diso[valid_diso]),
        color=GT_COLOR, lw=1.5, alpha=0.6, linestyle="--", label="GT course")
ax.plot(t_diso, np.degrees(diso_aligned), color=BLUE, lw=1.5, label="DISO −θ (aligné)")

# bande d'erreur ±1 std
t_ok = t_diso[valid_diso]
a_ok = np.degrees(diso_aligned[valid_diso])
r_ok = np.degrees(gt_diso[valid_diso])
ax.fill_between(t_ok, r_ok - avg_diso, r_ok + avg_diso,
                color=BLUE, alpha=0.12, label=f"±avg ({avg_diso:.1f}°)")

ax.set_ylabel("Cap (°)", fontsize=11)
ax.set_title(f"Cap DISO (prior GT, repère corrigé)  —  avg={avg_diso:.1f}°  med={med_diso:.1f}°", fontsize=11)
ax.legend(fontsize=9, loc="upper right")
ax.grid(True, alpha=0.3)
ax.set_xlim(0, max(t_cv[-1], t_diso[-1]))

# ── Subplot 3 : notre cap GT-free vs GT ────────────────────────────────────
ax = axes[2]
ax.plot(t_cv[valid_cv], np.degrees(gt_cv[valid_cv]),
        color=GT_COLOR, lw=1.5, alpha=0.6, linestyle="--", label="GT course")
ax.plot(t_cv, np.degrees(cv_aligned), color=ORANGE, lw=1.5, label="Notre cap (θ SLAM)")

t_ok2 = t_cv[valid_cv]
r_ok2 = np.degrees(gt_cv[valid_cv])
ax.fill_between(t_ok2, r_ok2 - avg_cv, r_ok2 + avg_cv,
                color=ORANGE, alpha=0.12, label=f"±avg ({avg_cv:.1f}°)")

ax.set_xlabel("Temps (s)", fontsize=11)
ax.set_ylabel("Cap (°)", fontsize=11)
ax.set_title(f"Notre cap GT-free (θ SLAM cmd_vel+USBL)  —  avg={avg_cv:.1f}°  med={med_cv:.1f}°", fontsize=11)
ax.legend(fontsize=9, loc="upper right")
ax.grid(True, alpha=0.3)
ax.set_xlim(0, max(t_cv[-1], t_diso[-1]))

plt.tight_layout()
out = os.path.join(CV_DIR, "heading_characterization.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"PNG : {out}")

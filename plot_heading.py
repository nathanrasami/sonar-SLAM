#!/usr/bin/env python3
"""plot_heading.py — erreur de cap au cours du temps (façon plot_trajectories.py mais pour le heading).

Compare par keyframe (alignés sur gt_heading.csv) :
  - SLAM theta       (trajectory.csv : theta)
  - cmd_vel heading  (trajectory.csv : dr_theta)
  - compas GT-free   (= -dr_theta, identité wz = -d(compas)/dt)
  - GT course        (gt_heading.csv : gt_course)
  - GT compas        (gt_heading.csv : gt_compass)

Sortie :
  heading_comparison.png   — cap brut au cours du temps + GT
  heading_error.png        — |erreur angulaire| vs GT compas par keyframe

Usage :
    SLAM_RESULTS_DIR=TESTS_image/run_... python3 plot_heading.py
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

results_dir = os.environ.get(
    "SLAM_RESULTS_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959")
)

def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))

def angular_err_deg(est, ref):
    return np.degrees(np.abs(wrap(est - ref)))

# ── Chargement ────────────────────────────────────────────────────────────────
traj_path = os.path.join(results_dir, "trajectory.csv")
gth_path  = os.path.join(results_dir, "gt_heading.csv")

if not os.path.exists(traj_path):
    raise SystemExit(f"trajectory.csv introuvable dans {results_dir}")
if not os.path.exists(gth_path):
    raise SystemExit(f"gt_heading.csv introuvable dans {results_dir} — lance reverse_cap.py d'abord")

traj = pd.read_csv(traj_path)
gth  = pd.read_csv(gth_path)

# Fusion sur keyframe_id (inner join → seulement les KF communs)
df = traj.merge(gth, on="keyframe_id", suffixes=("", "_gt"))
t = df["time"].to_numpy()
rel_t = t - t[0]

slam_theta  = df["theta"].to_numpy()
dr_theta    = df["dr_theta"].to_numpy()
compass_gf  = -dr_theta                       # 100% GT-free, = compas (identité)
gt_course   = df["gt_course"].to_numpy()
gt_compass  = df["gt_compass"].to_numpy()

# Référence angulaire = GT compas si dispo, sinon GT course
has_compass = not np.all(np.isnan(gt_compass))
ref = gt_compass if has_compass else gt_course
ref_label = "GT compas" if has_compass else "GT course"
print(f"Référence cap : {ref_label}")

series = [
    ("SLAM theta",       slam_theta,  "blue"),
    ("cmd_vel heading",  dr_theta,    "red"),
    ("compas GT-free",   compass_gf,  "darkgreen"),
    ("GT course",        gt_course,   "darkorange"),
]

def align_offset(vals, ref):
    """Soustrait l'offset médian (repère) pour ne montrer que l'erreur de suivi. Ignore les NaN."""
    valid = ~np.isnan(ref) & ~np.isnan(vals)
    if not valid.any():
        return vals
    offsets = wrap(vals[valid] - ref[valid])
    median_off = np.arctan2(np.sin(offsets).mean(), np.cos(offsets).mean())
    return wrap(vals - median_off)

# ── Plot 1 : cap aligné overtime (offset repère retiré) ────────────────────
fig, ax = plt.subplots(figsize=(14, 6))
ax.plot(rel_t, np.degrees(ref), "k-", lw=2, alpha=0.9, label=f"{ref_label} (référence)")
for label, vals, color in series:
    aligned = align_offset(vals, gt_compass)
    ax.plot(rel_t, np.degrees(aligned), color=color, lw=1.4, alpha=0.75, label=label)
ax.set_xlabel("Temps (s)", fontsize=12)
ax.set_ylabel("Cap aligné (°)", fontsize=12)
ax.set_title("Comparaison des sources de cap — alignées sur GT compas (offset repère retiré)", fontsize=13)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
plt.tight_layout()
out1 = os.path.join(results_dir, "heading_comparison.png")
plt.savefig(out1, dpi=150, bbox_inches="tight")
print(f"Cap aligné : {out1}")

# ── Plot 2 : erreur angulaire résiduelle vs GT compas ───────────────────────
fig, ax = plt.subplots(figsize=(14, 6))
stats_lines = []
for label, vals, color in series:
    aligned = align_offset(vals, ref)
    err = angular_err_deg(aligned, ref)
    valid_mask = ~np.isnan(err)
    ax.plot(rel_t[valid_mask], err[valid_mask], color=color, lw=1.5, alpha=0.85, label=label)
    e = err[valid_mask]
    line = f"{label}: avg={e.mean():.1f}°  med={np.median(e):.1f}°  max={e.max():.1f}°"
    stats_lines.append(line)
    print(f"[{label}] avg={e.mean():.1f}°  med={np.median(e):.1f}°  max={e.max():.1f}°")

ax.set_xlabel("Temps (s)", fontsize=12)
ax.set_ylabel("|Erreur cap résiduelle| (°)", fontsize=12)
ax.set_title(f"Erreur de cap résiduelle vs {ref_label} (après retrait offset repère)", fontsize=13)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.text(0.02, 0.98, "\n".join(stats_lines), transform=ax.transAxes,
        fontsize=9, va="top", bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.75))
plt.tight_layout()
out2 = os.path.join(results_dir, "heading_error.png")
plt.savefig(out2, dpi=150, bbox_inches="tight")
print(f"Erreur cap : {out2}")

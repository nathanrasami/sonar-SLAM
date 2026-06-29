#!/usr/bin/env python3
"""plot_theta_error.py — pourquoi le point cloud est mauvais : l'erreur de CAP.

Le cloud se rend keyframe par keyframe avec le cap de chaque keyframe. Si ce cap est
bruité, les scans se désalignent → "tourbillon" (swirl). Ce script trace, pour un run
Bruce_Sonar_USBL :

  (haut)  les caps au cours du temps : theta SLAM (celui qui REND le cloud par défaut)
          vs notre cap corrigé compas (-dr_theta, utilisé pour RViz + le CSV du cloud),
          alignés sur le cap GT (course) — on voit lequel suit le mieux.
  (bas)   l'ERREUR DE CAP LOCALE (variation scan-à-scan vs GT) : c'est elle qui cause le
          swirl. Plus elle est grande, plus le cloud bave.

Sortie : <run>/theta_error.png   |   Usage : SLAM_RESULTS_DIR=results/run_... python3 plot_theta_error.py
"""
import os, bisect
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = os.environ.get("SLAM_RESULTS_DIR", "results/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959")
def wrap(a): return np.arctan2(np.sin(a), np.cos(a))

traj = pd.read_csv(os.path.join(R, "trajectory.csv"))
gt = pd.read_csv(os.path.join(R, "groundtruth.csv")).sort_values("time").reset_index(drop=True)
t = traj.time.to_numpy(); rel_t = t - t[0]
theta = traj.theta.to_numpy()            # cap SLAM (rend le cloud par défaut)
compass = -traj.dr_theta.to_numpy()      # notre cap corrigé (RViz + CSV cloud), GT-free

if "theta" in gt.columns:
    gtt, gth = gt.time.to_numpy(), gt.theta.to_numpy()
    gth_unwrapped = np.unwrap(gth)
    gt_cap = np.interp(t, gtt, gth_unwrapped)
    gt_cap = wrap(gt_cap)
    ok = np.ones(len(gt_cap), dtype=bool)
else:
    # cap GT = course (atan2 du déplacement DGPS sur ±1.5 s) — fallback
    gtt, gx, gy = gt.time.to_numpy(), gt.x.to_numpy(), gt.y.to_numpy()
    def course(tq, dt=1.5):
        i0 = max(bisect.bisect_left(gtt, tq-dt), 0); i1 = min(bisect.bisect_left(gtt, tq+dt), len(gtt)-1)
        return np.arctan2(gy[i1]-gy[i0], gx[i1]-gx[i0]) if i1 > i0 else np.nan
    gt_cap = np.array([course(x) for x in t])
    ok = ~np.isnan(gt_cap)

def align(v, ref):
    """retire l'offset/réflexion de repère : teste +v et -v, garde le mieux aligné sur ref."""
    best = None
    for s in (1.0, -1.0):
        d = wrap(s*v[ok] - ref[ok]); off = np.arctan2(np.sin(d).mean(), np.cos(d).mean())
        e = np.abs(wrap(s*v - off - ref))
        if best is None or np.nanmean(e[ok]) < best[0]:
            best = (np.nanmean(e[ok]), s, off)
    _, s, off = best
    return wrap(s*v - off)

theta_a = align(theta, gt_cap)
compass_a = align(compass, gt_cap)

# erreur de cap INSTANTANÉE vs GT (theta est le cap qui rend le cloud)
err_th = np.degrees(np.abs(wrap(theta_a - gt_cap)))
err_th_ok = err_th[ok]

# ── Figure ──────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(2, 1, figsize=(13, 9))
fig.suptitle("Erreur de cap Bruce-SLAM → pourquoi le point cloud bave", fontsize=14, fontweight="bold")

# (haut) cap SLAM vs cap GT, dérroulés (unwrap) pour la lisibilité
ax[0].plot(rel_t[ok], np.degrees(np.unwrap(gt_cap[ok])), "k-", lw=2, alpha=.85, label="Cap GT (course)")
th_u = np.unwrap(np.where(ok, theta_a, np.nan)[ok])
ax[0].plot(rel_t[ok], np.degrees(th_u), color="#d62728", lw=1.3, alpha=.85,
           label="cap SLAM theta (rend le cloud)")
ax[0].set_ylabel("Cap déroulé (°)"); ax[0].legend(fontsize=10); ax[0].grid(alpha=.3)
ax[0].set_title("Le cap qui place chaque scan : SLAM vs GT", fontsize=11)

# (bas) erreur de cap au cours du temps + bavure équivalente
ax[1].plot(rel_t[ok], err_th_ok, color="#d62728", lw=1, alpha=.85)
ax[1].axhline(np.median(err_th_ok), color="k", ls="--", lw=1,
              label=f"médiane {np.median(err_th_ok):.1f}°  (moy {err_th_ok.mean():.1f}°, max {err_th_ok.max():.0f}°)")
ax[1].set_xlabel("Temps (s)"); ax[1].set_ylabel("|erreur de cap| (°)")
ax[1].legend(fontsize=10); ax[1].grid(alpha=.3)
smear = np.tan(np.radians(np.median(err_th_ok))) * 30.0
ax[1].set_title(f"Erreur de cap du SLAM → ~{smear:.1f} m de bavure à 30 m de portée = le swirl", fontsize=11)

plt.tight_layout()
out = os.path.join(R, "theta_error.png"); plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"cap SLAM : erreur médiane {np.median(err_th_ok):.1f}°  moy {err_th_ok.mean():.1f}°  max {err_th_ok.max():.0f}°")
print(f"          → ~{smear:.1f} m de bavure à 30 m (cause du point cloud swirl)")
print(f"PNG : {out}")

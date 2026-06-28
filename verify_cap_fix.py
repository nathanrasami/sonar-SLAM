#!/usr/bin/env python3
"""TEST conditions reelles de la correction de signe du cap : re-rendre le cloud GT-free
(150959) avec le cap COMPAS (signe corrige) et comparer au cap SLAM actuel + au DISO 120307
(reference quai en T). Metrique = distance NN mediane (sharpness locale, robuste a la collapse).
"""
import os, numpy as np, pandas as pd
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

C = "TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959"
D = "results/run_aracati_Bruce_DISO_Sonar_2026-06-16_120307"

traj = pd.read_csv(os.path.join(C, "trajectory.csv"))
cloud = pd.read_csv(os.path.join(C, "pointcloud.csv"))
gth = pd.read_csv(os.path.join(C, "gt_heading.csv"))
dcloud = pd.read_csv(os.path.join(D, "pointcloud.csv"))

idx = {int(k): i for i, k in enumerate(traj.keyframe_id)}
px, py, pth = traj.x.to_numpy(), traj.y.to_numpy(), traj.theta.to_numpy()
gt_comp = gth.gt_compass.to_numpy()
cid = cloud.keyframe_id.to_numpy().astype(int)
wx, wy = cloud.x.to_numpy(), cloud.y.to_numpy()

def render(cap_by_kf):
    nx, ny = np.empty(len(wx)), np.empty(len(wx))
    for k in np.unique(cid):
        if k not in idx: continue
        i = idx[k]; m = cid == k; c, s = np.cos(pth[i]), np.sin(pth[i])
        bx = c*(wx[m]-px[i])+s*(wy[m]-py[i]); by = -s*(wx[m]-px[i])+c*(wy[m]-py[i])
        nth = cap_by_kf[i] if not np.isnan(cap_by_kf[i]) else pth[i]
        c2, s2 = np.cos(nth), np.sin(nth)
        nx[m] = c2*bx - s2*by + px[i]; ny[m] = s2*bx + c2*by + py[i]
    return nx, ny

def sharpness(x, y, n=8000):
    """NN median sur un echantillon : sharp (lignes) -> petit ; swirl -> grand."""
    rng = np.random.default_rng(0)
    s = rng.choice(len(x), min(n, len(x)), replace=False)
    P = np.c_[x[s], y[s]]
    d, _ = cKDTree(P).query(P, k=2)
    return np.median(d[:, 1])

slam_x, slam_y = render(pth)
comp_x, comp_y = render(gt_comp)

print(f"{'carte':>22} | {'NN median (m)':>13} | {'pts':>8}")
print("-"*50)
print(f"{'SLAM actuel (swirl)':>22} | {sharpness(slam_x,slam_y):>12.3f} | {len(slam_x):>8}")
print(f"{'compas (signe corrige)':>22} | {sharpness(comp_x,comp_y):>12.3f} | {len(comp_x):>8}")
print(f"{'DISO 120307 (ref)':>22} | {sharpness(dcloud.x.to_numpy(),dcloud.y.to_numpy()):>12.3f} | {len(dcloud):>8}")

fig, ax = plt.subplots(1, 3, figsize=(26, 8.5))
ax[0].scatter(slam_x, slam_y, s=0.3, c="navy", alpha=0.22); ax[0].set_title("cap SLAM actuel (GT-free) — swirl")
ax[1].scatter(comp_x, comp_y, s=0.3, c="darkgreen", alpha=0.22); ax[1].set_title("cap COMPAS corrige (GT-free)")
ax[2].scatter(dcloud.x, dcloud.y, s=0.3, c="darkred", alpha=0.22); ax[2].set_title("DISO 120307 (reference quai en T)")
for a in ax: a.axis("equal"); a.grid(alpha=0.3); a.set_xlabel("x"); a.set_ylabel("y")
plt.tight_layout()
out = os.path.join(C, "verify_cap_fix.png")
plt.savefig(out, dpi=120); print(f"\nPNG : {out}")

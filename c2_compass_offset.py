#!/usr/bin/env python3
"""C — essai 2 : le recalage echoue (fond sans features). Le meilleur cap GT-free est le COMPAS.
Son repere a un offset constant vs le repere position (cmd_vel/USBL) -> chaque scan tourne de cet
offset autour de sa position -> smear. On CALIBRE l'offset (1 param GT-free) en minimisant la
sharpness NN mediane (robuste, contrairement a occupied-cells qui s'effondre)."""
import os, numpy as np, pandas as pd
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = os.environ.get("SLAM_RESULTS_DIR", "TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959")
traj = pd.read_csv(os.path.join(R, "trajectory.csv"))
cloud_path = os.path.join(R, "pointcloud.csv")
if not os.path.exists(cloud_path):
    raise SystemExit(f"pointcloud.csv introuvable dans {R}")
cloud = pd.read_csv(cloud_path)
idx = {int(k): i for i, k in enumerate(traj.keyframe_id)}
px, py, pth = traj.x.to_numpy(), traj.y.to_numpy(), traj.theta.to_numpy()
# 100% GT-FREE : -dr_theta = compas (identite wz=-d(compas)/dt), depuis cmd_vel, PAS /pose_gt.
compass = -traj.dr_theta.to_numpy()
cid = cloud.keyframe_id.to_numpy().astype(int)
wx, wy = cloud.x.to_numpy(), cloud.y.to_numpy()

# body + offsets pre-calc
bx = np.empty(len(wx)); by = np.empty(len(wx)); cap0 = np.empty(len(wx)); ox = np.empty(len(wx)); oy = np.empty(len(wx))
for k in np.unique(cid):
    if k not in idx: continue
    i = idx[k]; m = cid == k; c, s = np.cos(pth[i]), np.sin(pth[i])
    bx[m] = c*(wx[m]-px[i])+s*(wy[m]-py[i]); by[m] = -s*(wx[m]-px[i])+c*(wy[m]-py[i])
    cap0[m] = compass[i]; ox[m] = px[i]; oy[m] = py[i]

def render(off):
    c, s = np.cos(cap0+off), np.sin(cap0+off)
    return c*bx - s*by + ox, s*bx + c*by + oy

def nn_med(x, y, n=8000):
    rng = np.random.default_rng(0); sm = rng.choice(len(x), min(n,len(x)), replace=False)
    P = np.c_[x[sm], y[sm]]; d, _ = cKDTree(P).query(P, k=2); return np.median(d[:,1])

res = sorted(((round(np.degrees(o)), nn_med(*render(o))) for o in np.radians(np.arange(-180,180,3))),
             key=lambda r: r[1])
print("meilleurs offsets compas (deg -> NN median m) :")
for d, v in res[:6]: print(f"  {d:+4d}° -> {v:.3f}")
obest = np.radians(res[0][0]); xb, yb = render(obest)
print(f"\ncompas brut (offset 0) : NN = {nn_med(*render(0.0)):.3f}")
print(f"compas calibre {res[0][0]:+d}°  : NN = {res[0][1]:.3f}   (cible DISO+GT 0.242 ; SLAM 0.469)")

plt.figure(figsize=(10, 9))
plt.scatter(xb, yb, s=0.35, c="darkgreen", alpha=0.25); plt.plot(px, py, "k-", lw=0.5, alpha=0.4)
plt.title(f"C2 — cap cmd_vel calibre {res[0][0]:+d}° (NN {res[0][1]:.3f}) — 100% GT-free")
plt.axis("equal"); plt.grid(alpha=0.3); plt.tight_layout()
plt.savefig(os.path.join(R, "cloudmap_c2_gtfree.png"), dpi=140)
pd.DataFrame({"keyframe_id": cid, "x": xb, "y": yb}).to_csv(os.path.join(R, "cloudmap_c2_gtfree.csv"), index=False)
print(f"LIVRABLE : {os.path.join(R,'cloudmap_c2_gtfree.png')} + .csv")

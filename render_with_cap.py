#!/usr/bin/env python3
"""Test direct de la 'triche' cap : re-rendre le cloud en gardant les POSITIONS SLAM mais
en remplacant le CAP de chaque keyframe par un cap GT (course ou compas). Si un cap GT
nettoie la carte, le probleme est le cap ; sinon, le cap n'est pas en cause."""
import os, sys, bisect
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = sys.argv[1]
traj = pd.read_csv(os.path.join(R, "trajectory.csv"))
cloud = pd.read_csv(os.path.join(R, "pointcloud.csv"))
gth = pd.read_csv(os.path.join(R, "gt_heading.csv"))   # produit par reverse_cap.py

idx = {int(k): i for i, k in enumerate(traj.keyframe_id)}
px = traj.x.to_numpy(); py = traj.y.to_numpy(); pth = traj.theta.to_numpy()
dr = traj.dr_theta.to_numpy()                          # cap odometrie PUR (GT-free)
gt_course = gth.gt_course.to_numpy(); gt_comp = gth.gt_compass.to_numpy()

cid = cloud.keyframe_id.to_numpy().astype(int)
wx, wy = cloud.x.to_numpy(), cloud.y.to_numpy()

def render(cap_arr, name):
    """deproj body via theta SLAM, reproj avec cap_arr (position SLAM inchangee)."""
    nx, ny = np.empty(len(wx)), np.empty(len(wx))
    for k in np.unique(cid):
        if k not in idx: continue
        i = idx[k]; m = cid == k
        ox, oy, oth = px[i], py[i], pth[i]
        c, s = np.cos(oth), np.sin(oth)
        bx = c*(wx[m]-ox)+s*(wy[m]-oy); by = -s*(wx[m]-ox)+c*(wy[m]-oy)
        nth = cap_arr[i]
        if np.isnan(nth): nth = oth
        c2, s2 = np.cos(nth), np.sin(nth)
        nx[m] = c2*bx - s2*by + ox; ny[m] = s2*bx + c2*by + oy
    occ = len(set(zip(np.floor(nx/0.5).astype(int), np.floor(ny/0.5).astype(int))))
    print(f"  {name:>16} : cellules occupees = {occ}")
    return nx, ny

print("re-rendu cloud avec differents caps (positions SLAM figees) :")
base_x, base_y = render(pth, "SLAM actuel")
ndr_x, ndr_y = render(-dr, "-dr_theta GT-FREE")
cmp_x, cmp_y = render(gt_comp, "GT compas")

fig, ax = plt.subplots(1, 3, figsize=(24, 8))
for a, (X, Y, t) in zip(ax, [(base_x, base_y, "cap SLAM ACTUEL (swirl)"),
                              (ndr_x, ndr_y, "cap = -dr_theta = compas GT-FREE"),
                              (cmp_x, cmp_y, "cap GT COMPAS (reference)")]):
    a.scatter(X, Y, s=0.3, c="navy", alpha=0.25); a.set_title(t)
    a.axis("equal"); a.grid(alpha=0.3)
plt.tight_layout()
out = os.path.join(R, "render_with_cap.png")
plt.savefig(out, dpi=120); print(f"PNG : {out}")

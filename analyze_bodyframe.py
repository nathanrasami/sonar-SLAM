#!/usr/bin/env python3
"""Diagnostic carte "en arcs" : reconstruit les points de chaque keyframe dans
son repère BODY (en inversant la pose SLAM) pour discriminer l'origine du bazar :

- BODY = fan 130° propre avec murs nets, mais carte monde en arcs
    → poses/cap incohérents (timing ou miroir) — le problème est côté poses.
- BODY montre un arc dense à rayon ~constant proche de la portée max
    → frontière du fan qui survit au masque (échelle/marges du PNG fausses).
- BODY = speckle uniforme partout dans le fan
    → bruit CFAR (Pfa/threshold trop laxistes) — le problème est côté détection.

Usage : python3 analyze_bodyframe.py results/run_xxx
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

d = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("SLAM_RESULTS_DIR", "results")
traj = pd.read_csv(os.path.join(d, "trajectory.csv")).set_index("keyframe_id")
cloud = pd.read_csv(os.path.join(d, "pointcloud.csv"))

bx, by = [], []
for kid, g in cloud.groupby("keyframe_id"):
    if kid not in traj.index:
        continue
    x0, y0, th = traj.loc[kid, ["x", "y", "theta"]]
    dx = g["x"].to_numpy() - x0
    dy = g["y"].to_numpy() - y0
    c, s = np.cos(-th), np.sin(-th)
    bx.append(c * dx - s * dy)
    by.append(s * dx + c * dy)
bx = np.concatenate(bx)
by = np.concatenate(by)
r = np.hypot(bx, by)
bearing = np.degrees(np.arctan2(by, bx))

fig, axes = plt.subplots(1, 3, figsize=(20, 7))

# fan attendu : ouvert vers +x (haut), ±62° max, r < portée max
axes[0].scatter(by, bx, s=0.3, alpha=0.15, c="navy")
axes[0].set_title("Points en repère BODY (tous keyframes superposés)\n"
                  "attendu : fan 130° vers le haut, murs nets")
axes[0].set_xlabel("y body (m)")
axes[0].set_ylabel("x body (m)")
axes[0].axis("equal")
axes[0].grid(True)

axes[1].hist(r, bins=200, color="navy")
axes[1].set_title("Rayon body — pic isolé près de la portée max\n= frontière du fan mal masquée")
axes[1].set_xlabel("r (m)")
axes[1].grid(True)

axes[2].hist(bearing, bins=180, color="navy")
axes[2].set_title("Bearing body — attendu borné à ±62°\nau-delà = incohérence de repère")
axes[2].set_xlabel("bearing (°)")
axes[2].grid(True)

print(f"points : {len(r)}")
print(f"r      : p50={np.percentile(r,50):.1f}  p99={np.percentile(r,99):.1f}  max={r.max():.1f} m")
print(f"bearing: min={bearing.min():.1f}°  max={bearing.max():.1f}°  (attendu ±62°)")
frac_edge = (r > 0.92 * r.max()).mean()
print(f"fraction des points à r>92% du max : {frac_edge:.1%} (gros % = frontière du fan)")

plt.tight_layout()
out = os.path.join(d, "bodyframe_diag.png")
plt.savefig(out, dpi=130)
print(f"Saved to {out}")
plt.show()

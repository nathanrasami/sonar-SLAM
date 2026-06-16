#!/usr/bin/env python3
"""analyse_centre.py — analyse SANS Umeyama : alignement par CENTRAGE seul.

Contrairement à analyze_drift.py (qui utilise Umeyama = rotation + translation
+ réflexion optimales), ici on aligne UNIQUEMENT par translation de centroïdes :

    est_aligné = est - centroïde(est) + centroïde(GT)

C'est la meilleure translation possible (elle minimise le RMSE à rotation fixe),
mais AUCUNE rotation ni réflexion n'est appliquée. Donc :
  - si une trajectoire est tournée ou MIROIR par rapport à la GT (cas de DISO,
    qui sort en Y inversé), elle NE se superposera PAS → l'ATE sera gonflé.
  - c'est justement le but : voir l'accord géométrique BRUT, et mesurer ce que
    la rotation d'Umeyama corrige (comparer l'ATE d'ici à celui d'analyze_drift).

Le plot recentre tout par le MÊME point (centroïde GT) → positions relatives
préservées : si un début est décalé, ça se voit (pas de forçage trompeur à 0,0).

Usage :
    SLAM_RESULTS_DIR=results/run_aracati_... python3 analyse_centre.py
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from traj_eval import associer_par_temps, calculer_ate

results_dir = os.environ.get("SLAM_RESULTS_DIR", "results")


def charger(nom, cols):
    p = os.path.join(results_dir, nom)
    if not os.path.isfile(p):
        return None
    return pd.read_csv(p)


def centrer_sur(xy, centroide_cible):
    """Translation pure : amène le centroïde de xy sur centroide_cible. Pas de rotation."""
    return xy - xy.mean(axis=0) + centroide_cible


# ── Chargement ────────────────────────────────────────────────────────────────
gt = charger("groundtruth.csv", None)
if gt is None:
    raise SystemExit(f"groundtruth.csv introuvable dans {results_dir} — besoin de la GT.")
gt_t = gt["time"].values
gt_xy_full = gt[["x", "y"]].to_numpy()
c_gt = gt_xy_full.mean(axis=0)  # centroïde GT = référence commune

traj = charger("trajectory.csv", None)   # Bruce-SLAM : keyframe_id,time,x,y,...
odom = charger("odometry.csv", None)      # odométrie (DISO ou GT) : time,x,y


def ate_centre(d, label):
    """Associe au temps, aligne par centrage seul, retourne (xy_aligné, ATE)."""
    if d is None:
        return None, None
    t = d["time"].values
    xy = d[["x", "y"]].to_numpy()
    gt_assoc = associer_par_temps(t, gt_t, gt_xy_full[:, 0], gt_xy_full[:, 1])
    # centrage : même décalage relatif (centroïde de l'estimé → centroïde du GT associé)
    xy_al = xy - xy.mean(axis=0) + gt_assoc.mean(axis=0)
    ate = calculer_ate(xy_al, gt_assoc)
    print(f"{label:28s} ATE (centrage seul) = {ate:.2f} m   ({len(d)} points)")
    return xy_al, ate


print(f"=== analyse SANS Umeyama (centrage seul) — {results_dir} ===")
b_xy, b_ate = ate_centre(traj, "Bruce-SLAM")
o_xy, o_ate = ate_centre(odom, "Odométrie")

# ── Plot : tout recentré par le MÊME point (centroïde GT) ─────────────────────
fig, ax = plt.subplots(figsize=(10, 8))
g = centrer_sur(gt_xy_full, c_gt) - c_gt  # GT centrée sur (0,0) pour lisibilité
ax.plot(g[:, 0], g[:, 1], "r--", label="Ground truth")
ax.plot(g[-1, 0], g[-1, 1], "rX", ms=12)

if o_xy is not None:
    op = o_xy - c_gt
    lbl = f"Odométrie (ATE={o_ate:.1f} m)"
    ax.plot(op[:, 0], op[:, 1], color="orange", ls="-.", lw=1.5, label=lbl)
    ax.plot(op[-1, 0], op[-1, 1], marker="X", color="orange", ms=12)

if b_xy is not None:
    bp = b_xy - c_gt
    lbl = f"Bruce-SLAM (ATE={b_ate:.1f} m)"
    ax.plot(bp[:, 0], bp[:, 1], "k-", lw=1.5, label=lbl)
    ax.plot(bp[-1, 0], bp[-1, 1], "kX", ms=12)

ax.plot(0, 0, marker="*", color="green", ms=14, label="Centroïde GT")
ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Alignement par CENTRAGE seul (sans Umeyama)\n"
             "⚠ pas de rotation/réflexion → un Y-flip DISO restera visible")
ax.legend()
out = os.path.join(results_dir, "trajectory_plot_centre.png")
fig.tight_layout()
fig.savefig(out, dpi=150)
print(f"\nPlot : {out}")
print("NB : compare cet ATE à celui d'analyze_drift.py (Umeyama). L'écart = ce que")
print("     la rotation/réflexion d'Umeyama corrige (repère DISO Y-inversé, etc.).")

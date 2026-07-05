#!/usr/bin/env python3
"""Carte 3D INTERACTIVE (rotation à la souris, zoom molette — façon rotate3d MATLAB).

Ouvre une fenêtre matplotlib 3D : nuage coloré par z + trajectoire SLAM (z du CSV si
présent, sinon z=0) + départ/arrivée. Sauve aussi carte_3d.png (vue par défaut) pour
le dossier du run. Sans serveur d'affichage (ssh, batch), retombe automatiquement sur
la sauvegarde seule.

Usage : python3 analysis/view3d.py <run_dir> [--save-only]
        ./analyse.sh 3D <run>   (lancé en dernier, fenêtre bloquante)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_on_cloud import _load_cloud


def main(run_dir, save_only=False):
    interactive = not save_only and os.environ.get("DISPLAY", "") != ""
    import matplotlib
    if not interactive:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (active la projection 3d)

    P, z, src = _load_cloud(run_dir)
    if z is None:
        z = np.zeros(len(P))
    traj = np.genfromtxt(os.path.join(run_dir, "trajectory.csv"),
                         delimiter=",", names=True)
    tz = traj["z"] if "z" in traj.dtype.names else np.zeros(len(traj))

    # sous-échantillonnage pour garder la rotation fluide (~60k pts max)
    if len(P) > 60000:
        idx = np.random.default_rng(0).choice(len(P), 60000, replace=False)
        P, z = P[idx], z[idx]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(P[:, 0], P[:, 1], z, s=1.2, c=z, cmap="viridis",
                    alpha=0.55, linewidths=0)
    fig.colorbar(sc, ax=ax, label="z (m)", shrink=0.6)
    ax.plot(traj["x"], traj["y"], tz, color="red", lw=1.8, label="trajectoire SLAM")
    ax.scatter([traj["x"][0]], [traj["y"][0]], [tz[0]], color="red", marker="^",
               s=80, label="départ")
    ax.scatter([traj["x"][-1]], [traj["y"][-1]], [tz[-1]], color="red", marker="s", s=60)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title(f"{os.path.basename(run_dir.rstrip('/'))} — carte 3D ({src})")
    ax.legend(loc="upper left")
    # échelle ~isotrope (matplotlib 3D ne fait pas d'aspect égal tout seul)
    try:
        ax.set_box_aspect((np.ptp(P[:, 0]), np.ptp(P[:, 1]), max(np.ptp(z), 1.0)))
    except Exception:
        pass

    out = os.path.join(run_dir, "carte_3d.png")
    fig.savefig(out, dpi=140)
    print("->", out)
    if interactive:
        print("[view3d] fenêtre interactive — tourner à la souris, fermer pour quitter")
        plt.show()
    else:
        print("[view3d] pas d'affichage (DISPLAY vide ou --save-only) — PNG seulement")


if __name__ == "__main__":
    main(sys.argv[1], save_only="--save-only" in sys.argv[2:])

#!/usr/bin/env python3
"""Compare l'ATE de DISO entre deux runs (ex. VM vs Docker), même méthode.

Usage :
    python3 compare_diso_env.py <run_dir_1> <run_dir_2> [--gt <groundtruth.csv>]

Chaque run_dir doit contenir diso_trajectory.csv. La vérité terrain (GT) :
  - --gt si fourni,
  - sinon groundtruth.csv trouvé dans l'un des deux runs,
  - sinon le GT de référence du même bag (run_diso_2026-06-10_163336).

Alignement Umeyama avec réflexion (DISO sort en Y-flip) puis RMSE = ATE.
But : si les deux ATE diffèrent nettement, c'est l'ENVIRONNEMENT (pas DISO).
"""
import os
import sys
import argparse
import numpy as np
import pandas as pd
from traj_eval import associer_par_temps, umeyama, appliquer, calculer_ate

FALLBACK_GT = "results/run_diso_2026-06-10_163336/groundtruth.csv"


def find_gt(run_dirs, explicit):
    if explicit:
        return explicit
    for d in run_dirs:
        cand = os.path.join(d, "groundtruth.csv")
        if os.path.isfile(cand):
            return cand
    return FALLBACK_GT


def ate_of(run_dir, gt):
    traj = os.path.join(run_dir, "diso_trajectory.csv")
    if not os.path.isfile(traj):
        sys.exit(f"ERREUR: {traj} introuvable")
    d = pd.read_csv(traj)               # time,x,y,z,qx,qy,qz,qw
    t, xy = d["time"].values, d[["x", "y"]].to_numpy()
    gxy = associer_par_temps(t, gt["time"].values, gt["x"].values, gt["y"].values)
    s, R, tt = umeyama(xy, gxy)
    return calculer_ate(appliquer(s, R, tt, xy), gxy), len(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir_1")
    ap.add_argument("run_dir_2")
    ap.add_argument("--gt", default=None, help="chemin d'un groundtruth.csv (time,x,y)")
    args = ap.parse_args()

    runs = [args.run_dir_1.rstrip("/"), args.run_dir_2.rstrip("/")]
    gt_path = find_gt(runs, args.gt)
    if not os.path.isfile(gt_path):
        sys.exit(f"ERREUR: GT introuvable: {gt_path}")
    gt = pd.read_csv(gt_path)
    print(f"GT de référence : {gt_path}  ({len(gt)} points)\n")

    print(f"{'Run':45s} {'frames':>8s} {'ATE (m)':>9s}")
    print("-" * 64)
    ates = []
    for d in runs:
        a, n = ate_of(d, gt)
        ates.append(a)
        print(f"{os.path.basename(d):45s} {n:8d} {a:9.2f}")

    diff = abs(ates[0] - ates[1])
    print("-" * 64)
    print(f"Écart d'ATE : {diff:.2f} m")
    if diff > 1.0:
        print("=> Écart significatif : même DISO, même bag → c'est l'ENVIRONNEMENT")
        print("   (synchro ApproximateTime sonar/GT), pas l'algorithme.")
    else:
        print("=> ATE comparables → DISO se comporte pareil dans les deux environnements.")


if __name__ == "__main__":
    main()

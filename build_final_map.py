#!/usr/bin/env python3
"""TEMPS 2 — construction OFFLINE de la cloudmap propre à partir d'un run.

Découplage trajectoire/carte : le run (Temps 1) fige la trajectoire (cmd_vel + USBL +
loops). Ici on relit ses CSV et on filtre le nuage SANS toucher la trajectoire, autant
de fois qu'on veut, en quelques secondes (pas de re-run).

FILTRE = discriminant PHYSIQUE, 100% GT-free :
  pour chaque voxel monde, ECART-TYPE du range capteur des observations.
  - Structure réelle (réflecteur physique) : vue à des distances variées quand le ROV
    bouge → grande variance de range  → GARDÉE.
  - Fantôme de fond (retour au range de rasance ~constant, pas de réflecteur) :
    n'apparaît qu'à ce range → faible variance → RETIRÉE.
  + nb de keyframes distincts par voxel (un vrai objet est revu de plusieurs poses).

Validé sur run DISO 011733 (rstd≥9 & nkf≥5 → quai+appontements nets, swirl retiré).
ATTENTION : ne révèle des structures QUE si l'entrée en a (cohérence locale de pose
suffisante). Un nuage cmd_vel pur (quai étalé par le bruit USBL) ne donne qu'un blob.

Usage :
  SLAM_RESULTS_DIR=results/run_aracati_... python3 build_final_map.py
  python3 build_final_map.py results/run_aracati_...  [--rstd 9] [--nkf 5] [--res 1.5]
"""
import os, sys, argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def deproject_range(traj: pd.DataFrame, cloud: pd.DataFrame):
    """Range capteur (body) de chaque point, via la pose de son keyframe d'origine.
    world = pose_xy + R(theta)·body  ⇒  body = R(theta)^T·(world − pose_xy)."""
    pose = {int(k): (x, y, th)
            for k, x, y, th in zip(traj.keyframe_id, traj.x, traj.y, traj.theta)}
    cid = cloud.keyframe_id.to_numpy().astype(int)
    wx, wy = cloud.x.to_numpy(), cloud.y.to_numpy()
    miss = [c for c in np.unique(cid) if c not in pose]
    if miss:
        raise ValueError(f"{len(miss)} keyframe_id du cloud absents de trajectory.csv "
                         f"(ex: {miss[:5]}) — CSV de runs différents ?")
    ox = np.array([pose[c][0] for c in cid])
    oy = np.array([pose[c][1] for c in cid])
    oth = np.array([pose[c][2] for c in cid])
    ct, st = np.cos(oth), np.sin(oth)
    dx, dy = wx - ox, wy - oy
    bx = ct * dx + st * dy
    by = -st * dx + ct * dy
    return wx, wy, cid, np.hypot(bx, by)


def variance_filter(wx, wy, cid, rng, res, rstd_min, nkf_min):
    """Masque des points dont le voxel monde a rstd≥rstd_min ET nkf≥nkf_min."""
    cx = np.floor(wx / res).astype(np.int64)
    cy = np.floor(wy / res).astype(np.int64)
    df = pd.DataFrame({"cx": cx, "cy": cy, "kf": cid, "rng": rng,
                       "i": np.arange(len(wx))})
    stat = (df.groupby(["cx", "cy"])
              .agg(nkf=("kf", "nunique"), rstd=("rng", "std"))
              .reset_index())
    stat["rstd"] = stat["rstd"].fillna(0.0)
    m = df.merge(stat, on=["cx", "cy"]).sort_values("i")
    keep = (m["rstd"].to_numpy() >= rstd_min) & (m["nkf"].to_numpy() >= nkf_min)
    return keep, len(stat)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", nargs="?",
                    default=os.environ.get("SLAM_RESULTS_DIR"),
                    help="dossier du run (sinon $SLAM_RESULTS_DIR)")
    ap.add_argument("--rstd", type=float, default=9.0,
                    help="écart-type de range min par voxel (m). Défaut 9 (validé 011733)")
    ap.add_argument("--nkf", type=int, default=5,
                    help="nb min de keyframes distincts par voxel. Défaut 5")
    ap.add_argument("--res", type=float, default=1.5, help="taille voxel (m). Défaut 1.5")
    ap.add_argument("--imin", type=float, default=140.0,
                    help="gate d'INTENSITÉ par point (>= imin = retour structurel). "
                         "140 = quai/murs (cf. histogramme). 0 = off. Ignoré si le "
                         "CSV n'a pas de colonne 'intensity' (anciens runs).")
    args = ap.parse_args()
    if not args.run_dir:
        sys.exit("Donne un dossier de run (arg ou SLAM_RESULTS_DIR).")

    traj = pd.read_csv(os.path.join(args.run_dir, "trajectory.csv"))
    cloud = pd.read_csv(os.path.join(args.run_dir, "pointcloud.csv"))
    wx, wy, cid, rng = deproject_range(traj, cloud)

    # gate d'INTENSITÉ (discriminant structure le plus direct : le quai sature en
    # intensité, le fond non). Disponible seulement si le run a exporté la colonne.
    has_int = "intensity" in cloud.columns
    if has_int and args.imin > 0:
        igate = cloud["intensity"].to_numpy() >= args.imin
    else:
        igate = np.ones(len(wx), bool)
        if not has_int:
            print("  (pas de colonne 'intensity' → gate intensité ignoré ; re-run requis)")

    keep_var, nvox = variance_filter(wx, wy, cid, rng, args.res, args.rstd, args.nkf)
    keep = keep_var & igate

    print(f"run        : {args.run_dir}")
    print(f"filtre     : res={args.res}m  rstd>={args.rstd}m  nkf>={args.nkf}  imin={args.imin if has_int else 'n/a'}")
    print(f"voxels     : {nvox}")
    print(f"points     : {len(wx)} → intensité {int(igate.sum())} → +variance {int(keep.sum())} gardés ({100*keep.mean():.0f}%)")

    # CSV propre
    out_csv = os.path.join(args.run_dir, "cloudmap_clean.csv")
    pd.DataFrame({"keyframe_id": cid[keep], "x": wx[keep], "y": wy[keep]}).to_csv(
        out_csv, index=False)
    print(f"CSV        : {out_csv}")

    # PNG : trajectoire + cloud filtré
    fig, ax = plt.subplots(1, 2, figsize=(16, 7))
    ax[0].scatter(wx, wy, s=0.3, c="navy", alpha=0.2)
    ax[0].set_title(f"Cloud brut ({len(wx)} pts)")
    ax[1].scatter(wx[keep], wy[keep], s=0.5, c="darkred", alpha=0.4)
    ax[1].plot(traj.x, traj.y, "-", color="black", lw=0.8, alpha=0.6, label="trajectoire")
    ax[1].set_title(f"Cloudmap propre — rstd≥{args.rstd} & nkf≥{args.nkf} "
                    f"({100*keep.mean():.0f}%)")
    ax[1].legend(fontsize=8)
    for a in ax:
        a.set_xlabel("x (m)"); a.set_ylabel("y (m)"); a.axis("equal"); a.grid(True, alpha=0.3)
    plt.tight_layout()
    out_png = os.path.join(args.run_dir, "cloudmap_clean.png")
    plt.savefig(out_png, dpi=130)
    print(f"PNG        : {out_png}")


if __name__ == "__main__":
    main()

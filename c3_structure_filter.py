#!/usr/bin/env python3
"""C — essai 3 : cap C2 GT-free + FILTRE DE STRUCTURE (variance-range) pour faire ressortir
le quai en T complet (pas seulement la bande). Le filtre garde les voxels monde vus a des
distances capteur VARIEES (vraie structure = parallaxe quand le ROV bouge) et retire le fond
(retour a range ~constant de rasance). 100% GT-free (cap = cmd_vel calibre, range = geometrie)."""
import os, argparse
import numpy as np, pandas as pd
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = "TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rstd", type=float, default=6.0, help="ecart-type range min par voxel (m)")
    ap.add_argument("--nkf", type=int, default=4, help="nb keyframes distincts min par voxel")
    ap.add_argument("--res", type=float, default=1.5, help="taille voxel (m)")
    args = ap.parse_args()

    traj = pd.read_csv(os.path.join(R, "trajectory.csv"))
    c2 = pd.read_csv(os.path.join(R, "cloudmap_c2_gtfree.csv"))   # cloud rendu avec cap C2 GT-free
    pos = {int(k): (x, y) for k, x, y in zip(traj.keyframe_id, traj.x, traj.y)}

    cid = c2.keyframe_id.to_numpy().astype(int)
    wx, wy = c2.x.to_numpy(), c2.y.to_numpy()
    kx = np.array([pos[c][0] for c in cid]); ky = np.array([pos[c][1] for c in cid])
    rng = np.hypot(wx - kx, wy - ky)            # range capteur (cap-independant)

    # voxelisation + stats par voxel
    cxg = np.floor(wx/args.res).astype(np.int64); cyg = np.floor(wy/args.res).astype(np.int64)
    df = pd.DataFrame({"cx": cxg, "cy": cyg, "kf": cid, "rng": rng, "i": np.arange(len(wx))})
    stat = df.groupby(["cx", "cy"]).agg(nkf=("kf", "nunique"), rstd=("rng", "std")).reset_index()
    stat["rstd"] = stat["rstd"].fillna(0.0)
    m = df.merge(stat, on=["cx", "cy"]).sort_values("i")
    keep = (m["rstd"].to_numpy() >= args.rstd) & (m["nkf"].to_numpy() >= args.nkf)

    def nn_med(x, y, n=8000):
        if len(x) < 50: return float("nan")
        rngn = np.random.default_rng(0); s = rngn.choice(len(x), min(n, len(x)), replace=False)
        P = np.c_[x[s], y[s]]; d, _ = cKDTree(P).query(P, k=2); return np.median(d[:, 1])

    fx, fy = wx[keep], wy[keep]
    print(f"filtre : res={args.res} rstd>={args.rstd} nkf>={args.nkf}")
    print(f"points : {len(wx)} -> {keep.sum()} gardes ({100*keep.mean():.0f}%)")
    print(f"NN median : C2 complet {nn_med(wx,wy):.3f} -> C3 filtre {nn_med(fx,fy):.3f}  (DISO 0.242)")

    out_csv = os.path.join(R, "cloudmap_c3_gtfree.csv")
    pd.DataFrame({"keyframe_id": cid[keep], "x": fx, "y": fy}).to_csv(out_csv, index=False)

    fig, ax = plt.subplots(1, 2, figsize=(18, 9))
    ax[0].scatter(wx, wy, s=0.3, c="darkgreen", alpha=0.18); ax[0].set_title("C2 complet (cap GT-free, swirl retire)")
    ax[1].scatter(fx, fy, s=0.5, c="darkred", alpha=0.35)
    ax[1].plot(traj.x, traj.y, "k-", lw=0.5, alpha=0.4)
    ax[1].set_title(f"C3 = C2 + filtre structure ({keep.sum()} pts) — 100% GT-free")
    for a in ax: a.axis("equal"); a.grid(alpha=0.3)
    plt.tight_layout(); out = os.path.join(R, "cloudmap_c3_gtfree.png")
    plt.savefig(out, dpi=140); print(f"PNG : {out}\nCSV : {out_csv}")


if __name__ == "__main__":
    main()

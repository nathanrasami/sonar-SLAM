#!/usr/bin/env python3
"""TEMPS 2 — RECALAGE des blocs de structure pour reassembler le quai.

Constat (validé run 150959) : le filtre variance retire le fond et garde la STRUCTURE
(quai en T), mais chaque keyframe la voit à une position légèrement décalée (erreur de cap
résiduelle ~1.9°/kf × 30 m ≈ 1 m d'étalement) → le quai apparaît en N copies décalées au lieu
d'un seul morceau. Ici on RECALE chaque keyframe pour superposer ces copies, SANS toucher la
trajectoire globale (laisse sur la déviation de pose → ATE préservé, 100% GT-free).

Méthode : recalage structure-vers-carte itératif (scan-to-map ICP 2D).
Pour chaque keyframe, on cherche la PETITE rotation autour de SA position (+ micro-translation)
qui superpose ses points de structure sur ceux des AUTRES keyframes voisins. Rotation autour de
la position du keyframe = corrige le CAP (la vue pivote, les blocs lointains se rejoignent) sans
déplacer le keyframe → trajectoire et ATE inchangés.

Entrée  : <run>/trajectory.csv  +  <run>/cloudmap_clean.csv (structure filtrée, fond déjà retiré).
          → lancer build_final_map.py AVANT (produit cloudmap_clean.csv).
Sortie  : <run>/cloudmap_refined.csv  +  <run>/cloudmap_refined.png (avant/après).

Usage : python3 refine_cloud.py <run_dir> [--iters 6] [--leash_t 1.5] [--leash_th 10]
"""
import os, sys, argparse
import numpy as np, pandas as pd
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rigid_2d(src, dst):
    """Umeyama 2D sans échelle : R,t tels que R·src+t ≈ dst. src,dst : (N,2)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = (dst - mu_d).T @ (src - mu_s)
    U, _, Vt = np.linalg.svd(S)
    D = np.diag([1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    t = mu_d - R @ mu_s
    return R, t


def cross_kf_nn_dist(wx, wy, cid, sample=8000):
    """Netteté = distance médiane au plus proche voisin d'un AUTRE keyframe.
    Blocs superposés → petite distance. Mesure directe de 'les copies se rejoignent'."""
    n = len(wx)
    idx = np.random.RandomState(0).choice(n, min(sample, n), replace=False)
    tree = cKDTree(np.c_[wx, wy])
    d = []
    for i in idx:
        dd, jj = tree.query([wx[i], wy[i]], k=8)
        for dist, j in zip(dd[1:], jj[1:]):
            if cid[j] != cid[i]:
                d.append(dist); break
    return np.median(d), np.mean(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--iters", type=int, default=6, help="itérations externes")
    ap.add_argument("--neigh_r", type=float, default=12.0, help="rayon voisinage keyframe (m)")
    ap.add_argument("--max_corr", type=float, default=3.0, help="gate correspondance ICP (m)")
    ap.add_argument("--leash_t", type=float, default=1.5, help="déviation position max vs pose globale (m)")
    ap.add_argument("--leash_th", type=float, default=10.0, help="déviation cap max vs pose globale (deg)")
    ap.add_argument("--alpha", type=float, default=0.6, help="amortissement de la correction")
    args = ap.parse_args()

    traj = pd.read_csv(os.path.join(args.run_dir, "trajectory.csv"))
    clean = pd.read_csv(os.path.join(args.run_dir, "cloudmap_clean.csv"))

    # poses originales (globales, à préserver)
    kf_ids = traj.keyframe_id.to_numpy().astype(int)
    pose0 = {int(k): np.array([x, y, th]) for k, x, y, th in
             zip(traj.keyframe_id, traj.x, traj.y, traj.theta)}
    pose = {k: v.copy() for k, v in pose0.items()}

    # points de structure en BODY frame (pour re-projeter avec la pose corrigée)
    cid = clean.keyframe_id.to_numpy().astype(int)
    wx0, wy0 = clean.x.to_numpy(), clean.y.to_numpy()
    body = {}
    for k in np.unique(cid):
        if k not in pose0:
            continue
        m = cid == k
        ox, oy, oth = pose0[k]
        dx, dy = wx0[m] - ox, wy0[m] - oy
        c, s = np.cos(oth), np.sin(oth)
        bx = c * dx + s * dy
        by = -s * dx + c * dy
        body[k] = np.c_[bx, by]
    valid = [k for k in np.unique(cid) if k in body]

    def project_all():
        xs, ys, cs = [], [], []
        for k in valid:
            x, y, th = pose[k]
            c, s = np.cos(th), np.sin(th)
            P = body[k]
            xs.append(c * P[:, 0] - s * P[:, 1] + x)
            ys.append(s * P[:, 0] + c * P[:, 1] + y)
            cs.append(np.full(len(P), k))
        return np.concatenate(xs), np.concatenate(ys), np.concatenate(cs)

    def occupied(wx, wy, res=0.5):
        """Cellules occupées : blocs superposés → MOINS de cellules (= plus concentré)."""
        return len(set(zip(np.floor(wx/res).astype(int), np.floor(wy/res).astype(int))))

    wx, wy, cc = project_all()
    shp0 = cross_kf_nn_dist(wx, wy, cc)
    occ0 = occupied(wx, wy)
    print(f"netteté initiale  : NN cross-kf médian={shp0[0]:.3f} m | cellules occupées={occ0}")

    kf_xy = np.array([pose0[k][:2] for k in valid])
    kf_tree = cKDTree(kf_xy)
    leash_th = np.radians(args.leash_th)

    for it in range(args.iters):
        wx, wy, cc = project_all()
        tree = cKDTree(np.c_[wx, wy])
        moved = 0.0
        for ki, k in enumerate(valid):
            x, y, th = pose[k]
            c, s = np.cos(th), np.sin(th)
            P = body[k]
            sx = c * P[:, 0] - s * P[:, 1] + x   # points keyframe k en monde (pose courante)
            sy = s * P[:, 0] + c * P[:, 1] + y
            # correspondances : plus proche voisin d'un AUTRE keyframe, sous max_corr
            su, sv, tu, tv = [], [], [], []
            dd, jj = tree.query(np.c_[sx, sy], k=6)
            for p in range(len(sx)):
                for dist, j in zip(dd[p, 1:], jj[p, 1:]):
                    if cc[j] != k and dist < args.max_corr:
                        su.append(sx[p] - x); sv.append(sy[p] - y)   # relatif au keyframe
                        tu.append(wx[j] - x); tv.append(wy[j] - y)
                        break
            if len(su) < 12:
                continue
            src = np.c_[su, sv]; dst = np.c_[tu, tv]
            R, t = rigid_2d(src, dst)
            dth = np.arctan2(R[1, 0], R[0, 0])
            # amortissement
            dth *= args.alpha; t = t * args.alpha
            new_th = th + dth
            new_x, new_y = x + t[0], y + t[1]
            # laisse vs pose GLOBALE (préserve ATE)
            ox, oy, oth = pose0[k]
            ddx, ddy = new_x - ox, new_y - oy
            dn = np.hypot(ddx, ddy)
            if dn > args.leash_t:
                new_x, new_y = ox + ddx / dn * args.leash_t, oy + ddy / dn * args.leash_t
            dthg = np.arctan2(np.sin(new_th - oth), np.cos(new_th - oth))
            dthg = np.clip(dthg, -leash_th, leash_th)
            new_th = oth + dthg
            moved += abs(dth)
            pose[k] = np.array([new_x, new_y, new_th])
        wx, wy, cc = project_all()
        shp = cross_kf_nn_dist(wx, wy, cc)
        print(f"  iter {it+1}/{args.iters} : NN médian={shp[0]:.3f} m  (Σ|dθ|={np.degrees(moved):.0f}°)")

    # ATE preservation : déviation des poses vs global
    dev = np.array([np.hypot(*(pose[k][:2] - pose0[k][:2])) for k in valid])
    devth = np.array([abs(np.degrees(np.arctan2(np.sin(pose[k][2]-pose0[k][2]),
                                                np.cos(pose[k][2]-pose0[k][2])))) for k in valid])
    print(f"\ndéviation pose vs global : pos médian={np.median(dev):.2f} m (max {dev.max():.2f}) | "
          f"cap médian={np.median(devth):.1f}° (max {devth.max():.1f})")
    occf = occupied(wx, wy)
    print(f"netteté finale    : NN cross-kf médian={shp[0]:.3f} m (init {shp0[0]:.3f}) | "
          f"cellules occupées={occf} (init {occ0}) → concentration {100*(1-occf/occ0):.0f}%")

    wxf, wyf, ccf = project_all()
    pd.DataFrame({"keyframe_id": ccf.astype(int), "x": wxf, "y": wyf}).to_csv(
        os.path.join(args.run_dir, "cloudmap_refined.csv"), index=False)

    fig, ax = plt.subplots(1, 2, figsize=(18, 8))
    ax[0].scatter(wx0, wy0, s=0.6, c="navy", alpha=0.3)
    ax[0].set_title(f"AVANT (structure filtrée, blocs décalés) — NN {shp0[0]:.2f} m")
    ax[1].scatter(wxf, wyf, s=0.6, c="darkred", alpha=0.3)
    ax[1].plot(traj.x, traj.y, "-", color="black", lw=0.5, alpha=0.4)
    ax[1].set_title(f"APRÈS recalage (blocs réassemblés) — NN {shp[0]:.2f} m")
    for a in ax:
        a.axis("equal"); a.grid(alpha=0.3); a.set_xlabel("x (m)"); a.set_ylabel("y (m)")
    plt.tight_layout()
    out = os.path.join(args.run_dir, "cloudmap_refined.png")
    plt.savefig(out, dpi=120)
    print(f"\nCSV : cloudmap_refined.csv\nPNG : {out}")


if __name__ == "__main__":
    main()

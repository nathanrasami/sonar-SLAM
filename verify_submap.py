#!/usr/bin/env python3
"""VERIFICATION OFFLINE de l'architecture SUBMAP DISO-local + cmd_vel-global.

Idee : DISO recale les scans densement -> structure LOCALEMENT propre (quai en T net),
mais son cap GLOBAL derive (43.8deg GT-free). cmd_vel+USBL a une bonne traj GLOBALE
(ATE 1.43m GT-free) mais un cloud en swirl (cap local incoherent). On COMBINE :
  - on decoupe la trajectoire DISO en SOUS-CARTES (fenetres de W keyframes),
  - chaque sous-carte (structure DISO propre) est REPLACEE par transfo rigide pour que
    ses keyframes coincident avec les poses cmd_vel (appariees par timestamp),
  - la transfo rigide par sous-carte absorbe la derive globale DISO sans distordre la
    structure locale.
Resultat attendu : cloud propre (quai en T) place sur la trajectoire GT-free cmd_vel.

NB : ce test utilise DISO-120307 (prior GT) comme PROXY — la qualite LOCALE d'un scan-match
est independante du prior global, donc valide pour DISO GT-free. Si l'architecture marche ici,
l'etape suivante = re-run DISO GT-free (prior cmd_vel) pour des sous-cartes 100% GT-free.

Usage : python3 verify_submap.py <diso_run> <cmdvel_run> [--win 20]
"""
import os, sys, argparse, bisect
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rigid_2d(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = (dst - mu_d).T @ (src - mu_s)
    U, _, Vt = np.linalg.svd(S)
    D = np.diag([1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    return R, mu_d - R @ mu_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("diso_run")
    ap.add_argument("cmdvel_run")
    ap.add_argument("--win", type=int, default=20, help="taille sous-carte (keyframes)")
    ap.add_argument("--min_span", type=float, default=3.0, help="etendue min des ancres pour aligner (m)")
    args = ap.parse_args()

    dtraj = pd.read_csv(os.path.join(args.diso_run, "trajectory.csv"))
    dcloud = pd.read_csv(os.path.join(args.diso_run, "pointcloud.csv"))
    ctraj = pd.read_csv(os.path.join(args.cmdvel_run, "trajectory.csv"))

    # poses DISO + appariement temporel vers poses cmd_vel
    dk = dtraj.keyframe_id.to_numpy().astype(int)
    dpose = {int(k): (x, y) for k, x, y in zip(dtraj.keyframe_id, dtraj.x, dtraj.y)}
    dtime = {int(k): t for k, t in zip(dtraj.keyframe_id, dtraj.time)}
    ctimes = ctraj.time.to_numpy()
    cxy = np.c_[ctraj.x, ctraj.y]
    order = np.argsort(ctimes); ctimes = ctimes[order]; cxy = cxy[order]

    def cmdvel_at(t):
        i = bisect.bisect_left(ctimes, t)
        i = min(max(i, 1), len(ctimes)-1)
        t0, t1 = ctimes[i-1], ctimes[i]
        a = (t-t0)/(t1-t0) if t1 > t0 else 0
        return cxy[i-1]*(1-a) + cxy[i]*a

    # points DISO en monde, groupes par keyframe
    cid = dcloud.keyframe_id.to_numpy().astype(int)
    cwx, cwy = dcloud.x.to_numpy(), dcloud.y.to_numpy()
    pts_by_kf = {}
    for k in np.unique(cid):
        m = cid == k
        pts_by_kf[k] = np.c_[cwx[m], cwy[m]]

    kf_sorted = sorted(dpose.keys())
    placed_x, placed_y = [], []
    n_sub, n_skip = 0, 0
    for i in range(0, len(kf_sorted), args.win):
        chunk = kf_sorted[i:i+args.win]
        src, dst, pts = [], [], []
        for k in chunk:
            if k not in dpose:
                continue
            src.append(dpose[k])
            dst.append(cmdvel_at(dtime[k]))
            if k in pts_by_kf:
                pts.append(pts_by_kf[k])
        src, dst = np.array(src), np.array(dst)
        if len(src) < 3 or not pts:
            n_skip += 1; continue
        # etendue des ancres (eviter d'aligner sur une ligne degeneree)
        span = np.linalg.norm(src - src.mean(0), axis=1).max()
        if span < args.min_span:
            n_skip += 1; continue
        R, t = rigid_2d(src, dst)
        P = np.vstack(pts)
        Pn = (R @ P.T).T + t
        placed_x.append(Pn[:, 0]); placed_y.append(Pn[:, 1])
        n_sub += 1
    px = np.concatenate(placed_x); py = np.concatenate(placed_y)
    print(f"sous-cartes placees : {n_sub} (ignorees {n_skip}), points : {len(px)}")

    def occupied(x, y, res=0.5):
        return len(set(zip(np.floor(x/res).astype(int), np.floor(y/res).astype(int))))
    print(f"cellules occupees (res .5m) : submap-replace={occupied(px,py)}")

    fig, ax = plt.subplots(1, 2, figsize=(18, 8))
    ax[0].scatter(dcloud.x, dcloud.y, s=0.3, c="navy", alpha=0.25)
    ax[0].plot(dtraj.x, dtraj.y, "-", color="orange", lw=0.6, alpha=0.7)
    ax[0].set_title(f"DISO brut (cap GT, deja propre) — {len(dcloud)} pts")
    ax[1].scatter(px, py, s=0.3, c="darkred", alpha=0.25)
    ax[1].plot(ctraj.x, ctraj.y, "-", color="black", lw=0.6, alpha=0.6)
    ax[1].set_title(f"SUBMAP DISO-local + cmd_vel-global (win={args.win}) — GT-free")
    for a in ax: a.axis("equal"); a.grid(alpha=0.3); a.set_xlabel("x"); a.set_ylabel("y")
    plt.tight_layout()
    out = os.path.join(args.cmdvel_run, "verify_submap.png")
    plt.savefig(out, dpi=120)
    print(f"PNG : {out}")


if __name__ == "__main__":
    main()

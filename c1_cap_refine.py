#!/usr/bin/env python3
"""C — essai 1 : estimateur de cap GT-FREE par recalage scan-a-scan ROTATION SEULE, seede compas.

Hypothese : DISO diverge GT-free car il estime 3-DOF (x,y,theta) sous-determine. Ici la position
x,y est FIGEE (baseline cmd_vel+USBL, 1.43m) et on n'estime que le CAP (1-DOF), seede par le compas
(lisse, sans derive). Pour chaque keyframe on aligne son scan (rotation autour de sa position figee)
sur une sous-carte des keyframes precedents -> correction de cap locale. Puis pose-graph 1-DOF :
  cap_k - cap_{k-1} = mesure_recalage   (w_reg)
  cap_k = compas_k                       (w_prior, ancre sans derive)
On rend le cloud avec (x,y baseline + cap raffine). Metrique = NN median (robuste, cf verify_cap_fix).

GT-free : positions = cmd_vel/USBL ; cap seed = compas (capteur embarque, = -dr_theta). 0 /pose_gt
dans l'optimisation (le compas vient de gt_heading.csv mais c'est le capteur, pas la position DGPS).
Usage : python3 c1_cap_refine.py [--w_prior 0.3] [--win 4] [--iters 2]
"""
import os, argparse
import numpy as np, pandas as pd
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = "TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959"
def wrap(a): return np.arctan2(np.sin(a), np.cos(a))


def rot_fit(B, A_centered):
    """angle phi minimisant |R(phi)B - A_centered|^2 (B,A centres sur p_k). 2D Procrustes."""
    cross = B[:,0]*A_centered[:,1] - B[:,1]*A_centered[:,0]
    dot = B[:,0]*A_centered[:,0] + B[:,1]*A_centered[:,1]
    return np.arctan2(cross.sum(), dot.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--w_prior", type=float, default=0.3, help="poids ancre compas (vs recalage=1)")
    ap.add_argument("--win", type=int, default=4, help="nb keyframes precedents pour la sous-carte de ref")
    ap.add_argument("--iters", type=int, default=2, help="passes (re-recalage avec caps raffines)")
    ap.add_argument("--max_corr", type=float, default=2.0, help="distance max appariement ICP (m)")
    args = ap.parse_args()

    traj = pd.read_csv(os.path.join(R, "trajectory.csv"))
    cloud = pd.read_csv(os.path.join(R, "pointcloud.csv"))
    gth = pd.read_csv(os.path.join(R, "gt_heading.csv"))
    idx = {int(k): i for i, k in enumerate(traj.keyframe_id)}
    px, py, pth = traj.x.to_numpy(), traj.y.to_numpy(), traj.theta.to_numpy()
    compass = gth.gt_compass.to_numpy()          # cap seed GT-free (= -dr_theta a une const pres)
    N = len(px)

    # body points par keyframe (deproj via le theta SLAM qui a rendu le cloud)
    cid = cloud.keyframe_id.to_numpy().astype(int)
    wx, wy = cloud.x.to_numpy(), cloud.y.to_numpy()
    body = {}
    for k in np.unique(cid):
        if k not in idx: continue
        i = idx[k]; m = cid == k; c, s = np.cos(pth[i]), np.sin(pth[i])
        body[k] = np.c_[c*(wx[m]-px[i])+s*(wy[m]-py[i]), -s*(wx[m]-px[i])+c*(wy[m]-py[i])]
    kfs = sorted(body.keys())

    def world(k, cap):
        c, s = np.cos(cap), np.sin(cap); B = body[k]
        return np.c_[c*B[:,0]-s*B[:,1]+px[idx[k]], s*B[:,0]+c*B[:,1]+py[idx[k]]]

    def render(caps):
        xs, ys = [], []
        for k in kfs:
            W = world(k, caps[idx[k]]); xs.append(W[:,0]); ys.append(W[:,1])
        return np.concatenate(xs), np.concatenate(ys)

    def nn_med(x, y, n=8000):
        rng = np.random.default_rng(0); s = rng.choice(len(x), min(n,len(x)), replace=False)
        P = np.c_[x[s], y[s]]; d, _ = cKDTree(P).query(P, k=2); return np.median(d[:,1])

    caps = compass.copy()           # seed
    for it in range(args.iters):
        # 1) mesures de recalage : pour chaque k, aligner scan k sur sous-carte precedente
        z = np.zeros(N)             # cap absolu recale de k (dans le repere courant)
        valid = np.zeros(N, bool)
        for n, k in enumerate(kfs):
            if n == 0: continue
            ref_k = kfs[max(0,n-args.win):n]
            A = np.vstack([world(j, caps[idx[j]]) for j in ref_k])
            tree = cKDTree(A)
            i = idx[k]; cap = caps[i]; pk = np.array([px[i], py[i]])
            for _ in range(8):
                W = world(k, cap)
                d, j = tree.query(W, k=1); msk = d < args.max_corr
                if msk.sum() < 12: break
                # rotation autour de pk : B (body) -> A apparies, centres sur pk
                phi = rot_fit(body[k][msk], A[j[msk]] - pk)
                if abs(wrap(phi - cap)) < 1e-4: cap = phi; break
                cap = phi
            z[i] = cap; valid[i] = msk.sum() >= 12 if 'msk' in dir() else False
        # 2) pose-graph 1-DOF : cap_k-cap_{k-1}=dz (recalage) ; cap_k=compas (prior)
        # systeme lineaire creux : assemble H cap = b
        from scipy.sparse import lil_matrix
        from scipy.sparse.linalg import spsolve
        H = lil_matrix((N, N)); b = np.zeros(N)
        wp = args.w_prior
        for i in range(N):                       # prior compas
            H[i, i] += wp; b[i] += wp * compass[i]
        for n in range(1, len(kfs)):             # relatif recalage
            ka, kb = kfs[n-1], kfs[n]; ia, ib = idx[ka], idx[kb]
            if not valid[ib]: continue
            dz = wrap(z[ib] - z[ia])             # relatif mesure
            # residu (cap_ib - cap_ia - dz) ; pour rester lineaire on travaille en increments
            H[ib, ib] += 1; H[ia, ia] += 1; H[ia, ib] -= 1; H[ib, ia] -= 1
            b[ib] += dz; b[ia] -= dz
        # ancrage absolu : ajoute par le prior compas (wp). Resout.
        caps = spsolve(H.tocsr(), b)
        x, y = render(caps); print(f"  passe {it+1}: NN median = {nn_med(x,y):.3f} m", flush=True)

    # comparaisons
    xc, yc = render(compass); xr, yr = render(caps)
    print(f"\ncompas seul    : NN = {nn_med(xc,yc):.3f} m")
    print(f"cap raffine C1 : NN = {nn_med(xr,yr):.3f} m   (cible DISO+GT = 0.242)")
    print(f"correction cap : med={np.degrees(np.median(np.abs(wrap(caps-compass)))):.1f}°  "
          f"max={np.degrees(np.max(np.abs(wrap(caps-compass)))):.1f}°")

    fig, ax = plt.subplots(1, 2, figsize=(18, 8))
    ax[0].scatter(xc, yc, s=0.3, c="navy", alpha=0.22); ax[0].set_title(f"compas seul (NN {nn_med(xc,yc):.3f})")
    ax[1].scatter(xr, yr, s=0.3, c="darkgreen", alpha=0.22)
    ax[1].set_title(f"C1 cap raffine rotation-seule (NN {nn_med(xr,yr):.3f}) — GT-free")
    for a in ax: a.axis("equal"); a.grid(alpha=0.3)
    plt.tight_layout(); out = "/tmp/c1_cap_refine.png"; plt.savefig(out, dpi=120)
    print(f"PNG : {out}")


if __name__ == "__main__":
    main()

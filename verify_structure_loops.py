#!/usr/bin/env python3
"""VERIFICATION OFFLINE : les loop closures sur STRUCTURE assemblent-elles le quai
SANS gacher la trajectoire ? On simule la strategie back-end hors-ligne sur un run existant,
donc SANS re-run risque. On balaie le poids des loops pour voir le compromis.

Idee : le decalage des blocs = derive INTER-PASSAGES. On trouve les paires de keyframes qui
revisitent la meme zone (proches en espace, loin en temps), on recale leurs sous-cartes de
STRUCTURE (fond deja retire par le filtre variance) par ICP, et on injecte ces contraintes
dans un pose-graph 2D avec ancrage (prior) sur la trajectoire SLAM actuelle (= USBL+odom+loops,
ATE 1.43m). En augmentant le poids des loops :
  - assemblage du quai (cellules occupees ↓, structure concentree)
  - ATE vs GT (degrade ? = l'intuition a verifier)

Etapes imprimees : (1) submaps+paires (2) ICP qualite (3) sweep ATE/assemblage.
Usage : python3 verify_structure_loops.py <run_dir>
"""
import os, sys, argparse, bisect
import numpy as np, pandas as pd
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def wrap(a): return np.arctan2(np.sin(a), np.cos(a))


def rigid_2d(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = (dst - mu_d).T @ (src - mu_s)
    U, _, Vt = np.linalg.svd(S)
    D = np.diag([1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    return R, mu_d - R @ mu_s


def icp(src, dst, max_corr=2.0, iters=25):
    """ICP 2D point-point. src,dst (N,2) monde. Retourne (R,t, inliers, rms)."""
    tree = cKDTree(dst)
    R_tot = np.eye(2); t_tot = np.zeros(2)
    cur = src.copy()
    inl, rms = 0, 9.9
    for _ in range(iters):
        d, j = tree.query(cur, k=1)
        m = d < max_corr
        if m.sum() < 10:
            break
        R, t = rigid_2d(cur[m], dst[j[m]])
        cur = (R @ cur.T).T + t
        R_tot = R @ R_tot; t_tot = R @ t_tot + t
        inl = int(m.sum()); rms = float(np.sqrt((d[m]**2).mean()))
        if np.hypot(*t) < 1e-3 and abs(np.arctan2(R[1,0],R[0,0])) < 1e-4:
            break
    return R_tot, t_tot, inl, rms


def umeyama_ate(traj_xy, times, gt):
    gts = [g[0] for g in gt]
    src, dst = [], []
    for (x, y), ts in zip(traj_xy, times):
        i = bisect.bisect_left(gts, ts)
        if i <= 0 or i >= len(gt): continue
        t0, gx0, gy0 = gt[i-1]; t1, gx1, gy1 = gt[i]
        a = (ts-t0)/(t1-t0) if t1 > t0 else 0
        dst.append((gx0+a*(gx1-gx0), gy0+a*(gy1-gy0))); src.append((x, y))
    src, dst = np.array(src), np.array(dst)
    R, t = rigid_2d(src, dst)
    return np.linalg.norm((R@src.T).T + t - dst, axis=1).mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--revisit_r", type=float, default=8.0, help="rayon revisite (m)")
    ap.add_argument("--min_sep", type=int, default=40, help="ecart min en keyframes (passage different)")
    ap.add_argument("--win", type=int, default=4, help="demi-fenetre submap (keyframes)")
    ap.add_argument("--icp_max_corr", type=float, default=2.5)
    ap.add_argument("--min_inl", type=int, default=40, help="inliers ICP min pour garder la loop")
    ap.add_argument("--max_rms", type=float, default=0.8, help="RMS ICP max pour garder la loop")
    args = ap.parse_args()

    traj = pd.read_csv(os.path.join(args.run_dir, "trajectory.csv"))
    clean = pd.read_csv(os.path.join(args.run_dir, "cloudmap_clean.csv"))
    gt = sorted((t, x, y) for t, x, y in
                pd.read_csv(os.path.join(args.run_dir, "groundtruth.csv")).itertuples(index=False))

    kid = traj.keyframe_id.to_numpy().astype(int)
    P0 = np.c_[traj.x, traj.y, traj.theta]              # poses SLAM (ancre globale)
    times = traj.time.to_numpy()
    idx = {int(k): i for i, k in enumerate(kid)}        # keyframe_id -> ligne
    # points de structure BODY par keyframe (pour reprojeter)
    cid = clean.keyframe_id.to_numpy().astype(int)
    cwx, cwy = clean.x.to_numpy(), clean.y.to_numpy()
    body = {}
    for k in np.unique(cid):
        if k not in idx: continue
        m = cid == k; ox, oy, oth = P0[idx[k]]
        c, s = np.cos(oth), np.sin(oth)
        dx, dy = cwx[m]-ox, cwy[m]-oy
        body[k] = np.c_[c*dx+s*dy, -s*dx+c*dy]

    def submap(center_k, P):
        """structure (monde) des keyframes autour de center_k, pose P."""
        xs, ys = [], []
        for k in range(center_k-args.win, center_k+args.win+1):
            if k not in body or k not in idx: continue
            x, y, th = P[idx[k]]; c, s = np.cos(th), np.sin(th)
            Q = body[k]
            xs.append(c*Q[:,0]-s*Q[:,1]+x); ys.append(s*Q[:,0]+c*Q[:,1]+y)
        if not xs: return None
        return np.c_[np.concatenate(xs), np.concatenate(ys)]

    # ---- ETAPE 1 : paires de revisite (proche espace, loin temps) ----
    kf_xy = P0[:, :2]
    tree = cKDTree(kf_xy)
    pairs = set()
    for i in range(len(kid)):
        for j in tree.query_ball_point(kf_xy[i], args.revisit_r):
            if abs(kid[i]-kid[j]) >= args.min_sep:
                a, b = sorted((int(kid[i]), int(kid[j])))
                pairs.add((a, b))
    # garder une paire par "zone" : sous-echantillonner pour ne pas surcharger
    pairs = sorted(pairs)[::3]
    print(f"[1] paires de revisite (inter-passage) : {len(pairs)}", flush=True)

    # ---- ETAPE 2 : ICP structure submap-to-submap ----
    loops = []
    for (a, b) in pairs:
        Sa, Sb = submap(a, P0), submap(b, P0)
        if Sa is None or Sb is None or len(Sa) < 30 or len(Sb) < 30: continue
        R, t, inl, rms = icp(Sa, Sb, max_corr=args.icp_max_corr)
        if inl < args.min_inl or rms > args.max_rms: continue
        # transforme la correction (monde) en contrainte relative entre poses a et b.
        # apres ICP : pose_a corrigee -> T∘pose_a doit coincider avec pose_b.
        dth = np.arctan2(R[1,0], R[0,0])
        pa, pb = P0[idx[a]], P0[idx[b]]
        # nouvelle pose a apres ICP
        na_xy = R @ pa[:2] + t; na_th = pa[2] + dth
        # mesure relative a->b = inv(na) ∘ pb
        c, s = np.cos(na_th), np.sin(na_th)
        dxy = pb[:2] - na_xy
        rel = np.array([c*dxy[0]+s*dxy[1], -s*dxy[0]+c*dxy[1], wrap(pb[2]-na_th)])
        loops.append((idx[a], idx[b], rel, inl, rms))
    inls = [l[3] for l in loops]; rmss = [l[4] for l in loops]
    print(f"[2] loops STRUCTURE retenues : {len(loops)}/{len(pairs)} "
          f"(inliers med={np.median(inls) if inls else 0:.0f}, RMS med={np.median(rmss) if rmss else 0:.2f} m)")
    if len(loops) < 5:
        print("    trop peu de loops fiables — structure trop eparse. Stop.")
        return

    # contraintes d'odometrie (consecutives) depuis P0 (poses SLAM) : ainsi w_loop=0
    # reproduit EXACTEMENT P0 (odom+prior coherents) → baseline propre, effet des loops isole.
    odom = []
    for k in range(len(kid)-1):
        pa, pb = P0[k], P0[k+1]; c, s = np.cos(pa[2]), np.sin(pa[2])
        dxy = pb[:2]-pa[:2]
        odom.append((k, k+1, np.array([c*dxy[0]+s*dxy[1], -s*dxy[0]+c*dxy[1], wrap(pb[2]-pa[2])])))

    def occupied(P, res=0.5):
        xs, ys = [], []
        for k in body:
            x, y, th = P[idx[k]]; c, s = np.cos(th), np.sin(th); Q = body[k]
            xs.append(c*Q[:,0]-s*Q[:,1]+x); ys.append(s*Q[:,0]+c*Q[:,1]+y)
        wx, wy = np.concatenate(xs), np.concatenate(ys)
        return len(set(zip(np.floor(wx/res).astype(int), np.floor(wy/res).astype(int)))), wx, wy

    occ0, _, _ = occupied(P0)
    ate0 = umeyama_ate(P0[:, :2], times, gt)
    print(f"[3] base : ATE={ate0:.2f} m | cellules occupees={occ0}\n")
    print(f"{'w_loop':>7} | {'ATE (m)':>8} | {'dATE':>6} | {'occup.':>7} | {'assemblage':>10}")
    print("-"*55)

    N = len(kid)
    # contraintes relatives en tableaux pour un Gauss-Newton creux VECTORISE
    ea = np.array([a for (a, b, z) in odom] + [l[0] for l in loops])
    eb = np.array([b for (a, b, z) in odom] + [l[1] for l in loops])
    ez = np.array([z for (a, b, z) in odom] + [l[2] for l in loops])
    is_loop = np.array([0]*len(odom) + [1]*len(loops))   # masque type de contrainte
    WROT = 3.0   # poids relatif de la composante angulaire

    def solve_gn(w_loop, w_odom=1.0, w_prior=0.5, iters=8):
        """Gauss-Newton creux, jacobien analytique. Contraintes relatives + prior global."""
        from scipy.sparse import coo_matrix
        from scipy.sparse.linalg import spsolve
        P = P0.copy()
        we = np.where(is_loop == 1, w_loop, w_odom)
        for _ in range(iters):
            pa, pb = P[ea], P[eb]
            ca, sa = np.cos(pa[:, 2]), np.sin(pa[:, 2])
            dx, dy = pb[:, 0]-pa[:, 0], pb[:, 1]-pa[:, 1]
            px = ca*dx + sa*dy
            py = -sa*dx + ca*dy
            pth = wrap(pb[:, 2]-pa[:, 2])
            rx = (px - ez[:, 0]) * we
            ry = (py - ez[:, 1]) * we
            rth = wrap(pth - ez[:, 2]) * we * WROT
            # jacobiens (M,3) wrt a et b pour chaque composante
            M = len(ea)
            rows, cols, vals = [], [], []
            def add(rr, ci, vv):
                rows.append(rr); cols.append(ci); vals.append(vv)
            ridx = np.arange(M)
            # composante x : d/dxa=-ca, dya=-sa, dθa=py ; dxb=ca, dyb=sa
            add(ridx*3+0, ea*3+0, -ca*we); add(ridx*3+0, ea*3+1, -sa*we); add(ridx*3+0, ea*3+2, py*we)
            add(ridx*3+0, eb*3+0,  ca*we); add(ridx*3+0, eb*3+1,  sa*we)
            # composante y : dxa=sa, dya=-ca, dθa=-px ; dxb=-sa, dyb=ca
            add(ridx*3+1, ea*3+0,  sa*we); add(ridx*3+1, ea*3+1, -ca*we); add(ridx*3+1, ea*3+2, -px*we)
            add(ridx*3+1, eb*3+0, -sa*we); add(ridx*3+1, eb*3+1,  ca*we)
            # composante θ : dθa=-1, dθb=1
            add(ridx*3+2, ea*3+2, -np.ones(M)*we*WROT); add(ridx*3+2, eb*3+2, np.ones(M)*we*WROT)
            nr = M*3
            # prior global (ancre = trajectoire SLAM, embarque USBL)
            pr = (P - P0); pr[:, 2] = wrap(pr[:, 2])
            rprior = (pr * np.array([1, 1, 2]) * w_prior).ravel()
            pri = np.arange(N*3)
            rows.append(nr+pri); cols.append(np.arange(N*3))
            vals.append(np.tile([1, 1, 2], N)*w_prior)
            R = np.concatenate([rx, ry, rth])
            # reordonner R en (x,y,th) par contrainte
            Rc = np.empty(M*3); Rc[0::3]=rx; Rc[1::3]=ry; Rc[2::3]=rth
            rfull = np.concatenate([Rc, rprior])
            rows = np.concatenate([np.atleast_1d(x) for x in rows])
            cols = np.concatenate([np.atleast_1d(x) for x in cols])
            vals = np.concatenate([np.atleast_1d(x) for x in vals])
            J = coo_matrix((vals, (rows, cols)), shape=(nr+N*3, N*3)).tocsr()
            H = J.T @ J; g = J.T @ rfull
            H = H + 1e-6*coo_matrix((np.ones(N*3), (np.arange(N*3), np.arange(N*3))), shape=(N*3, N*3))
            delta = spsolve(H.tocsc(), -g)
            P = P + delta.reshape(N, 3); P[:, 2] = wrap(P[:, 2])
        return P

    best = None
    for w_loop in [0.0, 0.5, 1.0, 2.0, 4.0, 8.0]:
        P = solve_gn(w_loop)
        ate = umeyama_ate(P[:, :2], times, gt)
        occ, wx, wy = occupied(P)
        asm = 100*(1-occ/occ0)
        if w_loop > 0 and ate <= ate0*1.10 and (best is None or asm > best[3]):
            best = (w_loop, P, ate, asm, wx, wy)
        print(f"{w_loop:>7.1f} | {ate:>7.2f} | {ate-ate0:>+5.2f} | {occ:>7} | {asm:>+9.0f}%", flush=True)

    print()
    if best is None:
        print("VERDICT : aucun poids n'assemble en preservant l'ATE (≤+10%). "
              "L'intuition se confirme : les loops structure tirent contre la trajectoire.")
        return
    w, P, ate, asm, wx, wy = best
    print(f"VERDICT : MEILLEUR compromis w_loop={w} → ATE {ate:.2f}m (base {ate0:.2f}) | "
          f"assemblage +{asm:.0f}%")
    print("  → si assemblage franc ET ATE preserve : la strategie est VRAIE et NON destructrice.")

    fig, ax = plt.subplots(1, 2, figsize=(18, 8))
    _, wx0, wy0 = occupied(P0)
    ax[0].scatter(wx0, wy0, s=0.6, c="navy", alpha=0.3); ax[0].set_title(f"base (ATE {ate0:.2f}m)")
    ax[1].scatter(wx, wy, s=0.6, c="darkred", alpha=0.3)
    ax[1].set_title(f"loops structure w={w} (ATE {ate:.2f}m, assemblage +{asm:.0f}%)")
    for a in ax: a.axis("equal"); a.grid(alpha=0.3)
    plt.tight_layout(); out = os.path.join(args.run_dir, "verify_structure_loops.png")
    plt.savefig(out, dpi=120); print(f"PNG : {out}")


if __name__ == "__main__":
    main()

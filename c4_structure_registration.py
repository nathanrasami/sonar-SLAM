#!/usr/bin/env python3
"""C — essai 4 : recalage rotation-seule sur les POINTS DE STRUCTURE UNIQUEMENT (gate
range-variance), seede C2. C1 echouait car il recalait sur le fond (66%, sans signal). Ici on
ne garde que les voxels structure (range-std eleve = vu a distances variees) et on aligne le cap
sur ces points-la. Pose-graph 1-DOF avec LISSAGE (cap voisin ~ similaire) pour eviter la collapse.
Juge au VISUEL (les metriques mentent par clustering)."""
import os, argparse
import numpy as np, pandas as pd
from scipy.spatial import cKDTree
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = "TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959"
def wrap(a): return np.arctan2(np.sin(a), np.cos(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--w_prior", type=float, default=0.2)
    ap.add_argument("--w_smooth", type=float, default=2.0, help="lissage cap voisins (anti-collapse)")
    ap.add_argument("--rstd", type=float, default=8.0, help="gate structure : range-std min (m)")
    ap.add_argument("--nkf", type=int, default=5)
    ap.add_argument("--win", type=int, default=6)
    ap.add_argument("--iters", type=int, default=3)
    args = ap.parse_args()

    traj = pd.read_csv(os.path.join(R, "trajectory.csv"))
    cloud = pd.read_csv(os.path.join(R, "pointcloud.csv"))
    idx = {int(k): i for i, k in enumerate(traj.keyframe_id)}
    px, py, pth = traj.x.to_numpy(), traj.y.to_numpy(), traj.theta.to_numpy()
    # cap C2 GT-free = -dr_theta + offset calibre (recalcule rapide via NN deja fait : +162°)
    compass = -traj.dr_theta.to_numpy() + np.radians(162)
    N = len(px)

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
        xs = [world(k, caps[idx[k]]) for k in kfs]; W = np.vstack(xs); return W[:,0], W[:,1]

    # gate STRUCTURE : voxels a forte range-variance (sur le rendu C2)
    def structure_mask(caps):
        x, y = render(caps)
        kf_all = np.concatenate([np.full(len(body[k]), k) for k in kfs])
        kx = np.array([px[idx[k]] for k in kf_all]); ky = np.array([py[idx[k]] for k in kf_all])
        rng = np.hypot(x-kx, y-ky); res = 1.5
        cxg = np.floor(x/res).astype(np.int64); cyg = np.floor(y/res).astype(np.int64)
        df = pd.DataFrame({"cx":cxg,"cy":cyg,"kf":kf_all,"rng":rng,"i":np.arange(len(x))})
        st = df.groupby(["cx","cy"]).agg(nkf=("kf","nunique"),rstd=("rng","std")).reset_index()
        st["rstd"]=st["rstd"].fillna(0)
        m = df.merge(st,on=["cx","cy"]).sort_values("i")
        return (m["rstd"].to_numpy()>=args.rstd)&(m["nkf"].to_numpy()>=args.nkf), kf_all

    def nn_med(x,y,n=8000):
        if len(x)<50: return float("nan")
        rg=np.random.default_rng(0); s=rg.choice(len(x),min(n,len(x)),replace=False)
        P=np.c_[x[s],y[s]]; d,_=cKDTree(P).query(P,k=2); return np.median(d[:,1])

    caps = compass.copy()
    # points de structure par keyframe (indices dans body[k]) — fige sur le seed
    smask, kf_all = structure_mask(caps)
    struct_body = {}
    off = 0
    for k in kfs:
        n = len(body[k]); mk = smask[off:off+n]; off += n
        if mk.sum() >= 8: struct_body[k] = body[k][mk]
    print(f"keyframes avec structure exploitable : {len(struct_body)}/{len(kfs)}", flush=True)

    for it in range(args.iters):
        z = {};
        for n, k in enumerate(kfs):
            if n == 0 or k not in struct_body: continue
            ref = [j for j in kfs[max(0,n-args.win):n] if j in struct_body]
            if not ref: continue
            Alist = []
            for j in ref:
                cj, sj = np.cos(caps[idx[j]]), np.sin(caps[idx[j]]); Bj = struct_body[j]
                Alist.append(np.c_[cj*Bj[:,0]-sj*Bj[:,1]+px[idx[j]], sj*Bj[:,0]+cj*Bj[:,1]+py[idx[j]]])
            A = np.vstack(Alist)
            tree = cKDTree(A); cap = caps[idx[k]]; pk = np.array([px[idx[k]], py[idx[k]]])
            for _ in range(10):
                c, s = np.cos(cap), np.sin(cap); B = struct_body[k]
                W = np.c_[c*B[:,0]-s*B[:,1]+pk[0], s*B[:,0]+c*B[:,1]+pk[1]]
                d, j = tree.query(W, k=1); mk = d < 3.0
                if mk.sum() < 6: break
                Ac = A[j[mk]] - pk
                phi = np.arctan2((B[mk,0]*Ac[:,1]-B[mk,1]*Ac[:,0]).sum(),
                                 (B[mk,0]*Ac[:,0]+B[mk,1]*Ac[:,1]).sum())
                if abs(wrap(phi-cap)) < 1e-4: cap = phi; break
                cap = phi
            z[k] = cap
        # pose-graph 1-DOF : prior compas + lissage + mesure structure
        H = lil_matrix((N,N)); b = np.zeros(N)
        for i in range(N): H[i,i]+=args.w_prior; b[i]+=args.w_prior*compass[i]
        for n in range(1,len(kfs)):       # lissage
            ia,ib = idx[kfs[n-1]], idx[kfs[n]]
            H[ia,ia]+=args.w_smooth; H[ib,ib]+=args.w_smooth; H[ia,ib]-=args.w_smooth; H[ib,ia]-=args.w_smooth
            d=wrap(compass[ib]-compass[ia]); b[ib]+=args.w_smooth*d; b[ia]-=args.w_smooth*d
        for k,zc in z.items():            # mesure absolue structure (poids 1)
            i=idx[k]; H[i,i]+=1.0; b[i]+=zc
        caps = spsolve(H.tocsr(), b)
        x,y = render(caps); print(f"  passe {it+1}: NN={nn_med(x,y):.3f}  corr_med={np.degrees(np.median(np.abs(wrap(caps-compass)))):.1f}°", flush=True)

    xr,yr = render(caps); xc,yc = render(compass)
    fig,ax=plt.subplots(1,2,figsize=(18,9))
    ax[0].scatter(xc,yc,s=0.3,c="darkgreen",alpha=0.2); ax[0].set_title(f"C2 cap (NN {nn_med(xc,yc):.3f})")
    ax[1].scatter(xr,yr,s=0.3,c="purple",alpha=0.2); ax[1].plot(px,py,"k-",lw=0.5,alpha=0.4)
    ax[1].set_title(f"C4 recalage structure-only (NN {nn_med(xr,yr):.3f}) — GT-free")
    for a in ax: a.axis("equal"); a.grid(alpha=0.3)
    plt.tight_layout(); plt.savefig("/tmp/c4_structure.png",dpi=130); print("PNG : /tmp/c4_structure.png")


def world_struct(k, cap, sb):
    import numpy as np
    c, s = np.cos(cap), np.sin(cap); B = sb[k]
    # position recuperee globalement — passe via closure n'est pas dispo ici, on recompose plus haut
    return np.c_[c*B[:,0]-s*B[:,1], s*B[:,0]+c*B[:,1]]


if __name__ == "__main__":
    main()

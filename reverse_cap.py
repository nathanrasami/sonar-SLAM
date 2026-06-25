#!/usr/bin/env python3
"""RETRO-INGENIERIE du cap : quelle transformation manque pour passer du cap GT-FREE
(cmd_vel) au cap GT (qui donne la bonne carte) ? On compare par keyframe et on ajuste
des modeles simples (offset, derive lineaire, reflexion, echelle). Le residu apres le
meilleur modele dit si l'ecart est une transformation GLOBALE corrigeable (residu faible)
ou du BRUIT LOCAL non-parametrable (residu ~ l'incoherence qui fait le swirl).

Sort : <run>/gt_heading.csv (cap GT sauve) + diagnostic console + reverse_cap.png.
Usage : python3 reverse_cap.py <cmdvel_run> [--gt_compass /tmp/gt_pose_compass.csv]
"""
import os, sys, argparse, bisect
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt


def wrap(a): return np.arctan2(np.sin(a), np.cos(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", nargs="?", default=os.environ.get("SLAM_RESULTS_DIR", "."))
    ap.add_argument("--gt_compass", default="/tmp/gt_pose_compass.csv")
    args = ap.parse_args()

    traj = pd.read_csv(os.path.join(args.run_dir, "trajectory.csv"))
    gtp = pd.read_csv(os.path.join(args.run_dir, "groundtruth.csv")).sort_values("time").reset_index(drop=True)
    t = traj.time.to_numpy()
    free_dr = traj.dr_theta.to_numpy()      # cap GT-free PUR (odometrie cmd_vel)
    free_slam = traj.theta.to_numpy()        # cap apres SLAM (rendu de la carte)

    # cap GT = COURSE (atan2 de la vitesse GT), lisse sur +-1.5s autour de chaque keyframe
    gtt = gtp.time.to_numpy(); gx = gtp.x.to_numpy(); gy = gtp.y.to_numpy()
    def gt_course(tq, dt=1.5):
        i0 = bisect.bisect_left(gtt, tq-dt); i1 = bisect.bisect_left(gtt, tq+dt)
        i0 = max(i0, 0); i1 = min(i1, len(gtt)-1)
        if i1 <= i0: return np.nan
        return np.arctan2(gy[i1]-gy[i0], gx[i1]-gx[i0])
    gt_crs = np.array([gt_course(tq) for tq in t])

    # cap GT = COMPAS (optionnel — fichier externe bag-specific)
    has_compass = os.path.exists(args.gt_compass)
    if has_compass:
        comp = pd.read_csv(args.gt_compass).sort_values("time").reset_index(drop=True)
        ct = comp.time.to_numpy(); cc = np.unwrap(comp.compass.to_numpy())
        gt_cmp = wrap(np.interp(t, ct, cc))
    else:
        print(f"[reverse_cap] gt_compass non trouve ({args.gt_compass}) — gt_compass=NaN dans gt_heading.csv")
        gt_cmp = np.full(len(t), np.nan)

    # sauver le cap GT
    out_csv = os.path.join(args.run_dir, "gt_heading.csv")
    pd.DataFrame({"keyframe_id": traj.keyframe_id, "time": t,
                  "gt_course": gt_crs, "gt_compass": gt_cmp}).to_csv(out_csv, index=False)
    print(f"cap GT sauve : {out_csv}\n")

    # --- comparaison + modeles de transformation ---
    valid = ~np.isnan(gt_crs)
    def analyse(free, ref, name):
        d = wrap(free[valid] - ref[valid])
        tt = t[valid] - t[valid][0]
        du = np.unwrap(d)
        # modeles : offset / lineaire(derive) / reflexion / echelle
        off = np.median(du)
        A = np.c_[tt, np.ones_like(tt)]; (a, b), *_ = np.linalg.lstsq(A, du, rcond=None)
        res_off = np.degrees(np.std(wrap(du - off)))
        res_lin = np.degrees(np.std(du - (a*tt + b)))
        # reflexion : free ~ -ref + c  => free+ref ~ c
        s = wrap(free[valid] + ref[valid]); res_ref = np.degrees(np.std(wrap(s - np.median(s))))
        # echelle : free ~ k*ref + c
        Ar = np.c_[np.unwrap(ref[valid]), np.ones(valid.sum())]
        (k, c), *_ = np.linalg.lstsq(Ar, np.unwrap(free[valid]), rcond=None)
        res_sc = np.degrees(np.std(np.unwrap(free[valid]) - (k*np.unwrap(ref[valid])+c)))
        print(f"[{name} vs {('cap='+name)}] ecart median={np.degrees(off):+.1f}°  RMS brut={np.degrees(np.std(d)):.1f}°")
        print(f"    residu apres OFFSET constant   : {res_off:.1f}°")
        print(f"    residu apres DERIVE lineaire    : {res_lin:.1f}°  (derive {np.degrees(a)*60:.2f}°/min)")
        print(f"    residu apres REFLEXION          : {res_ref:.1f}°")
        print(f"    residu apres ECHELLE k={k:.3f}    : {res_sc:.1f}°")
        return d, off

    print("=== cap GT-FREE PUR (dr_theta) vs GT-course ===")
    d_dr_crs, _ = analyse(free_dr, gt_crs, "dr/course")
    print("\n=== cap SLAM (theta, rendu carte) vs GT-course ===")
    d_sl_crs, _ = analyse(free_slam, gt_crs, "slam/course")

    fig, ax = plt.subplots(2, 1, figsize=(15, 9))
    tt = t - t[0]
    ax[0].plot(tt, np.degrees(np.unwrap(free_dr)), label="GT-free dr_theta", lw=1)
    ax[0].plot(tt, np.degrees(np.unwrap(np.where(valid, gt_crs, np.nan))), label="GT course", lw=1)
    ax[0].set_ylabel("cap (deg, unwrap)"); ax[0].grid(alpha=0.3)
    ax[0].set_title("Caps : GT-free vs GT (course)")
    if has_compass:
        print("\n=== cap GT-FREE PUR (dr_theta) vs GT-compas ===")
        d_dr_cmp, _ = analyse(free_dr, gt_cmp, "dr/compas")
        ax[0].plot(tt, np.degrees(np.unwrap(gt_cmp)), label="GT compas", lw=1, alpha=0.7)
        ax[1].plot(tt[valid], np.degrees(d_dr_cmp), label="dr - compas", lw=0.8, alpha=0.7)
    ax[0].legend()
    ax[1].plot(tt[valid], np.degrees(d_dr_crs), label="dr - course", lw=0.8)
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("ecart cap (deg)"); ax[1].legend(); ax[1].grid(alpha=0.3)
    ax[1].set_title("Ecart GT-free - GT (cherche : constant ? derive ? bruit ?)")
    plt.tight_layout(); out = os.path.join(args.run_dir, "reverse_cap.png")
    plt.savefig(out, dpi=120); print(f"\nPNG : {out}")


if __name__ == "__main__":
    main()

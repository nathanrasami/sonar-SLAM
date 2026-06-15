#!/usr/bin/env python3
"""Ablation DISO/GT : la sortie DISO (nourrie d'une GT DÉRIVÉE) suit-elle la
dérive (=> dépendant GT) ou la corrige-t-elle par le sonar (=> vraie odométrie) ?

Compare diso_nogt à DEUX références (vraie GT et GT dérivée reconstruite),
alignement Umeyama avec réflexion autorisée (DISO sort en Y-flip)."""
import sys
import numpy as np

DRIFT_RATE = 0.003  # m/s latéral (cf gt_drift_node.py)

# IMPORTANT : comparer DISO normal et DISO dérivé DANS LE MÊME ENVIRONNEMENT.
# DISO est très sensible à l'environnement (VM ~2 m vs Docker ~4,5 m, cf STAGE.md),
# donc REFDISO doit être le run normal Docker, pas le run VM 06-10 (sinon le delta
# mélange "corruption GT" et "VM→Docker" → conclusion fausse).
NOGT = "results/run_diso_nogt_2026-06-15_101108/diso_trajectory.csv"   # Docker, GT dérivée
REFGT = "results/run_diso_2026-06-10_163336/groundtruth.csv"           # vraie GT (même bag)
REFDISO = "results/run_diso_2026-06-15_105930/diso_trajectory.csv"     # Docker, GT normale


def load_xy(path, cols=(0, 1, 2)):
    a = np.loadtxt(path, delimiter=",", skiprows=1, usecols=cols)
    return a[:, 0], a[:, 1:3]  # t, (x,y)


def resample(t_src, xy_src, t_q):
    """Interpole xy_src(t_src) aux instants t_q (clip aux bornes)."""
    x = np.interp(t_q, t_src, xy_src[:, 0])
    y = np.interp(t_q, t_src, xy_src[:, 1])
    return np.column_stack([x, y])


def umeyama(src, dst, allow_reflection=True):
    """Aligne src->dst (sim transform). Retourne dst_pred et RMSE (ATE)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s, d = src - mu_s, dst - mu_d
    cov = d.T @ s / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(2)
    if not allow_reflection and np.linalg.det(U @ Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    var_s = (s ** 2).sum() / len(src)
    c = (D * np.diag(S)).sum() / var_s
    t = mu_d - c * R @ mu_s
    pred = (c * (R @ src.T).T) + t
    ate = np.sqrt(((pred - dst) ** 2).sum(1).mean())
    return ate


# --- charge ---
t_gt, gt = load_xy(REFGT, cols=(0, 1, 2))     # vraie GT (time,x,y)
t_d, dn = load_xy(NOGT)                         # diso nogt
t_dref, dref = load_xy(REFDISO)                # diso normal (06-10, vraie GT)
t0 = t_gt.min()

# GT dérivée reconstruite : y += rate*(t-t0)
gt_drift = gt.copy()
gt_drift[:, 1] = gt[:, 1] + DRIFT_RATE * (t_gt - t0)

# fenêtre temporelle commune
lo = max(t_gt.min(), t_d.min(), t_dref.min())
hi = min(t_gt.max(), t_d.max(), t_dref.max())
mask = (t_gt >= lo) & (t_gt <= hi)
tq = t_gt[mask]
gt_q, gtd_q = gt[mask], gt_drift[mask]

dn_q = resample(t_d, dn, tq)
dref_q = resample(t_dref, dref, tq)

# --- ATE ---
ate_nogt_real = umeyama(dn_q, gt_q)
ate_nogt_drift = umeyama(dn_q, gtd_q)
ate_ref_real = umeyama(dref_q, gt_q)   # référence "bon DISO" vs vraie GT

print(f"Durée analysée      : {hi-lo:.0f} s  ({mask.sum()} points)")
print(f"Dérive injectée fin : {DRIFT_RATE*(hi-t0):.2f} m latéral + 3° yaw\n")
print(f"DISO normal (Docker) vs vraie GT   : ATE = {ate_ref_real:.2f} m   [référence]")
print(f"DISO nogt (dérive)   vs vraie GT   : ATE = {ate_nogt_real:.2f} m")
print(f"DISO nogt (dérive)   vs GT dérivée : ATE = {ate_nogt_drift:.2f} m\n")

# Effet PUR de la corruption GT (à environnement constant) + % corrigé par le sonar
injected = DRIFT_RATE * (hi - t0)               # dérive latérale injectée (m)
degradation = ate_nogt_real - ate_ref_real      # dégradation due à la corruption
corrected = 100.0 * (1.0 - degradation / injected)
print(f"Effet pur corruption GT : +{degradation:.2f} m  (sur {injected:.2f} m injectés)")
print(f"=> le sonar corrige ~{corrected:.0f}% de la dérive injectée\n")

if ate_nogt_drift + 0.5 < ate_nogt_real:
    print("=> DISO nogt colle MIEUX à la GT DÉRIVÉE qu'à la vraie GT")
    print("   => DISO SUIT le prior => DÉPENDANT de la GT (le sonar ne corrige pas).")
elif corrected >= 60.0:
    print("=> DISO corrige la majeure partie de la dérive injectée")
    print("   => vraie odométrie sonar (prior GT = ancre faible : init + scale).")
else:
    print("=> Cas intermédiaire : prior GT influent mais sonar partiellement correcteur.")

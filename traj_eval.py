"""Évaluation de trajectoire SLAM (standard TUM/evo).

Association temporelle exacte par interpolation + alignement rigide optimal
(Umeyama 1991). Remplace l'alignement manuel fragile (flip Y + centroïde).
"""
import numpy as np


def associer_par_temps(t_est, t_gt, x_gt, y_gt):
    """Interpole la position GT à chaque timestamp estimé.

    Résout les tailles différentes (ex: DISO 14434, Bruce 189, GT 14436) et
    le sous-échantillonnage des keyframes. Retourne un tableau (N, 2).
    Pré-requis : t_gt croissant (cas des timestamps Unix).
    """
    t_est = np.asarray(t_est, dtype=float)
    gx = np.interp(t_est, t_gt, x_gt)
    gy = np.interp(t_est, t_gt, y_gt)
    return np.column_stack([gx, gy])


def umeyama(src, dst, with_scale=False, allow_reflection=True):
    """Alignement rigide optimal src -> dst (nuages (N, 2)).

    allow_reflection=True : ne force PAS det(R)=+1, donc une réflexion
    (ex: axe Y inversé de DISO) est absorbée automatiquement → aucun flip
    manuel nécessaire.
    Retourne (s, R, t) tel que : dst ≈ s * (R @ src.T).T + t.
    """
    src, dst = np.asarray(src, dtype=float), np.asarray(dst, dtype=float)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    cov = (D.T @ S) / len(src)
    U, Dg, Vt = np.linalg.svd(cov)
    W = np.eye(2)
    if not allow_reflection and np.linalg.det(U @ Vt) < 0:
        W[-1, -1] = -1
    R = U @ W @ Vt
    s = (Dg * np.diag(W)).sum() / (S ** 2).sum() * len(src) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def appliquer(s, R, t, xy):
    """Applique la transformation (s, R, t) à un nuage (N, 2).

    Sert pour la trajectoire ET le point cloud (même transfo → cohérence).
    """
    xy = np.asarray(xy, dtype=float)
    return (s * (R @ xy.T).T) + t


def calculer_ate(est_aligne, gt_xy):
    """ATE = RMSE des distances euclidiennes après alignement."""
    return np.sqrt(np.mean(np.sum((est_aligne - gt_xy) ** 2, axis=1)))

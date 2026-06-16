# Alignement GT - SLAM

## Alignement Umeyama
**Script :** `analyze_drift.py` → produit `trajectory_umeyama.csv` + `trajectory_plot.png`

Trouve la transformation (rotation + translation + échelle optionnelle) qui **minimise globalement**
la distance entre la trajectoire estimée et la GT. C'est un optimum au sens des moindres carrés
sur l'ensemble du trajet.

**Formule :**
```
ATE = RMSE( R·est + t , gt )   avec (R, t) = argmin RMSE
```

**Ce que ça fait concrètement :**
- Fait pivoter + déplacer toute la trajectoire SLAM pour qu'elle colle au mieux à la GT
- Le point de départ n'est PAS forcément à (0,0) — c'est là où Umeyama place la trajectoire optimalement
- Résultat : ATE **optimiste** (meilleur fit possible)

**Interprétation :** si Umeyama donne 3,5 m, c'est la dérive résiduelle **après avoir corrigé
le biais de cap et de position**. Utile pour comparer des algos entre eux (même correction appliquée à tous).

---

## Alignement Origine (départ aligné)
**Script :** `analyze_origine.py` → produit `trajectory_centre.csv` + `trajectory_plot_origine.png`

Aligne **uniquement le DÉBUT** de chaque trajectoire sur la GT, sans optimisation globale.
On ajuste une transfo fixe (rotation OU réflexion, **sans échelle**) sur les ~15 % premiers
points seulement, on l'applique à toute la trajectoire, puis on recentre à (0,0).

**Formule :**
```
(R) = umeyama( est[:n] , gt[:n] )       # n = 15% des points, réflexion autorisée, sans échelle
est_aligné = (R · est) - (R · est)[0]
gt_aligné  = gt - gt[0]
ATE = RMSE( est_aligné , gt_aligné )
```

**Pourquoi pas une translation pure ?** Une translation seule laisse les flips/rotations de
repère (DISO swap x/y, /cmd_vel sans cap absolu) → plots illisibles, ATE de 30 m+. Ajuster le
**cap de départ** règle ça pour toutes les méthodes, avec une seule règle :
- run main (odom = GT relayée) → R ≈ identité (rien à corriger)
- DISO (swap x/y) → R = réflexion (corrige le flip)
- Odom pure (/cmd_vel) → R = rotation du cap initial

**Ce que ça fait concrètement :**
- On ne corrige QUE le début → la **dérive en aval reste visible** (≠ Umeyama qui la masque)
- Résultat : ATE **conservateur** (on n'optimise pas globalement)

**Interprétation :** si Origine donne 7 m alors qu'Umeyama donne 3,5 m, l'écart représente
la **dérive de cap** accumulée que Bruce-SLAM produit et qu'Umeyama efface par rotation globale.

---

## Comparaison des deux

| | Umeyama | Origine |
|---|---|---|
| Ajustement | Global (tous les points) | Début seulement (~15 %) |
| Transformation | Rotation/réflexion + translation + échelle opt. | Rotation/réflexion (sans échelle), recentré (0,0) |
| Point de départ | Libre (placé par l'optimum) | Toujours (0,0) |
| ATE | Optimiste = **min(ATE)** | Conservateur (dérive visible) |
| Dérive de cap | Masquée (rotation globale l'absorbe) | Visible (on ne corrige que le début) |
| Rôle | **Chiffre officiel** (standard TUM/KITTI) | **Diagnostic** de dérive |

**Les deux ATE encadrent la réalité.** On génère SYSTÉMATIQUEMENT les deux à chaque test :
Umeyama = chiffre publiable, Origine = lecture honnête de la dérive depuis le départ.

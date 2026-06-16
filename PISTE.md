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

## Alignement Origine (départ commun)
**Script :** `analyze_origine.py` → produit `trajectory_centre.csv` + `trajectory_plot_origine.png`

Épingle le **premier point** de chaque trajectoire à (0,0) en soustrayant simplement la première
coordonnée. Aucune rotation.

**Formule :**
```
est_aligné = est - est[0]
gt_aligné  = gt  - gt[0]
ATE = RMSE( est_aligné , gt_aligné )
```

**Ce que ça fait concrètement :**
- Tout le monde part du même point (0,0) — on "colle" les débuts
- Aucune rotation : si SLAM a un biais de cap dès le départ, il reste dans l'ATE
- Résultat : ATE **conservateur/honnête** (on sait où on a démarré, pas de triche)

**Interprétation :** si Origine donne 16 m alors qu'Umeyama donne 3,5 m,
l'écart (≈12 m) représente principalement la **dérive de cap** que Bruce-SLAM accumule
et qu'Umeyama efface par rotation.

---

## Comparaison des deux

| | Umeyama | Origine |
|---|---|---|
| Transformation | Rotation + Translation | Translation seule |
| Point de départ | Libre (placé par l'optimum) | Toujours (0,0) |
| ATE | Optimiste (meilleur fit) | Conservateur (dérive réelle) |
| Sensible au biais de cap | Non (rotation l'absorbe) | Oui |
| Utile pour | Comparer algos, publication | Évaluer dérive depuis départ connu |

**Les deux ATE encadrent la réalité.**
La vérité terrain d'un SLAM opérationnel se situe entre les deux.

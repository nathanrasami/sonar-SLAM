# Résumé complet — SIO-UV: Rapid and Robust Sonar Inertial Odometry for Underwater Vehicles

**Titre :** *SIO-UV: Rapid and Robust Sonar Inertial Odometry for Underwater Vehicles*
**Auteurs :** Jibo Bai, Daqi Zhu, Mingzhi Chen, Chaomin Luo
**Institutions :** University of Shanghai for Science and Technology (Chine), Mississippi State University (USA)
**Publié :** IEEE Transactions on Industrial Electronics, 2025 (DOI 10.1109/TIE.2025.3561817)
**Code source :** non publié

---

## Abstract

SIO-UV est un framework d'**odométrie sonar-inertielle** (IMU + sonar imageur multifaisceaux FLS)
construit sur un **factor graph**. Il fusionne la pré-intégration IMU (qui fournit une pose
initiale) et l'observation sonar (qui la raffine) pour estimer la trajectoire et construire une
carte. Trois apports : (1) un débruitage **MCFAR** (multiscale constant false alarm rate),
(2) une conversion des images sonar 2D en **nuages de points 3D** par "reverse regression
mapping", (3) une odométrie sonar de type LOAM (features corner/surface). Validé en simulation
(Gazebo) et sur un ROV réel en piscine. **Bat directement Bruce-SLAM** dans toutes les
expériences.

---

## I. Introduction

### Contexte

Sous l'eau, l'optique et le LiDAR sont inutilisables (absorption, diffusion de la lumière). Le
**sonar multifaisceaux frontal (FLS)** est le capteur de référence. Mais ses données sont
**rares et bruitées** → le matching d'images sonar est peu fiable. On le couple donc à un **IMU**
pour stabiliser l'estimation, dans un **factor graph** (optimisation MAP).

### Limites des méthodes existantes (que SIO-UV vise à corriger)

1. Le **matching d'images sonar par features** (ex. A-KAZE) devient peu fiable en milieu
   bruité → erreurs de mise en correspondance, robustesse compromise.
2. Le **matching de nuages 2D par ICP** (= exactement le SSM/NSSM de Bruce) est
   **sous-optimal** à cause de la rareté des points 2D.
3. L'estimation IMU par **filtrage** (EKF/PF) accumule les erreurs sur les longues trajectoires.

### Idée centrale

Reconstruire un nuage **3D** (au lieu de 2D) pour densifier l'information de matching, débruiter
proprement avant extraction (MCFAR), et tout fusionner avec l'IMU dans un factor graph optimisé
par iSAM2.

---

## II. Related Work

Deux familles de fusion IMU+FLS :

- **Filtre** (EKF, PF, RBPF…) : sensible à la complexité des mises à jour et aux erreurs de
  linéarisation. **Bruce-SLAM est dans cette catégorie** (EKF pour l'IMU).
- **Graphe** (ASFM, RIAUL…) : représente clairement les relations état/observation, plus
  robuste, mais repose sur le matching de features sonar — si le matching échoue, tout le
  système est compromis.

→ SIO-UV se place côté **graphe**, mais remplace le matching de features fragile par un matching
de nuages **3D** densifiés.

---

## III. Méthodologie — Rapid and Robust Sonar Inertial Odometry

### A. Framework (3 threads)

Entrée (IMU + images FLS) → **calibration temporelle** → puis trois threads parallèles :
1. **Pré-intégration IMU** : donne la transformation initiale `T_imu` entre deux keyframes sonar.
2. **FLS Reverse Regression Mapping** : MCFAR → nuage 3D → extraction de features.
3. **Graph optimization** (BackEnd) : matching keyframes ↔ sous-carte, donne la pose sonar
   `T_sonar`, et **renvoie les biais corrigés vers la pré-intégration IMU** (boucle de
   rétroaction).

> Architecture identique à Bruce sur le principe : **l'IMU fournit l'init, le sonar raffine.**
> C'est précisément ce qui manque sur Aracati (pas d'IMU).

### B. Pré-intégration IMU

État du véhicule :

$$x = [P, R, V, b_a, b_g]^T$$

(position, rotation, vitesse, biais accéléromètre, biais gyroscope). La pré-intégration (formule
standard de Forster et al.) accumule les incréments entre deux keyframes `i` et `j` pour éviter
de réintégrer à chaque itération d'optimisation :

$$\Delta\tilde{P}_{ij} = R_i^T\left(P_j - P_i - V_i\Delta t_{ij} - \tfrac{1}{2}g\Delta t_{ij}^2\right) + \delta P_{ij}$$
$$\Delta\tilde{V}_{ij} = R_i^T\left(V_j - V_i - g\Delta t_{ij}\right) + \delta V_{ij}$$
$$\Delta\tilde{R}_{ij} = R_i^T R_j \, \mathrm{Exp}(\delta\phi_{ij})$$

où `δP, δV` sont les termes de bruit et `Exp(δφ)` la carte exponentielle Lie convertissant un
petit vecteur de rotation en matrice.

### C. Reverse Regression Mapping — le cœur du papier

#### 1. MCFAR (débruitage multi-échelle)

La force du signal `x` suit une loi exponentielle (bruit acoustique) :

$$f(x) = \frac{1}{\sigma}e^{-\frac{x}{\sigma}}, \quad x \geq 0$$

À chaque échelle `s`, on estime le bruit `σ̂_s` comme moyenne d'une fenêtre de taille `N_s`
autour du pixel :

$$\hat{\sigma}_s = \frac{1}{N_s}\sum_{i=1}^{N_s} x_i$$

On en déduit un seuil de détection `T_s = α_s · σ̂_s` (éq. 5), où le **facteur de seuil** dérive
directement du taux de fausse alarme voulu `P_fa` (éq. 6-9) :

$$P_{fa} = e^{-\alpha_s} \quad\Rightarrow\quad \alpha_s = -\ln(P_{fa})$$

**Fusion multi-échelle** (éq. 10) : on prend le seuil **minimum** sur les 3 échelles :

$$T_{final} = \min_s T_s$$

Logique : petites échelles `s` = sensibles aux petites cibles mais bruitées ; grandes échelles =
robustes mais ratent les petites cibles. Le **min** garantit qu'une cible détectée à *une seule*
échelle survit → on évite les **détections manquées** (miss-detection) en fond bruité complexe.

> Différence avec le CFAR de Bruce : Bruce utilise un CFAR **mono-échelle** (SOCA). MCFAR ajoute
> la dimension multi-échelle + le min des seuils.

#### 2. Conversion 2D → 3D (reverse regression mapping)

Un point sonar en sphérique `p[r, θ, φ]` (range, azimuth, élévation) devient cartésien (éq. 11) :

$$P = \begin{bmatrix} X \\ Y \\ Z \end{bmatrix} = R \begin{bmatrix} \cos\theta\cos\phi \\ \sin\theta\cos\phi \\ \sin\phi \end{bmatrix}$$

Problème : l'image FLS **perd l'élévation φ**. Astuce de SIO-UV : comme φ est petit (10-20°) et
la portée verticale ~1 m, ils **étirent verticalement** le nuage horizontal — ils empilent
(stacking) le scan horizontal le long de la verticale en supposant un balayage vertical analogue.

> **C'est une 3D *supposée*, pas mesurée.** Les auteurs le reconnaissent : "reasonable
> assumption, not true vertical information". Ça densifie le nuage pour le matching, mais ça ne
> reconstruit pas la vraie géométrie verticale (ex. la courbure réelle d'un barrage).

#### 3. Extraction de features (façon LOAM)

Sur le nuage 3D, on extrait par courbure : **corner features** `F_p` (courbure forte) et
**surface features** `F_s` (courbure faible), plus les features horizontales `F_h` (façon scan
LiDAR 2D).

### D. Factor Graph Optimization

Graphe à un seul type de nœud (pose `x_i`) et trois facteurs : IMU `u_i`, odométrie sonar `s_i`,
loop closure `l_i`. MAP :

$$X^* = \arg\max_x p(x_0)\prod_k p(u_k|x_{k-1},x_k)\prod_i p(s_i|x_{i-1},x_i)\prod_j p(l_j|x_*,x_{**})$$

Sous hypothèse gaussienne, devient un moindres carrés non-linéaire (éq. 15) résolu par
**Gauss-Newton / Levenberg-Marquardt** via **iSAM2** (comme Bruce).

**Odométrie sonar fusionnée** (éq. 16) — la nouveauté : deux matchings combinés par poids,
- `T_h` : features horizontales (façon LiDAR 2D) via **PL-ICP**,
- `T_c` : corner+surface via point-surface matching (façon **LIO-SAM/LOAM**),

$$T_{sonar} = w_1 T_h + w_2 T_c$$

> Plus riche que l'ICP unique de Bruce. Le matching se fait entre la keyframe courante et la
> sous-carte locale (SubMap-Match), initialisé par `T_imu`.

**Loop closure** : recherche par distance euclidienne entre keyframes (comme Bruce), puis ICP
3D pour valider et créer le facteur de boucle.

---

## IV. Évaluation expérimentale

### A. Simulation (Gazebo, ROS Noetic)

ROV Rexrov + sonar BlueView M900 simulé (Dave platform, FOV horizontal 130°, FOV vertical 15°,
range 10 m, 512 faisceaux horizontaux, 256 verticaux). Deux scènes : 40×40 m bien distincte,
30×30 m à faible distinguabilité (dégradée).

**Résultats (Table I) — métriques APE (Absolute Pose Error) et RMSE :**

| Scène | SIO-UV (RMSE) | Bruce-SLAM (RMSE) | ASFM-SLAM (RMSE) |
|-------|---------------|-------------------|------------------|
| 1 (40×40, distincte) | **1.79 m** | 8.48 m | 15.3 m |
| 2 (30×30, dégradée)  | **1.86 m** | 9.30 m | 16.1 m |

### B. Expérience réelle (piscine 25×25 m)

ROV avec BlueView M900 (900-2250 kHz, FOV 130°, range 10-100 m) + IMU MEMS, FLS à 5 Hz, IMU à
100 Hz.

**Résultats (Table III) :**

| Scène | SIO-UV (RMSE) | Bruce-SLAM (RMSE) | ASFM-SLAM (RMSE) |
|-------|---------------|-------------------|------------------|
| Piscine | **1.68 m** | **22.2 m** | 27.7 m |

> **Pourquoi Bruce s'effondre ici (point critique).** SIO-UV fait tourner Bruce avec
> **SOCA-CFAR + ICP sur nuage 2D**, et son erreur de matching ICP **explose dans les virages**
> et en environnement à faible distinguabilité (§IV-D). Autrement dit : ce papier démontre
> **exactement** la thèse de mon stage — *l'odométrie ICP front-end de Bruce est le maillon
> faible*. SIO-UV y répond par MCFAR+3D-LOAM ; moi par DISO. Deux réponses concurrentes au même
> diagnostic.

### C. Comparaison des débruitages (Table II)

| Méthode | Échelles | Temps (ms) | Erreur de matching ICP |
|---------|----------|-----------|------------------------|
| CFAR mono-échelle | 1 | 5 | 29-35 % |
| **MCFAR** | 3 | 11 | **18.6-21.4 %** |
| SOCA-CFAR (celui de Bruce) | 1 | 6 | 23-26 % |

→ MCFAR réduit l'erreur de matching de ~12 % vs CFAR mono-échelle et ~5 % vs SOCA-CFAR, au prix
d'un temps ~2× (mais accéléré CUDA, donc temps-réel compatible).

### D. Time efficiency

FrontEnd 15-22 ms, BackEnd 50-60 ms. ~5 ms de plus que les concurrents, accepté vu le gain de
robustesse. MCFAR accéléré CUDA (×3 vs sans).

---

## V. Conclusion

SIO-UV = odométrie sonar-inertielle sur factor graph, avec MCFAR + conversion 3D + odométrie
LOAM, battant Bruce-SLAM et ASFM-SLAM en simulation et en réel. Perspective : intégrer
vision + USBL pour réduire la dérive.

---

## Lien avec mon projet (Bruce_SLAM + DISO) et comparaison avec Sonar Context

### Ce que SIO-UV partage avec mon diagnostic

Ce papier **bat Bruce-SLAM** précisément parce que l'**ICP front-end de Bruce est défaillant**
(RMSE 22 m en piscine, explosion dans les virages). C'est la **preuve externe** que mon choix de
remplacer le SSM/ICP par DISO est justifié → **à citer dans la présentation** pour légitimer
DISO devant le jury.

### Les trois briques de SIO-UV, évaluées pour mon projet

| Brique SIO-UV | Intégrable sur Aracati ? | Verdict |
|---------------|--------------------------|---------|
| **MCFAR** (débruitage multi-échelle) | **Oui** (remplace le CFAR mono-échelle de `feature_extraction.py`) | **À tester — la seule brique vraiment portable** |
| Conversion 2D→3D par stacking vertical | Oui mais inutile (Aracati 2D) | Plan B léger pour HoloOcean ; **fausse 3D** (hauteur supposée, pas la courbure réelle d'un barrage) |
| Odométrie 3D-LOAM (T_h + T_c) | Non sans IMU | **Redondant avec DISO** + dépend de l'IMU |

### Blocage majeur : pas d'IMU sur Aracati

SIO-UV repose **entièrement** sur l'IMU (init de pré-intégration + correction de biais en boucle
fermée). **Aracati n'a pas d'IMU.** On ne peut donc pas reproduire SIO-UV tel quel sur le dataset
actif. La seule brique portable aujourd'hui est **MCFAR**.

### SIO-UV vs Sonar Context : lequel choisir ?

**Ils ne sont PAS en compétition — ils agissent à des endroits différents de la pipeline.**

| | Sonar Context (Kim, mon plan actuel) | SIO-UV (ce papier) |
|---|---|---|
| Cible dans la pipeline | **Détection de loop closure** (remplace la détection NSSM géométrique) | **Odométrie + débruitage** (remplace ICP front-end + CFAR) |
| Apporte | Information *nouvelle* (fermetures de boucle par apparence) | Meilleure odométrie *locale* |
| Besoin IMU | Non | **Oui (bloquant sur Aracati)** |
| Effort d'intégration | Moyen (étapes 2-5 entamées) | MCFAR faible, le reste lourd/impossible |

**Raisonnement décisif** (cf. `FABLE.md §2`) : sans loop closure valide, **rien** ne peut faire
passer Bruce+DISO sous DISO standalone, car l'odométrie seule ne crée pas d'information nouvelle.
SIO-UV améliore l'**odométrie** → même intégré parfaitement, on resterait plafonné par l'absence
de fermetures de boucle. **Sonar Context reste donc la priorité n°1** : c'est le seul des deux
qui attaque le verrou réel (le loop closure).

### Recommandation

1. **Garder Sonar Context comme axe principal** (loop closure = le levier).
2. **Piocher UNIQUEMENT MCFAR dans SIO-UV**, comme amélioration *orthogonale* du front-end :
   3 tailles de fenêtre CFAR + seuil minimum, en remplacement du CFAR mono-échelle de
   `feature_extraction.py`. Bénéficie à DISO *et* à Sonar Context (features plus propres).
   Testable en une demi-journée, un papier de plus à citer.
3. **Citer SIO-UV** comme preuve externe que l'ICP front-end de Bruce est défaillant.
4. **Ignorer** les briques 2D→3D-stacking (fausse 3D) et 3D-LOAM (redondant avec DISO, dépend
   de l'IMU).

---

## Glossaire

| Terme | Définition |
|-------|-----------|
| **FLS** | Forward-Looking Sonar — sonar imageur multifaisceaux frontal |
| **MCFAR** | Multiscale CFAR — débruitage CFAR à plusieurs échelles, seuil = min des seuils |
| **CFAR / SOCA-CFAR** | Constant False Alarm Rate — détection à seuil adaptatif (SOCA = celui de Bruce) |
| **Pré-intégration IMU** | Accumulation des mesures IMU entre deux keyframes pour éviter de réintégrer à chaque optimisation (Forster et al.) |
| **Reverse regression mapping** | Conversion des images sonar 2D en nuage 3D par stacking vertical (élévation supposée) |
| **LOAM / corner-surface features** | Extraction de features par courbure (arêtes vs surfaces), façon LiDAR Odometry |
| **PL-ICP** | Point-to-Line ICP — variante d'ICP adaptée aux structures linéaires |
| **iSAM2** | Incremental Smoothing and Mapping — solveur incrémental de factor graph (utilisé aussi par Bruce) |
| **ASFM** | Acoustic Structure from Motion — méthode comparative (features A-KAZE) |
| **APE / RMSE** | Absolute Pose Error / Root Mean Square Error — métriques de précision de trajectoire |

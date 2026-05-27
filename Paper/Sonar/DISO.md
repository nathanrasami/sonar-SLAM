# Résumé complet — DISO: Direct Imaging Sonar Odometry

**Titre :** *DISO: Direct Imaging Sonar Odometry*
**Auteurs :** Shida Xu, Kaicheng Zhang, Ziyang Hong, Yuanchang Liu, Sen Wang
**Institutions :** Imperial College London, Heriot-Watt University, University College London
**Publié :** IEEE ICRA 2024, Yokohama, Japon
**Code source :** https://github.com/SenseRoboticsLab/DISO

---

## Abstract

DISO est un système d'odométrie sonar qui estime la transformation spatiale relative entre deux images sonar consécutives **sans extraire de features géométriques**. Au lieu de ça, il minimise l'erreur d'intensité acoustique des points à fort gradient. Le système intègre aussi une optimisation multi-frames (fenêtre glissante), une stratégie d'association de données, et un rejet d'outliers basé sur l'intensité.

Testé en simulation et sur le dataset réel **Aracati2017** (BlueView P900-130), DISO surpasse les méthodes basées ICP (dont BlueROV SLAM) sur toutes les séquences.

---

## I. Introduction

### Contexte

Le SLAM sous-marin avec sonar imageur est difficile car :
- Le sonar produit des images **basse résolution**, **bruitées**, **1 canal** (intensité seulement)
- Le signal-to-noise ratio est faible → features géométriques peu fiables
- Les méthodes classiques (ORB, SIFT) sous-performent sur les images sonar

### Approches existantes

**Feature-based :**
- Détectent des keypoints dans l'image sonar → descripteurs → matching
- Problème : faible SNR → matching instable

**ICP-based :**
- Convertissent l'image sonar en nuage de points → ICP pour l'alignement
- Problème : **très sensible aux conditions initiales** → minimum local fréquent
- La qualité du nuage de points dépend de l'extraction → bruit sonar amplifié

### Idée centrale de DISO

Trois motivations pour une approche directe sur l'intensité :
1. L'intensité acoustique est une fonction des propriétés physiques du landmark (coefficient de réflexion, orientation normale, distance) → **stable à court terme**
2. Utiliser l'intensité globale + le gradient est plus robuste que matcher des pixels individuels
3. Pas besoin de calculer des descripteurs → moins coûteux en calcul

---

## II. Méthodologie

### A. Modèle du sonar imageur et coordonnées

Le sonar émet des impulsions acoustiques et reçoit les échos. Les données sont encodées en image polaire (axes : azimuth θ et range r).

Pour un point $_{S}\mathbf{p} = [r, \theta]^T$ en polaire, sa position cartésienne est :

$$_{C}\mathbf{p} = [x, y]^T = [r\cos\theta, r\sin\theta]^T$$

La transformation vers les pixels de l'image cartésienne (largeur w, hauteur h) est une **similarité 2D** $\mathbf{S}_{IC}$ :

$$_{I}\mathbf{p} = \mathbf{S}_{IC} \cdot _{C}\mathbf{p} = \begin{bmatrix} s\cos\omega & -s\sin\omega & t_x \\ s\sin\omega & s\cos\omega & t_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}$$

où $s = h/r_{max}$ (échelle), $t_x = w/2$, $t_y = h/2$.

> **Note importante :** L'élévation (angle vertical φ) est perdue dans l'image sonar → z est approximé à 0.

---

### B. Optimisation directe de pose sonar

#### 1. Résidu d'intensité

Étant donné N keypoints $\mathcal{K} = \{_{S}\mathbf{p}^i\}_{i \in \mathcal{N}}$ sur l'image sonar courante, on cherche la transformation $\mathbf{T}_{S_c, S_r}$ entre le frame de référence $S_r$ et le frame courant $S_c$ qui minimise :

$$E_I = \sum_{_I\mathbf{p}^i \in \mathcal{K}} \|I_c(_{I}\mathbf{p}^i) - I_r(_{I}\mathbf{p}^i)\|^2$$

- $I_r$ : intensité dans l'image de référence
- $I_c$ : intensité dans l'image courante
- $_{I}\mathbf{p}^i$ : pixel dans l'image courante correspondant au keypoint $_{S}\mathbf{p}^i$

**Différence clé avec ICP :** on minimise l'erreur d'*intensité*, pas la distance géométrique entre points.

#### 2. Jacobien (pour la descente de gradient)

Le résidu $\mathbf{r}_I = I_c(_{I}\mathbf{p}) - I_r(_{I}\mathbf{p})$ est linéarisé par rapport à la rotation $\mathbf{R}_{S_c S_r}$ et à la translation $_{S_c}\mathbf{p}_{S_c S_r}$ :

$$\frac{\partial r_I}{\partial \mathbf{R}_{S_c S_r}} = -\frac{\partial I_c(_{I}\mathbf{p})}{\partial _{I}\mathbf{p}} \cdot s \cdot \mathbf{R}_{IC} \begin{bmatrix}1&0\\0&1\\0&0\end{bmatrix} [\mathbf{R}_{S_c S_r} \cdot _{S_r}\mathbf{p}^i]^\wedge$$

$$\frac{\partial r_I}{\partial _{S_c}\mathbf{p}_{S_c S_r}} = \frac{\partial I_c(_{I}\mathbf{p})}{\partial _{I}\mathbf{p}} \cdot \mathbf{R}_{IC} \begin{bmatrix}1&0\\0&1\\0&0\end{bmatrix}$$

où $\frac{\partial I_c}{\partial _{I}\mathbf{p}}$ est le **gradient d'intensité** (calculé par Sobel), $\mathbf{R}_{IC}$ est la rotation de $\mathbf{S}_{IC}$, et $\wedge$ désigne la matrice antisymétrique (skew-matrix).

---

### C. Optimisation en fenêtre (Window Optimization)

#### Pourquoi une fenêtre ?

Le Frame-to-Frame seul accumule du drift rapidement. La fenêtre glissante optimise conjointement les poses et les landmarks sur plusieurs frames passées.

#### Erreur de reprojection (au lieu de l'intensité)

Dans la fenêtre, on minimise l'**erreur de reprojection** (pas l'intensité) pour deux raisons :
1. L'intensité peut changer entre frames distantes (atténuation acoustique variable)
2. L'erreur de reprojection ne nécessite pas de ré-interpoler l'image → plus rapide

$$E_P = \sum_{i \in \mathcal{N}, j \in \mathcal{M}} \|\mathbf{S}_{IC} \Pi(\mathbf{T}_{S_0 S_j}^{-1} \cdot _{S_0}\mathbf{l}^i) - _{I}\mathbf{p}^i\|^2$$

où $\mathcal{L} = \{_{S_0}\mathbf{l}^i\}$ sont les landmarks 3D dans le frame initial $S_0$, $\mathcal{M}$ est l'ensemble des frames dans la fenêtre.

---

### D. Implémentation du système

#### Vue d'ensemble (Fig. 3 du papier)

```
Image polaire → Image cartésienne → Gradient Sobel → Sélection keypoints
                                                            ↓
DVL + IMU → Filtre de Kalman étendu → Pose odom → Frame-to-Frame Tracking
                                                            ↓
                                               Frame-to-Window Tracking
                                                            ↓
                                            Sélection keyframe ?
                                          oui ↓           non → continuer
                                    Window Optimization (back-end)
```

#### 1. Sélection des keypoints

- Calcul du gradient Sobel sur l'image cartésienne
- Points dont le gradient dépasse un seuil → candidats
- Filtre par grille : dans chaque cellule, garder le point avec le gradient le plus élevé → distribution uniforme

#### 2. Frame-to-Frame Tracking

- Estimée initiale : pose fournie par l'odométrie inertielle (DVL + IMU via filtre de Kalman étendu)
- Optimisation directe par descente de gradient (Eq. 2) sur l'erreur d'intensité
- Stratégie multi-échelle (pyramide) pour robustesse aux grands mouvements : grossier → fin

#### 3. Frame-to-Window Tracking

- Utilise le résultat Frame-to-Frame comme initialisation
- Optimise la pose courante par rapport aux keyframes de la fenêtre active (Eq. 5)
- Rejette les landmarks outliers (erreur d'intensité > seuil $\sigma_{th}$)

#### 4. Sélection de keyframe

Une keyframe est ajoutée si :
- L'intervalle temporel depuis la dernière keyframe dépasse un seuil
- **OU** le nombre d'inliers tombe sous un seuil (image de qualité insuffisante)

#### 5. Association de données

Quand une nouvelle keyframe est créée, ses points sont associés aux landmarks existants dans la fenêtre active :
- Utilise les poses Frame-to-Window comme initialisation
- Optimisation directe entre la nouvelle keyframe et toutes les keyframes de la fenêtre
- Points avec erreur d'intensité < $\sigma_{th}$ → association établie → utilisés dans la Window Optimization

#### 6. Window Optimization (back-end)

- Minimise l'erreur de reprojection (Eq. 5) conjointement sur poses et landmarks
- Peut intégrer un résidu d'odométrie (Eq. 6-7) si DVL+IMU disponible
- Architecture multi-threadée (front-end et back-end en parallèle)

---

## III. Évaluation expérimentale

### A. Simulation

**Environnement :** DAVE (simulateur sous-marin open-source), structures 3D sous-marines, sonar simulé (BlueView M900), DVL, IMU intégrés dans un RexROV.

**3 séquences** testées.

**Résultats (Table I) :**

| Méthode | Translation error (%) | Rotation error (°/m) |
|---------|----------------------|----------------------|
| **DISO** | **3.40 / 4.29 / 4.18** | **0.276 / 0.41 / 0.488** |
| BlueROV SLAM (ICP) | 10.00 / 12.18 / 9.37 | 0.44 / 1.29 / 0.89 |
| Odométrie | 6.15 / 14.48 / 8.47 | 0.43 / 1.29 / 0.68 |

**DISO est 2-3× meilleur que ICP en simulation.**

---

### B. Dataset réel — Aracati2017

**Robot :** LBV 300-5 ROV avec BlueView P900-130 (130° FOV, 50m range), boussole magnétique, DGPS (ground truth).

**Odométrie :** calculée depuis la vitesse et la boussole magnétique.

**Spécificité :** Aracati2017 fournit uniquement des images cartésiennes (pas polaires) → DISO adapte la similarité $\mathbf{S}_{IC}$.

**3 séquences** de ~15 min chacune (total 44 min).

**Résultats (Table II) :**

| Méthode | Translation error (%) | Rotation error (°/m) |
|---------|----------------------|----------------------|
| **DISO** | **5.91 / 9.08 / 7.28** → **Overall : 8.69%** | **0.17 / 0.19 / 0.16** → **0.25°/m** |
| BlueROV SLAM (ICP) | 11.64 / 10.77 / 19.37 → 16.25% | 0.19 / 0.46 / 0.22 → 0.32°/m |
| Odométrie | 17.97 / 16.63 / 13.83 → 17.69% | NaN |

**DISO est ~2× meilleur que ICP et ~2× meilleur que l'odométrie seule sur Aracati2017.**

> **Note :** BlueROV SLAM nécessite des messages OculusPing — les auteurs ont converti les images cartésiennes en nuages de points pour le faire tourner, ce qui introduit des erreurs supplémentaires.

---

## IV. Discussions

### Pourquoi DISO surpasse ICP ?

**1. Sensibilité aux conditions initiales :**
- ICP converge vers des minima locaux si l'initialisation est mauvaise
- Sur sonar : beaucoup de points → initialisation souvent insuffisante → minimum local fréquent
- DISO utilise le gradient d'intensité + optimisation coarse-to-fine → plus robuste

**2. Robustesse aux outliers :**
- ICP est sensible aux outliers (points mal extraits du sonar)
- DISO utilise un schéma de sélection d'inliers basé sur l'intensité → rejette les outliers naturellement

### Limitations

- **Pas de loop closure** — DISO est un système d'odométrie pure, pas de SLAM global
- **Hypothèse 2D planaire** (z=0) — ne gère pas les changements de profondeur
- Extension 3D et loop closure = perspectives futures

---

## V. Conclusion

DISO est une méthode directe d'odométrie sonar qui :
- Évite l'extraction de features géométriques (sensible au bruit sonar)
- Minimise directement l'erreur d'intensité acoustique
- Intègre une fenêtre d'optimisation multi-frames
- Surpasse ICP sur Aracati2017 : **8.69% vs 16.25% erreur de translation**

**Code open source :** https://github.com/SenseRoboticsLab/DISO

---

## Glossaire

| Terme | Définition |
|-------|-----------|
| **Odométrie directe** | Estimation de pose sans extraction de features — minimise directement une erreur sur les pixels |
| **Gradient d'intensité** | Variation locale de l'intensité sonar (calculée par Sobel) — indique les zones à fort contraste |
| **Keypoint** | Point à fort gradient sélectionné comme point de suivi |
| **Frame-to-Frame** | Estimation de la transformation entre deux images consécutives |
| **Frame-to-Window** | Estimation de la transformation entre une image et une fenêtre de keyframes passées |
| **Window Optimization** | Optimisation conjointe des poses et landmarks sur une fenêtre glissante (back-end) |
| **Erreur de reprojection** | Distance entre un landmark projeté et son observation dans l'image |
| **BlueROV SLAM** | Implémentation open-source de SLAM sonar basée ICP — utilisée comme référence comparative |
| **Aracati2017** | Dataset réel BlueView P900-130, marina de Rio Grande (Brésil) — https://github.com/matheusbg8/aracati2017 |

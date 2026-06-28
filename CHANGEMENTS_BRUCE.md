# Résumé des modifications du code Bruce-SLAM (Aracati2017)

Point de départ : **Bruce-SLAM original** (jake3991/sonar-SLAM), conçu pour un robot avec
**IMU + DVL** et un sonar **OculusPing polaire**. Aracati2017 n'a **ni IMU ni DVL** et publie
des images sonar **cartésiennes** → il a fallu modifier le code (pas seulement des launch files).

Cap des résultats (ATE Umeyama, GT-free) : Bruce brut ~12.8 m → avec nos changements **~1.4 m**.

---

## 1. Pont Aracati (rendre Bruce capable de tourner sur ce dataset)

| Fichier | Changement | Pourquoi |
|---|---|---|
| `cmd_vel_odom.py` (**nouveau**) | intègre `/cmd_vel` (modèle unicycle) → `nav_msgs/Odometry` | Aracati n'a pas d'IMU/DVL : on fabrique l'odométrie attendue par Bruce à partir des vitesses commandées (le `wz` vient du compas du véhicule) |
| `feature_extraction.py` | **mode cartésien** : lit `/son/compressed`, CFAR sur image cartésienne métrique | Aracati publie des images cartésiennes, pas du `OculusPing` polaire |

→ Sans ça, Bruce ne démarre pas sur Aracati. C'est l'adaptation **minimale** au capteur.

---

## 2. USBL — l'ajout principal (ce que la tutrice demande)

**Idée :** Aracati fournit `/usbl_point` = position **acoustique absolue** (≈1.4 m de bruit,
**indépendante de la GT**). C'est le seul ancrage absolu GT-free du dataset. On l'a intégré à
**deux endroits**, ce qui a demandé du code :

### 2a. USBL back-end — facteur de position dans le factor-graph (le plus important)

| Fichier | Changement |
|---|---|
| `slam.py` (resp. `slam_ros.py`) | méthode **`add_usbl()`** : ajoute un **facteur prior de position** `PriorFactorPose2` sur chaque keyframe, à la position du fix USBL le plus proche en temps |

- Modèle de bruit **robuste (Cauchy)**, σ ≈ 1.4 m, qui ne contraint **que x,y** (σ_θ énorme → le cap reste libre).
- gtsam (iSAM2) **recolle** alors la trajectoire sur les ancres USBL en moyennant leur bruit sur tout le graphe et en rejetant les outliers acoustiques.
- Association fix↔keyframe par fenêtre temporelle ; rejet d'outliers (sauts ~73 m) par gate de vitesse.
- **Effet : borne la dérive de l'odométrie → ATE ~1.4 m** (vs ~12.8 m sans).

### 2b. USBL front-end — seed + filtre complémentaire (dans l'odométrie)

| Fichier | Changement |
|---|---|
| `cmd_vel_odom.py` | **seed GT-free** : position initiale = 1er fix USBL, cap initial = *course-over-ground* (atan2 du déplacement USBL) ; **filtre complémentaire** optionnel : correction de position low-gain vers chaque fix |

- Le seed **aligne le repère** de l'odométrie cmd_vel sur le repère USBL/monde (sinon les fixes seraient dans un repère tourné). **100% GT-free** (l'USBL est acoustique, pas de `/pose_gt`).
- ⚠️ **Piège trouvé** : USBL front-end **ET** back-end en même temps = **double ancrage** → l'odométrie snappe sur chaque fix bruité → zigzag (ATE 1.45 → 4.66 m). **Règle : un seul ancrage continu** (le back-end), le front-end ne sert qu'au seed.

---

## 3. Cap / point cloud — correction en post-traitement (rendu)

Constat : le cloud "tourbillonne" parce que le `theta` optimisé par iSAM2 est tiré vers la
**direction de trajectoire**, alors que le sonar a besoin du **cap réel (compas)**.

| Fichier | Changement |
|---|---|
| `slam_ros.py` | option **`cloud/use_compass_cap`** : rend chaque scan avec le cap **compas** (`-dr_theta + offset`) au lieu du `theta` iSAM2 ; position iSAM2 conservée → **trajectoire inchangée** |
| `c2_compass_offset.py`, `filter_cloud.py` | post-traitement : calibration de l'offset cap + filtre structure (intensité + persistance) → cloud dé-swirlé |

- Découple **cap-de-rendu** et **cap-de-trajectoire** : n'affecte que l'affichage du cloud (RViz + CSV), pas le SLAM.
- **Limite établie** : retire le tourbillon (baseline du quai nette) mais pas les "doigts" du quai → ceux-ci exigent un cap **local** par scan que seul le scan-matching GT donne.

---

## 4. Loop closure + outillage

| Fichier | Changement |
|---|---|
| `sonar_context.py` (**nouveau**), `feature_extraction.py` | descripteur **SONAR Context** (place recognition) pour des loop closures plus robustes que features+ICP |
| `slam_ros.py` | **export CSV** (trajectory / pointcloud / groundtruth) à l'arrêt → évaluation ATE |
| `plot_heading_comparison.py`, `plot_trajectories.py`, `traj_eval.py` | analyse : **courbe d'erreur de cap**, trajectoires, ATE Umeyama |

---

## En une phrase (pour la tutrice)

> Pour faire tourner Bruce sur Aracati (sans IMU/DVL, sonar cartésien) j'ai écrit un pont
> d'odométrie et un mode sonar cartésien ; puis j'ai **intégré l'USBL** comme **facteur de
> position absolue robuste dans le factor-graph** (`add_usbl`) + un seed/filtre côté odométrie,
> ce qui fait passer l'ATE de ~12.8 m à ~1.4 m. La correction de cap (compas) est un
> post-traitement qui ne touche que l'affichage du point cloud.

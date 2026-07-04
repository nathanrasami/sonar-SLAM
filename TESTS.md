# Tests & Expériences — Bruce-SLAM

Fichier de suivi des modifications de paramètres et observations.
Référence : configs `bruce_slam/config/feature.yaml`, `icp.yaml`, `slam.yaml`

---

## Baseline

Paramètres par défaut (`feature.yaml`) :
- `Pfa: 0.1` — taux de fausse alarme CFAR (filtre relatif au voisinage local)
- `threshold: 65` — seuil minimum d'intensité brute sonar (0-255), filtre absolu post-CFAR
- `resolution: 0.5` — taille du voxel downsampling (mètres), 1 point conservé par cellule
- `radius: 1.0` / `min_points: 5` — outlier rejection : point gardé si ≥5 voisins dans 1m

**Résultat :** trajectoire cohérente, loop closures visibles (traits rouges), nuage de points dense.

![Baseline](TESTS_image/base.png)
---

## Test 1 — Pfa réduit (0.01)

| Paramètre | Valeur baseline | Valeur testée |
|-----------|----------------|---------------|
| Pfa | 0.1 | 0.01 |

**Observation :**
- Moins de points détectés (nuage orange moins dense)
- Trajectoire globale identique
- Légères différences locales → ICP a moins de correspondances
- Loop closures inchangés globalement

**Conclusion :** Pfa impacte la précision locale mais iSAM2 + loop closure corrigent globalement.

---

## Test 2 — Pfa très réduit (0.001)

| Paramètre | Valeur baseline | Valeur testée |
|-----------|----------------|---------------|
| Pfa | 0.1 | 0.01 |

![Pfa 0.01](TESTS_image/pfa0_001.png)

**Observation :**
- Beaucoup moins de points détectés
- Trajectoire globale inchangée malgré la très faible densité
- Loop closures toujours présents

**Conclusion :** Même avec très peu de points, ICP + iSAM2 maintiennent une trajectoire cohérente. Le système est robuste à la réduction de features.

---

## Test 3 — Threshold bas (30)

| Paramètre | Valeur baseline | Valeur testée |
|-----------|----------------|---------------|
| threshold | 65 | 30 |

![Threshold 30](TESTS_image/threshold30.png)

**Observation :**
- Beaucoup de points (nuage dense, bleu/vert/orange/jaune) → beaucoup de faux positifs acceptés
- Moins de loop closures que la baseline
- Début de drift visible, zigzags sur la trajectoire

**Conclusion :** Un threshold trop bas inonde ICP de faux positifs → recalage bruité → drift local. Paradoxalement moins de loop closures car les scans bruités se ressemblent moins.

---

## Test 4 — Threshold élevé (90)

| Paramètre | Valeur baseline | Valeur testée |
|-----------|----------------|---------------|
| threshold | 65 | 90 |

![Threshold 90](TESTS_image/threshold90.png)

**Observation :**
- Moins de points détectés
- Moins de loop closures
- Trajectoire propre, similaire à la baseline

**Conclusion :** threshold (filtre absolu) a plus d'impact que Pfa (filtre relatif). À 90, on garde seulement les réflexions fortes → ICP propre → bonne trajectoire mais moins de loop closures.

---

## Test 5 — Resolution fine (0.1)

| Paramètre | Valeur baseline | Valeur testée |
|-----------|----------------|---------------|
| resolution | 0.5 | 0.1 |

**Définition :** Plus la valeur est basse, plus le nuage de points est dense (voxels plus petits).

![Resolution 0.1](TESTS_image/resolution01.png)

**Observation :**
- Beaucoup plus de points dans le nuage
- Trajectoire plus géométrique (angles nets, formes précises)
- Beaucoup de loop closures, y compris entre le début et la fin du parcours

**Conclusion :** Plus de points → ICP plus précis → meilleure géométrie et plus de loop closures. Coût : temps de calcul plus élevé.

---

## Test 6 — Resolution grossière (2.0)

| Paramètre | Valeur baseline | Valeur testée |
|-----------|----------------|---------------|
| resolution | 0.5 | 2.0 |

**Définition :** Plus la valeur est haute, moins il y a de points conservés (voxels plus grands).

![Resolution 2.0](TESTS_image/resolution2.png)

**Observation :**
- Presque aucun point dans le nuage
- Trajectoire pourtant cohérente
- Loop closures uniquement au tout début du parcours

**Conclusion :** ICP fonctionne encore avec très peu de points grâce au dead reckoning (DVL/IMU), mais perd la capacité de détecter des loop closures tardifs.

---

## Test 7 — Radius bas (0.3)

| Paramètre | Valeur baseline | Valeur testée |
|-----------|----------------|---------------|
| radius | 1.0 | 0.3 |

**Définition :** radius = rayon de recherche pour l'outlier rejection. Un point est supprimé s'il n'a pas assez de voisins dans ce rayon. Valeur basse → zone de recherche réduite → plus de points supprimés.

![Radius 0.3](TESTS_image/radius0_3.png)

**Observation :**
- Peu de points (beaucoup supprimés car zone trop petite)
- Trajectoire décalée par rapport à la baseline
- Pas de loop closure

**Conclusion :** Trop peu de points → ICP perd en précision → drift. Sans loop closure, pas de correction globale.

---

## Test 8 — Radius élevé (3.0)

| Paramètre | Valeur baseline | Valeur testée |
|-----------|----------------|---------------|
| radius | 1.0 | 3.0 |

**Définition :** Valeur haute → zone de recherche large → points isolés conservés, moins de suppression.

![Radius 3](TESTS_image/radius3.png)

**Observation :**
- Nombre de points normal
- Trajectoire décalée par rapport à la baseline
- Loop closure visibles et actifs (correction visible en temps réel)
- Nombre de loop closures normal

**Conclusion :** Avec un radius trop grand, des faux points (outliers) sont conservés → ICP bruité → trajectoire décalée. Les loop closures compensent partiellement mais ne suffisent pas à corriger complètement.

---

## Test 9 — ICP maxDist outlier bas (1.0)

| Paramètre | Fichier | Valeur baseline | Valeur testée |
|-----------|---------|----------------|---------------|
| MaxDistOutlierFilter maxDist | icp.yaml | 3.0 | 1.0 |

**Définition :** distance max acceptée entre deux points associés par ICP. Au-delà → paire rejetée comme outlier.

![Outlier 1.0](TESTS_image/outlier1.png)

**Observation :**
- Nombre de points similaire à la baseline
- Plus de loop closures détectés
- Trajectoire similaire à la baseline

**Conclusion :** Seuil plus strict → ICP garde seulement les correspondances très proches → recalage plus précis → meilleure détection de loop closures.

---

## Test 10 — ICP maxDist outlier élevé (6.0)

| Paramètre | Fichier | Valeur baseline | Valeur testée |
|-----------|---------|----------------|---------------|
| MaxDistOutlierFilter maxDist | icp.yaml | 3.0 | 6.0 |

![Outlier 6.0](TESTS_image/outlier6.png)

**Observation :**
- Nombre de points similaire à la baseline
- Trajectoire similaire à la baseline
- Résultat global peu différent de la baseline

**Conclusion :** À 6m le filtre est permissif mais ICP converge quand même grâce au TrimmedDist qui filtre les 20% pires correspondances.

---

## Test 11 — ICP ratio TrimmedDist bas (0.5)

| Paramètre | Fichier | Valeur baseline | Valeur testée |
|-----------|---------|----------------|---------------|
| TrimmedDistOutlierFilter ratio | icp.yaml | 0.8 | 0.5 |

**Définition :** garde seulement les X% meilleures correspondances ICP. 0.5 = garde 50%.

![Ratio 0.5](TESTS_image/ratio05.png)

**Observation :** Peu de différences par rapport à la baseline.

**Conclusion :** ICP est robuste à ce paramètre — les 50% meilleures correspondances suffisent à converger correctement.

---

## Test 12 — ICP ratio TrimmedDist élevé (0.95)

| Paramètre | Fichier | Valeur baseline | Valeur testée |
|-----------|---------|----------------|---------------|
| TrimmedDistOutlierFilter ratio | icp.yaml | 0.8 | 0.95 |

![Ratio 0.95](TESTS_image/ratio0_95.png)

**Observation :** Peu de différences par rapport à la baseline.

**Conclusion :** Garder 95% des correspondances n'introduit pas assez de bruit supplémentaire pour dégrader ICP — MaxDistOutlierFilter filtre déjà les pires cas en amont.

---

## À tester

- [ ] Paramètres loop closure / iSAM2 (`slam.yaml`)

---

## Dataset Aracati2017 — Test SSM + NSSM paramètres défaut (27 mai 2026)

### Paramètres actifs

| Fichier | Paramètre | Valeur |
|---------|-----------|--------|
| `feature_aracati.yaml` | threshold | 30 |
| `feature_aracati.yaml` | resolution | 0.5 |
| `feature_aracati.yaml` | skip | 5 |
| `feature_aracati.yaml` | Pfa | 0.1 |
| `feature_aracati.yaml` | cartesian_mode | True (BlueView P900-130) |
| `slam_aracati.yaml` | SSM enable | **True** |
| `slam_aracati.yaml` | SSM max_translation | 3.0 m |
| `slam_aracati.yaml` | SSM max_rotation | 30° |
| `slam_aracati.yaml` | NSSM enable | **True** |
| `slam_aracati.yaml` | NSSM max_translation | 10.0 m |
| `slam_aracati.yaml` | NSSM max_rotation | 60° |
| `icp.yaml` | errorMinimizer | PointToPointErrorMinimizer |
| `icp.yaml` | MaxDistOutlierFilter maxDist | 3.0 |
| `icp.yaml` | TrimmedDistOutlierFilter ratio | 0.8 |

### Résultats

![Trajectoire](TESTS_image/aracati_ssm_trajectory.png)

![Carte point cloud](TESTS_image/aracati_ssm_pointcloud.png)

### Observations

- **Odométrie (bleu pointillé)** : bonne forme générale, proche du GT, drift attendu
- **Trajectoire SLAM (noir)** : dérive largement, forme incohérente avec GT et odométrie
- **Carte point cloud** : lignes droites visibles dans chaque scan ✅ mais orientées dans toutes les directions → ICP ne recale pas les scans entre eux
- SSM introduit des erreurs au lieu de les corriger
- NSSM produit de faux loop closures (warning sklearn "covariance not full rank")

### Conclusion

**ICP (SSM+NSSM) dégrade la trajectoire par rapport à l'odométrie seule** sur Aracati2017. Causes :
- Image sonar BlueView basse résolution + bruit élevé → mauvaise extraction de points
- ICP sensible aux conditions initiales → converge vers minimum local
- Sans DVL/IMU (odométrie depuis `/cmd_vel`), l'initialisation ICP est moins précise

**Confirmé par le papier DISO (Xu et al., ICRA 2024)** : BlueROV SLAM (= ICP) donne 16.25% d'erreur de translation sur Aracati2017 vs 8.69% pour DISO. La méthode directe par intensité sonar est nécessaire pour ce dataset.

---

## Indications doctorante — Aracati2017 (25 mai 2026)

Objectif : la trajectoire Bruce-SLAM doit se situer **entre odométrie et GT** (pas identique à l'odo, pas aussi bonne que le GT).

Problème actuel : avec SSM+NSSM activés, la trajectoire Bruce-SLAM a une forme complètement différente de GT et odométrie → quelque chose ne va pas.

### Plan de débogage (dans l'ordre)

- [ ] **1. Désactiver NSSM, garder SSM uniquement** — déboguer SSM seul d'abord
- [ ] **2. Ajuster le seuil de sélection SSM vs odométrie** — Bruce-SLAM choisit entre facteur odométrie et facteur SSM selon la qualité de l'ICP ; essayer d'ajuster ce seuil
- [ ] **3. Ajuster les paramètres CFAR/débruitage** (`feature_aracati.yaml`) — la conversion image sonar → points peut introduire du bruit
- [ ] **4. Exporter la carte point cloud** (points cumulés comme dans RViz, hors trajectoire) dans `results/pointcloud_map.csv` et générer une image 2D
  - La carte doit montrer des **lignes droites** (murs du port) et non des arcs
  - Si la carte montre des arcs → problème dans la conversion pixels→mètres de l'image BlueView
- [ ] **5. Une fois SSM OK, réactiver NSSM**

---

## Dataset Aracati2017 — Tests d'intégration

Dataset : BlueView P900-130, FOV 130°, max range 50m, marina Yacht Club Rio Grande (Brésil).
Différence principale vs sample_data : odométrie par intégration de `/cmd_vel` (pas de DVL/IMU) → plus grande erreur de dead reckoning.

### Modifications apportées à bruce_slam

- `feature_extraction.py` : ajout du mode `cartesian_mode=True` pour accepter les images PNG cartésiennes du BlueView directement (sans `OculusPing`)
- `odom_bridge.py` : nœud de conversion `/odom_pose` (PoseStamped) → `nav_msgs/Odometry` sur `/bruce/slam/localization/odom`
- `launch/aracati.launch` : launch file combinant aracati2017 + bruce_slam
- `config/feature_aracati.yaml` : paramètres feature extraction pour le BlueView (threshold=65, cartesian_mode=True, FOV=130°, max_range=50m)
- `config/slam_aracati.yaml` : paramètres SLAM adaptés (odom_sigmas augmentés, min_pcm=4)

### Test 1 — Odométrie pure (SSM=False, NSSM=False)

**Observation :** Trajectoire cohérente, forme similaire au ground truth GPS (vert). Drift visible mais attendu (odométrie seule sans correction sonar).

**Conclusion :** Le bridge odométrie fonctionne correctement. La forme globale est cohérente avec la réalité.

### Test 2 — SSM activé (NSSM=False)

**Observation :** Trajectoire dégradée — SSM introduit des erreurs. ICP ne converge pas correctement sur ce dataset.

**Conclusion :** Les paramètres ICP calibrés pour l'Oculus M750d ne sont pas adaptés au BlueView P900-130. Le recalage local échoue et dégrade la trajectoire.

### Test 3 — SSM + NSSM activés

**Observation :** Faux loop closures nombreux (traits rouges partant tous vers le début). Trajectoire incohérente avec le ground truth.

**Conclusion :** NSSM détecte de fausses correspondances — les scans BlueView sont trop différents des scans Oculus pour que le NSSM fonctionne avec ces paramètres. `min_pcm=4` ne suffit pas à rejeter les faux positifs.

### Conclusion générale

Bruce-SLAM intégré avec Aracati2017 fonctionne en mode odométrie pure. SSM et NSSM nécessitent un recalibrage des paramètres ICP spécifique au sonar BlueView P900-130 pour fonctionner correctement sur ce dataset.

---

## Dataset Aracati2017 — DISO Standalone (2026-06-03)

Front-end DISO uniquement (pas de back-end iSAM2). Odométrie directe par intensité sonar (pas ICP).

### Configuration

| Composant | Valeur |
|-----------|--------|
| Nœud | `direct_sonar_odometry/aracati2017_node` |
| Config | `config/config_aracati2017.yaml` |
| Back-end | aucun (pas de loop closure, pas de iSAM2) |
| Topic pose | `/direct_sonar/pose` |

### Résultats

**Trajectoire DISO vs GT :**
![DISO Trajectoires](TESTS_image/diso_trajectories.png)

**Carte point cloud DISO :**
![DISO Point cloud](TESTS_image/diso_pointcloud.png)

### Observations

- Trajectoire DISO suit la forme générale du GT (GPS)
- Dérive progressive visible (pas de loop closure pour corriger)
- Point cloud cohérent — les structures du port sont reconnaissables

### ATE

**ATE = 6.5 m**

---

## Dataset Aracati2017 — DISO + Bruce_SLAM (iSAM2, SSM=off, NSSM=off) (2026-06-03)

DISO comme source d'odométrie sonar, back-end iSAM2 de Bruce_SLAM. SSM (ICP) désactivé car DISO le remplace. NSSM désactivé (faux loop closures).

### Configuration

| Composant | Valeur |
|-----------|--------|
| Front-end odométrie | DISO (`/direct_sonar/pose` → `odom_bridge` → `/bruce/slam/localization/odom`) |
| SSM | **False** (DISO remplace ICP) |
| NSSM | **False** (faux loop closures sur Aracati) |
| Back-end | iSAM2 |
| Covariance odom_bridge | 0.1 (position), 0.05 (rotation) |

### Résultats

![DISO + Bruce_SLAM trajectoire](TESTS_image/diso_bruce_slam_trajectory.png)

### Observations

- Sans loop closure, iSAM2 propage les poses DISO sans correction globale
- Résultat légèrement moins bon que DISO standalone (latence bridge + overhead iSAM2 sans boucle)
- Le gain réel de Bruce_SLAM viendra quand NSSM sera réactivé avec Sonar Context

### ATE

**ATE = 29.9 m**

### Conclusion

**Sans NSSM, DISO+Bruce_SLAM est nettement pire que DISO standalone (29.9 m vs 6.5 m).** iSAM2 sans loop closure ne fait que propager les erreurs avec overhead supplémentaire (latence bridge, désynchronisation temporelle possible). Le gain réel de Bruce_SLAM viendra quand NSSM sera réactivé avec Sonar Context pour corriger la dérive globale.

> ⚠️ Note rétrospective : les ATE 29.9 m / 6.5 m de cette section ont été calculés avec
> l'ancienne méthode d'alignement (flip Y manuel + association `linspace`), peu fiable.
> Voir la section suivante pour la méthode d'évaluation robuste (Umeyama + interpolation
> temporelle) et des valeurs correctes.

---

## Méthode d'évaluation ATE — passage à Umeyama (2026-06-04)

L'ancien alignement (flip Y hardcodé + rotation par centroïde + association des points par
`np.linspace`) était fragile : un simple décalage en X faussait l'ATE, et le flip Y devait
être géré à la main.

**Nouvelle méthode (standard TUM/evo)** implémentée dans `traj_eval.py` :
1. **Association temporelle** par interpolation sur la colonne `time` → compare chaque pose
   estimée à la pose GT au même instant (résout les tailles différentes : DISO ~14k poses,
   Bruce ~600 keyframes, GT ~14k).
2. **Alignement rigide Umeyama** (SVD) → trouve rotation + translation optimales et **absorbe
   automatiquement la réflexion Y** (det R = -1) → plus aucun flip manuel.
3. **Sans échelle** (s=1) → ATE métrique honnête en mètres.

Fait confirmé : `corr(diso_y, gt_y) = -0.95` → l'axe Y de DISO est l'opposé du GT (réflexion
physique du repère, pas un bug). Umeyama la gère seul.

`analyze_diso.py` et `analyze_drift.py` utilisent désormais ce module partagé.

---

## Run DISO + Bruce_SLAM — config affinée, ATE Umeyama (2026-06-04)

Run propre (aucun saut/trou temporel, contrairement aux runs précédents avec paramètres
NSSM trop conservateurs). Archivé dans `TESTS_image/run_diso_bruce_2026-06-04_ate5.4/`
(CSV + PNG).

### Configuration complète

| Fichier | Paramètre | Valeur |
|---------|-----------|--------|
| `slam_aracati.yaml` | keyframe_duration | 1.0 s |
| `slam_aracati.yaml` | **keyframe_translation** | **1.0 m** (était 3.0 → plus de keyframes) |
| `slam_aracati.yaml` | keyframe_rotation | deg(30) |
| `slam_aracati.yaml` | prior_sigmas | [0.1, 0.1, 0.01] |
| `slam_aracati.yaml` | odom_sigmas | [0.5, 0.5, 0.05] |
| `slam_aracati.yaml` | icp_odom_sigmas | [0.1, 0.1, 0.01] |
| `slam_aracati.yaml` | point_resolution | 0.5 |
| `slam_aracati.yaml` | **ssm enable** | **False** (DISO remplace ICP) |
| `slam_aracati.yaml` | **nssm enable** | **False** |
| `slam_aracati.yaml` | min_pcm | 4 |
| `slam_aracati.yaml` | pz_detection_rate | 0.3 |
| `feature_aracati.yaml` | CFAR Pfa | 0.1 |
| `feature_aracati.yaml` | CFAR alg | SOCA (Ntc=40, Ngc=10, rank=10) |
| `feature_aracati.yaml` | threshold | 30 |
| `feature_aracati.yaml` | resolution | 0.5 |
| `feature_aracati.yaml` | radius / min_points | 1.0 / 5 |
| `feature_aracati.yaml` | skip | 5 |
| `feature_aracati.yaml` | cartesian_mode | True (FOV 130°, max_range 50 m) |
| `odom_bridge.py` | covariance | 0.1 (position), 0.05 (rotation) |
| `config_aracati2017.yaml` (DISO) | Tbs | identité |
| `config_aracati2017.yaml` (DISO) | OdomTopic / SonarTopic | /pose_gt / /son |

### Résultats

![Trajectoires DISO + Bruce_SLAM](TESTS_image/run_diso_bruce_2026-06-04_ate5.4/trajectory_plot.png)

| Trajectoire | ATE (Umeyama, sans échelle) | N points |
|-------------|-----------------------------|----------|
| DISO standalone | **3.0 m** | ~14 400 |
| DISO + Bruce_SLAM (iSAM2, sans loop closure) | **5.4 m** | 619 keyframes |

### Observations

- Run propre : 619 keyframes, dt max 23 s, **aucun trou temporel** (les sauts des runs
  précédents venaient des paramètres NSSM conservateurs, maintenant remis à la config de base).
- Bruce suit visuellement bien la forme du GT.
- DISO standalone (3.0 m) reste meilleur que DISO+Bruce (5.4 m) car sans loop closure
  iSAM2 ne corrige pas — il propage les poses DISO avec l'overhead du bridge.

### Conclusion

Le pipeline tourne proprement et l'évaluation est maintenant fiable (Umeyama). Tant que NSSM
est désactivé, Bruce_SLAM n'apporte pas de gain sur DISO. Prochaine étape : intégrer Sonar
Context dans le NSSM pour activer le loop closure et faire descendre l'ATE Bruce sous celui
de DISO.

---

## Run DISO + Bruce_SLAM — NSSM ON (2026-06-04)

Test avec le loop closure NSSM réactivé. Archivé dans
`TESTS_image/run_diso_bruce_2026-06-04_nssm_on_ate6.2/`.

### Configuration (différences vs run NSSM off précédent)

| Fichier | Paramètre | Valeur |
|---------|-----------|--------|
| `slam_aracati.yaml` | **nssm enable** | **True** |
| `slam_aracati.yaml` | nssm min_st_sep | 8 |
| `slam_aracati.yaml` | nssm min_points | 100 |
| `slam_aracati.yaml` | nssm max_translation | 5.0 m |
| `slam_aracati.yaml` | nssm max_rotation | deg(30) |
| `slam_aracati.yaml` | nssm source_frames | 5 |
| `slam_aracati.yaml` | min_pcm | 6 |
| (ssm reste off, reste identique au run NSSM off) | | |

### Résultats

![Trajectoires NSSM ON](TESTS_image/run_diso_bruce_2026-06-04_nssm_on_ate6.2/trajectory_plot.png)

| Run | ATE Bruce | Loop closures (nssm_constraints) | N keyframes | Trous > 30s |
|-----|-----------|----------------------------------|-------------|-------------|
| NSSM **off** | 5.4 m | 0 | 619 | 0 |
| NSSM **on** | **6.2 m** | **0** | 569 | 2 |
| DISO standalone | 3.0 m | — | — | — |

### Observations

- **NSSM n'a détecté AUCUN loop closure** (0 constraints) malgré son activation.
- L'ATE est légèrement pire (6.2 m vs 5.4 m), pas meilleur. Causes probables :
  - 2 trous temporels (dt max 32.9 s) dans ce run alors que le run NSSM off n'en avait aucun.
  - Overhead du NSSM (recherche de candidats + ICP de vérification) sans aucun bénéfice
    puisque 0 boucle trouvée.
  - Variance run-to-run (timing temps réel différent).

### Conclusion

**Le NSSM natif de Bruce_SLAM est inopérant sur Aracati2017** : sa détection de boucle
(features + ICP) ne trouve aucune correspondance sur les scans BlueView, donc 0 correction
et seulement de l'overhead. C'est exactement le problème que **Sonar Context** (ICRA 2023)
résout — détection de lieu robuste sans ICP, testée sur Aracati2017. Confirme la pertinence
de l'intégrer dans le NSSM.

---

## Run DISO + Bruce_SLAM — NSSM ON, params relâchés (2026-06-04)

Re-run NSSM on avec des seuils **plus permissifs** que le run précédent, pour tenter de
déclencher des loop closures. Commit : `Run NSSM on, min_points 100 max_translation 8,
max_rotation 45, min_pcm 4`. Archivé dans
`TESTS_image/run_diso_bruce_2026-06-04_nssm_on_ate5.5/`.

### Configuration

| Fichier | Paramètre | Run précédent (6.2 m) | Ce run (5.5 m) |
|---------|-----------|-----------------------|----------------|
| `slam_aracati.yaml` | nssm min_points | 100 | 100 |
| `slam_aracati.yaml` | nssm max_translation | 5.0 m | **8.0 m** |
| `slam_aracati.yaml` | nssm max_rotation | deg(30) | **deg(45)** |
| `slam_aracati.yaml` | min_pcm | 6 | **4** |

Seuils relâchés (zone de recherche plus large, moins de votes PCM requis) = conditions
plus favorables à la détection de boucle.

### Résultats

![Trajectoires NSSM ON params relâchés](TESTS_image/run_diso_bruce_2026-06-04_nssm_on_ate5.5/trajectory_plot.png)

| Run NSSM on | max_trans / max_rot / min_pcm | ATE Bruce | Loop closures | N kf | Trous > 30s |
|-------------|-------------------------------|-----------|---------------|------|-------------|
| #1 (strict) | 5.0 / 30° / 6 | 6.2 m | 0 | 569 | 2 |
| #2 (relâché) | 8.0 / 45° / 4 | **5.5 m** | **0** | 579 | 1 |

### Observations

- **Même en relâchant les seuils, 0 loop closure** détecté → NSSM natif confirmé inopérant
  quels que soient les paramètres testés.
- ATE 5.5 m (relâché) vs 6.2 m (strict) : la légère amélioration vient surtout du nombre de
  trous (1 vs 2), pas du NSSM (toujours 0 boucle).
- Reste au-dessus de DISO standalone (3.0 m).

### Conclusion

Le NSSM natif ne trouve **aucune boucle sur Aracati2017, ni avec des seuils stricts ni
relâchés**. Le problème n'est pas le réglage mais la méthode de détection elle-même
(features + ICP, inadaptée au sonar BlueView). C'est exactement ce que **Sonar Context**
(ICRA 2023) corrige.

> ⚠️ **Correction (voir run suivant)** : cette conclusion était ERRONÉE. Le vrai problème
> n'était pas la méthode NSSM mais le paramètre `skip: 5` qui rendait 69 % des keyframes
> vides (pas de features → rien à matcher). Avec `skip: 1`, le NSSM trouve bien des boucles.

---

## Run DISO + Bruce_SLAM — skip=1, NSSM trouve des boucles ! (2026-06-05)

**Découverte majeure.** Le paramètre `skip: 5` de `feature_aracati.yaml` ne traitait qu'1 scan
sonar sur 5 → 69 % des keyframes étaient vides → le NSSM n'avait aucune feature à matcher → 0
boucle. En passant à **`skip: 1`**, tous les scans sont traités, les keyframes ne sont plus
vides, et le NSSM **détecte enfin des loop closures**.

### Configuration

| Fichier | Paramètre | Valeur |
|---------|-----------|--------|
| `feature_aracati.yaml` | **skip** | **1** (était 5) |
| `feature_aracati.yaml` | threshold | 30 |
| `slam_aracati.yaml` | ssm enable | False |
| `slam_aracati.yaml` | nssm enable | True |
| `slam_aracati.yaml` | nssm min_points | 30 |
| `slam_aracati.yaml` | nssm max_translation | 8.0 m |
| `slam_aracati.yaml` | nssm max_rotation | deg(45) |
| `slam_aracati.yaml` | nssm source_frames | 5 |

### Résultats

![Trajectoires skip=1 NSSM](TESTS_image/run_diso_bruce_2026-06-05_skip1_nssm8loops_ate11.3/trajectory_plot.png)

| Métrique | Runs précédents (skip=5) | Ce run (skip=1) |
|----------|--------------------------|-----------------|
| Keyframes vides | 69 % | **0 %** |
| **Loop closures NSSM** | **0** | **8** |
| N keyframes | ~570 | 551 |
| ATE Bruce | 5.4–6.2 m | 11.3 m |
| ATE DISO standalone | 3.0 m | 3.0 m |

### Observations

- **Le NSSM fonctionne** : 8 loop closures détectées (vs 0 avant). Le diagnostic « features+ICP
  inadaptés » était faux — il manquait juste des features (cause : `skip: 5`).
- **Comportement de la trajectoire** (qualitatif) :
  - **Départ erroné** : très mauvaise initialisation (Bruce part vers y ≈ -60, loin du GT).
  - **Milieu** : se recalibre progressivement grâce aux loop closures détectées.
  - **Fin** : trajectoire bien alignée sur le GT.
- L'**ATE global (11.3 m) est élevé à cause du mauvais départ**, pas du loop closure (qui
  améliore au contraire la fin). Le RMSE est dominé par l'erreur d'initialisation.
- 2 trous temporels subsistent (dt max 34.8 s).

### Conclusion

`skip: 1` débloque le NSSM natif (0 → 8 boucles). Le loop closure recale bien la trajectoire en
cours de route. **Deux problèmes restants** : (1) la mauvaise initialisation de départ qui plombe
l'ATE, (2) seulement 8 boucles, dont l'effet reste limité. C'est là que **Sonar Context**
apportera plus : une détection de boucle plus dense et robuste → recalage plus précoce et plus
fort. Prochaine étape : investiguer l'init de départ, puis tester d'autres seuils NSSM
maintenant que les features sont disponibles.

### Analyse approfondie du run skip=1 (les 8 boucles sont FAUSSES)

En analysant la forme de la trajectoire (corrélation avec le GT) :

| Trajectoire | corr_x | corr_y | Forme |
|-------------|--------|--------|-------|
| DISO standalone | +0.99 | -0.99 | excellente |
| Bruce `dr_` (DISO via bridge, AVANT NSSM) | +0.99 | **-0.88** | bonne |
| Bruce `x/y` (APRÈS les 8 boucles NSSM) | -0.80 | **-0.12** | **cassée** |

→ Le NSSM **dégrade** la trajectoire : avant correction la forme est bonne (-0.88), après les 8
boucles elle est cassée (-0.12). Les 8 loop closures (toutes groupées kf 49-65) sont donc
**fausses / mal contraintes** : elles tordent la trajectoire au lieu de la corriger. Le
« départ à y=-60 » du plot est un artefact d'alignement Umeyama causé par cette déformation
globale, pas une vraie erreur d'initialisation du SLAM (kf0 brut = (0,0) = correct).

---

## Run DISO + Bruce_SLAM — skip=1, min_pcm=6 : fausses boucles filtrées (2026-06-05)

Suite du diagnostic : les 8 boucles du run précédent étant fausses, on durcit le filtre PCM
(Pairwise Consistency Maximization) de `min_pcm: 4` à **`min_pcm: 6`** pour les rejeter.

### Configuration

| Fichier | Paramètre | Valeur |
|---------|-----------|--------|
| `feature_aracati.yaml` | skip | 1 |
| `slam_aracati.yaml` | nssm enable | True |
| `slam_aracati.yaml` | nssm min_points | 30 |
| `slam_aracati.yaml` | nssm max_translation | 8.0 m |
| `slam_aracati.yaml` | nssm max_rotation | deg(45) |
| `slam_aracati.yaml` | **min_pcm** | **6** (était 4) |

### Résultats

![Trajectoires skip=1 min_pcm=6](TESTS_image/run_2026-06-05_skip1_minpcm6_ate5.2/trajectory_plot.png)

| Métrique | min_pcm=4 | **min_pcm=6** |
|----------|-----------|---------------|
| Loop closures | 8 (fausses) | **0** |
| corr_y avant NSSM | -0.88 | -0.98 |
| corr_y après NSSM | -0.12 (cassée) | **-0.98 (préservée)** |
| ATE Bruce | 11.3 m | **5.2 m** |
| Trous > 30 s | 2 | **0** |

### Observations

- `min_pcm: 6` **rejette les 8 fausses boucles** → la forme de la trajectoire est préservée
  (corr_y reste -0.98) → ATE divisé par ~2 (11.3 → 5.2 m). Run parfaitement propre (0 trou).
- Conséquence : retour à **0 loop closure**. Le PCM filtre TOUT car les 8 boucles étaient
  toutes fausses ; il n'y a aucune vraie boucle à valider.

### Conclusion

Le NSSM natif sur Aracati2017 ne propose que des **fausses boucles** : soit on les garde
(min_pcm 4 → trajectoire cassée, ATE 11.3 m), soit on les filtre (min_pcm 6 → 0 boucle, ATE
5.2 m mais aucun gain de loop closure). Dans les deux cas, **aucun bénéfice réel du loop
closure**. C'est l'argument définitif pour **Sonar Context** : une détection de lieu qui
propose de *vraies* boucles robustes, validables par le PCM. Le meilleur réglage actuel
(min_pcm 6, ATE 5.2 m) sert de **baseline** à battre avec Sonar Context.

---

## Branche Bruce_Sonar_USBL — Run 1 : USBL facteurs seuls (loops OFF) (2026-06-17)

Branche dédiée à la fusion USBL par facteurs dans le graphe gtsam. Odométrie cmd_vel,
SSM off, loops (nssm + sonar_context) off — on valide les facteurs USBL **isolément**.
Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-17_215206/`.

### Configuration

| Paramètre | Valeur |
|---|---|
| Odométrie | cmd_vel (intégration /cmd_vel, seed GT à t=0 uniquement) |
| SSM | False |
| NSSM | False |
| Sonar Context | False |
| USBL facteurs | **True** |
| usbl.sigma | 1.4 m |
| usbl.max_dt | 1.0 s |
| usbl.max_speed | 3.0 m/s |

### Résultats

![Trajectoire USBL facteurs](TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-17_215206/trajectory_plot.png)

| Métrique | Valeur |
|---|---|
| **ATE brut** (trajectory.csv, sans alignement) | **2.69 m** |
| ATE Umeyama (trajectory_umeyama.csv) | 1.96 m |
| Trajet GT | 614.4 m |
| Trajet BRUT | 612.5 m (ratio 0.997) |
| Umeyama facteur d'échelle s | **1.018** (≈ 1) |
| \|centre BRUT − centre GT\| | 1.69 m |

### Observations

- **ATE brut = 2.69 m sans aucun alignement** : le SLAM sort nativement dans le bon repère
  métrique. Umeyama n'apporte qu'un léger recentrage (0.7 m de gain).
- **Pas de dilatation d'échelle** : s=1.018 ≈ 1. Les facteurs USBL ancrent directement en
  coordonnées métriques absolues → en vraie vie sans GT, la trajectoire est exploitable.
- Conforme au sandbox `usbl_sim.py` (~3 m attendu → 2.69 m mesuré).

### Conclusion

**Les facteurs USBL fonctionnent.** Résultat réel SLAM (non-artefact Umeyama), à la bonne
échelle, sans dépendance GT. Prochaine étape : activer loops (nssm + sonar_context) pour
le Run 2 (USBL + loops).

---

## Branche Bruce_Sonar_USBL — Run 2 : USBL + loops (2026-06-18)

NSSM + Sonar Context réactivés sur la base du Run 1. Objectif : corriger le cap via les
loop closures pour améliorer ATE et qualité du point cloud.
Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_loops_2026-06-18_100943/`.

### Configuration (différences vs Run 1)

| Paramètre | Run 1 | Run 2 |
|---|---|---|
| nssm enable | False | **True** |
| sonar_context enable | False | **True** |
| min_st_sep | 8 | 8 |
| gate_distance | — | 20.0 m |
| dist_threshold | — | 0.65 |

### Résultats

![Trajectoire USBL + loops](TESTS_image/run_aracati_Bruce_Sonar_USBL_loops_2026-06-18_100943/trajectory_plot.png)

| Métrique | Run 1 (USBL seul) | Run 2 (USBL + loops) |
|---|---|---|
| **ATE brut** | 2.69 m | **2.53 m** |
| ATE Umeyama | 1.96 m | 1.9 m |
| Trajet BRUT / GT | 612.5 / 614.4 m (0.997) | 609.0 / 625.9 m (0.973) |
| Boucles acceptées (kf) | 0 | 19 (3 clusters) |
| SC candidats retenus / total | — | 244 / 659 |

### Observations

- ATE légèrement amélioré (2.69 → 2.53 m) mais trajectoire **déformée** (courbe visible).
- **Fausses boucles courte-terme** : les premiers candidats SC ciblent des kf à ~10 kf
  d'écart (kf21→kf12, kf25→kf17…) — le ROV n'a pas encore fait de vraie revisite.
  L'ICP les valide (scans similaires localement) → la trajectoire est tordue.
- `gate_distance: 20.0` calibrée sur DISO (cap précis) est trop permissive avec cmd_vel :
  244/659 candidats passent, dont beaucoup de faux court-terme.
- Point cloud toujours tourbillon : le cap reste imprécis malgré les loops.

### Conclusion

Les fausses boucles court-terme dégradent la trajectoire. Paramètres à ajuster pour Run 3 :
`min_st_sep: 30` (séparation temporelle ~200 s), `gate_distance: 10.0`, `dist_threshold: 0.60`.

---

## Branche Bruce_Sonar_USBL — DISO odométrie seule (2026-06-18, 161831)

Run **diagnostic décisif**. Après que cmd_vel (tourbillon) et GT (duplication en
translation) aient tous deux échoué à produire un point cloud propre, retour à **DISO**
(Direct Sonar Odometry) comme front-end d'odométrie — la config du bon run du 06-14.
**NSSM off, Sonar Context off, USBL off** : odométrie pure, aucune correction.
Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-18_161831/`.

> ⚠️ **CORRECTION (2026-06-23) — CE RUN N'EST PAS GT-FREE.** DISO a besoin d'un *prior
> de mouvement* et, à cette date, sa seule config (`DISO/config/config_aracati2017.yaml`)
> avait `OdomTopic: /pose_gt`. La variante GT-free (prior `/cmd_vel/pose`) n'a été créée
> que le **22 juin**. Donc DISO recalait le sonar **autour de la vérité terrain**. Le label
> "DISO odométrie seule" est trompeur : le cloud propre est **assisté par la GT**. Idem
> pour le run **120307** (16 juin). Le 1er cloud propre RÉELLEMENT GT-free est obtenu le
> 23 juin (cmd_vel + seuil intensité 140 + filtre de persistance, section dédiée).

### Configuration

| Paramètre | Valeur |
|---|---|
| odom_source | **diso** |
| DISO `OdomTopic` (prior) | **`/pose_gt`** ⚠️ (= GT dans la boucle) |
| nssm / sonar_context / usbl | tous **False** |

### Résultats

![Trajectoire DISO seule](TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-18_161831/trajectory_plot.png)
![Point cloud DISO](TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-18_161831/pointcloud_map.png)

| Métrique | Valeur |
|---|---|
| **ATE brut** | 35.11 m *(désalignement de repère DISO, pas l'erreur réelle)* |
| **ATE Umeyama (aligné)** | **3.16 m** |
| Étendue traj / GT | x[-44.7, 35.1] / x[-42.5, 35.1] — formes ≈ identiques |
| Boucles | 0 (NSSM off) |
| Point cloud | **propre** (port reconnaissable, lignes droites) |

### Observations — percée

- **Point cloud propre** : DISO donne un cap *scan-consistent* (recalage scan-à-scan,
  **autour du prior GT**) → les scans s'accumulent au bon angle.
  > ⚠️ **CORRECTION (2026-06-23)** : la conclusion "le tourbillon cmd_vel venait de la
  > dérive de cap" est **FAUSSE** (établie ensuite à la source). Test poses-GT : avec des
  > poses GT PARFAITES le cloud tourbillonne quand même. Le swirl = le **backscatter du
  > fond du sonar** (intensité ~20-86, retour spéculaire à ~34m qui balaie en arcs), PAS
  > l'odométrie. DISO paraissait propre car (a) prior GT + (b) trajectoire plus grande. La
  > vraie solution GT-free : seuil intensité 140 (les structures sont à >140) + filtre de
  > persistance. cf. section "Cloud propre GT-free (23 juin)".
- **ATE brut 35 m trompeur** : DISO démarre dans un repère tourné → désalignement
  global. Après Umeyama, l'erreur réelle est **3.16 m** (forme correcte).
- **Compromis central** mis en évidence :

  | Config | Point cloud | ATE aligné | Cause |
  |---|---|---|---|
  | DISO seul | **propre** | 3.16 m | cap scan-consistent, mais aucune correction de dérive |
  | cmd_vel + loops + USBL | tourbillon | ~1.9 m | USBL+loops corrigent la dérive, mais cap cmd_vel sale |

### Conclusion

DISO résout le point cloud ; il manque la correction de dérive pour baisser l'ATE.
**Prochaine étape** : empiler loops + USBL *sur DISO* (config cible Bruce_DISO_Sonar_USBL)
→ viser point cloud propre ET ATE bas. Tests NSSM sur DISO (min_st_sep 8 puis 30) ont
créé de la dérive (fausses boucles) → privilégier Sonar Context (apparence + PCM) plutôt
que la détection géométrique NSSM.

---

## Branche Bruce_Sonar_USBL — DISO + Sonar Context (2026-06-18, 200146)

Étape 1 de la config cible : loops sur DISO (USBL encore off). NSSM on (machinerie),
Sonar Context on (détection par apparence), USBL off.
Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-18_200146/`.

### Configuration

| Paramètre | Valeur |
|---|---|
| odom_source | diso |
| nssm enable | True (machinerie de boucle) |
| sonar_context enable | True |
| usbl enable | False |
| min_st_sep / gate / dist_threshold | 30 / 10 m / 0.60 |

### Résultats

| Métrique | DISO seul (161831) | DISO + SC (200146) |
|---|---|---|
| **ATE Umeyama** | 3.16 m | **5.97 m** (+89 %, pire) |
| ATE brut | 35.1 m | 38.3 m |
| Candidats SC retenus (SC+ICP) | — | 104 / 437 |
| **Boucles appliquées (post-PCM)** | 0 | **10** |

### Observations

- **min_st_sep=30 a éliminé les faux court-terme** : séparation des 104 retenus min 37,
  médiane 272 kf, 103/104 sont des revisites ≥100 kf. Plus aucun faux court-terme.
- **Mais les loops dégradent l'ATte** : PCM rejette 94/104, **10 boucles appliquées**
  suffisent à passer 3.16 → 5.97 m.
- **Cause racine** : DISO est déjà localement précis. Les retenus ont une distance SC
  médiane 0.475 (43/104 borderline >0.50). Ces boucles "moyennement ressemblantes"
  portent une erreur ICP **supérieure** à la précision locale DISO → le `BetweenFactor`
  bruité tire la trajectoire optimisée loin de l'estimé DISO déjà bon.
- Séparation nette retenus/rejetés (max 0.600 / min 0.606) : le seuil 0.60 coupe net.

### Conclusion

Sur Aracati, **DISO n'a pas besoin de loop closures** : localement précis, les boucles SC
borderline injectent plus de bruit qu'elles n'en corrigent. Ce qui manque à DISO est
l'**ancrage absolu** (l'erreur brute de 35 m est une dérive *globale*, pas locale) — rôle
de l'**USBL**. Prochaine étape : **DISO + USBL sans loops** pour écraser l'offset global.

---

## Branche Bruce_Sonar_USBL — DISO + USBL (2026-06-18, 225022)

DISO odométrie + facteurs USBL de position absolue, **loops off**. Objectif : ancrer la
dérive globale de DISO (offset brut 35 m) sans recourir aux loops.
Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-18_225022/`.

### Configuration

| Paramètre | Valeur |
|---|---|
| odom_source | diso |
| nssm / sonar_context | False |
| **usbl.enable** | **True** (σ=1.4 m, Cauchy) |

### Résultats — USBL casse DISO

| Métrique | DISO seul | DISO + SC | **DISO + USBL** | cmd_vel + USBL (Run 1) |
|---|---|---|---|---|
| ATE brut | 35.1 m | 38.3 m | **16.2 m** | 2.69 m |
| ATE Umeyama | **3.16 m** | 5.97 m | **13.87 m** | **1.96 m** |
| cov finale (xx/yy) | 202 / 338 | — | **3.5 / 9.1** | — |
| Point cloud | propre | propre | propre | tourbillon |

### Diagnostic — incompatibilité de repère

L'USBL réduit l'offset brut (35→16 m) mais **détruit la forme** (Umeyama 3.16→13.87 m) :

- DISO est localement précis mais son repère **tourne globalement** vs le monde (l'offset
  brut de 35 m est un quasi-pur décalage **rigide**, d'où l'excellent Umeyama de DISO seul).
- Les facteurs USBL sont des priors de **position absolue seulement** (σ_θ=1e6, cap non
  contraint). gtsam tire la chaîne DISO tournée vers des positions monde → **aucune rotation
  unique ne satisfait tous les fixes** → l'optimiseur **gauchit** la trajectoire (non-rigide).
- Preuve : DISO seul s'aligne parfaitement (offset rigide alignable, 3.16 m) ; après USBL
  l'alignement échoue (13.87 m). La covariance chute (202→3.5) → USBL contraint bien la
  position, mais au prix de la déformation.
- USBL marchait sur **cmd_vel** (Run 1, 1.96 m) car cmd_vel est seedé GT → déjà en repère
  monde. DISO non → dérive en rotation que l'USBL (position-only) ne peut pas redresser.

### Conclusion

**USBL et DISO ne se combinent pas** : USBL contraint la position mais pas le cap, et le
repère DISO tourne. Sur Aracati (ni IMU ni boussole), DISO manque d'une référence de cap
absolue. Compromis acté :
- **DISO seul** = carte propre + meilleure forme (3.16 m), mais offset global.
- **cmd_vel + USBL** = meilleur ATE (1.96 m), mais carte tourbillon.

Test à venir : **USBL plus doux** (σ 1.4→4-5 m) — laisser USBL corriger la dérive grossière
sans gauchir, voir si la forme DISO est préservée.

---

## Branche Bruce_Sonar_USBL — Run 3 : cmd_vel + USBL + Sonar Context (2026-06-18, 120154)

Meilleur résultat sur pipeline **100% GT-free** (seed position initiale uniquement).
cmd_vel comme odométrie, USBL pour l'ancrage absolu, Sonar Context pour les loop closures.
Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-18_120154/`.

### Configuration

| Paramètre | Valeur |
|---|---|
| Odométrie | **cmd_vel** (seed position GT à t=0 uniquement) |
| SSM | False |
| NSSM | True (machinerie loop closure) |
| Sonar Context | **True** |
| USBL facteurs | **True** (σ=1.4 m) |
| min_st_sep | 30 |
| gate_distance | 10.0 m |
| dist_threshold | 0.60 |
| min_pcm | 6 |

### Résultats

![Trajectoire](TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-18_120154/trajectory_plot.png)
![Point cloud](TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-18_120154/pointcloud_map.png)

| Métrique | Run 1 (USBL seul) | Run 2 (USBL+SC, gate 20m) | **Run 3 (USBL+SC, gate 10m)** |
|---|---|---|---|
| ATE brut | 2.69 m | 2.53 m | — |
| **ATE Umeyama** | 1.96 m | 1.90 m | **1.44 m** |
| Odométrie ATE | — | — | 11.51 m |
| Boucles acceptées (graphe) | 0 | 19 | **12** |

> ⚠️ Correction : **12** boucles FINALES (facteurs `nssm_constraints`, vérifié sur
> trajectory.csv). Le « 467 » précédent était le nombre de **candidats Sonar Context retenus**
> (colonne `retenu` de loops_detected.csv), AVANT ICP+PCM. Ne pas confondre les deux métriques.

### Observations

- **Meilleur ATE sur pipeline cmd_vel : 1.44 m** (−26 % vs Run 1 USBL seul).
- 12 boucles finales acceptées dans le graphe (467 candidats SC retenus → 12 passent ICP+PCM).
- **Point cloud tourbillon** : ce N'EST PAS le cap (prouvé offline : cap cmd_vel ≈ cap DISO,
  ~6° médian vs course GT). Le swirl = **backscatter du fond** (retour iso-range ~30 m balayé le
  long de la trajectoire). Ni le cap ni le lissage de trajectoire ne le retirent. cf. analyse 06-23.

### Conclusion

Pipeline cmd_vel + USBL + Sonar Context = **meilleur résultat GT-free** obtenu (1.44 m).
Le problème restant est le point cloud (cap imprécis de cmd_vel). Résolu uniquement par DISO (cap scan-consistent) mais DISO nécessite un prior de qualité.

---

## Branche Bruce_Sonar_USBL — Run 4 : DISO + USBL (2026-06-20, 011733)

⚠️ **RÉSULTAT TROMPEUR — PAS GT-FREE**

Meilleur résultat combiné (cloud propre + trajectoire serrée) mais obtenu en trichant :
DISO utilisait `/pose_gt` comme prior de mouvement.

Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-20_011733/`.

### Configuration

| Paramètre | Valeur |
|---|---|
| Odométrie | **DISO** (front-end sonar direct) |
| **DISO prior** | **⚠️ `/pose_gt`** (GROUND TRUTH — donc PAS GT-free) |
| DISO Range | 48.2896 m (calibré SIM3) |
| DISO GradientThreshold | 170 |
| SSM | False |
| NSSM | False |
| Sonar Context | False |
| **USBL facteurs** | **True** (σ ≈ 1.4 m) |
| flip_y | True (repère DISO réfléchi Y) |
| Feature extraction | baseline (Pfa 0.1, threshold 65, res 0.5) |
| Loop closures acceptés | 0 |

### Résultats

| Métrique | Valeur |
|---|---|
| DISO odométrie seul | ATE 5.5 m |
| **DISO + USBL** | **ATE 0.9 m** |
| Point cloud | **propre** — quai + bassin, marina reconnaissable |
| GT-free ? | **❌ NON** — GT est le prior de DISO |

### Pourquoi le cloud est propre

DISO s'aligne **par intensité d'image** (méthode directe) → les scans successifs sont
cohérents localement → pas de swirl. Mais cela ne fonctionne bien ici que parce que le prior
GT lui donne la pose globale correcte à chaque scan.

### Conclusion

Ce run représente le **plafond théorique avec GT** : cloud propre + 0.9 m de trajectoire.
Sert de référence pour évaluer les résultats GT-free. **Ne pas présenter comme GT-free.**

---

## Branche Bruce_Sonar_USBL — Run 5 : cmd_vel + filtres (2026-06-23, 095710)

**Premier cloud propre 100% GT-free.** Compromis : 6 boucles seulement → trajectoire ≈ odométrie.

Archivé dans `results/run_aracati_2026-06-23_095710/`.

### Configuration

| Paramètre | Valeur |
|---|---|
| Odométrie | **cmd_vel** (seed position GT à t=0 uniquement) |
| SSM | False |
| NSSM | True (machinerie loop closure) |
| Sonar Context | **True** (gate 10 m, min_st_sep 30, dist 0.60, min_pcm 6) |
| USBL facteurs | **True** (σ = 5.0 m) |
| **Feature threshold** | **140** (baseline = 65) — structure-only |
| **Map persistence** | **True** — res 3 m, min_obs 35 |
| Bag rate | 1× (rate complet) |

### Résultats

| Métrique | Valeur |
|---|---|
| Keyframes | 654 |
| Loop closures acceptés | **6** (vs **12** au run 120154 — boucles finales graphe) |
| **ATE Umeyama** | **5.2 m** |
| Points cloud (filtré) | 16 937 |
| Point cloud | blocs de voxels + stries iso-range (encore ~66 % de fond > 25 m) |
| GT-free ? | **✅ OUI** |

### Pourquoi la trajectoire s'est dégradée (1.44 → 5.2 m) — analyse honnête

Ce run a changé PLUSIEURS paramètres vs 120154, pas seulement le seuil :

| Paramètre | 120154 (1.44 m) | 095710 (5.2 m) | Effet |
|---|---|---|---|
| `usbl.sigma` | 1.4 m | **5.0 m** | ancre USBL 3.5× plus molle → **cause principale** de la dérive |
| `odom_sigmas` | [0.5] | **[0.2]** | trust accru de l'odométrie cmd_vel (cap imprécis) → aggrave |
| `threshold` | 65 | 140 | loops 12 → 6 (effet **mineur** sur l'ATE) |

→ La régression d'ATE vient surtout du **σ_usbl mou** (et σ_odom serré), **pas** de la famine de
loops (12→6 est mineur). L'ancien récit « le seuil affame les loops → mauvaise traj » était trompeur.

### Le point cloud

Le seuil 140 + persistance N'A PAS donné un cloud propre : il reste des blocs de voxels (bords
de la grille de persistance, res 3 m) et des **stries iso-range** = backscatter du fond balayé le
long de l'axe (que la persistance ne retire pas quand le ROV avance droit). Vérifié offline
(range body médian 30 m, 66 % des points > 25 m). cf. analyse 06-23.

### Problème ouvert

Découpler les features SLAM (threshold 65, dense) des features carte (threshold 140, fort + persistance).
Première tentative effectuée (infrastructure portée par Z du PointCloud2) — pas encore concluant :
les features denses incluent aussi du fond persistant, et l'ICP s'aligne dessus plutôt que sur les structures.

### Conclusion

Run NON concluant : ni cloud propre, ni bonne trajectoire. Sa valeur est **diagnostique** :
il confirme que le swirl = backscatter du fond (pas l'odométrie, pas le cap), MAIS que
seuil 140 + persistance ne suffisent PAS à le retirer (stries iso-range le long de l'axe
survivent). Décision : repartir de la config 120154 (meilleure traj GT-free) et attaquer
le cloud séparément. cf. analyse 06-23 (cap, lissage, variance-de-range).

---

## Branche Bruce_Sonar_USBL — Run 6 : odom_pose (baseline aracati2017) (2026-06-24, 135228)

**Retour à la config 120154** (la meilleure traj GT-free), avec l'odométrie en mode `odom_pose` :
intégration cmd_vel façon nœud `odom` original d'aracati2017 (`main_odom.cpp`). Sert de
baseline propre, reconnaissable par la doctorante.

Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_135228/`.

### Configuration

| Paramètre | Valeur |
|---|---|
| Odométrie | **odom_pose** = cmd_vel intégré (seed position+cap GT à t=0 uniquement) |
| SSM | False |
| NSSM | True (machinerie loop closure) |
| Sonar Context | True (gate 10 m, min_st_sep 30, dist 0.60, min_pcm 6) |
| USBL facteurs (back-end) | **True** (σ = 1.4 m) — **front-end USBL OFF** |
| Feature threshold | 65 (baseline dense) |
| Map persistence | False |
| Bag rate | 1× |

### Résultats

| Métrique | Valeur |
|---|---|
| **ATE Umeyama** | **1.49 m** (≈ 120154 à 0.04 m près) |
| Odométrie brute ATE | 11.51 m (cmd_vel pur, sans front-end USBL) |
| Loop closures | 469 candidats (cf. loops_detected.csv) |
| GT-free ? | **⚠️ Sauf seed t=0** (position + cap initial venant de /pose_gt). Reste à enlever (priorité #1 : seed USBL) |
| Point cloud | swirl du fond (non traité, priorité #2) |

### Légitimité de l'ATE bas (audit GT)

ATE 1.49 m **n'est pas une fuite GT** : audit complet du code → `/pose_gt` n'entre JAMAIS
dans le back-end gtsam (seul usage = `_gt_callback` → export CSV pour calcul d'ATE a posteriori).
L'ancrage vient de l'**USBL** (`/usbl_point`, capteur acoustique réel, dérivé de `/usbl` NavSatFix,
indépendant du DGPS). **Erreur USBL mesurée vs GT : médiane 1.39 m** (moyenne 2.38, p90 4.21, max 73).
Donc ATE ≈ erreur de l'ancre : c'est le comportement attendu d'un SLAM bien ancré, pas une triche.

### ⚠️ Piège évité : double ancrage USBL

Premier essai (run 111133) lancé avec `USBL=true` → fusion USBL AUSSI dans le front-end
(`cmd_vel_odom`) en plus du back-end → l'odométrie snappe sur chaque fix bruité → **ZIGZAG**,
ATE 1.45 → **4.66 m**. Corrigé en retirant `USBL=true` (front-end = dead-reckoning lisse,
back-end = facteurs USBL). Documenté dans run_slam.sh et aracati.launch.

### Seul GT restant

Le seed t=0 (`_seed_cb`) prend de `/pose_gt` : position de départ + cap initial (`atan2` du
1er déplacement). Ponctuel mais c'est encore du GT. **Priorité #1 : seed 100% GT-free** =
position du 1er fix USBL + cap = course-over-ground des premiers fixes USBL (acoustique, pas GT).
→ **Réalisé au Run 7 (150959).**

---

## Branche Bruce_Sonar_USBL — Run 7 : seed USBL 100% GT-FREE (2026-06-24, 150959)

**Jalon : SLAM sonar VRAIMENT GT-free.** Plus AUCUN `/pose_gt` dans la boucle — ni seed, ni
odométrie, ni ancrage. Le dernier GT (seed de pose à t=0) est remplacé par l'USBL acoustique.

Archivé dans `TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959/`.

### Configuration

| Paramètre | Valeur |
|---|---|
| Odométrie | cmd_vel intégré |
| **Seed initial** | **`seed_from_usbl=true`** : position = 1er fix `/usbl_point` ; cap = course-over-ground (atan2 du déplacement jusqu'à `usbl_seed_min_disp=1.0` m). **GT-FREE.** |
| SSM | False |
| NSSM | True |
| Sonar Context | True (gate 10 m, min_st_sep 30, dist 0.60, min_pcm 6) |
| USBL facteurs (back-end) | True (σ = 1.4 m) — front-end USBL OFF |
| Feature threshold | 65 |
| Map persistence | False |
| Bag rate | 1× |

### Résultats

| Métrique | Valeur |
|---|---|
| **ATE Umeyama** | **1.43 m** (≤ baseline GT-seed 1.45-1.49 m) |
| Odométrie brute ATE | 11.49 m (cmd_vel pur) |
| Keyframes | 665 |
| Loop closures | 465 candidats |
| Seed réel | x=1.30 y=1.08 (= 1er fix USBL après 1 m de déplacement, t+12.6 s) |
| **GT-free ?** | **✅ 100% — aucune trace de /pose_gt dans le code actif** |

### Validation offline AVANT le run (test_usbl_seed + test_integration sur le bag)

| Test | Résultat |
|---|---|
| Seed position vs GT | 0.93 m (dans le bruit USBL) |
| Seed cap vs GT-seed | 13° d'écart (bruit USBL + courbure ; corrigé par back-end) |
| Front-end USBL-seed vs GT-seed (ATE brute) | **10.75 m vs 10.78 m** → équivalents, seed non cassé |
| Robustesse glitch (gate vitesse 3 m/s) | fenêtre de seed propre, 0 rejet |

**Découverte : le ROV vire dès le départ** → fenêtre de seed COURTE = meilleur cap
(1 m → 13° ; 3 m → la course-over-ground capte déjà le virage → 78° d'erreur). D'où `min_disp=1.0`.

### Conclusion

L'écart de cap initial de 13° est **entièrement absorbé par le back-end USBL** (ancrage absolu
sur toute la trajectoire) : ATE 1.43 m, identique/meilleur que le seed GT. **Preuve que le seed
GT n'était pas une triche déguisée** — le remplacer par l'USBL ne change pas le résultat.
Baseline GT-free de référence. Reste : le point cloud (priorité #2).

---

## Priorité #2 (cloud) — Bilan et livrable B : carte GT-assistée (2026-06-24)

### Ce qui NE marche PAS pour un cloud propre 100% GT-free (tout vérifié offline)

| Approche | Résultat | Cause |
|---|---|---|
| Filtre variance / intensité / close-range / densité | blobs / arcs | structure lointaine (~30m) étalée |
| Recalage local offline (refine_cloud) | 8% puis -6% | dérive inter-passages invisible au local |
| Loop closures sur structure (verify_structure_loops) | ATE 1.24→1.98m | ICP épars globalement incohérent |
| **DISO GT-free** (173049 rate=1, 203041 rate=0.5) | **swirl** | scan-matching diverge SANS prior GT |
| Correction signe du cap (compas) | swirl réduit mais flou | corrige le cap GLOBAL, pas l'alignement LOCAL |

**Cause racine identifiée :** le swirl = fond sonar (66% des points à ~30m) + **bruit de cap local
±12°**. DISO+GT était propre car le **prior GT donnait le cap local exact** → scans alignés. GT-free,
aucune source de cap (cmd_vel=-compas, DISO GT-free, compas) n'est assez précise localement.
Découverte annexe : `dr_theta = -compas` (cap réfléchi, identité `wz=-d(compas)/dt`).

### Livrable B : carte GT-ASSISTÉE (transparente)

Fusion pose-graph **DISO-120307 local (prior GT) + cmd_vel-150959 global (GT-free)** :
`python3 verify_fusion.py results/run_aracati_Bruce_DISO_Sonar_2026-06-16_120307 \
  TESTS_image/run_aracati_Bruce_Sonar_USBL_2026-06-24_150959 --w_anchor 0.05 --save`

| Métrique | Valeur |
|---|---|
| ATE poses fusionnées vs GT | **0.88 m** |
| Cloud | **quai en T net** (210 398 pts) → `cloudmap_fusion.csv` + `cloudmap_fusion_final.png` |
| Statut | **trajectoire GT-free, carte GT-assistée** (DISO utilise /pose_gt comme prior de mouvement) |

C'est le meilleur cloud présentable. Le cloud propre **100% GT-free** reste ouvert → exploré en
option C (estimateur de cap GT-free custom, branche dédiée).

---
---

# PARTIE 2 — RECENTRALISATION FINALE (2026-07-04) : de la chiralité au champion 1.47 m

> **LE document central des résultats.** Les dossiers des runs conservés sont dans
> `TESTS_image/` (CSV + images, non suivis par git — évaluables par
> `python3 analysis/paper_eval.py TESTS_image/<run>`). Ménage du 07-04 : les runs
> intermédiaires de juin (saga DISO 06-04→06-15 de la partie 1, essais 06-17→06-29
> remplacés depuis) ont été **supprimés** — leurs chiffres restent dans la partie 1
> et dans le tableau maître quand ils comptent. Protocole d'évaluation : ATE Umeyama
> SE(2) s=1 (primaire) + sections S1-S3 + RE + carte vs poses GT — expliqué et justifié
> dans `Paper/MiniPapier/MINI_PAPIER.md` §6.2. Pièges à lire avant tout run : `PIEGES.md`.

## 2.0 Le fil de l'histoire en 5 découvertes

1. **Chiralité (07-02, LE déblocage)** : les scans étaient peints en MIROIR du cap
   (identité R(θ)M = MR(−θ)) → « tourbillon » et détection de boucles morte (6 PCM).
   Fix 1 paramètre (`flip_bearing`), validation : run `141223`. Détail : PIEGES §1.
2. **Sonar Context (branche Bruce_Sonar_USBL)** : détection de boucles par apparence
   (descripteur densité, AUC 0.55→0.86) + porte géométrique → champion 1.2a 1.50 m.
3. **Le σ d'ancre USBL dépend du pipeline** (mesuré 5×) : loops SC seuls → raide
   (1.4-1.8) ; SSM/NSSM natifs → doux (2.5) ; adaptatif par fix → perd (RU5).
   Balayage final : 1.4→1.50 · **1.8→1.47** · 2.0→1.60 · 2.5(+SSM)→3.13.
4. **Rendu compas U1 (branche Bruce_Ultime, 0 run)** : position optimisée + cap compas
   recalé (δ auto-fit GT-free) → carte méd 0.075 / p90 0.413 = la borne GT. Sortie
   standard d'`analyse.sh` (`pointcloud_compass.csv/png`).
5. **Deux échecs instructifs (07-04)** : union des détecteurs sans porte → faux
   positifs (PIEGES §12) ; densification des keyframes sans rescaler les fenêtres
   NSSM (en KEYFRAMES !) → fausses loops court-terme (PIEGES §11).

## 2.1 TABLEAU MAÎTRE — les 21 runs conservés (dossiers dans `TESTS_image/`)

| Dossier (TESTS_image/) | Branche | Lancement | Config clé | Résultats | Verdict / filiation |
|---|---|---|---|---|---|
| `…Bruce_DISO_Sonar_2026-06-16_120307` | BSU (époque DISO) | `ODOM_SOURCE=diso DISO_PRIOR=gt` | DISO + prior GT | cloud propre (submap fusion, partie 1) | réf. VISUELLE carte GT-assistée |
| `…Bruce_Sonar_USBL_2026-06-18_120154` | BSU | `GT_FREE_SEED=false ./run_slam.sh` | seed /pose_gt t=0, σ1.4 | ATE 1.44 | baseline GT-seed (PRÉ-fix : cloud tourbillon) |
| `…Bruce_Sonar_USBL_2026-06-24_150959` | BSU | `./run_slam.sh` | seed USBL (100% GT-free), σ1.4 | ATE 1.43 | baseline GT-free historique (PRÉ-fix) |
| `…Bruce_Sonar_USBL_2026-06-20_011733` | BSU | `ODOM_SOURCE=diso DISO_PRIOR=gt` + loops + USBL | GT-assisté complet | **ATE 0.89**, carte 0.11/0.40, cap 1.6° | ★ **BORNE du dataset** (GT = DGPS planche flottante) |
| `run_aracati_2026-07-01_150034` | BSU | `./run_slam.sh` | PRÉ-fix chiralité | tourbillon + `pointcloud_demiroir.png` | pièce à conviction du miroir (Fig. 1 des papiers) |
| `run_aracati_2026-07-02_141223` | BSU | `./run_slam.sh` | **POST-fix**, SC 0.60 | 1.53, NN 0.365→0.203, PCM 6→82 | ★ validation du fix chiralité |
| `run_aracati_2026-07-02_194559` (**A**) | **Bruce** | `SSM=true NSSM=true USBL=false ./run_slam.sh` | Bruce pur, **zéro USBL**, KF 3.0 | 1.95, RE 5.89 %, carte 0.09/**0.67** | ★ Bruce original sans USBL (cas doctorante), meilleure carte de la branche |
| `run_aracati_2026-07-02_214846` (A-bis) | Bruce | idem A | — | 2.04 | répétabilité : ±0.1 m (ICP non seedé) |
| `run_aracati_2026-07-02_204329` (B) | Bruce | A + `USBL=true USBL_GAIN=0 USBL_BACKEND=true`, σ1.0 | ancre RAIDE | 2.03 | l'ancre raide dégrade A |
| `run_aracati_2026-07-03_120352-1` (**B′**) | Bruce | idem B, σ2.5 (yaml) | ancre DOUCE | **1.88**, 130 loops, carte 0.09/0.74 | ★ champion Bruce pur (amélioration de A) |
| `run_aracati_2026-07-03_003823` (**1.2a**) | **BSU** | `./run_slam.sh` (config figée) | SC **0.70**, σ1.4, SSM off | **1.50**, 116 c., cap 2.6°, carte 0.11/0.99 | ★ champion BSU (améliore 141223 : seuil SC 0.60→0.70 = +108 candidats vrais, 0 faux) |
| `run_aracati_2026-07-03_015742` (1.3) | BSU | 1.2a + `ssm/enable: True` (yaml) | SC + SSM, σ1.4 | 2.14, **NN 0.173** | champion cloud pré-compas ; SSM troque l'ATE contre la carte |
| `run_aracati_2026-07-03_140908-2` (1.4) | BSU | 1.3 + σ2.5 | SSM + doux | 3.13 | rejeté — σ doux incompatible avec SC |
| `run_aracati_2026-07-03_151239-3` (loterie) | BSU | `ODOM_SOURCE=diso DISO_PRIOR=cmd_vel` (+invert_wz) | DISO GT-free | 4.56 (odom brute 39.2) | piste DISO GT-free CLOSE (tag archive/Bruce_DISO_wz) |
| `run_aracati_2026-07-04_125434-RU1` | **Ultime** | `USBL_SIGMA=1.8` (désormais : `./run_slam.sh` nu) | SC 0.70, **σ1.8**, SSM off | **🏆 1.47**, 125 c., carte compas **0.075/0.413**, NN compas **0.172** | 🏆 **CHAMPION FINAL** (améliore 1.2a) — config figée dans le yaml Ultime |
| `run_aracati_2026-07-04_134157-RU2` | Ultime | `USBL_SIGMA=2.0` | σ2.0 | 1.60 | borne l'optimum σ (~1.8) |
| `run_aracati_2026-07-04_145924-RU3` | Ultime | `LOOP_UNION=true` | union SC + gating natif | 2.91, 615 KF (CPU) | rejeté — faux positifs non gatés (PIEGES §12) |
| `run_aracati_2026-07-04_114439-RU4` (B″) | Bruce | commande B′ + `keyframe_translation: 1.0` | KF densifiées | 17.17 | rejeté — fenêtres NSSM en keyframes (PIEGES §11) ; rollback fait ; recette B″-bis dans ABLATION.md |
| `run_aracati_2026-07-04_161907-RU5` | Ultime | `USBL_ADAPTIVE=true` | σ adaptatif par fix (MAD) | 1.62, carte 0.078/0.484 | rejeté — le σ fixe 1.8 gagne (code conservé, défaut off) |
| `run_holoocean_2026-07-03_015206` | **holoocean** | `./run_slam.sh holoocean` | odom **dvl** (défaut) | ATE 0.13, 4 murs reconstruits | ★ config holoocean retenue |
| `run_holoocean_2026-07-03_015417` | holoocean | + `nssm:=true` | loops natives | dérive > dvl seul | NSSM natif n'aide pas (bag court) |

## 2.2 Lecture par branche

- **`Bruce`** (Bruce-SLAM original adapté — papier : `BRUCE_SLAM.md` sur cette branche) :
  A (1.95, sans USBL) → B′ (1.88, ancre douce) ; B et B″ rejetés. Sans USBL = §7.1 du
  papier (cas doctorante) ; avec = §7.2. La carte de A est la meilleure de la branche.
- **`Bruce_Sonar_USBL`** (contribution Sonar Context — mini-papier) : baselines pré-fix
  (1.43/1.44, cloud tourbillon) → fix chiralité (141223) → 1.2a champion 1.50.
  Variantes SSM (1.3/1.4) et DISO GT-free (loterie) rejetées.
- **`Bruce_Ultime`** (fusion des 2 mondes — plan : `ULTIME.md`) : 1.2a + σ1.8 + rendu
  compas = RU1 **1.47 / 0.075 / 0.413**. Union (RU3) et σ adaptatif (RU5) rejetés.
  **`./run_slam.sh` nu sur cette branche reproduit le champion.**
- **`holoocean`** (simulation, préparation 3D) : dvl 0.13 m ; chaîne 3D prête
  (`sonar_source:=points3d`), bag 3D attendu du collègue (`HOLOOCEAN_3D_GUIDE.md`).

## 2.3 🎬 RUNS FINAUX (2 par branche — arrêt auto en fin de bag, UN run à la fois)

Après CHAQUE run : `./analyse.sh <nom_du_run>` puis
`python3 analysis/paper_eval.py results/<nom_du_run>` ; renommer le dossier si voulu
(nom littéral, sans espaces) ; reporter la ligne dans le tableau maître ci-dessus ;
déplacer dans `TESTS_image/` quand validé.

```bash
# ── Bruce_Ultime : 2× le champion (répétabilité du livrable) ──────────────
git checkout Bruce_Ultime
./run_slam.sh          # run 1 — attendu ~1.47 (RU1 : 1.47/0.075/0.413)
./run_slam.sh          # run 2 — variance ICP attendue ±0.1 m

# ── Bruce_Sonar_USBL : 2× le champion 1.2a ────────────────────────────────
git checkout Bruce_Sonar_USBL
./run_slam.sh          # run 1 — attendu ~1.50
./run_slam.sh          # run 2

# ── Bruce : 1× SANS USBL (cas doctorante) + 1× AVEC (champion B′) ─────────
git checkout Bruce
SSM=true NSSM=true USBL=false ./run_slam.sh                                # attendu ~1.95-2.05
SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true ./run_slam.sh   # attendu ~1.88

# ── holoocean : 2× la config dvl (bag test.bag, câblé par défaut) ─────────
git checkout holoocean
./run_slam.sh holoocean     # run 1 — attendu ~0.13
./run_slam.sh holoocean     # run 2
```

⚠ Rappels : jamais `USBL=true` sans `USBL_GAIN=0` (double ancrage, PIEGES §2) ; ne pas
toucher au dépôt pendant un run (PIEGES §4) ; les yaml des 4 branches sont FIGÉS sur
leur champion — aucune édition nécessaire.


## 2.4 ✅ RUNS FINAUX (07-04/05) — répétabilité, verdicts, images

10 runs, 2+ par branche, configs figées, renommés à la main. Tous archivés dans
`TESTS_image/`. Chiffres au protocole du mini-papier (`paper_eval.py`).

| Run (TESTS_image/) | Branche / config | ATE um. (m) | Cap méd | Carte compas méd/p90 | NN compas |
|---|---|---|---|---|---|
| `…182358_Bruce_Ultime_1` | Ultime, σ1.8 | 1.54 | 3.5° | 0.075 / 0.454 | 0.176 |
| `…192645_Bruce_Ultime_2` | Ultime, σ1.8 | 1.60 | 4.6° | 0.078 / 0.459 | 0.180 |
| `…201541_Bruce_Sonar_USBL_1` | BSU, σ1.4 | **1.45** | 3.0° | 0.075 / **0.425** | 0.177 |
| `…210733-Bruce_Sonar_USBL_2` | BSU, σ1.4 | 1.52 | 2.8° | 0.074 / 0.434 | 0.176 |
| `…215353_Bruce_1` | Bruce SANS USBL | 2.07 | 3.0° | 0.090 / 0.628* | — |
| `…223959_Bruce_2` | Bruce SANS USBL | 1.88 | **2.2°** | 0.092 / 0.579* | — |
| `…233119_Bruce_USBL_1` | Bruce + USBL σ2.5 | 1.74 | 2.0° | 0.095 / 0.628* | — |
| `…001730_Bruce_USBL_2` | Bruce + USBL σ2.5 | 1.98 | 1.8° | 0.098 / 0.730* | — |
| `run_holoocean_…010436_1` | holoocean, dvl | **0.13** | 0.0° | 0.07 / 0.16 (θ opt) | — |
| `run_holoocean_…010647_2` | holoocean, dvl | **0.13** | 0.0° | 0.07 / 0.16 (θ opt) | — |

*sur la branche Bruce le rendu compas n'apporte rien (le SSM tient déjà le cap local,
NN se dégrade légèrement) — on garde le rendu θ optimisé comme carte de référence Bruce.

### Conclusions (les vraies, avec la répétabilité)

1. **Les pipelines Sonar Context (BSU σ1.4 et Ultime σ1.8) sont ÉQUIVALENTS dans la
   variance ICP** : 6 runs cumulés → 1.45 / 1.47 / 1.50 / 1.52 / 1.54 / 1.60 (méd 1.51,
   ±0.08). Le « champion RU1 1.47 » était en partie un bon tirage ; le vrai livrable est
   **ATE 1.5 ± 0.1 m, carte compas 0.075 / 0.43 ± 0.02** — reproductible par
   `./run_slam.sh` nu sur l'une ou l'autre branche. Meilleure trajectoire jamais mesurée :
   BSU_1 (1.45).
2. **Bruce pur** : sans USBL 1.88-2.07 (4 runs avec A/A-bis : méd 1.97) ; avec ancre douce
   1.74-1.98 (3 runs : méd 1.88) → l'ancre gagne ~0.1 m en médiane. Le cap est SA force
   (1.8-3.0° méd, records du stage) grâce au SSM. **L'écart contribution vs original
   tient en multi-runs : ~1.5 vs ~1.9 = +0.4 m.**
3. **HoloOcean : 0.13 m parfaitement répétable** (2 runs identiques au cm) — la chaîne
   dvl+imu GT-free est validée en simulation.
4. Note : `Bruce_USBL_1` a une RE translation anormale (23.6 %) avec pourtant le meilleur
   ATE Bruce (1.74) — artefact du seed de cap USBL course-over-ground de CE run (β 112°
   vs ~94° habituels) : le repère odométrique est né tourné de ~16°, l'ancre a recollé les
   positions mais θ garde sa convention → la RE (qui exprime les déplacements dans le
   repère de chaque pose) est gonflée. Sans effet sur l'ATE ni la carte.

![Meilleure trajectoire GT-free du stage — BSU_1](TESTS_image/run_aracati_2026-07-04_201541_Bruce_Sonar_USBL_1/bilan_run.png)
*BSU_1 : ATE 1.45 m — trajectoire, cloud, erreur de cap.*

![Carte livrable — rendu compas de BSU_1](TESTS_image/run_aracati_2026-07-04_201541_Bruce_Sonar_USBL_1/pointcloud_compass.png)
*Le rendu compas (droite) vs θ optimisé (gauche) : NN 0.196 → 0.177, carte p90 0.425.*

![Carte vs carte vraie — BSU_1](TESTS_image/run_aracati_2026-07-04_201541_Bruce_Sonar_USBL_1/run_aracati_2026-07-04_201541_Bruce_Sonar_USBL_1_cloud_vs_gt.png)
*Erreur de carte de BSU_1 : superposition au « nuage vrai » et coloration par distance.*

![Bruce sans USBL — Bruce_2](TESTS_image/run_aracati_2026-07-04_223959_Bruce_2/bilan_run.png)
*Bruce original pur (sans USBL) : 1.88 m, cap 2.2° — le cas d'étude de la doctorante.*

![HoloOcean dvl](TESTS_image/run_holoocean_2026-07-05_010436_1/bilan_run.png)
*Simulation HoloOcean, odométrie DVL+IMU GT-free : 0.13 m.*

## 2.5 📖 LEXIQUE des sorties d'un run (qui produit quoi, comment)

- **`trajectory.csv`** : poses SLAM aux keyframes (x,y,θ optimisés iSAM2) + `dr_*` =
  dead-reckoning (intégration /cmd_vel) au même instant + `nssm_constraints` (loops).
- **`odometry.csv`** : l'odométrie brute à PLEINE fréquence (pas seulement aux keyframes).
- **`groundtruth.csv`** : /pose_gt (DGPS) + θ = cap compas — ÉVALUATION SEULEMENT.
- **`pointcloud.csv`** : détections CFAR de chaque keyframe projetées au monde avec la
  pose optimisée (+ colonne intensité sur BSU/Ultime).
- **`pointcloud_filtered.csv/png`** (analyse.sh → `filter_cloud.py`) : le MÊME nuage,
  filtré par **seuil d'intensité ≥ 255** (on ne garde que les retours saturés =
  structures fortes ; le fond marin/backscatter part). Aucune géométrie modifiée —
  c'est un tri de lignes du CSV. Sert à comparer les nuages au MÊME seuil (PIEGES §8).
- **`pointcloud_compass.csv/png`** (analyse.sh → `render_compass_cloud.py`, U1) :
  re-rendu 100 % GT-free. Chaque scan est ramené dans le repère LOCAL de sa keyframe
  (en inversant la pose optimisée), puis re-projeté avec **la position optimisée mais le
  cap compas recalé** θ_rendu = dr_theta + δ, où δ = moyenne circulaire de
  (θ_optimisé − dr_theta) sur tout le run (recalage de convention INTERNE au run, aucune
  GT). Pourquoi : le θ d'iSAM2 est optimisé pour la POSITION (ancres USBL + loops) et
  transitoire jusqu'à 40° en virage → scans « smearés » ; le cap compas est lisse et
  vrai. Validé : NN 0.20→0.176, carte p90 0.99→0.43 (= le niveau du rendu au cap GT).
- **`<run>_cloud_vs_gt.png`** (paper_eval.py) : ÉVALUATION de la carte. Le « nuage
  vrai » n'est PAS un run GT — on n'en a jamais fait. C'est **nos propres détections,
  re-rendues aux poses GT** en post-traitement : chaque point est ramené au repère local
  de sa keyframe (via la pose SLAM), puis re-projeté à la pose GT de cette keyframe
  (position DGPS ramenée dans le repère carte par l'Umeyama inverse, cap = compas
  converti). La carte d'erreur colore chaque point de NOTRE nuage par sa distance au
  plus proche voisin de ce nuage vrai → mesure la dégradation de la carte causée par
  les erreurs de POSE, à détections identiques. (Mini-papier §6.2f.)
- **`<run>_traj.png` / `<run>_err_time.png`** (paper_eval.py) : trajectoires alignées +
  erreurs au cours du temps ; plein/tirets = alignement Umeyama, pointillés = ancré au
  départ (convention des papiers, comparable à la Fig. 8 de Sonar Context).
- **`bilan_run.png`** (analyse.sh → bilan_run.py) : le résumé 1 image — trajectoire+ATE,
  cloud+NN (auto-cohérence : distance médiane au plus proche voisin, 8000 pts), erreur
  de cap dans le temps (fit s·θ+β, offset de convention retiré).

## 2.6 🔒 AUDIT GT-FREE (07-05) — vérifié dans le code, branche par branche

**Verdict : avec les défauts, les 3 pipelines aracati et holoocean sont 100 % GT-free
et ne consomment QUE des capteurs présents sur un vrai UV.**

Topics consommés par les nœuds lancés PAR DÉFAUT :

| Branche | Nœud | Topics consommés | Statut |
|---|---|---|---|
| Bruce | cmd_vel_odom | `/cmd_vel`, `/usbl_point` | bord + acoustique ✓ |
| Bruce | feature_extraction | image sonar | sonar ✓ |
| Bruce | slam | features + odom (+`/usbl_point` si backend) | ✓ |
| BSU/Ultime | cmd_vel_odom | `/cmd_vel`, `/usbl_point` (seed course-over-ground) | ✓ |
| BSU/Ultime | slam | features + odom + `/usbl_point` (facteurs) | ✓ |
| holoocean | dvl_imu_odom | `/dvl`, `/imu`, `/depth` | capteurs UV ✓ |
| toutes | slam_ros `_gt_callback` | `/pose_gt` → **UNIQUEMENT groundtruth.csv** (éval) | ⚠ éval only |

- `/pose_gt` (DGPS planche flottante — n'existe pas sur un vrai UV) : le SEUL subscriber
  actif est le logger d'évaluation de slam_ros (buffer → CSV à l'arrêt) ; `gt_poses`
  n'est lu par AUCUN chemin de calcul (vérifié par grep sur les 3 branches).
- Les chemins GT existent mais sont des modes **diagnostic non-défaut**, tous vérifiés
  OFF par défaut : `gt_free_seed:=false` (seed /pose_gt t=0), `heading_from_compass`
  (deltas de cap /pose_gt.orientation), `odom_source:=gt` (relais GT), `diso_prior:=gt`.
  Avec les défauts, le subscriber GT de cmd_vel_odom n'est même pas créé
  (`if seed_from_usbl → USBL ; … ; elif seed_from_gt → GT` : branche non prise).
- Nuance ASSUMÉE ET DÉCLARÉE (mini-papier §1.1) : le wz de `/cmd_vel` est dérivé du
  **compas magnétique du véhicule** (README aracati2017) — capteur embarqué légitime,
  mais toute comparaison « sonar-only » doit le mentionner (même statut que les
  baselines Odom+Mag de DISO/ISOPoT).
- HoloOcean : `/ground_truth` alimente gt_odom_to_pose → `/pose_gt` (éval) et le relais
  odométrie SEULEMENT si `odom_source:=gt` (défaut : `dvl`).

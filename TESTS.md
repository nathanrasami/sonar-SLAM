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

Calculé par `analyze_diso.py` (voir `plot1_trajectories.png` pour la valeur exacte).

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

### Conclusion

**Sans NSSM, DISO+Bruce_SLAM ≈ DISO standalone** avec overhead. La prochaine étape est d'intégrer Sonar Context (ICRA 2023) dans le NSSM pour activer le loop closure sur Aracati2017.

# Cadrage migration SLAM 2D → 3D (Bruce-SLAM / HoloOcean)

**MAJ 2026-07-02** — branche dédiée : **`holoocean`** (ex `slam3-d`). À exécuter **après**
la clôture d'Aracati. Bag cible : **`test_2.bag`** (le dernier reçu du collègue).

**Cible : HoloOcean uniquement.** Aracati2017 reste 2D (sonar matériel sans élévation).

**MAJ 07-03 — vérifié sur test_2.bag : `/sonar_points` a z=0 PARTOUT** (projection plate,
~120k pts/msg, portée max 39.9 m) → **pas de vraie 3D possible avec ce bag**. La demande
au collègue (§ ci-dessous) reste LE prérequis de la vraie 3D. En attendant, la **2.5D est
implémentée** : z GT et z=-/depth (dvl_imu_odom) alimentent pose3 ; trajectory.csv,
groundtruth.csv et pointcloud.csv exportent une colonne `z` ; extraction assouplie
(threshold 50, min_points 5 — les murs lointains étaient détectés mais épars, ~29 pts/KF).
Validation : `./run_slam.sh holoocean` (61 s) puis vérifier la colonne z non nulle.

---

## Plan HoloOcean en 2 temps (branche `holoocean`)

### Temps 1 — SLAM 2D sur test_2.bag (livrable minimal)
Bruce-SLAM tel quel sur le bag : RViz (trajectoire + cloud) + les CSV standard
(`groundtruth.csv`, `trajectory.csv`, `pointcloud.csv`, `loops_detected.csv`).
- Vérifier d'abord les topics du bag (`rosbag info`) : format du sonar (image polaire type
  Oculus ? cartésienne ? PointCloud2 ?), odométrie dispo (IMU/DVL simulés ?), GT.
- Adapter `slam_holoocean.yaml`/launch en conséquence (le pont
  `holoocean_sonar_bridge.py` existe déjà).
- Dans `test_2.bag` le véhicule ne bouge pas en z et le sonar est à élévation fixe →
  la **projection horizontale 2D est exactement valide** (pas une approximation dégradée).
- ⚠ Leçon d'Aracati : vérifier la **chiralité** (signe du y latéral des features vs convention
  de cap de l'odométrie) AVANT tout run long — cf. FABLE.md §1. Un `bilan_run.py` sur un
  run court suffit à le voir (fan ±FOV/2, carte qui « tourbillonne » = signe inversé).

### Temps 2 — 3D : deux niveaux
1. **« 2.5D » (recommandé, faible coût)** : z (capteur de pression simulé) + roll/pitch (IMU)
   injectés dans `dr_pose3` → trajectoire 3D et carte extrudée SANS back-end Pose3 (c'est
   l'architecture voulue par Bruce, cf. §ci-dessous). Étapes 0-2 du plan historique.
2. **Vraie 3D volumétrique** : nécessite de l'information d'élévation → voir proposition
   collègue ci-dessous, puis étapes 3-5 du plan historique.

### 📨 Proposition à transmettre au collègue HoloOcean (pour une vraie 3D)
Pour que le SLAM produise une carte 3D réelle (pas une extrusion), il faut de l'élévation.
Trois options, par ordre de préférence :
1. **Publier le PointCloud2 3D du simulateur** (HoloOcean sait donner les points sonar avec
   élévation) → on court-circuite l'extraction 2D, gain maximal pour zéro traitement.
2. **Osciller le tilt du sonar** (pitch du capteur balayé, ex. ±15° à ~0.5 Hz) pendant que le
   véhicule avance → chaque ping échantillonne une tranche différente, la carte devient
   volumétrique avec le pipeline existant + l'angle de tilt publié (indispensable).
3. **Trajectoire avec variation de z** (le véhicule plonge/remonte) → même effet, plus lent.
En complément dans le bag : **IMU** (roll/pitch/yaw + biais réalistes), **pression** (z),
**DVL** si possible (vx,vy,vz), GT avec z et orientation, et **figer les conventions** :
repère monde (ENU ou NED ?), sens de z, convention du quaternion — c'est LE point qui nous a
coûté 3 semaines sur Aracati (bug de miroir).

---

## Bonne nouvelle : l'échafaudage 3D existe déjà

Une partie du code est déjà prête pour la 3D — rien à coder dessus :

- `conversions.py` : `r2g`, `g2r`, `n2g`, `g2n` retournent/lisent déjà des `gtsam.Pose3` (6 DOF)
- `slam_objects.py` (classe `Keyframe`) : champs `pose3`, `points3D`, `transf_points3D` et
  méthode `transform_points_3D()` déjà présents

→ La migration est surtout **mécanique** sur le back-end. Le vrai morceau dur est l'ICP 3D du
loop closure (à faire en dernier).

---

## Changements MÉCANIQUES (remplacement quasi mot-à-mot, faible risque)

### Back-end gtsam — `bruce_slam/src/bruce_slam/slam.py`
- ligne ~434 : `PriorFactorPose2` → `PriorFactorPose3`
- ligne ~450 : `BetweenFactorPose2` (odométrie) → `BetweenFactorPose3`
- ligne ~1120 : `BetweenFactorPose2` (loop closure) → `BetweenFactorPose3`
- `create_noise_model` (~1191) : générique, **rien à changer** → lui passer 6 sigmas

### `bruce_slam/src/bruce_slam/slam_ros.py`
- ligne ~190 : `values.atPose2(X(key))` → `atPose3`

### Configs `slam_holoocean.yaml`
- `prior_sigmas`, `odom_sigmas`, `icp_odom_sigmas` : de **3 à 6 valeurs**
- ⚠️ **PIÈGE** : ordre gtsam Pose3 = **[roll, pitch, yaw, x, y, z]** (rotation EN PREMIER),
  différent de Pose2 (x, y, theta). À vérifier sur la version gtsam installée.

### Export CSV — `slam_ros.py`
- trajectory.csv : ajouter `z` (+ `roll`, `pitch`) — dispo via `kf.pose3`
- pointcloud.csv : ajouter `z` — depuis `kf.transf_points3D`
- groundtruth.csv : ajouter `z` (+ orientation)

### `traj_eval.py`
- `umeyama()` : `W = np.eye(2)` → `W = np.eye(d)` avec `d = src.shape[1]` (généraliser)
- `associer_par_temps()` : interpoler aussi `z` → retour (N, 3)
- `calculer_ate()` : **déjà générique**, rien à changer

---

## Changements STRUCTUREL (refonte, plus de soin)

### `slam_objects.py` — `Keyframe.update()` (~137)
Aujourd'hui : garde z/roll/pitch du dead reckoning, ne prend que x,y,yaw du SLAM.
À faire : accepter un `gtsam.Pose3` optimisé **complet** et mettre à jour `pose3` directement.

### Point cloud 3D
- `feature_extraction.py` (~189, ~321) : `np.zeros(...)` → vrai Z
  - **OU** court-circuité : si le simulateur publie un `PointCloud2` 3D (cf. specs ami), on
    consomme le nuage tel quel sans repasser par l'extraction polaire 2D
- `slam_ros.py` (~175) : `np.c_[points[:,0], -1*points[:,2]]` → garder **x, y, z**
  (en respectant la convention d'axe/sens de z décidée avec l'ami)

### Loop closure NSSM 3D — POINT DUR
- Détection + ICP actuellement 2D
- 3D = recalage de nuages 3D (ICP / GICP / NDT), transfo `Pose3`, covariance 6×6, PCM 3D
- Fragile (convergence, initialisation)
- **Ne pas attaquer en premier** — laisser `nssm.enable: False` (déjà le cas dans
  `slam_holoocean.yaml`)

---

## Ordre d'implémentation recommandé

| Étape | Contenu | Difficulté |
|-------|---------|-----------|
| **0** | Figer les specs + conventions de repère avec l'ami | — |
| **1** | Trajectoire 3D "GT-driven" : propager z+orientation, export CSV+z, ATE 3D, plot 3D (pas de modif back-end) | facile |
| **2** | Nuage 3D : brancher PointCloud2 3D, garder z, exporter, visualiser le volume | moyen |
| **3** | Back-end gtsam 6DOF : Pose2→Pose3, sigmas 3→6, atPose3, update() 6DOF (NSSM off) | moyen |
| **4** | Odométrie 6DOF réelle : fusion IMU+DVL(+depth) (le "EKF later" du launch) | difficile |
| **5** | Loop closure 3D : ICP/GICP 3D + covariance 6×6 | difficile |

Étapes 1–3 majoritairement mécaniques (grâce à l'échafaudage existant). Coût structurel
concentré sur 5 (ICP 3D) puis 4 (EKF 6DOF).

---

## Vérification par étape

- **Étape 1** : `trajectory.csv` / `groundtruth.csv` ont une colonne `z` non nulle ; plot
  matplotlib 3D montre la variation de profondeur ; ATE 3D calculé.
- **Étape 2** : `pointcloud.csv` avec `z` ; le nuage affiché montre un volume (hauteur visible)
  — un barrage 3D, pas une ligne.
- **Étape 3** : le SLAM optimise 6 DOF (NSSM off), trajectoire 3D cohérente avec le GT.

---

## Fichiers clés

| Fichier | Rôle |
|---------|------|
| `bruce_slam/src/bruce_slam/slam.py` | facteurs gtsam (434, 450, 1120), noise (1191) |
| `bruce_slam/src/bruce_slam/slam_ros.py` | nuage (175), atPose2 (190), export CSV (376-430), GT (371) |
| `bruce_slam/src/bruce_slam/slam_objects.py` | Keyframe pose3/points3D/update (échafaudage 3D) |
| `bruce_slam/src/bruce_slam/utils/conversions.py` | déjà 6DOF — référence |
| `bruce_slam/scripts/holoocean_sonar_bridge.py` | entrée sonar (à adapter selon specs ami) |
| `traj_eval.py` | umeyama 2D à généraliser, associer_par_temps à étendre au z |

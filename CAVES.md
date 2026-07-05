# CAVES.md — branche `caves` : Bruce-SLAM dans la grotte (dataset CIRS Underwater Caves)

> Le document de LA branche (comme ULTIME.md/BRUCE_SLAM.md ailleurs) : principe, ce qui
> change par rapport à Aracati dans le code, particularités (3D ?), état de validation.
> Dataset : Mallios et al., IJRR 2017 — https://cirs.udg.edu/caves-dataset/
> Fichiers locaux : `caves.bag` (377 Mo, racine) + `caves_txt/` (CSV, mêmes topics).

## 1. Le principe

Un AUV Sparus guidé par plongeur traverse une **grotte sous-marine** (Cala Viuda,
Majorque) : l'environnement le plus contraint et le plus intéressant pour un SLAM sonar
(parois partout, aucune visibilité, GPS impossible — le GT-free n'est pas une contrainte
d'exercice ici, c'est la réalité). Bruce-SLAM y est appliqué avec le **même cœur**
qu'Aracati/holoocean ; seule l'ENTRÉE change (sonar mécanique + odométrie du dataset).

**Inspection du bag réel (07-05, `analysis/inspect_bag.py`) — hypothèses confirmées :**

| Topic | Type | Contenu vérifié |
|---|---|---|
| `/sonar_micron_ros` | LaserScan | **1 faisceau/msg** (angle dans angle_min), 397 bins, portée 20 m, pas 1.8°, sens +, 23.3 Hz → **200 faisceaux = 1 tour 360° en ~8.6 s** ; intensités 0-191 |
| `/odometry` | Odometry | dead-reckoning DVL+IMU du dataset, repère `world`, 11.2 Hz, **z ≈ 13.9 m** (profondeur) |
| `/imu_adis_ros`, `/imu_xsens_mti_ros` | Imu | 10 Hz (option odométrie maison) |
| `/sonar_seaking_ros` | LaserScan | **profiler VERTICAL** SeaKing, 97 501 faisceaux, 50 Hz — inutilisé en v1, cf. §3 |
| `/dvl_linkquest`, `/depth_sensor`, `/imu_*` (non-_ros), `/tf` | types custom `cirs_girona_cala_viuda` | PAS nécessaires (les topics `_ros` standards suffisent) |

Mission : **32.6 min**, ~500 m de galerie avec retour au point d'entrée (revisite en fin
de parcours = LE test de loop closure).

**Dry-run validé sans ROS** : l'assemblage des faisceaux reproduit hors-ligne sur le bag
(1200 faisceaux → 7 tours, image 397×~200) montre déjà les parois de la grotte à 5-8 m.
Le format supposé par le bridge est le bon.

## 2. Ce qui change par rapport à Aracati dans le code Bruce-SLAM

Le cœur (CFAR → keyframes → ICP → SSM/NSSM → shgo/ICP/PCM → iSAM2) est INCHANGÉ.
Les différences, toutes en AMONT du SLAM :

| | Aracati (branches Bruce/BSU) | Caves (cette branche) |
|---|---|---|
| Sonar | BlueView P900 : **image cartésienne complète** à chaque ping (5 Hz) | Tritech Micron **MSIS : 1 faisceau à la fois** → NOUVEAU `msis_scan_bridge.py` : assemblage en tours complets → image polaire → chaîne feature POLAIRE (celle d'holoocean, pas le mode cartésien) |
| Cadence effective | 5 scans/s | **1 scan / 8.6 s** (un tour) → keyframes pilotées par la distance, pas le temps |
| Odométrie | intégration `/cmd_vel` maison (consignes + compas), seed USBL | `/odometry` du dataset (DVL+IMU, EKF du bord) **relayé tel quel** — pas de cmd_vel_odom |
| Ancrage absolu | USBL (`/usbl_point`) → facteurs Cauchy | **AUCUN** (pas d'USBL en grotte) → la méthode `bruce_sonar_usbl` = Sonar Context SEUL (loops par apparence, usbl/enable False verrouillé) |
| Chiralité | fix `flip_bearing` (chemin cartésien) | chemin polaire natif : le fix cartésien ne s'applique pas, mais la chiralité reste À VÉRIFIER au 1er run (checklist GARDE_FOU §6.1 — si carte en tourbillon : inverser l'ordre/le signe des bearings dans le bridge) |
| GT / évaluation | DGPS continu → ATE Umeyama, sections, RE | **GT éparse** (cônes revus par la caméra, non fournis dans full_dataset.zip) → PAS d'ATE en v1 ; on juge la CARTE (cohérence de la galerie, fermeture au retour) — case dédié dans analyse.sh |
| Lancement | `./run_slam.sh` | `./run_slam.sh caves [Bruce\|Bruce_Sonar_USBL]` |

Fichiers propres à la branche : `msis_scan_bridge.py`, `caves.launch`,
`feature_caves.yaml`, `slam_caves.yaml` (bases holoocean, **seuils à calibrer au 1er
run** : CFAR threshold 80 vs intensités max 191 ; keyframe_translation vs vitesse
~0.2 m/s — et si on densifie, rescaler les fenêtres NSSM, PIEGES §11).

## 3. Particularités — et la question « 3D ? »

- **2.5D immédiat** : la branche hérite du pipeline holoocean → le z de `/odometry`
  (~14 m, variable dans la grotte) est porté dans les CSV/figures ; le sonar Micron est
  HORIZONTAL → la carte est une coupe horizontale peinte à profondeur variable.
  Les figures passent en 3D automatiquement si std(z) > 0.2 m (pipeline unifié).
- **La vraie porte vers la 3D : le profiler SeaKing VERTICAL** (97 501 faisceaux
  inutilisés en v1). Chemin balisé : profils verticaux projetés le long de la
  trajectoire SLAM → carte 3D de la galerie reconstruite OFFLINE (aucun changement du
  graphe). C'est le pendant caves du « mode 3d » holoocean, et l'étape 2 naturelle de
  cette branche.
- **Distorsion de balayage** : 8.6 s/tour à ~0.2 m/s → jusqu'à ~1.7 m de déplacement
  PENDANT un tour. v1 assume (le SLAM voit un scan « moyenné ») ; si les scans sortent
  smeared ou l'ICP décroche : compensation de mouvement par interpolation `/odometry`
  par faisceau (recette du papier ULCDfMS, `Paper/Loop/`) = TODO n°1 connu.
- **Topologie tunnel** : revisites rares avant le retour final — s'attendre à un graphe
  quasi-odométrique longtemps, puis UNE grande fermeture. min_st_sep et le gate SC
  devront le refléter (calibration §GARDE_FOU 7 sur loops_detected.csv).

## 4. Pourquoi ce dataset (et pas les autres) — verdicts du 07-05

- **Caves ✅** : seul candidat avec sonar imageur + odométrie en ROS bag standard.
- Aqualoc ❌ : caméra+IMU+pression, AUCUN sonar (dataset visuel-inertiel).
- ACFR ❌ : stéréo + multibeam bathymétrique descendant, pas de FLS, pas de bags.
- Bonus possibles plus tard : bags natifs des auteurs de Bruce (effort 0),
  Aracati2014 (chaîne aracati quasi telle quelle).

## 5. État et prochaine étape

- [x] Câblage complet (bridge, launch, configs, run_slam, analyse.sh, inspect_bag)
- [x] Bag inspecté : format confirmé, bridge réglé (350°/tour, échelle 1.0)
- [x] Dry-run assemblage hors-ROS : parois visibles
- [ ] **PREMIER RUN** : `./run_slam.sh caves` puis `./analyse.sh 3D run_caves_<date>`
      → checklist chiralité (GARDE_FOU §6.1) sur carte_finale, calibration CFAR/keyframes
- [ ] Ensuite : `./run_slam.sh caves Bruce_Sonar_USBL` (recalibrer τ SC via
      loops_detected.csv) ; compensation de balayage si nécessaire ; SeaKing → 3D.

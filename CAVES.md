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
| Sonar | BlueView P900 : **image cartésienne complète** à chaque ping (5 Hz) | Tritech Micron **MSIS : 1 faisceau à la fois** → NOUVEAU `msis_scan_bridge.py` : assemblage en tours 360° + **CFAR et conversion EN POLAIRE, features publiées DIRECTEMENT** (la chaîne image de feature_extraction suppose un secteur <180° et s'effondre en 360° — PIEGES §13, découvert aux runs de validation) |
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

- **⚠ le nuage Micron n'est PAS de la vraie 3D — c'est un RUBAN 2.5D** (mesuré,
  07-06) : std(z) INTRA-scan = 0.0000 m — chaque tour est une tranche horizontale
  plate posée au z de la pose ; les 4.3 m de variance globale viennent uniquement de
  la profondeur de la trajectoire. C'est PHYSIQUE, pas un bug : le Micron n'encode
  aucune élévation (son ouverture verticale ~35° fait même de chaque tranche une
  APPROXIMATION). Impossible d'en tirer de la vraie 3D. Rôles : le Micron = le SLAM
  (trajectoire, loops) ; la 3D = le SeaKing ci-dessous. `grotte_3d.html --with-map`
  superpose les deux (ruban orange + parois colorées).
- **La vraie 3D : ✅ FAITE (07-06) — `analysis/caves_3d.py`** : le profiler SeaKing
  VERTICAL (97 501 faisceaux) projeté le long de la trajectoire SLAM interpolée →
  la cavité 3D « comme sur le site du dataset », OFFLINE, sans toucher au graphe.
  Détection = retour le plus fort par faisceau (>60, champ proche 1 m ignoré) ;
  géométrie validée en coupe transverse (plafond au-dessus, plancher au-dessous du
  véhicule). Sorties : `grotte_3d.csv` + `grotte_3d.html` (plotly interactif),
  appelé automatiquement par `./analyse.sh <run_caves>` si caves.bag est présent.
  ⚠ En RViz LIVE, la carte Micron reste une tranche 2D à plat : NORMAL (sonar
  horizontal, le topic cloud est 2D) — le relief vit dans les produits offline
  (carte_3d.html = carte Micron en 2.5D ; grotte_3d.html = la cavité SeaKing).
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

## 5. État — ✅ CHAÎNE VALIDÉE EN RUN COMPLET (07-05, 3 runs de validation conteneur)

- [x] Bag inspecté, bridge validé (dry-run + runs réels)
- [x] **Leçon des runs 1-2** : chaîne image feature_extraction inutilisable en 360°
      (nuages bit-identiques de 672 pts — PIEGES §13) → bridge v2 : CFAR + conversion
      EN POLAIRE, features directes. feature_caves.yaml supprimé (les paramètres
      d'extraction vivent dans le bridge : seuil 60, downsample 0.5, outliers 1.5 m/3,
      calibrés étape par étape sur un tour réel : 11 811 pics CFAR → 456 → 180 → ~160).
- [x] **Run de validation complet SANS erreur** (rate 8, headless) : 227 KF, 493 m
      (~les 500 m de la galerie), 27 048 points, z 2.3→19.2 m ; carte : les parois
      ENVELOPPENT la trajectoire, cohérentes aller/retour → **chiralité OK
      (flip_y=False)**. Run de référence : `results/run_caves_VALIDATION3`.
- [x] Panneau sonar RViz : le bridge publie l'image du tour assemblé (disque 360°
      + détections rouges) sur feature_img — rendu par la MÊME formule que les
      features (alignement garanti). Panneau « aerial view » : pas de vue aérienne
      d'une grotte — bloc image_publisher PRÊT EN COMMENTAIRE dans caves.launch
      (poser une image dans bruce_slam/maps/caves_survey.png et décommenter,
      ex. le plan de coupe du papier IJRR).
- [ ] Run visuel de Nathan (RViz) : `./run_slam.sh caves` + `./analyse.sh 3D run_caves_<date>`
- [ ] Ensuite : loops (nssm:=true, min_st_sep à adapter à la topologie tunnel) ;
      `./run_slam.sh caves Bruce_Sonar_USBL` (recalibrer τ SC via loops_detected.csv) ;
      compensation de balayage si besoin ; SeaKing → 3D.

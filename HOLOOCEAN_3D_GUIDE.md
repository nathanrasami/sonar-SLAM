# HOLOOCEAN 3D — Bags livrés : contenu détaillé pour le SLAM (v3, 07-07 soir)

> **Pour** : Nathan (côté SLAM). Ce document remplace le guide v2 (objectifs) : les
> deux bags sont **générés et validés** — voici exactement ce qu'ils contiennent,
> comment chaque topic a été fabriqué, et les pièges connus.
> Historique v1/v2 : récupérable via git.

---

## 1. Les livrables

| Fichier | Taille | Durée sim | Contenu |
|---|---|---|---|
| `Scripts_HoloOcean/BAG_files/holoocean_3d_traj1.bag` | 3.6 Go | 676 s (11 min 16) | Traj 1 — pseudo-3D (z par la trajectoire) |
| `Scripts_HoloOcean/BAG_files/holoocean_3d_traj2.bag` | 3.6 Go | 676 s | Traj 2 — vraie 3D (profiler vertical) + tous les topics de traj 1 |

Les deux bags partagent **exactement la même trajectoire** (même seed de bruit) ;
traj 2 ajoute uniquement le capteur profileur et son topic. Les checklists complètes
sont dans `Scripts_HoloOcean/validation_3d/full_run.log` ; les deux bags sont
**PASS sur tous les critères** (détail §5).

Monde : `Bruce_slam_nathan` (package Charuco, re-cook du 07-07 22:03 — murs
extérieurs corrigés en double-face).

---

## 2. La trajectoire (identique dans les 2 bags)

**Rectangle à coins arrondis sur les lignes MÉDIANES du couloir**, mesurées au
sonar (échos symétriques des deux côtés depuis ces lignes) :

- Centre `(-14.8, 14.5)` m, côtés `LX=30.2` (x), `LY=32.4` (y), virages `R=4` m
- Côtés du chemin : sud `y=-1.7`, nord `y=+30.7`, est `x=+0.3`, ouest `x=-29.9`
- Clearance murs/bloc : **2.2 à 2.8 m** de chaque côté (mesuré)
- Profondeur : `z(s) = -3.5 + 2.0·sin(2π·3·s/PERIM)` → z ∈ [-5.5, -1.5],
  **std(z) = 1.41 m**, 3 cycles par tour (entier → boucle fermée exacte)
- **roll = 0 et pitch = 0 partout** (« grande route ») ; yaw = tangente du chemin,
  |dyaw/dt| max = 5 °/s (virages)
- Vitesse d'avance 0.35 m/s ; périmètre 118.3 m ; **2 tours** ; départ à
  `(-24.7, -1.7, -3.5)` (dans le couloir sud, côté ouest) ; **fermeture 0.01 m**
- Génération par **teleport** à 20 Hz sur la trajectoire analytique (pas de PID,
  pas de dynamique) → les capteurs inertiels sont synthétisés analytiquement (§3)

⚠ **Pourquoi la médiane et pas la ligne du bag manuel** : l'ancien pilotage manuel
longeait le bloc central à ~0.7 m, DANS la zone aveugle du sonar (RangeMin 0.5 m +
champ proche) — c'est ça (et non un défaut du bloc) qui rendait le « mur gauche »
muet/fuyant sur les anciens bags. Depuis la médiane, échos symétriques 2.4/2.4 m.

---

## 3. Topics — contenu détaillé

Tous les stamps = **temps simulé** croissant, `stamp = t_sim + 1.0 s` (offset pour
éviter t=0). Fréquences : 20 Hz pour GT/IMU/DVL/depth, **5 Hz** pour les topics sonar.

### 3.1 `/ground_truth` — nav_msgs/Odometry, 20 Hz, 13 524 msgs
- `header.frame_id = "map"`, `child_frame_id = "auv0"`
- `pose.pose.position` : position EXACTE (m), **ENU, z vers le HAUT** (sous l'eau z<0)
- `pose.pose.orientation` : quaternion **xyzw** exact, construit depuis
  (roll=0, pitch=0, yaw) — convention R = Rz(yaw)·Ry(pitch)·Rx(roll)
- `twist.twist.linear` : vitesse **repère MONDE** (m/s, dérivée analytique exacte,
  sans bruit) ; `twist.angular` = (0,0,0) (non rempli)
- Covariances : zéros (pose exacte)

### 3.2 `/imu` — sensor_msgs/Imu, 20 Hz, 13 524 msgs
- `frame_id = "auv0"` (repère véhicule : x avant, y gauche, z haut)
- `orientation` : quaternion xyzw **exact** (même source que le GT — pas de dérive)
- `angular_velocity` : ω exact + bruit gaussien **σ = 0.002 rad/s**.
  roll=pitch=0 ⇒ ω = (0, 0, dyaw/dt) : nul en ligne droite, ~±0.087 rad/s en virage
- `linear_acceleration` : **gravité seule projetée dans le repère véhicule**
  `R^T·[0,0,9.81]` + bruit **σ = 0.02 m/s²**. ⚠ Les accélérations propres
  (centripète en virage ~0.03 m/s², heave de la sinusoïde z) ne sont PAS incluses —
  si ton filtre intègre l'accélération, il verra un accéléromètre « au repos »
- Covariances : zéros

### 3.3 `/dvl` — geometry_msgs/TwistStamped, 20 Hz, 13 524 msgs
- `frame_id = "auv0"` ; `twist.linear` = vitesse **repère VÉHICULE**
  (vx avant ≈ 0.35, vy gauche ≈ 0, vz haut = dérivée de la sinusoïde z, ±0.11 m/s max)
  = `R^T · v_monde` + bruit gaussien **σ = 0.01 m/s** par axe
- Pas de ranges de fond (contrairement au vrai DVL HoloOcean) ; `twist.angular` = 0

### 3.4 `/depth` — std_msgs/Float64, 20 Hz, 13 524 msgs
- Profondeur **POSITIVE vers le bas** = `−z_GT` + bruit **σ = 0.02 m**
- Varie entre ~1.5 et ~5.5 m

### 3.5 `/sonar` — sensor_msgs/Image, 5 Hz, 3 380 msgs
- **Encodage `32FC1`** (float32), `frame_id = "auv0"`, 512 lignes × 512 colonnes
- Image POLAIRE du sonar principal (ImagingSonar « SonarFin ») :
  **lignes = range 0.5 → 40 m** (ligne 0 = 0.5 m), **colonnes = azimut −60° → +60°**
- Intensités [0,1] ; dans ce monde les échos de murs plafonnent à **~0.47**
- Config capteur : `Azimuth 120, Elevation 6, RangeMin 0.5, RangeMax 40,
  RangeBins 512, AzimuthBins 512, AddSigma 0.01, MultSigma 0.01, RangeSigma 0,
  MultiPath false, AzimuthStreaks 0, ScaleNoise false`, monté à plat
  (rotation [0,0,0]) sur SonarSocket
- ⚠ Bruit **0.01** (vs 0.05 du guide v1/v2) : mesuré indispensable — à 0.05 le
  bruit noie les échos (~0.25-0.47 max ici) et remplit le nuage de fantômes

### 3.6 `/sonar_points` — sensor_msgs/PointCloud2, 5 Hz, 3 380 msgs
- `frame_id = "map"` — **points 3D repère MONDE** (déjà transformés, rien à faire)
- Champs : `x, y, z, intensity`, tous float32, `point_step = 16`, height=1,
  width = nombre de points (variable), is_dense=true
- Fabrication : pixels de l'image `/sonar` avec **intensité > 0.10**, projetés en
  supposant l'écho **dans le plan d'élévation 0 du capteur** (fan fin ±3°) :
  `p_capteur = r·[cos a, −sin a, 0]` (x avant, y gauche ; azimut positif à droite),
  puis `p_monde = p_robot + R_monde_robot · p_capteur`
- Traj 1 : ~890 pts/ping en moyenne (3.0 M points au total), z ∈ [-5.5, -1.5]
- ⚠ **std(z) INTRA-message ≈ 0 par construction** (roll=pitch=0 → chaque ping est
  une tranche horizontale au z du robot). C'est le pseudo-3D **assumé** du contrat
  v2 §2 : le z 3D vient de la trajectoire. La vraie 3D intra-ping est dans
  `/profiler_points` (traj 2)

### 3.7 `/profiler_points` — sensor_msgs/PointCloud2, 5 Hz, 3 380 msgs (traj 2 SEULEMENT)
- Même format exact que `/sonar_points` (xyzi float32, `frame_id = "map"`, monde)
- Source : **ProfilingSonar « ProfilerVert » monté VERTICAL** (rotation [90,0,0] =
  roll 90°) : son fan ±60° balaye le plan **x-z du véhicule** (avant/haut-bas)
- Config : `Azimuth 120, Elevation 1, RangeMin 0.5, RangeMax 40, RangeBins 512,
  AzimuthBins 240`, mêmes bruits que le SonarFin
- Projection : `p_capteur = R_roll(90°) · r·[cos a, −sin a, 0]` puis passage monde
- **Chaque message est une SECTION VERTICALE de l'environnement** (fond + mur +
  partie émergée du mur) : 1.19 M points, z ∈ [-11.2, +6.1],
  **std(z) intra-message médian = 2.45 m, 100 % des pings > 0.5 m** — le critère
  « vraie 3D » du contrat v2 §3 est prouvé
- C'est la matière première de la reconstruction « profiler le long de la
  trajectoire » (type grotte)

---

## 4. Pièges connus / à filtrer côté SLAM

1. **Points à z > 0 dans `/profiler_points`** (jusqu'à +6.1 m) : les murs émergent
   de l'eau et HoloOcean ne modélise PAS la réflexion de surface → le fan vertical
   voit la partie aérienne des murs. **Filtre `z < 0`** si ta reconstruction n'aime pas.
2. **Asymétrie en incidence rasante** : en ligne droite, le bloc central (intérieur)
   renvoie mieux que le mur extérieur au bord du fan (±60°). En perpendiculaire les
   deux sont identiques (2.4 m / i=0.47). Effet visible : le côté intérieur du
   couloir est plus dense dans `/sonar_points`.
3. **`/sonar_points` traj 1 = tranches horizontales** (intra-z nul) — c'est voulu,
   ne pas s'étonner ; la 3D par capteur est dans traj 2.
4. **IMU sans accélérations propres** (gravité seule + bruit) — voir §3.2.
5. **RangeMin 0.5 m + champ proche** : tout objet à moins de ~0.7 m du sonar est
   invisible. La trajectoire médiane garde tout à ≥ 2.2 m, mais souviens-t'en si tu
   rejoues d'anciens bags manuels (qui frôlaient le bloc à 0.7 m — d'où leur
   « fuite mur gauche » historique).
6. Échos max ~0.47 (pas 1.0) : seuils d'intensité côté SLAM à caler en conséquence.

---

## 5. Validation (résultats des checklists, log complet dans `validation_3d/full_run.log`)

| Critère (contrat v2) | traj 1 | traj 2 |
|---|---|---|
| roll GT ≈ 0 (max mesuré) | 0.00° ✅ | 0.00° ✅ |
| Boucle fermée < 2 m | 0.01 m ✅ | 0.01 m ✅ |
| Enveloppe (dans le couloir, marges) | ✅ | ✅ |
| std(z) trajectoire > 1 m | 1.41 m ✅ | 1.41 m ✅ |
| Pings non vides | 3380/3380 ✅ | 3380/3380 ✅ |
| Sections verticales (std intra > 0.5 m sur ≥1/3 pings) | n/a | **100 %** ✅ |
| §0 image propre (pas d'arcs, pas de retours au-delà des murs) | ✅ (PNG dans `validation_3d/`) | ✅ |

---

## 6. Reproduire / régénérer (si besoin)

Scripts dans `Scripts_HoloOcean/` (pur Python, `rosbags`, pas de ROS requis) :

| Script | Rôle |
|---|---|
| `gen_bag_3d.py --traj 1\|2 [--test N]` | Générateur (trajectoire + capteurs synthétiques + écriture bag). `--test 60` = bag court de validation |
| `check_bag_3d.py <bag> --traj 1\|2` | Checklist PASS/FAIL automatique du contrat v2 |
| `check_sonar_frames.py <bag>` | PNG de frames sonar + métrique de fuite gauche/droite (§0) |
| `run_full_bags.bat` | Les 2 runs complets + validations, en batch détaché |

Paramètres clés en tête de `gen_bag_3d.py` (centre/côtés du rectangle, Z0/Z_AMP,
V_FWD, bruits). ⚠ Si le monde est re-cooké : supprimer le cache octree
(`%LOCALAPPDATA%/holoocean/2.3.0/worlds/Charuco/Windows/Holodeck/Octrees/Bruce_slam_nathan`)
sinon le sonar continue de voir l'ancienne géométrie.

## 7. Historique du débogage (résumé, détail dans `Scripts_HoloOcean/DIAGNOSTIC_3D_murs.md`)

1. Murs extérieurs invisibles au sonar → géométrie **simple face** dans UE →
   corrigé par re-cook (07-07).
2. Bloc central « muet »/« fuite mur gauche » → le bloc était sain : les anciennes
   trajectoires le frôlaient à 0.7 m, dans la **zone aveugle RangeMin** → corrigé
   par la trajectoire médiane (mesurée au sonar, 4 côtés).
3. Bruit 0.05 → nuages remplis de fantômes (échos réels ~0.25-0.47) → réduit à 0.01.
4. Zone navigable établie depuis le `/ground_truth` du bag manuel du 30-06
   (30 688 poses) puis affinée par sondes sonar perpendiculaires.

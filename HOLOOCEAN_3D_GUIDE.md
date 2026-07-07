# HOLOOCEAN 3D — Objectifs pour le bag 3D (v2, 07-07)

> **Pour** : [le collègue HoloOcean] + son agent (Fable 5). Ce document donne des
> **objectifs clairs et vérifiables**, pas du code : ton agent saura implémenter.
> Chaque objectif a son critère PASS/FAIL — c'est le contrat.
> Côté SLAM (Nathan), tout est prêt pour consommer les bags (`./run_slam.sh holoocean [3D]`).
>
> **v2 (07-07)** : remplace le guide détaillé v1 (récupérable via git). Changements :
> fix « fuite » mur gauche ajouté ; départ DANS la structure ; **plus d'acrobaties —
> le robot reste à plat (« grande route »)** ; deux trajectoires redéfinies :
> **Traj 1 = pseudo-3D (spirale en profondeur)** puis **Traj 2 = vraie 3D (par le sonar)**.

---

## 0. PRÉALABLE — corriger le rendu sonar (avant toute trajectoire)

Deux artefacts constatés sur les bags actuels, à corriger et à prouver sur UNE image :

1. **Arcs** dans l'image sonar brute (visibles sur le carré).
   Pistes dans l'ordre : `MultiPath: false`, `AzimuthStreaks: 0`,
   bruits `AddSigma/MultSigma ≤ 0.05` + `RangeSigma: 0`.
2. **« Fuite » sur le mur parallèle au robot — mur GAUCHE uniquement** : des retours
   traversent/débordent le mur quand le robot lui est parallèle. Asymétrique (gauche
   seulement) → suspects : réflexion acoustique (multipath), géométrie/normales de ce
   mur dans la scène, ou position du socket sonar. À investiguer et corriger.

**Critère PASS** : une image `/sonar` où les murs = lignes nettes, sans arcs, sans
points au-delà des murs, symétrique gauche/droite quand le robot est centré.

---

## 1. Contraintes de trajectoire (valables pour TOUTES les trajectoires)

- **Départ DANS la structure** (pas à l'extérieur comme les tests actuels) : première
  pose entre les murs, sonar voyant déjà l'environnement.
- **« Grande route » : le bas du robot regarde TOUJOURS le bas** (aligné gravité).
  Roll ≈ 0 en permanence — pas de vrille, pas de roulis avec la structure. Pitch
  autorisé mais modéré (|pitch| ≤ 20°). Analogie : le robot roule sur une route qui
  monte/descend, il ne tourne jamais sur lui-même.
- Vitesses réalistes : |Δyaw|/Δt < 30°/s, avance ~0.3–0.5 m/s.
- Marges : ≥ 1.5 m des murs, ≥ 1 m de la surface et du fond.
- **Boucle fermée** : point d'arrivée ≈ point de départ (< 2 m) — le SLAM a besoin
  de revisites. Durée visée ~10 min simulées (≈ 2 tours).
- Génération par **teleport sur trajectoire analytique** (pas de PID) → synthétiser
  IMU/DVL analytiquement (ω et v exacts + bruit gaussien réaliste), le sonar est rendu
  par le moteur à la pose téléportée.

---

## 2. TRAJ 1 — pseudo-3D : « spirale » en profondeur (robot à plat)

**Objectif** : le robot suit le parcours (carré, coins arrondis) en **variant sa
profondeur en continu** (montées/descentes cycliques le long du chemin — une « spirale »
verticale sur la route), **roll = 0 tout du long**.

- Le z 3D vient de la **trajectoire** (profondeur), pas du capteur : c'est du **2.5D
  assumé** (chaque ping reste une tranche ~horizontale posée au z du robot).
- Ce bag valide toute la chaîne : GT 6-DOF, `/depth`, `/dvl`, `/imu`, projection z,
  export CSV, figures 3D côté SLAM.

**Critères PASS** :
- std(z) de la TRAJECTOIRE (GT) > 1 m ; z toujours dans [fond+1 m, surface−1 m].
- roll GT ≈ 0 partout (|roll| < 2°).
- std(z) INTRA-message de `/sonar_points` ~ 0 : NORMAL ici (c'est la définition du
  pseudo-3D — ne pas « tricher » pour le gonfler).

## 3. TRAJ 2 — vraie 3D : le balayage vient du SONAR

**Objectif** : même style de trajectoire (à plat, boucle fermée), mais la couverture
verticale vient du **capteur**, pas du robot. Deux options, au choix selon ce que le
simulateur permet le plus simplement :

- **A (recommandée)** : un **second sonar profileur monté VERTICAL** (type
  ProfilingSonar, élévation ~1°) publié sur `/profiler_points` — chaque message est
  une **section verticale** de l'environnement (recette validée sur un dataset réel
  de grotte ; la reconstruction côté SLAM existe déjà).
- **B** : le sonar principal **tilté ou oscillant en pitch** (le plan de scan monte et
  descend) — la 3D vient du balayage du plan.

**Critères PASS** :
- Un message de `/profiler_points` (A) affiché en 3D dessine une **section verticale**
  (arc), pas une ligne horizontale.
- (B) std(z) INTRA-message de `/sonar_points` > 0.5 m sur les pings tiltés.
- ⚠ Piège connu (mesuré sur dataset réel) : un std(z) GLOBAL élevé peut venir de la
  seule profondeur de trajectoire — **seul le critère intra-message prouve la 3D**.

---

## 4. Contrat d'interface (inchangé — ne rien renommer)

| Topic | Type | Convention |
|---|---|---|
| `/sonar` | Image 32FC1 (R×Az) | polaire, range 0.5→40 m, azimut ±60° — **si RangeMax/Azimuth changent, le dire** (valeurs codées côté SLAM) |
| `/sonar_points` | PointCloud2 xyzi | points 3D **repère monde**, z réel |
| `/profiler_points` | PointCloud2 xyzi | (traj 2A) sections verticales, repère monde |
| `/ground_truth` | Odometry | 6-DOF exact, **ENU, z vers le HAUT** (sous l'eau z<0), quaternion xyzw |
| `/imu` | Imu | orientation absolue + gyro/accel (repère véhicule) |
| `/dvl` | TwistStamped | vitesse repère véhicule (vx avant, vy gauche, vz haut) |
| `/depth` | Float64 | profondeur POSITIVE vers le bas (= −z) |

Stamps = temps simulé croissant ; mêmes noms de topics que les bags actuels (drop-in).

## 5. Checklist avant envoi (5 min, par bag)

- [ ] §0 : une image `/sonar` propre (pas d'arcs, pas de fuite mur gauche)
- [ ] départ dans la structure ; roll GT ≈ 0 partout ; jamais hors murs/eau (min/max GT)
- [ ] boucle fermée (< 2 m) ; durée ~10 min
- [ ] Traj 1 : std(z) trajectoire > 1 m
- [ ] Traj 2 : critère intra-message (§3) — c'est LE test qui prouve la vraie 3D
- [ ] `rosbag info` : topics du §4 présents, ~5 Hz sonar

## 6. Côté SLAM (déjà prêt, pour info)

`./run_slam.sh holoocean` (2D) / `./run_slam.sh holoocean 3D` (consomme `/sonar_points`).
CSV avec z, figures auto-3D, carte interactive `./analyse.sh 3D <run>`. Reconstruction
« profiler le long de la trajectoire » déjà écrite et validée (dataset grotte réel).
Envoie les bags, c'est branché.

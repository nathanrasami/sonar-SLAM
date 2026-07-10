# HOLOOCEAN 3D — Demande en cours : ajouter un 2ᵉ sonar VERTICAL à l'avant

> **Pour** : le collègue HoloOcean + son agent.
> Le bag `holoocean_3d_traj3.bag` actuel est **validé** côté SLAM (ATE 0.03 m, carte 3D GT-free
> à 5.7 cm de la carte GT). **Tout le reste est INCHANGÉ** : trajectoire (errance aléatoire,
> seed 42), monde PierHarbor, bruits, teleport. **Ne touche à rien d'autre que le §1.**
> Historique complet des itérations précédentes : `git log` sur ce fichier.

---

## 1. LA DEMANDE — un 2ᵉ sonar, fan vertical, vers l'avant

**Objectif de Nathan** : cartographier en 3D **ce que le ROV voit DEVANT** (pas le fond sous lui).

### 1.1 Géométrie

Garder le sonar SLAM **inchangé**, et ajouter **à côté, à l'avant, un DEUXIÈME sonar** — même
capteur, mêmes paramètres — simplement **tourné de 90° autour de l'axe d'avance x** :
`rotation = [90, 0, 0]` (roll 90°).

Vu de face (le long de x) : le sonar SLAM balaie en `−` (plan x-y, gauche↔droite) et le nouveau
balaie en `|` (plan **x-z**, haut↔bas). Les deux regardent devant → un `+`. Son ouverture de 120°
est donc en **élévation** (±60° haut/bas) ; son ambiguïté est en **azimut** (les échos hors axe
sont rabattus sur le plan vertical central — c'est connu, attendu, et accepté).

**C'est le mount v3 que tu as déjà codé** (`R_MOUNT_PROF`, rotation `[90,0,0]`, bags traj1/traj2) —
pas de nouvelle géométrie à inventer. Vérifié côté SLAM sur `holoocean_3d_traj2.bag` :
`std(x)=7.19  std(y)=0.00  std(z)=1.74` → fan bien dans le plan x-z, et la reconstruction donne
des **murs verticaux nets de z = −11 à +6 m**. C'est exactement ce qu'on veut.

### 1.2 Topics à publier

| Topic | Type | Fréquence | Convention |
|---|---|---|---|
| `/sonar_vert` | sensor_msgs/Image 32FC1 | 5 Hz | polaire, lignes=range, colonnes=**élévation** ±60° |
| `/sonar_vert_points` | PointCloud2 xyzi | 5 Hz | **repère VÉHICULE, `frame_id="auv0"`** |

- Repère véhicule = **x avant, y GAUCHE, z HAUT** (ENU), comme `/imu`, `/dvl`, `/sonar_points`.
- 🚨 **`frame_id="auv0"` OBLIGATOIRE.** Si tu publies en repère monde (`frame_id=map`, comme les
  vieux bags v3/traj2), **la pose GT est cuite dans les points** → la carte 3D n'est plus GT-free
  → **résultat inutilisable pour le stage** (contrainte absolue : capteurs embarqués seulement,
  la GT ne sert QU'À l'évaluation). C'est l'erreur exacte de traj2.
- ⚠ **Vérifie les SIGNES, ne les suppose pas.** On a déjà eu un axe **y inversé** sur
  `/profiler_points` (les murs atterrissaient du mauvais côté, en miroir). Le CHECK E3 ci-dessous
  existe pour ça.

### 1.3 Réalisme obligatoire (si la 3D marche, Nathan la fera en vrai)

On ne se contente pas de « le simulateur ignore le crosstalk » : on **choisit un matériel réel où
le crosstalk n'existe pas**, pour que la simu soit *fidèle* et non *optimiste*.

- **Deux têtes à fréquences porteuses DIFFÉRENTES** → elles pinguent simultanément sans se brouiller,
  exactement comme le simulateur le fait déjà.
  - `/sonar` (SLAM, horizontal) : classe **~750 kHz**, portée **0.5–40 m** — *inchangé*.
  - `/sonar_vert` (vertical 3D) : classe **~1.2 MHz**, portée **0.5–20 m**, **résolution en distance
    plus fine**. C'est le compromis physique réel (haute fréquence = meilleure résolution, moins de
    portée) et c'est exactement ce qu'on veut pour la 3D de ce qui est devant.
  - Si HoloOcean n'expose pas la fréquence : **émule-la** par `RangeMax = 20 m` + un nombre de cases
    de distance plus élevé sur `/sonar_vert`, et **note-le au §4**.
- **Bras de levier déclaré** : IRL deux têtes ne peuvent pas être au même point. Monte `/sonar_vert`
  avec un **décalage constant d'environ 0.15 m** du sonar SLAM, et **déclare la valeur exacte** (§4).
- **Tilt du sonar SLAM = 0** : le tilt mécanique oscillant était un bricolage pour grappiller de
  l'élévation ; elle vient désormais du sonar vertical. **Supprime-le** (et `/sonar_tilt` avec).

### 1.4 Ce qui change autour

- **Supprime le profiler transverse** (`/profiler_points`) : Nathan ne veut pas cartographier le fond.
  Ça libère aussi le coût de rendu → `{sonar SLAM + sonar vertical}` = **même nombre de capteurs
  qu'aujourd'hui**, donc **pas de surcoût**. (Si tu le gardes quand même : sache que son axe y est
  inversé, on le corrige côté SLAM.)
- **Pas de conflit entre les 2 sonars** : HoloOcean rend chaque capteur par lancer de rayons
  indépendant, il ne simule pas la propagation acoustique entre capteurs. Preuve : le bag actuel fait
  déjà tourner deux sonars simultanés sans pollution croisée. Et avec des porteuses différentes (§1.3),
  il n'y en aurait pas non plus en vrai.
- **Piège côté SLAM (info)** : notre bridge est câblé en dur sur `/sonar`, donc `/sonar_vert` n'ira
  jamais polluer le SLAM. Mais ne réutilise pas ce bridge pour `/sonar_vert` : ses colonnes sont
  l'ÉLÉVATION (pas l'azimut), et il aurait la même trappe `32FC1 [0,1] → ×255`.

---

## 2. Checks PASS/FAIL — sur un bag court `--test 60` AVANT les 18 min

**Ne régénère PAS le bag complet tant que E1–E4 ne sont pas tous PASS.** Logge les 4 mesures dans
`full_run.log` + une entrée datée au §4.

- **E1 — plan du fan** : sur `/sonar_vert_points` en repère véhicule,
  **PASS si `std(y) < 0.3` m ET `std(x) > 1` m ET `std(z) > 1` m** (le fan est bien dans x-z).
- **E2 — ouverture** : les angles d'élévation `atan2(z, x)` doivent couvrir **±60° ± 3°**.
  PASS si min < −57° et max > +57°.
- **E3 — signe (anti-miroir)** : reprojette en monde via la pose GT. Un écho du **fond devant** doit
  tomber à `z_monde < z_robot` ; un écho de la **surface** à `z_monde > z_robot`.
  **PASS si > 95 % des échos du fond sont sous le robot.** Si inversé → transpose la matrice de
  projection.
- **E4 — structure verticale** : place le ROV face à un mur connu, reprojette en monde.
  **PASS si le mur apparaît comme une ligne VERTICALE** (étalement horizontal < 1 m sur ≥ 5 m de
  hauteur), pas comme des rayons radiaux. **C'est le test qui prouve que la 3D avant est réelle.**

---

## 3. Contrat inchangé (rappel court — ne rien casser)

| Topic | Type | Fréquence | Convention |
|---|---|---|---|
| `/ground_truth` | nav_msgs/Odometry | 20 Hz | ENU, z HAUT (sous l'eau z<0), `frame_id=map` — **évaluation seulement** |
| `/imu` | sensor_msgs/Imu | 20 Hz | repère véhicule, `frame_id=auv0` |
| `/dvl` | geometry_msgs/TwistStamped | 20 Hz | vitesse repère véhicule |
| `/depth` | std_msgs/Float64 | 20 Hz | profondeur POSITIVE vers le bas (= −z) |
| `/sonar` | sensor_msgs/Image 32FC1 | 5 Hz | polaire, range 0.5→40 m, azimut ±60° |
| `/sonar_points` | PointCloud2 xyzi | 5 Hz | `frame_id=auv0` |

**Bruits déjà calibrés — garder** : sonar `AddSigma/MultSigma 0.01`, `RangeSigma 0` (0.05 noie les
échos) ; DVL σ 0.01 m/s ; IMU gyro σ 0.002 rad/s, accel σ 0.02 m/s².
Les échos de murs plafonnent ~0.47 (pas 1.0) — cale les seuils d'intensité en conséquence.

**Pièges à ne pas réintroduire** :
- `RangeMin 0.5 m` : tout objet à < 0.7 m du sonar est invisible → garder ≥ 1.2 m de marge.
- Génération par **teleport** sur trajectoire analytique (pas de PID) → IMU/DVL synthétisés
  analytiquement depuis la même trajectoire. Stamps = temps simulé croissant.
- `octree_min: 0.1` (10 cm) : à 2 cm la génération explose (29 Go de JSON). Ne pas y retoucher.
- Trajectoire : errance aléatoire bornée, **seed 42**, 2 tours, roll = pitch = 0. Inchangée.

---

## 4. Dialogue (tes réponses / blocages → Nathan)

Écris ici : les 4 mesures E1–E4, le bras de levier exact, et comment tu as géré la fréquence
(paramètre natif ou émulation par `RangeMax` + cases de distance).

**Blocage ?** Ne pas improviser une variante silencieuse : note-le ici et demande.

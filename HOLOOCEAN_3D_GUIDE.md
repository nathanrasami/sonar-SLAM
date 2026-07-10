# HOLOOCEAN 3D — Demande en cours : ajouter un 2ᵉ sonar VERTICAL à l'avant

> **Pour** : le collègue HoloOcean + son agent.
> Le bag `holoocean_3d_traj3.bag` actuel est **validé** côté SLAM (ATE 0.03 m, carte 3D GT-free
> à 5.7 cm de la carte GT). **Deux changements, et deux seulement** : le 2ᵉ sonar (§1) et la
> trajectoire (§1.5 : calibration + couverture). Monde PierHarbor, bruits, teleport : INCHANGÉS.
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

Cette config est celle de *Fusing Concurrent Orthogonal Wide-aperture Sonar Images for Dense
Underwater 3D Reconstruction* (McConnell, Martin & Englot, IROS 2020 — le groupe d'origine de
Bruce-SLAM) : deux sonars à axes d'incertitude ORTHOGONAUX observent les mêmes points ; dans le
volume de recouvrement on résout azimut ET élévation → vraie 3D à chaque paire de pings. Nathan
fera cette fusion côté SLAM — d'où deux exigences supplémentaires au §1.2 (stamps, ouvertures).

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
- **Stamps appariés** : `/sonar` et `/sonar_vert` pinguent aux MÊMES instants (paires exactes à
  5 Hz — trivial en teleport analytique). La fusion 3D apparie les pings deux à deux ; un ping
  orphelin est perdu. Le CHECK E5 mesure ça.
- **Déclare au §4 les ouvertures exactes** : ouverture en ÉLÉVATION du sonar SLAM et ouverture en
  AZIMUT de `/sonar_vert` (valeurs de config HoloOcean). Elles définissent l'épaisseur du volume
  de recouvrement où la fusion opère — on ne peut pas les deviner depuis le bag.

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

### 1.5 Trajectoire « traj4 » — calibration puis couverture (REMPLACE l'errance seed 42)

Le fan vertical est FIN en azimut : il ne met en 3D que ce qu'on BALAIE. La couverture vient donc
du MOUVEMENT (yaw-sweeps), pas d'un capteur en plus. Invariants conservés : roll = pitch = 0
partout, teleport analytique, vitesses et bruits de traj3.

**Phase A — calibration, en DÉBUT de bag (~90 s). Le bag court des checks DOIT la contenir.**
- **C1 — statique** (10 s) : immobile face à un mur de quai à ~10 m, à mi-profondeur d'eau.
  Sert aux checks E4 (mur vertical) et E6 (recouvrement des 2 sonars).
- **C2 — yaw-sweep 360°** (~36 s à 10°/s) au même point : le fan vertical balaie tout l'azimut
  comme un phare. Sert au check E7.
- **C3 — ascenseur** (±3 m en z, ~20 s) au même point, cap fixe : le sonar SLAM produit des
  strates à plusieurs profondeurs (3D latérale en couches).

**Phase B — trajectoire principale** : circuit en CARRÉ entre les deux quais, 2 tours :
- tour 1 à z ≈ −4 m, tour 2 à z ≈ −8 m, transitions douces le long des côtés ;
- à CHAQUE coin : pause + **yaw-sweep ±90°** (pivot sur place) avant de prendre le virage —
  c'est ce qui donne la 3D des quais que l'on longe ;
- UNE approche frontale du bateau garé le long d'un quai : s'arrêter à ~8 m, **yaw-sweep ±45°**
  face à lui — le fan vertical voit la coque sur ±60° d'élévation → coupes 3D du bateau.

---

## 2. Checks PASS/FAIL — sur un bag court `--test 150` AVANT les 18 min

Le bag court doit contenir **toute la phase A (§1.5) + ~1 min de carré**.
**Ne régénère PAS le bag complet tant que E1–E7 ne sont pas tous PASS.** Logge les 7 mesures dans
`full_run.log` + une entrée datée au §4.

- **E1 — plan du fan** : sur `/sonar_vert_points` en repère véhicule,
  **PASS si `std(y) < 0.3` m ET `std(x) > 1` m ET `std(z) > 1` m** (le fan est bien dans x-z).
- **E2 — ouverture** : les angles d'élévation `atan2(z, x)` doivent couvrir **±60° ± 3°**.
  PASS si min < −57° et max > +57°.
- **E3 — signe (anti-miroir)** : reprojette en monde via la pose GT. Un écho du **fond devant** doit
  tomber à `z_monde < z_robot` ; un écho de la **surface** à `z_monde > z_robot`.
  **PASS si > 95 % des échos du fond sont sous le robot.** Si inversé → transpose la matrice de
  projection.
- **E4 — structure verticale (pendant C1)** : reprojette le mur en monde.
  **PASS si le mur apparaît comme une ligne VERTICALE** (étalement horizontal < 1 m sur ≥ 5 m de
  hauteur), pas comme des rayons radiaux. **C'est le test qui prouve que la 3D avant est réelle.**
- **E5 — synchro** : chaque stamp de `/sonar` a son jumeau `/sonar_vert`.
  **PASS si 100 % des pings sont appariés à |Δt| < 20 ms.**
- **E6 — recouvrement (pendant C1)** : le mur d'en face apparaît dans LES DEUX images polaires,
  colonnes centrales. **PASS si |range(/sonar) − range(/sonar_vert)| < 0.3 m.**
- **E7 — couverture sweep (pendant C2)** : reprojette `/sonar_vert_points` en monde sur les 36 s.
  **PASS si les échos couvrent > 300° d'azimut** autour du point de pivot (le « phare » balaie).

---

## 3. Contrat du bag CIBLE (topics attendus après le §1)

Table complète : ce que le nouveau bag doit contenir. La colonne « état » dit ce qui change.
Tout ce qui est marqué *inchangé* : **ne rien casser**.

| Topic | Type | Fréquence | Convention | État |
|---|---|---|---|---|
| `/ground_truth` | nav_msgs/Odometry | 20 Hz | ENU, z HAUT (sous l'eau z<0), `frame_id=map` — **évaluation seulement** | inchangé |
| `/imu` | sensor_msgs/Imu | 20 Hz | repère véhicule, `frame_id=auv0` | inchangé |
| `/dvl` | geometry_msgs/TwistStamped | 20 Hz | vitesse repère véhicule, `frame_id=auv0` | inchangé |
| `/depth` | std_msgs/Float64 | 20 Hz | profondeur POSITIVE vers le bas (= −z, bruit σ 0.02 m) | inchangé |
| `/sonar` | sensor_msgs/Image 32FC1 | 5 Hz | polaire 512×512, range 0.5→40 m, azimut ±60° | inchangé, mais **tilt = 0** (§1.3) |
| `/sonar_points` | PointCloud2 xyzi | 5 Hz | `frame_id=auv0` | inchangé |
| `/sonar_vert` | sensor_msgs/Image 32FC1 | 5 Hz | polaire, colonnes = **élévation** ±60°, range 0.5→20 m | 🆕 **NOUVEAU** (§1.2) |
| `/sonar_vert_points` | PointCloud2 xyzi | 5 Hz | **`frame_id=auv0`** | 🆕 **NOUVEAU** (§1.2) |
| `/sonar_tilt` | std_msgs/Float64 | 5 Hz | *(était ±0.26 **rad** = ±14.9°)* | ❌ **SUPPRIMÉ** (§1.3) |
| `/profiler_points` | PointCloud2 xyzi | 5 Hz | *(était `frame_id=auv0`, fan transverse)* | ❌ **SUPPRIMÉ** (§1.4) |

⚠ **Conséquence attendue du tilt = 0, à ne PAS « corriger »** : `/sonar_points` redevient une tranche
plate (`std(z) ≈ 0` intra-message, cf. traj2). C'est normal — la 3D vient désormais de
`/sonar_vert_points`. Ne réintroduis pas le tilt oscillant pour « récupérer » de l'élévation.

**Bruits déjà calibrés — garder** : sonar `AddSigma/MultSigma 0.01`, `RangeSigma 0` (0.05 noie les
échos) ; DVL σ 0.01 m/s ; IMU gyro σ 0.002 rad/s, accel σ 0.02 m/s².
Les échos de murs plafonnent **~0.44** (mesuré sur traj3 : max 0.437, pas 1.0) — cale les seuils
d'intensité en conséquence. Applique les **mêmes bruits** à `/sonar_vert`.

**Pièges à ne pas réintroduire** :
- `RangeMin 0.5 m` : tout objet à < 0.7 m du sonar est invisible → garder ≥ 1.2 m de marge.
- Génération par **teleport** sur trajectoire analytique (pas de PID) → IMU/DVL synthétisés
  analytiquement depuis la même trajectoire. Stamps = temps simulé croissant.
- `octree_min: 0.1` (10 cm) : à 2 cm la génération explose (29 Go de JSON). Ne pas y retoucher.
- Trajectoire : **REMPLACÉE par traj4 (§1.5)** — phase A calibration (C1/C2/C3) + carré 2 tours
  à z −4/−8 m, yaw-sweeps aux coins + face au bateau. roll = pitch = 0 : toujours vrai.

---

## 4. Dialogue (tes réponses / blocages → Nathan)

Écris ici : les 7 mesures E1–E7, le bras de levier exact, les DEUX ouvertures (élévation du
sonar SLAM, azimut de `/sonar_vert`), et comment tu as géré la fréquence (paramètre natif ou
émulation par `RangeMax` + cases de distance).

**Blocage ?** Ne pas improviser une variante silencieuse : note-le ici et demande.

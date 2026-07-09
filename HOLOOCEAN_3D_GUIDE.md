
# HOLOOCEAN 3D — Demande traj 3 (v4, 07-08) : PierHarbor + errance aléatoire + tilt sonar

> **Pour** : le collègue HoloOcean + son agent (tout est
> explicite ci-dessous, avec code de référence et critères PASS/FAIL binaires).
> Les bags précédents (traj 1/2, couloir `Bruce_slam_nathan`) sont validés côté SLAM
> (ATE 0.03 m ×2 ; carte 3D GT-free à 0.021 m de la carte GT) — merci, ce doc ne
> revient pas dessus. Historique complet récupérable via `git log` si besoin.

---

## 1. Contrat topics à CONSERVER (sauf les 3 changements du §2)

Mêmes conventions que les bags précédents, sauf mention contraire au §2 :

| Topic | Type | Fréquence | Convention |
|---|---|---|---|
| `/ground_truth` | nav_msgs/Odometry | 20 Hz | ENU, z HAUT (sous l'eau z<0), quaternion xyzw, `frame_id=map` |
| `/imu` | sensor_msgs/Imu | 20 Hz | repère véhicule (x avant, y gauche, z haut), `frame_id=auv0` |
| `/dvl` | geometry_msgs/TwistStamped | 20 Hz | vitesse repère véhicule |
| `/depth` | std_msgs/Float64 | 20 Hz | profondeur POSITIVE vers le bas (= −z) |
| `/sonar` | sensor_msgs/Image 32FC1 | 5 Hz | polaire, lignes=range 0.5→40 m, colonnes=azimut ±60° |
| `/sonar_points` | PointCloud2 xyzi | 5 Hz | voir §2.3 (repère change) |
| `/profiler_points` | PointCloud2 xyzi | 5 Hz | voir §2.3 (repère change) |

Bruits déjà calibrés à garder : sonar `AddSigma/MultSigma 0.01, RangeSigma 0` (0.05
noie les échos — mesuré) ; DVL σ 0.01 m/s ; IMU gyro σ 0.002 rad/s, accel σ 0.02 m/s².
Échos de murs plafonnent ~0.47 (pas 1.0) — seuils d'intensité à caler en conséquence.
Stamps = temps simulé croissant.

**Pièges à ne pas réintroduire** :
- `RangeMin 0.5 m` + champ proche : tout objet à < 0.7 m du sonar est invisible —
  garder ≥ 1.2 m de marge à toute structure dans la nouvelle trajectoire (§2.1).
- IMU sans accélérations propres (gravité seule projetée + bruit) — connu, pas bloquant.
- Génération par **teleport** sur trajectoire analytique (pas de PID/dynamique) →
  IMU/DVL synthétisés analytiquement à partir de la même trajectoire.

---

## 2. LA DEMANDE v4 — trois changements

Base de départ = le générateur `gen_bag_3d.py` existant. On modifie ces trois points,
tout le reste (§1) est inchangé.

### 2.1 Changement 1 — trajectoire « errance aléatoire bornée »

Remplace la sinusoïde de profondeur : excursions **ALÉATOIRES dans les 4 directions**
(gauche/droite = latéral, haut/bas = profondeur). Exemple voulu : « 0.7 m gauche,
0.1 m haut, 0.6 m bas, 0.7 m haut, 0.6 m droite » — ordre et amplitudes au hasard,
toujours dans les marges du couloir/quai :

- **Latéral** : offset `lat(s)` ⊥ au chemin, tiré au hasard, borné selon la largeur
  réelle de la zone navigable de PierHarbor (à mesurer, §2.4) − marge ≥ 1.2 m.
- **Vertical** : `z(s)` tiré au hasard dans une plage à définir selon la profondeur
  du site (marge surface/fond ≥ 1 m).
- **Lisse** : une « décision » aléatoire tous les ~8 m d'abscisse, interpolée
  **PCHIP** (pas d'à-coups, pas d'overshoot hors bornes — contrairement à une spline
  cubique classique qui déborde entre les nœuds).
- **roll = 0 et pitch = 0 PARTOUT** (« grande route ») ; yaw = tangente du chemin réel
  (médiane+offset), |dyaw/dt| < 30°/s.
- **Boucle fermée** : offsets forcés à 0 aux deux extrémités du périmètre ; 2 tours ;
  même tirage aux 2 tours par défaut (plus simple pour les loops SLAM).
- **SEED loggée** (reproductibilité obligatoire) dans `validation_3d/full_run.log`.

```python
# ─── errance aléatoire bornée (à intégrer dans gen_bag_3d.py) ──────────────
import numpy as np
from scipy.interpolate import PchipInterpolator

SEED   = 42      # ⚠ logger la valeur ; changer = autre trajectoire
N_MAX  = 1.0     # offset latéral max (m) — À AJUSTER selon la largeur réelle mesurée
Z_MIN, Z_MAX = -5.2, -1.8   # À AJUSTER selon la profondeur réelle du site
L_SEG  = 8.0     # distance moyenne entre 2 décisions aléatoires (m)

def offsets_aleatoires(perim, seed=SEED):
    """lat(s), z(s) aléatoires lisses, bornés, nuls aux extrémités (fermeture)."""
    rng = np.random.default_rng(seed)
    n = max(8, int(perim / L_SEG))
    s_nodes = np.linspace(0.0, perim, n + 1)
    lat = rng.uniform(-N_MAX, N_MAX, n + 1)
    z   = rng.uniform(Z_MIN, Z_MAX, n + 1)
    lat[0] = lat[-1] = 0.0
    z[0]  = z[-1]  = 0.5 * (Z_MIN + Z_MAX)
    return PchipInterpolator(s_nodes, lat), PchipInterpolator(s_nodes, z)

def pose_at_v4(t, V_FWD, perim, chemin_median, f_lat, f_z):
    """chemin_median(s) -> (c [x,y], t_hat unitaire) : fonction existante du générateur.
    Même tirage aux 2 tours : s = (V_FWD*t) % perim."""
    s = (V_FWD * t) % perim
    c, t_hat = chemin_median(s)
    n_hat = np.array([-t_hat[1], t_hat[0]])          # normale GAUCHE du chemin
    p = c + f_lat(s) * n_hat
    s2 = (s + 0.5) % perim                             # yaw = tangente réelle (diff finie 0.5 m)
    c2, t2 = chemin_median(s2)
    p2 = c2 + f_lat(s2) * np.array([-t2[1], t2[0]])
    yaw = np.arctan2(p2[1] - p[1], p2[0] - p[0])
    return p[0], p[1], float(f_z(s)), yaw            # roll = pitch = 0
# ⚠ DVL : vz et la vitesse latérale ne sont PLUS nuls → v_monde par différence
#   finie centrale sur pose_at_v4 (dt=0.05), puis v_vehicule = R^T · v_monde.
```

### 2.2 Changement 2 — vraie 3D par le SONAR PRINCIPAL : tilt oscillant

L'ImagingSonar ne reste plus à plat : son plan de scan **oscille en pitch** →
information d'élévation intra-ping → le SLAM lui-même (pas seulement la carte
offline) voit de la 3D.

- `tilt(t) = 15° · sin(2π · t / 10 s)` (amplitude 15°, période 10 s).
- **Implémentation préférée** : si HoloOcean permet de changer la rotation du
  capteur au tick, `rotation = [0, tilt_deg(t), 0]` (vrai rendu incliné). Si
  impossible techniquement, **le dire avant de coder une variante alternative**
  (§4 dialogue).
- **Nouveau topic `/sonar_tilt`** (`std_msgs/Float64`, radians, même stamp que
  `/sonar`) — indispensable : sans lui le SLAM ne peut pas projeter les pings.
- Projection : `p_capteur = Ry(tilt) · r·[cos a, −sin a, 0]`, avec en ENU véhicule
  (x avant, y gauche, z haut) : `Ry(θ): x' = x·cosθ + z·sinθ ; z' = −x·sinθ + z·cosθ`.
  ⚠ Vérifier le signe : tilt > 0 doit donner des échos à z > z_robot.

### 2.3 Changement 3 — points en repère VÉHICULE (plus de GT cachée)

`/sonar_points` et `/profiler_points` : **`frame_id = "auv0"`**, points en repère
capteur/véhicule, SANS transformation monde (avant, la pose GT était cachée dans
les PointCloud2 — obligeait à dé-projeter côté SLAM ; un vrai sonar logge en
repère capteur). `/ground_truth` reste publié tel quel (évaluation seulement).
Le profiler garde son `R_roll(90°)` avant sortie (repère véhicule quand même).

### 2.3bis ⚠ CHANGEMENT CRUCIAL (08-08) — profiler en TRANSVERSE, pas vers l'avant

**Constat sur traj3** : le profiler y était monté pour balayer le plan **x-z du
véhicule (avant/haut-bas)** — mesuré côté SLAM : std x=8.9, y=0.00, z=12.7 m. Résultat :
il regarde VERS L'AVANT, voit le fond marin devant lui → la reconstruction 3D fait des
**« fans » radiaux** et on ne peut PAS reconstruire une carte propre.

**La reconstruction 3D propre (méthode validée sur grottes réelles, `analysis/caves_3d.py`)
exige un profiler TRANSVERSE** : le fan doit balayer le plan **y-z du véhicule
(GAUCHE-DROITE + haut-bas), PERPENDICULAIRE au sens de marche** — comme un vrai profiler
de coupe (SeaKing sur le dataset grotte). Chaque ping = une **section transverse** de
l'environnement ; empilées le long de la trajectoire SLAM → volume 3D net.

- **À faire** : monter le ProfilerVert pour que son fan soit dans le plan **y-z véhicule**
  (perpendiculaire à l'axe d'avance x). En repère véhicule, un point de paroi doit être
  `(0, r·cosφ, r·sinφ)` (x≈0), PAS `(r·cosφ, 0, r·sinφ)`.
- **Critère PASS** : sur `/profiler_points` en repère véhicule, **std(x) ≈ 0** et std(y),
  std(z) grands (l'inverse de traj3). Un ping affiché = un arc TRANSVERSE (coupe), pas un
  éventail vers l'avant.
- Côté SLAM, `caves_3d.py` reconstruit alors directement (1 faisceau → retour le plus fort
  = paroi, projeté le long de la traj) — c'est LA méthode propre, déjà écrite.

### 2.3ter ⚠ ITÉRATION 2 (09-08) — profiler : couverture + surface (traj3 régénéré à corriger)

Le bag traj3 « transverse » est meilleur mais la carte reconstruite a **2 défauts mesurés
côté SLAM** (à corriger dans `gen_bag_3d_v4.py`). Pour ton agent : chaque défaut a sa cause,
son fix, et un **check PASS/FAIL automatique** à lancer sur le bag régénéré (tu as la GT).

**Défaut A — le fan du profiler vise trop VERS LE HAUT.**
- Mesuré : **58 % des points `/profiler_points` retombent à z_monde > 0** (au-dessus de la
  surface) → le profiler gaspille la majorité de ses faisceaux sur le navire/surface au lieu
  des murs immergés. Résultat : carte noyée dans une bouillie jaune, structures profondes rares.
- Cause probable : après le mount transverse, le boresight du fan (±60° azimut) est incliné
  vers le haut, pas horizontal.
- **Fix** : orienter le fan pour qu'il balaye du HORIZONTAL vers le BAS (côté + fond), pas vers
  le haut. Concrètement, ajuster la rotation de montage du ProfilerVert (le 3ᵉ angle) et/ou
  réduire l'élévation, puis **vérifier par le check ci-dessous** (ne pas deviner : mesurer).
- **CHECK A (PASS/FAIL)** : sur le bag, reprojeter `/profiler_points` en MONDE via la pose GT
  du même stamp ; **PASS si < 20 % des points ont z_monde > 0** (aujourd'hui : 58 % = FAIL).

**Défaut B — couverture asymétrique (profiler regarde UN seul côté).**
- Mesuré : le côté gauche du parcours est capté en profondeur (jusqu'à −35 m), le côté droit
  reste à z≈0 (seulement le haut). Un profiler transverse fixe ne voit qu'un bord.
- **Fix (au choix)** : (1) **DEUX ProfilerVert**, un de chaque côté (fan y-z gauche ET droite,
  nouveau topic `/profiler_points_r` ou fusionnés) ; OU (2) garder un seul profiler mais faire
  un **parcours qui longe chaque structure du bon côté** (le côté que le profiler regarde).
  Option (1) recommandée = plus simple et symétrique.
- **CHECK B (PASS/FAIL)** : découper la zone en 2 (gauche/droite du centre du parcours) ;
  **PASS si les DEUX moitiés ont des points profonds** (z_monde < −15 m, au moins quelques
  milliers chacune). Aujourd'hui : une seule moitié = FAIL.

**Défaut C — le signe z du profiler n'a JAMAIS été vérifié** (seul le tilt du sonar principal
l'a été, §2.6.4).
- **CHECK C (PASS/FAIL)** : prendre un ping quand le robot est au-dessus du fond (fond connu
  par la GT/zone) ; reprojeter en monde ; **PASS si les retours du fond tombent à z_monde <
  z_robot** (sous le robot), pas au-dessus. Si inversé → transposer la matrice de projection
  du profiler (comme le fix R_y du sonar principal).

**Rappel méthode de travail (pour ton agent, §2.6.8)** : bag court `--test 60` d'abord, lancer
CHECK A/B/C dessus, n'écrire le bag complet QUE si les 3 sont PASS, et **logger les 3 chiffres
mesurés** dans `full_run.log` + une entrée datée au §3 dialogue. Ne PAS régénérer les 18 min si
un check court échoue.

### 2.3quater ⚠ ITÉRATION 3 (09-09) — `/profiler_points` a l'axe **y INVERSÉ** (miroir)

Le bag traj3 §2.3ter est bon (A/B/C PASS), la carte 3D marche — MAIS côté SLAM on a trouvé
que **`/profiler_points` est en miroir sur l'axe y** (gauche↔droite) par rapport à la
convention véhicule attendue (x avant, **y GAUCHE**, z haut, ENU — la même que `/imu`, `/dvl`,
`/sonar_points`). Conséquence : les structures verticales imagées par le profiler (murs/treillis
des quais) sont projetées **du mauvais côté** → elles apparaissent AU CENTRE, entre les 2 quais,
alors que la scène PierHarbor n'a rien entre les quais.

**Preuve mesurée (côté SLAM, indiscutable)** :
- Structures hautes du profiler reconstruites à x≈**+2 / +38** ; vrais quais (sonar horizontal
  `/sonar_points`) à x≈**−10 / +59**. Décalage = image miroir à travers la trajectoire.
- Persiste même en reprojetant avec la **pose GT** → ce n'est ni le SLAM ni la pose.
- Un ping isolé près d'un quai : le profiler place le mur **6 m à DROITE** du véhicule alors que
  le quai réel est **6 m à GAUCHE**.
- Test décisif : **négation de `y`** des points profiler → les structures retombent PILE sur les
  quais réels. Donc `y_profiler = −y_véhicule`.

**Fix (côté générateur)** : dans le transform qui met les points du profiler en repère `auv0`,
**l'axe y est inversé**. Très probablement la matrice `R_MOUNT_DOWN =
[[0,0,1],[0,1,0],[-1,0,0]]` (ou la composition avec la rotation native du capteur) a une
**mauvaise handedness sur y**. Corrige le signe de la 2ᵉ composante (ex. `R_MOUNT_DOWN =
[[0,0,1],[0,-1,0],[-1,0,0]]`, à VÉRIFIER par le check ci-dessous — ne pas deviner, mesurer).
Ne touche PAS aux autres topics (`/sonar_points`, `/imu`, `/dvl` sont corrects).

**CHECK D (PASS/FAIL)** — sur le bag régénéré, reprojette `/profiler_points` ET `/sonar_points`
en MONDE via la pose GT du même stamp ; garde les points « hauts » (structure, z_monde > fond+3 m)
de chaque source ; **PASS si les structures hautes du profiler tombent à < 3 m des murs de
`/sonar_points`** (mêmes positions de quai), et NON décalées vers le centre. Aujourd'hui (avant
fix) : profiler à +2/+38 vs sonar à −10/+59 = FAIL.

**En attendant** : côté SLAM on applique un patch (`carte_3d.py` : `pts[:,1] = -pts[:,1]`) pour
avoir une carte juste tout de suite. Quand tu livres un bag qui passe CHECK D, **dis-le au §3** :
on retirera le patch (sinon double inversion). Même classe de bug que le miroir latéral cmd_vel
déjà vu sur ce projet.

### 2.3quinquies ⭐ ITÉRATION 4 (09-09) — AJOUTER un 2ᵉ sonar AVANT, fan VERTICAL (le « + »)

**Objectif de Nathan** : cartographier en 3D **ce que le ROV voit DEVANT** (pas le fond sous lui).
Le profiler transverse (§2.3ter) cartographie ce qu'on *longe* — ce n'est pas le besoin.

**La demande, en une phrase** : garder le sonar SLAM **inchangé**, et ajouter **à côté, à l'avant,
un DEUXIÈME sonar identique** (mêmes paramètres : portée 0.5–40 m, ouverture 120°, mêmes bruits),
simplement **tourné de 90° autour de l'axe d'avance x** → `rotation = [90, 0, 0]` (roll 90°).

Vu de face (le long de x) : le sonar SLAM balaie en `−` (plan x-y, gauche↔droite) et le nouveau
balaie en `|` (plan **x-z**, haut↔bas). Les deux regardent devant → un `+`. L'ouverture de 120°
du nouveau est donc en **élévation** (±60° haut/bas) ; son ambiguïté est en **azimut** (les échos
hors axe sont rabattus sur le plan vertical central — c'est connu et accepté).

**C'est le mount v3 que tu as déjà codé** (`R_MOUNT_PROF`, rotation `[90,0,0]`, bags traj1/traj2
du couloir) — pas de nouvelle géométrie à inventer. Vérifié côté SLAM sur `holoocean_3d_traj2.bag` :
`std(x)=7.19  std(y)=0.00  std(z)=1.74` → fan bien dans le plan x-z, et la reconstruction montre
des **murs verticaux nets de z=−11 à +6 m** (vraie 3D prouvée). C'est exactement ce qu'on veut.

**Livrables** :
| Topic | Type | Fréquence | Convention |
|---|---|---|---|
| `/sonar_vert` | sensor_msgs/Image 32FC1 | 5 Hz | polaire, lignes=range, colonnes=**élévation** ±60° |
| `/sonar_vert_points` | PointCloud2 xyzi | 5 Hz | **repère véhicule, `frame_id=auv0`** |

- Repère véhicule = **x avant, y GAUCHE, z HAUT** (ENU), comme `/imu`, `/dvl`, `/sonar_points`.
- 🚨 **`frame_id = "auv0"` OBLIGATOIRE, points en repère VÉHICULE.** Si tu publies en repère monde
  (`frame_id=map`, comme les vieux bags v3/traj2), la **pose GT est cuite dans les points** et la
  carte 3D n'est plus GT-free → **résultat inutilisable pour le stage** (contrainte absolue :
  capteurs embarqués seulement, la GT ne sert QU'À l'évaluation). C'est l'erreur exacte de traj2.
- ⚠ **Attention au miroir** (§2.3quater) : vérifie le SIGNE de z, ne le suppose pas.
- Position : à côté du sonar SLAM (avant), avec un **bras de levier déclaré** (voir « réalisme » plus bas).
- `/sonar` (SLAM) **inchangé**. Recommandation : mets son **tilt à 0** (fixe) — le tilt oscillant
  de §2.2 devient inutile puisque l'élévation vient maintenant du sonar vertical, et un sonar SLAM
  fixe est plus proche du vrai Bruce. (Si tu le gardes, garde `/sonar_tilt`.)
- Le profiler transverse (§2.3ter) : **optionnel**, garde-le si c'est gratuit (il donne le fond),
  mais ce n'est plus la priorité.

**Checks PASS/FAIL (à lancer sur un bag court `--test 60` AVANT le bag complet)** :
- **CHECK E1 — plan du fan** : sur `/sonar_vert_points` en repère véhicule,
  **PASS si `std(y) < 0.3` m ET `std(x) > 1` m ET `std(z) > 1` m** (le fan est bien dans x-z).
  (Le transverse §2.3ter donnait l'inverse : `std(x)=0.00`. Ne pas confondre.)
- **CHECK E2 — ouverture** : les angles d'élévation `atan2(z, x)` des points doivent couvrir
  **±60° ± 3°**. PASS si min < −57° et max > +57°.
- **CHECK E3 — signe (anti-miroir)** : reprojette en monde via la pose GT. Un écho du **fond
  devant** doit tomber à `z_monde < z_robot` ; un écho de la **surface** à `z_monde > z_robot`.
  **PASS si > 95 % des échos du fond sont sous le robot.** Si inversé → transposer la matrice
  de projection (comme le fix R_y du sonar principal, §3 entrée 08-07).
- **CHECK E4 — structure verticale** : place le ROV face à un mur connu ; reprojette en monde.
  **PASS si le mur apparaît comme une ligne VERTICALE** (étalement horizontal < 1 m sur ≥ 5 m de
  hauteur), pas comme des rayons radiaux. C'est le test qui prouve que la 3D avant est réelle.

**Ne PAS régénérer les 18 min** tant que E1–E4 ne sont pas tous PASS sur le bag court. Logge les
4 mesures dans `full_run.log` + une entrée datée au §3.

**⭐ Contrainte de Nathan : le montage doit être RÉALISTE — si la 3D marche, on la fera en vrai.**
Donc on ne se contente pas de « le simulateur ignore le crosstalk » : on **choisit un matériel réel
où le crosstalk n'existe pas**, pour que la simu soit *fidèle* et non *optimiste*.

- **Deux têtes à fréquences porteuses DIFFÉRENTES** → elles pinguent simultanément sans se brouiller,
  exactement comme le simulateur le fait déjà.
  - `/sonar` (SLAM, horizontal) : classe **~750 kHz**, portée **0.5–40 m** (inchangé).
  - `/sonar_vert` (vertical 3D) : classe **~1.2 MHz**, portée **0.5–20 m**, **résolution en distance
    plus fine**. C'est le compromis physique réel (haute fréquence = meilleure résolution, moins de
    portée) et c'est exactement ce qu'on veut pour la 3D de ce qu'on voit devant.
  - Si HoloOcean n'expose pas la fréquence : **émule-la** par `RangeMax = 20 m` + un nombre de cases
    de distance plus élevé (résolution plus fine) sur `/sonar_vert`, et **note-le au §3**.
- **Bras de levier déclaré** : IRL deux têtes ne peuvent pas être au même point. Monte `/sonar_vert`
  avec un **décalage constant d'environ 0.15 m** par rapport au sonar SLAM, et **déclare la valeur
  exacte** (§3 + `full_run.log`). Ne prétends pas qu'elles sont confondues.
- **Tilt du sonar SLAM = 0** : un tilt mécanique oscillant est une vraie complication matérielle,
  et l'élévation vient désormais du sonar vertical. Supprime-le (garde `/sonar_tilt` seulement si
  tu gardes le tilt).

**Conflits entre les 2 sonars ? Non — ni en simu, ni IRL avec ce montage.** HoloOcean rend chaque
capteur par lancer de rayons indépendant contre l'octree ; il ne simule PAS la propagation
acoustique entre capteurs. Preuve : le bag traj3 actuel fait DÉJÀ tourner deux sonars simultanés
(`/sonar` + profiler, 5 Hz chacun) sans aucune pollution croisée. Et avec des porteuses différentes
(ci-dessus), il n'y en aurait pas non plus en vrai. ⚠ Si tu gardais la MÊME fréquence sur les deux,
il y aurait un vrai crosstalk IRL (le simulateur ne le montrerait pas) → à éviter.

**Coût de rendu** : un sonar imageur de plus = du ray-cast en plus (tu as déjà bataillé avec
l'octree). **Recommandé : SUPPRIME le profiler transverse** et garde `{imageur SLAM + sonar
vertical}` → même nombre de capteurs qu'aujourd'hui, **coût inchangé**.

**Piège côté SLAM (pour info)** : notre bridge est câblé en dur sur `/sonar`, donc `/sonar_vert`
n'ira jamais polluer le SLAM. Mais ne réutilise pas ce bridge pour `/sonar_vert` : ses colonnes
sont l'ÉLÉVATION (pas l'azimut), et il aurait la même trappe `32FC1 [0,1] → ×255`.

### 2.3quinquies ⚠ ITÉRATION 4 (09-09) — VOIR AUTOUR du ROV, pas seulement dessous

**Constat côté SLAM** : le profiler actuel (boresight droit vers le BAS, fan ±60°) n'image
qu'un **coin de 120° vers le bas** → il voit le fond + le bas des murs, mais PAS l'anneau à la
profondeur du ROV. Résultat : un **trou vertical** entre la trajectoire (profondeur ROV) et les
structures (en dessous), et les objets à hauteur du ROV hors trajectoire (ex. la coque d'un
bateau) sont invisibles au profiler. Objectif : que le profiler voie **autour** du véhicule
(murs jusqu'à la hauteur du ROV, idéalement la section transverse complète qui enveloppe la traj,
comme le SeaKing du dataset grottes). **En 2 phases (fais la phase 1 ET valide-la AVANT la 2)**.

**PHASE 1 (A) — élargir le fan à ~180°** (change UNE variable) :
- Garde le mount transverse boresight-bas (§2.3quater, y à corriger) ; augmente l'**ouverture
  azimutale** du ProfilerVert de **120° → ~180°** (±90° dans le plan transverse y-z).
- ⚠ **À vérifier** : HoloOcean plafonne peut-être l'azimut de l'`ImagingSonar`/`ProfilingSonar`
  (souvent ~120-170°). Si le max est < 180°, prends le max possible et **dis-le au §3** (ne force
  pas une valeur refusée en silence).
- Ça fait couvrir **horizontale-gauche → fond → horizontale-droite** → les 2 murs de quai
  jusqu'à la profondeur du ROV + le fond. Le trou se referme pour la moitié basse.
- **CHECK E (PASS/FAIL)** : reprojette `/profiler_points` en monde via la pose GT ; **PASS si des
  points de STRUCTURE (murs, pas le fond) atteignent la profondeur du ROV** (z_structure ≳ z_ROV −
  1 m quelque part le long du parcours), c.-à-d. plus de trou vertical entre la traj et le haut
  des structures. Vérifie aussi que CHECK D (pas de miroir y) tient toujours.

**PHASE 2 (B) — profiler ROTATIF (le vrai « autour », rendu tunnel grottes)** :
- Fais **tourner le fan autour de l'axe d'avance x** du véhicule au fil du temps (balayage
  mécanique, comme un vrai SeaKing) → sur quelques secondes il couvre le **cercle transverse
  complet (360°)** : mur gauche, haut, mur droit, fond = section qui **enveloppe** la trajectoire.
- La rotation autour de x **préserve x≡0** (le fan reste dans le plan y-z, il tourne dedans) →
  sors les points en repère `auv0` avec la rotation DÉJÀ appliquée (comme aujourd'hui) : côté SLAM
  rien ne change, on compose avec la pose. Publie quand même l'angle de rotation
  (`/profiler_roll`, std_msgs/Float64, même stamp) au cas où.
- ⚠ Pièges (on vient d'en vivre un) : le **signe** de la rotation et de y doivent rester cohérents
  (sinon miroir/tourbillon). Valide sur un bag court AVANT les 18 min.
- **CHECK F (PASS/FAIL)** : sur une portion droite du parcours, la section transverse reconstruite
  doit **entourer** la trajectoire (points au-dessus ET en dessous ET des 2 côtés du z_ROV), et la
  coque du bateau (hors trajectoire, à hauteur ROV) doit apparaître dans `/profiler_points` (plus
  seulement dans le sonar horizontal).

**Pourquoi A d'abord** : A valide la géométrie fan-large sur un capteur STATIQUE (pas de facteur
timing de rotation) et sert de repli si la rotation de B est instable ; B n'ajoute alors qu'UNE
variable (la rotation) sur une base saine. Ne saute pas A.

### 2.4 Monde cible : **PierHarbor** (officiel HoloOcean, pas de cooking)

Package `Ocean`, scénario de base `PierHarbor-HoveringImagingSonar` (à surcharger
avec les capteurs du §1/§2.2) — port + quai sur pilotis, jumeau simulé de la
mission réelle Aracati (quai en T).

- À refaire pour ce monde (même méthode que pour le couloir) : sonder la zone
  navigable au sonar + GT, définir le parcours (rectangle ou longée du quai) et
  les bornes `N_MAX`/`Z_MIN`/`Z_MAX` du §2.1 en conséquence.
- **Contrôle** : teleport sur trajectoire analytique (méthode déjà validée) — PAS
  le contrôleur PD intégré de l'HoveringAUV pour ces bags (dérive PID = IMU/DVL
  synthétiques faux). Le PD existe pour plus tard si on veut de la dynamique réaliste.

### 2.5 Livrables attendus

| Fichier | Contenu |
|---|---|
| `holoocean_3d_traj3.bag` | PierHarbor + errance aléatoire + tilt sonar + profiler + `/sonar_tilt`, points en repère véhicule, 2 tours, seed loggée |
| (option, si temps) `holoocean_3d_traj3b.bag` | idem, seed différente |

### 2.6 Ordre de travail imposé + critères PASS/FAIL

1. Bag court (`--test 60`) d'abord, puis les checks 2-6 ci-dessous.
2. **Enveloppe** : distance à toute structure ≥ 1.2 m sur toutes les poses GT ;
   |dyaw/dt| < 30°/s.
3. **Errance** : std(lat) > 0.4 m ET std(z traj) > 0.8 m ET au moins 6 extrema
   locaux de z par tour (pas une sinusoïde triviale).
4. **Tilt** : `/sonar_tilt` présent, amplitude ±15°±1°, période 10 s ±0.5 ;
   std(z) INTRA-message des points du sonar principal > 0.5 m sur les pings à
   |tilt| > 8° ; signe vérifié (tilt>0 → échos au-dessus du z robot).
5. **Repères** : `frame_id == "auv0"` sur les 2 topics de points ; un point de
   mur reprojeté monde via la pose GT du même stamp doit tomber à < 0.3 m du
   plan du mur connu.
6. **Fermeture** : ‖p(fin tour) − p(début)‖ < 2 m ; roll = pitch = 0 exact.
7. Si 2-6 PASS : générer les 2 tours complets, relancer les checks + PNG de
   frames sonar, tout logger dans `full_run.log`.
8. Blocage (API rotation au tick indisponible, signe ambigu…) → **ne pas
   improviser une variante silencieuse** : noter au §4 dialogue et demander.

### Suggestions optionnelles (si le reste est PASS et qu'il reste du temps)

- Bruits « réalistes v2 » : DVL σ 0.01→0.02 m/s, gyro σ 0.002→0.005 rad/s (dérive
  DR visible → les loops SLAM deviennent enfin utiles).
- IMU : ajouter les accélérations propres (centripète + heave).
- Un 3ᵉ tour à vitesse différente (0.5 m/s) : robustesse temporelle.

---

## 3. Dialogue (réponses/blocages du collègue → Nathan)

### 08-07 12:27 — LIVRÉ : `holoocean_3d_traj3.bag` VALIDE (tous checks §2.6 PASS)

**Zone navigable mesurée (PierHarbor, sonde auto)** : anneau rectangulaire
centre `(495, -647)`, chemin médian **54 × 42 m** (près du spawn officiel du
scénario), clearance min **5.0 m**, fond le plus haut **-18.5 m**. Errance :
`N_MAX = 1.2 m`, z ∈ [-11.6, -2.3]. **SEED = 42** (loggée). 2 tours, 1058 s sim,
5289 pings, fermeture **0.00 m**.

**Checks finaux mesurés** : std(z traj) = 2.31 m, 31 extrema z ; tilt 15.0° /
10.00 s ; std(z) intra-ping médiane **0.84 m** sur 2830 pings |tilt|>8° ;
**signe : 100 %** ; frames `auv0` sur les 2 topics points ; max |dyaw/dt| = 6.7 °/s.

**Choix/écarts à connaître (§2.6.8)** :
1. **Signe pitch UE inversé** : `rotate([0, +30, 0])` incline le plan vers le
   BAS (vérifié : fond à 10.5 m sous le robot apparaît à ~18 m ≈ 10.5/sin30°).
   La commande envoyée est donc `-tilt`. `rotate()` fonctionne, latence ~3 ticks
   (0.15 s = 5° de phase sur la période 10 s, non corrigée).
2. **Projection** : la formule Ry du §2.2 du guide donne z<0 pour tilt>0 (mesuré
   0 % au check signe) — on utilise sa **transposée** ; `/sonar_tilt` publie le
   tilt « physique » (positif = plan vers le haut), cohérent avec la projection.
3. **Octree 10 cm** (`octree_min: 0.1`) au lieu du défaut 2 cm : la génération
   2 cm avait atteint 29 Go de JSON sans finir ; 10 cm suffit (r_res sonar
   7.7 cm). Si tu régénères : le cache est sous
   `%LOCALAPPDATA%/holoocean/2.3.0/worlds/Ocean/.../Octrees/PierHarbor/`.
4. **Yaw** calculé sur une base ±0.25 m (le code §2.1 du guide, base 0.5 m,
   était la bonne idée : une base mm amplifie la courbure PCHIP en jitter).
5. Bag test intermédiaire : seuil intra-ping 0.25 m (segment droit unique) ;
   le bag complet est bien validé au seuil contractuel 0.5 m.
6. Suggestions optionnelles (bruits v2, accels propres, 3ᵉ tour) : non faites,
   restent dispo dans `gen_bag_3d_v4.py` si besoin (`--seed N` pour traj3b).

### 09-07 11:09 — FIX §2.3bis appliqué : `holoocean_3d_traj3.bag` RÉGÉNÉRÉ (profiler transverse)

Cause confirmée par un test dédié avant de toucher au générateur : avec l'ancien
mount (rotation capteur `[90,0,0]`, celui de v3/couloir), le profiler garde son
boresight aligné sur l'axe d'avance du véhicule — correct pour un couloir à cap
fixe, mais en errance aléatoire (cap qui varie sans cesse) ça donne un fan qui
« suit » toujours l'avant, d'où les éventails radiaux constatés.

**Fix** : rotation capteur `[90, 0, 90]` (ajout d'un yaw 90° pour rediriger le
boresight du profiler vers le côté, plan y-z du véhicule) + nouvelle matrice de
projection locale au script v4 :
```python
R_MOUNT_TRANSVERSE = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)
```
(le `R_MOUNT_PROF` de `gen_bag_3d.py` v3 n'est pas touché — toujours utilisé par
les bags traj1/traj2 du couloir, cap fixe, où il est correct.)

**Validation en 2 temps** :
1. Test isolé sur couloir `Bruce_slam_nathan` (murs à distance connue, ~2.4 m) :
   `std(x)=0.00` exact, `std(z)` étalé ±5.3 m — mount confirmé avant de relancer
   tout le pipeline PierHarbor (économise le temps de rendu si ç'avait été faux).
2. Critère `§2.3bis` ajouté à `check_bag_v4.py` (`std(x)<0.3`, `std(y)>0.5`,
   `std(z)>0.5`) — **mesuré sur le bag complet régénéré** :
   `std(x)=0.00 std(y)=8.67 std(z)=5.48 m` → **PASS**.

Tous les autres critères §2.6 restent PASS (zone, seed=42, errance, tilt,
signe, repères, cinématique — inchangés, cf. entrée du 08-07 ci-dessus).
`holoocean_3d_traj3.bag` livré est donc la version corrigée, prête pour
`caves_3d.py`.

### 09-07 16:xx — FIX §2.3ter appliqué : `holoocean_3d_traj3.bag` RÉGÉNÉRÉ (profiler boresight vers le bas)

Le mount transverse (fix précédent) laissait le boresight **horizontal** —
symétrique autour de l'horizontale, donc la moitié de l'azimut regardait
« vers le haut » (défaut A : 58 % des points au-dessus de la surface) et
`vehicule_y = r·cos(a)` garde toujours le même signe sur ±60°, donc un seul
côté du véhicule était couvert (défaut B).

**Fix géométrique** : boresight redirigé **droit vers le bas** au lieu
d'horizontal. Rotation capteur `[90, 90, 90]` (roll, pitch, yaw) — dérivation :
avec ce mount, `vehicule = (0, -r·sin(a), -r·cos(a))` pour `a ∈ [-60°, 60°]`.
Deux conséquences immédiates de la formule :
- `vehicule_z = -r·cos(a) < 0` **toujours** (`cos(a) ≥ cos(60°) = 0.5`) →
  plus aucun retour au-dessus de l'horizontale (fixe A).
- `vehicule_y = -r·sin(a)` **change de signe** selon `a` → un seul capteur
  couvre les deux côtés (fixe B, pas besoin de 2 profilers).

Nouvelle matrice locale à `gen_bag_3d_v4.py` :
```python
R_MOUNT_DOWN = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)
```

**Tentatives précédentes écartées** (composition Euler non triviale, pas de
doc formelle de la convention HoloOcean — testées empiriquement, rejetées sur
mesure, pas par intuition) :
- Deux profilers `[90,-30,±90]` (pitch bas + yaw miroir) : le signe du pitch
  est allé dans le mauvais sens (100 % des points au-dessus, pas <20 %) ET le
  yaw miroir ne séparait pas gauche/droite comme attendu — changer roll/yaw
  après le mount `[90,0,90]` casse la propriété `x≈0` (démontré par calcul :
  `vehicule_x` devient fonction de `sin(Δroll)` ou `sin(Δyaw)`, non nul dès
  qu'on s'écarte de `[90,·,90]`). Seul **pitch** (avec roll=90, yaw=90 fixés)
  préserve `x≡0` tout en tournant le fan dans le plan y-z — d'où `[90,90,90]`.

**Validation en 2 temps** (comme le fix précédent) :
1. Test isolé sur la zone réelle (formule appliquée à la main sur un cas
   simple, `r=10` fixe) : `z` toujours négatif, `y` couvre les deux signes —
   confirmé par un test HoloOcean dédié avant de toucher au générateur :
   `vehicule_z ∈ [-14.0,-13.3]` (jamais >0), `y ∈ [-24.3,+23.3]`
   (208 pts gauche / 229 pts droite).
2. **Bug trouvé dans mes propres checks** en testant le bag régénéré (pas
   dans le mount) :
   - L'ancien critère `§2.3bis` (`std(y)>0.5`, `std(z)>0.5`) est devenu faux
     pour ce mount : un plancher plat donne un `z` **quasi constant** par
     ping (géométrie : `r = profondeur/cos(a)` ⟹ `z = -r·cos(a) = -profondeur`,
     invariant en `a`) — c'est le comportement attendu (balayage propre d'un
     plancher plat), pas un défaut. Critère réduit à `std(x)≈0` seul.
   - `CHECK B` comparait au monde via un `x` fixe (centre de zone) : sur un
     segment court/rectiligne qui reste entièrement d'un côté de ce point,
     *tout* tombe « à gauche » quel que soit ce que capte réellement le
     capteur des deux côtés du véhicule. Corrigé pour comparer au **signe de
     y en repère véhicule** (gauche/droite du véhicule, pas du monde).

**Checks finaux mesurés sur le bag complet régénéré (2 tours, 5289 pings)** :
```
[OK] §2.3ter CHECK A : 0 % des points profiler au-dessus de la surface (attendu <20 %)
[OK] §2.3ter CHECK B : points profonds (z<-15m) gauche=1 087 018 droite=1 256 824 (attendu >1000 chacun)
[OK] §2.3ter CHECK C (signe) : 100 % des points profiler sous le robot (attendu >95 %)
```
Tous les autres critères §2.6 restent PASS (inchangés). `holoocean_3d_traj3.bag`
livré est la version finale, prête pour `caves_3d.py`.

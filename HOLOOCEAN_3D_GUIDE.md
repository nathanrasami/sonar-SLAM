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

*(section à remplir par l'agent du collègue : blocages §2.6.8, choix faits,
valeurs mesurées aux checks — une entrée datée par itération)*

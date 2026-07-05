# HOLOOCEAN 3D — Guide complet pour générer un bag 3D (10 min)

> **Pour** : [le collègue HoloOcean] — document autoportant, à suivre avec un agent
> (Opus) ou à la main. **But** : remplacer le pilotage manuel (12 min réel / 1 min simu,
> intenable pour 10 min de bag) par un **script autonome** qui génère un bag **vraie 3D**.
> Côté SLAM (Nathan), tout est déjà prêt pour le consommer (`sonar_source:=points3d`).
>
> **MISE À JOUR 07-06 — DEUX STRATÉGIES (faire la 1, puis la 2) :**
> - **Stratégie 1 (§3)** : run auto « spirale » ~10 min, hélice autour du chemin carré,
>   **point d'arrivée = point de départ** (N_LAPS entier → la boucle se referme seule).
>   C'est la plus simple et elle suffit pour valider la chaîne 3D.
> - **Stratégie 2 (§3bis, NOUVELLE) — « façon grotte »** : nous avons validé sur un
>   dataset réel de grotte (CIRS caves) une reconstruction 3D par **profiler VERTICAL
>   projeté le long de la trajectoire** — on exporte la recette : un 2ᵉ sonar
>   (ProfilingSonar monté vertical) + une trajectoire d'**exploration aléatoire LISSE**
>   qui fait le tour de l'environnement en variant la profondeur et **revient au
>   départ**. Côté SLAM tout est déjà prêt (outil de reconstruction type grotte_3d).

---

## 1. Contexte et objectif

**État actuel** : bag 2D (`test_2.bag`) — le robot est piloté à la main dans le carré,
le sonar regarde à élévation fixe, et `/sonar_points` publie z=0 partout (projection
plate). Vérifié côté SLAM : **aucune information 3D exploitable**.

**Objectif** : un bag de **10 minutes simulées** où le sonar **balaye l'environnement en
3D** — c'est-à-dire que chaque point publié a un **z réel**, parce qu'on connaît à chaque
ping le plan exact que le sonar regarde.

**L'idée d'ingénieur (validée)** : ce n'est pas le capteur qui balaye, c'est **le
mouvement**. Trajectoire hélicoïdale : imagine un **cylindre** dont l'axe est le chemin
carré (centré entre les murs) ; le robot est SUR la surface du cylindre et **tourne
autour de l'axe en avançant** (tire-bouchon). Le sonar, fixe sur le robot, voit son plan
de scan **rouler** avec le robot → toutes les élévations sont couvertes au fil de
l'avance. Comme la pose 6-DOF est connue (GT), chaque ping se projette exactement en 3D.

```
   mur extérieur                       coupe transversale du "cylindre" :
  ┌───────────────────┐                      z↑
  │   axe = carré     │                 ●────┼──── surface du cylindre
  │  ┌─────────┐      │                /     │    \      ● = robot (roll = φ)
  │  │ mur int.│      │               │   axe(carré)  │  r_c = rayon
  │  └─────────┘      │                \     │    /
  └───────────────────┘                 ─────┼─────
```

---

## 2. Configuration du sonar (scénario JSON)

Deux changements par rapport au scénario actuel, + le fix des artefacts.

### 2.1 Élévation FINE (le cœur de la 3D)

L'ImagingSonar intègre tout ce qui est dans son ouverture verticale (`Elevation`, défaut
20°) : un pixel = un ARC d'élévation → ambiguïté. Pour la 3D on veut un plan FIN :

```json
"sensors": [
    {
        "sensor_type": "ImagingSonar",
        "sensor_name": "SonarFin",
        "socket": "SonarSocket",
        "rotation": [0.0, 0.0, 0.0],
        "configuration": {
            "Azimuth": 120,
            "Elevation": 6,
            "RangeMin": 0.5,
            "RangeMax": 40,
            "RangeBins": 512,
            "AzimuthBins": 512,
            "AddSigma": 0.05,
            "MultSigma": 0.05,
            "RangeSigma": 0,
            "MultiPath": false,
            "AzimuthStreaks": 0,
            "ScaleNoise": false,
            "InitOctreeRange": 50,
            "ViewRegion": false
        }
    }
]
```

- `Elevation: 6` (au lieu de 20) : chaque pixel ≈ un point dans un plan de ±3° —
  l'ambiguïté résiduelle à 40 m est de ±2 m, à 15 m de ±0.8 m. (Descendre à 3° si le
  rendu reste assez dense ; remonter à 10° si les murs deviennent trop clairsemés.)
- La couverture verticale perdue est **rendue par le roulis** de la trajectoire (§3).
- Garde les mêmes `RangeMax`/`Azimuth` que le bag actuel pour que le pont côté SLAM
  (`~range_m=40`, `~fov_deg=120`) reste valable — **si tu changes ces valeurs, dis-le**.

### 2.2 ⚠ Fix des ARCS (artefact constaté sur le bag actuel)

Des arcs sont visibles dans l'image sonar BRUTE du bag actuel → ça vient de la config
du simulateur, pas du pipeline SLAM. Les 3 coupables possibles, dans l'ordre :
1. **`MultiPath: true`** → échos fantômes en arcs. **Mettre `false`.**
2. **`AzimuthStreaks: 1` ou `-1`** → stries/artefacts d'azimut. **Mettre `0`.**
3. `AddSigma`/`MultSigma`/`RangeSigma` trop forts → speckle en anneaux. Garder ≤ 0.05
   et `RangeSigma: 0` pour un premier bag propre (on pourra re-bruiter ensuite).
Valide sur UNE image avant de lancer les 10 min (checklist §6).

### 2.3 (Option B, si l'hélice pose problème) ProfilingSonar tilté

Si le roulis s'avère gênant, l'alternative : un `ProfilingSonar` (Elevation 1° natif,
RangeMax 75) monté avec `"rotation": [0, -30, 0]` (pitch fixe vers le bas) — la 3D vient
alors de l'avance + des virages seulement (couverture moindre, mais zéro acrobatie).
Le reste du document marche pareil (le code §4 lit l'angle de tilt dans la config).

---

## 3. La trajectoire hélicoïdale (script de run)

### 3.1 Paramètres (à caler sur TON scénario)

Va lire dans ton code/scénario la **taille du carré** (côté du chemin entre les murs) et
la profondeur d'eau, puis règle :

```python
L        = 30.0    # côté du CHEMIN carré (m) = axe du cylindre, centré entre les murs
R_TURN   = 4.0     # rayon de l'arc aux coins (m) — quart de cercle, pas d'angle vif
R_CYL    = 1.5     # rayon du cylindre (m). CONTRAINTE : R_CYL + 1 m < distance axe→mur
Z_AXIS   = -3.0    # profondeur de l'axe (m, z NÉGATIF vers le bas en ENU).
                   # CONTRAINTE : |Z_AXIS| - R_CYL > 1 m sous la surface,
                   #              |Z_AXIS| + R_CYL < profondeur du fond - 1 m
V_FWD    = 0.35    # vitesse d'avance le long de l'axe (m/s)
T_ROLL   = 12.0    # période de rotation autour de l'axe (s) — voir §3.3
DT       = 0.05    # pas de temps simulé (s) → 20 Hz de poses
SONAR_HZ = 5.0     # cadence sonar visée (comme le bag actuel : 307 pings / 61 s)
```

Durée d'un tour : périmètre = 4·(L − 2·R_TURN) + 2π·R_TURN ≈ 4L − 1.7·R_TURN.
Avec L=30, R_TURN=4 : ≈ 113 m → à 0.35 m/s ≈ **5 min 23 s par tour** → **2 tours ≈ 10 min 45**
(2 tours = revisites → loop closures pour le SLAM : parfait). Ajuste V_FWD pour viser
ta durée (`T_total = 2·périmètre / V_FWD`).

### 3.2 Le chemin carré avec coins arrondis (abscisse curviligne)

Le chemin de l'axe = 4 segments droits + 4 quarts de cercle de rayon R_TURN. On le
paramètre par l'abscisse curviligne s ∈ [0, périmètre[ → position de l'axe `c(s)`,
tangente `t̂(s)` (direction d'avance, yaw = atan2(t̂_y, t̂_x)). **Aux coins, t̂ tourne
continûment le long de l'arc** — c'est ça, « bien gérer les virages » : pas de saut de
yaw, et l'hélice (φ) continue d'avancer sans discontinuité.

### 3.3 L'hélice autour du chemin

À l'abscisse s (instant t = s/V_FWD) :
- normale horizontale : `n̂(s) = t̂(s) × ẑ` (unitaire, pointe vers l'extérieur du virage) ;
- phase de roulis : `φ(t) = 2π · t / T_ROLL` ;
- **position robot** : `p(t) = c(s) + R_CYL·(cos φ · n̂ + sin φ · ẑ)` ;
- **orientation robot** : yaw = cap de t̂ ; **roll = φ** ; pitch = 0.
  (Le sonar étant fixe sur le robot, son plan de scan roule avec φ → balayage 3D.)

Choix de `T_ROLL` : pendant un tour de roulis le robot avance `V_FWD·T_ROLL` = 4.2 m
(valeurs ci-dessus) → un « pas de vis » de 4.2 m, et à 5 Hz sonar, 60 pings par tour de
roulis (un ping tous les 6° de roulis) : couverture dense. Resserre T_ROLL si tu veux
un pas plus fin (mais garde ≥ 8 s pour des vitesses angulaires raisonnables ~45°/s).

### 3.4 Pourquoi TELEPORT (et pas le contrôleur)

Piloter cette hélice au contrôleur de l'HoveringAUV = réglage PID pénible et dérive.
Pour GÉNÉRER UN BAG, la voie robuste : **teleporter le robot sur la trajectoire
analytique à chaque tick** (`agent.teleport(location, rotation)` — API HoloOcean ; si ta
version ne l'a pas sur l'agent, `env.agents["auv0"].teleport(...)` ou l'action
`env.step(...)` avec le mode de contrôle « position » selon la doc de ta version).
Conséquence : la dynamique n'est plus simulée → **l'IMU/DVL du simulateur ne sont plus
fiables** → on les **synthétise analytiquement** (on connaît la trajectoire exacte,
donc ω et v exacts + bruit gaussien réaliste) : voir le code §4. Le sonar, lui, est
rendu par le moteur à la pose téléportée : c'est tout ce qui compte.

---

## 3bis. STRATÉGIE 2 — exploration aléatoire « façon grotte » (NOUVEAU, 07-06)

### 3bis.1 Le principe (copié d'un dataset réel de grotte)

Sur le dataset CIRS caves, la cavité 3D est reconstruite ainsi : un sonar horizontal
fait le SLAM (carte + trajectoire), un **profiler VERTICAL** balaye la section
transverse, et chaque faisceau est projeté avec la pose interpolée → le tunnel 3D.
On reproduit ça en simulation :

1. **2ᵉ capteur dans le scénario** : un ProfilingSonar monté pour balayer le plan
   VERTICAL transverse (roll 90°) :

```json
{
    "sensor_type": "ProfilingSonar",
    "sensor_name": "ProfilerVert",
    "socket": "SonarSocket",
    "rotation": [90.0, 0.0, 0.0],
    "configuration": {
        "Azimuth": 120, "Elevation": 1,
        "RangeMin": 0.5, "RangeMax": 40,
        "RangeBins": 512, "AzimuthBins": 240,
        "AddSigma": 0.03, "MultSigma": 0.03, "RangeSigma": 0,
        "MultiPath": false, "AzimuthStreaks": 0, "ScaleNoise": false,
        "InitOctreeRange": 50, "ViewRegion": false
    }
}
```
   (⚠ vérifier sur TA version l'axe de `rotation` qui met le fan à la VERTICALE :
   symptôme si faux = les points profiler dessinent un plan horizontal. Un tick
   suffit à le voir.)

2. **Trajectoire aléatoire LISSE qui fait le tour et revient au départ** —
   « aléatoire » ≠ brownien : le SLAM veut de la continuité. Recette :
   des **waypoints ordonnés le long du tour** (comme la stratégie 1) mais avec un
   **jitter aléatoire** latéral et VERTICAL à chaque waypoint, puis une spline
   Catmull-Rom reparamétrée à vitesse constante. Premier point = dernier point
   (boucle fermée exacte) ; seed fixé (reproductible).

```python
# ─── Trajectoire stratégie 2 : waypoints jitterés + Catmull-Rom ────────────────
SEED       = 42
N_WPT      = 24          # waypoints le long du tour (6 par côté)
JIT_XY     = 3.0         # jitter latéral max (m) — GARDE >= 1.5 m des murs !
Z_MIN, Z_MAX = -6.0, -1.5   # bande de profondeur explorée (ENU, z<0 sous l'eau)
PITCH_MAX  = np.deg2rad(20) # ne pas cabrer plus (sonar + réalisme)

def make_waypoints():
    rng = np.random.default_rng(SEED)
    wpts = []
    for i in range(N_WPT):
        s = PERIM * i / N_WPT
        c2, t2 = path(s)                      # le chemin carré de la stratégie 1
        n2 = np.array([t2[1], -t2[0]])
        lat = rng.uniform(-JIT_XY, JIT_XY)
        z   = rng.uniform(Z_MIN, Z_MAX)
        wpts.append(np.array([c2[0] + lat*n2[0], c2[1] + lat*n2[1], z]))
    wpts.append(wpts[0].copy())               # RETOUR AU DÉPART (boucle fermée)
    return np.array(wpts)

def catmull_rom(wpts, n_dense=6000):
    """spline C1 fermée passant par les waypoints, échantillonnée dense puis
    reparamétrée à l'abscisse curviligne (vitesse constante)."""
    W = np.vstack([wpts[-2], wpts, wpts[1]])  # padding cyclique
    pts = []
    for i in range(1, len(W) - 2):
        P0, P1, P2, P3 = W[i-1], W[i], W[i+1], W[i+2]
        for u in np.linspace(0, 1, n_dense // (len(W)-3), endpoint=False):
            pts.append(0.5*((2*P1) + (-P0+P2)*u + (2*P0-5*P1+4*P2-P3)*u*u
                            + (-P0+3*P1-3*P2+P3)*u**3))
    pts = np.array(pts)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s_cum = np.concatenate([[0], np.cumsum(seg)])
    return pts, s_cum                          # position(s) par interpolation sur s_cum

# pose_at(t) version stratégie 2 :
#   s = V_FWD * t  →  p = interp(s_cum, pts) ; tangente t̂ = dérivée numérique ;
#   yaw = atan2(t̂_y, t̂_x) ; pitch = clip(-asin(t̂_z), ±PITCH_MAX) ; roll = 0
#   (le balayage vertical est fait par le ProfilerVert, plus besoin de rouler —
#    garder le roll de la stratégie 1 est possible mais pas nécessaire ici).
```

3. **Dans la boucle principale** (mêmes topics + un de plus) : à chaque tick sonar,
   lire aussi `state["ProfilerVert"]`, convertir comme `sonar_to_points3d` mais avec
   la rotation de montage : `pts_b = R_mount @ [r·cos a, r·sin a, 0]` où
   `R_mount = R_roll(90°)` — puis écrire dans le bag :
   `/profiler_points` (PointCloud2 x,y,z,intensity, points MONDE, comme /sonar_points).

### 3bis.2 Contraintes de faisabilité (validées, à respecter)

- **Lissité** : Catmull-Rom + vitesse constante → pas de saut de cap ; vérifier
  |Δyaw| < 30°/s et |pitch| ≤ 20° sur la trajectoire générée AVANT de rendre (5 lignes
  numpy, cf. checklist).
- **Collisions** : JIT_XY borné pour garder ≥ 1.5 m des murs ; Z_MIN/Z_MAX pour rester
  ≥ 1 m sous la surface et ≥ 1 m au-dessus du fond. Après génération : assert sur les
  min/max de la spline (pas seulement des waypoints — la spline peut dépasser
  légèrement : prendre 0.5 m de marge en plus).
- **Boucle fermée** : premier = dernier waypoint → le SLAM aura SA grande loop closure
  (comme la grotte : une seule, au retour).
- **Durée** : la spline fait ~PERIM ± 20 % ; régler V_FWD pour ~10 min, 1 seul tour
  suffit (la revisite = le retour au départ).

## 4. LE CODE — `gen_bag_3d.py` (générateur complet)

Un seul script : trajectoire → teleport → capture sonar → capteurs synthétiques →
**écriture DIRECTE du rosbag** (pas de ROS en live : robuste au ratio 12:1, le bag est
en temps SIMULÉ propre). À adapter : nom du scénario, nom de l'agent, chemin de sortie.

```python
#!/usr/bin/env python3
"""Génère un bag ROS1 3D depuis HoloOcean : trajectoire hélicoïdale autour d'un
chemin carré, sonar à élévation fine, capteurs synthétiques analytiques.
Usage : python3 gen_bag_3d.py  (env conda holoocean + rosbag/rospy dispo)
"""
import numpy as np
import holoocean
import rosbag
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pc2

# ─── PARAMÈTRES (cf. §3.1 — CALE L, Z_AXIS, R_CYL sur TON scénario) ────────────
SCENARIO   = "TON_SCENARIO"     # ton json (avec le SonarFin du §2)
AGENT      = "auv0"
OUT_BAG    = "test_3d.bag"
L, R_TURN, R_CYL = 30.0, 4.0, 1.5
Z_AXIS     = -3.0
V_FWD      = 0.35
T_ROLL     = 12.0
DT         = 0.05               # 20 Hz poses/IMU/DVL
SONAR_HZ   = 5.0
N_LAPS     = 2
RANGE_MAX, AZIMUTH_DEG = 40.0, 120.0   # = configuration du SonarFin
SIGMA_GYRO, SIGMA_ACC, SIGMA_DVL, SIGMA_DEPTH = 0.002, 0.02, 0.01, 0.02

# ─── Chemin carré à coins arrondis, paramétré par l'abscisse s ────────────────
Ls = L - 2 * R_TURN                       # longueur d'un segment droit
PERIM = 4 * Ls + 2 * np.pi * R_TURN
# coins du carré (chemin centré sur l'origine)
def path(s):
    """retourne c(s) (2D), t_hat(s) (2D) pour s dans [0, PERIM["""
    s = s % PERIM
    seg = Ls + (np.pi / 2) * R_TURN       # segment + quart de cercle
    k = int(s // seg)                     # quel côté (0..3)
    u = s - k * seg
    # repère du côté k : départ P0, direction d, normale gauche g
    dirs = [np.array([1, 0]), np.array([0, 1]), np.array([-1, 0]), np.array([0, -1])]
    d = dirs[k]; g = dirs[(k + 1) % 4]
    half = L / 2
    starts = [np.array([-half + R_TURN, -half]), np.array([half, -half + R_TURN]),
              np.array([half - R_TURN, half]), np.array([-half, half - R_TURN])]
    P0 = starts[k]
    if u <= Ls:                            # segment droit
        return P0 + u * d, d
    # quart de cercle : centre = fin du segment + R_TURN * g
    a = (u - Ls) / R_TURN                  # angle parcouru [0, pi/2]
    C = P0 + Ls * d + R_TURN * g
    c = C + R_TURN * (np.sin(a) * d - np.cos(a) * g)
    t = np.cos(a) * d + np.sin(a) * g
    return c, t / np.linalg.norm(t)

def pose_at(t):
    """pose 6-DOF analytique à l'instant t : p (3D), (roll, pitch, yaw), v_body, omega_body"""
    s = V_FWD * t
    c2, t2 = path(s)
    yaw = np.arctan2(t2[1], t2[0])
    phi = 2 * np.pi * t / T_ROLL                       # phase de roulis
    n2 = np.array([t2[1], -t2[0]])                     # normale horizontale (droite du chemin)
    p = np.array([c2[0] + R_CYL * np.cos(phi) * n2[0],
                  c2[1] + R_CYL * np.cos(phi) * n2[1],
                  Z_AXIS + R_CYL * np.sin(phi)])
    roll = phi % (2 * np.pi)
    # vitesses exactes (différences finies analytiques suffisent à ce pas)
    eps = 1e-3
    def pos_only(tt):
        ss = V_FWD * tt; cc, ttan = path(ss)
        ph = 2 * np.pi * tt / T_ROLL
        nn = np.array([ttan[1], -ttan[0]])
        return np.array([cc[0] + R_CYL * np.cos(ph) * nn[0],
                         cc[1] + R_CYL * np.cos(ph) * nn[1],
                         Z_AXIS + R_CYL * np.sin(ph)])
    v_world = (pos_only(t + eps) - pos_only(t - eps)) / (2 * eps)
    omega_body = np.array([2 * np.pi / T_ROLL, 0.0, 0.0])   # roll rate dominant
    return p, (roll, 0.0, yaw), v_world, omega_body

def rpy_to_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r/2), np.sin(r/2), np.cos(p/2), np.sin(p/2), np.cos(y/2), np.sin(y/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)   # x, y, z, w

def R_from_rpy(r, p, y):
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

# ─── Sonar polaire -> points 3D monde (LE code 2D→3D) ─────────────────────────
def sonar_to_points3d(img, p, rpy, thresh=0.15):
    """img : matrice (RangeBins, AzimuthBins) intensités [0,1] de l'ImagingSonar.
    Chaque pixel (i,j) = (range r_i, azimut a_j) DANS LE PLAN du sonar (élévation
    fine ≈ 0). Point capteur = r[cos a, sin a, 0] ; monde = p + R_wb @ point.
    (Si sonar monté tourné : multiplier par R_mount ici.)"""
    nR, nA = img.shape
    rr = np.linspace(0.5, RANGE_MAX, nR)
    aa = np.deg2rad(np.linspace(-AZIMUTH_DEG/2, AZIMUTH_DEG/2, nA))
    ii, jj = np.nonzero(img > thresh)
    if len(ii) == 0:
        return np.zeros((0, 4), np.float32)
    r = rr[ii]; a = aa[jj]; inten = img[ii, jj].astype(np.float32)
    pts_b = np.stack([r*np.cos(a), r*np.sin(a), np.zeros_like(r)], axis=1)
    Rwb = R_from_rpy(*rpy)
    pts_w = pts_b @ Rwb.T + p
    return np.column_stack([pts_w.astype(np.float32), inten])

# ─── Boucle principale ────────────────────────────────────────────────────────
env = holoocean.make(SCENARIO)
bag = rosbag.Bag(OUT_BAG, "w")
T_TOTAL = N_LAPS * PERIM / V_FWD
print(f"périmètre={PERIM:.1f} m, durée totale={T_TOTAL/60:.1f} min simulées")
t, next_sonar = 0.0, 0.0
rng = np.random.default_rng(0)
try:
    while t < T_TOTAL:
        p, rpy, v_w, w_b = pose_at(t)
        env.agents[AGENT].teleport(location=p.tolist(),
                                   rotation=list(np.degrees(rpy)))  # HoloOcean: degrés
        state = env.tick()
        stamp = rospy.Time.from_sec(t + 1.0)      # éviter t=0 exactement

        q = rpy_to_quat(*rpy)
        # /ground_truth (Odometry, ENU, z NÉGATIF sous l'eau)
        od = Odometry(); od.header.stamp = stamp; od.header.frame_id = "world"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = p
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
         od.pose.pose.orientation.z, od.pose.pose.orientation.w) = q
        bag.write("/ground_truth", od, stamp)

        # /imu : orientation exacte + gyro/accel bruités (synthétiques)
        imu = Imu(); imu.header.stamp = stamp; imu.header.frame_id = "imu"
        (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) = q
        gx, gy, gz = w_b + rng.normal(0, SIGMA_GYRO, 3)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gx, gy, gz
        acc = R_from_rpy(*rpy).T @ np.array([0, 0, 9.81]) + rng.normal(0, SIGMA_ACC, 3)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acc
        bag.write("/imu", imu, stamp)

        # /dvl : vitesse dans le repère VÉHICULE + bruit
        v_b = R_from_rpy(*rpy).T @ v_w + rng.normal(0, SIGMA_DVL, 3)
        dv = TwistStamped(); dv.header.stamp = stamp; dv.header.frame_id = "dvl"
        dv.twist.linear.x, dv.twist.linear.y, dv.twist.linear.z = v_b
        bag.write("/dvl", dv, stamp)

        # /depth : PROFONDEUR POSITIVE VERS LE BAS (= -z) + bruit
        bag.write("/depth", Float64(-p[2] + rng.normal(0, SIGMA_DEPTH)), stamp)

        # sonar à SONAR_HZ
        if t >= next_sonar and "SonarFin" in state:
            img = np.asarray(state["SonarFin"], dtype=np.float32)   # (RangeBins, AzimuthBins)
            im = Image(); im.header.stamp = stamp; im.header.frame_id = "sonar"
            im.height, im.width = img.shape
            im.encoding = "32FC1"; im.is_bigendian = 0; im.step = im.width * 4
            im.data = img.tobytes()
            bag.write("/sonar", im, stamp)

            pts = sonar_to_points3d(img, p, rpy)
            fields = [PointField("x", 0, PointField.FLOAT32, 1),
                      PointField("y", 4, PointField.FLOAT32, 1),
                      PointField("z", 8, PointField.FLOAT32, 1),
                      PointField("intensity", 12, PointField.FLOAT32, 1)]
            hdr = im.header
            pc = pc2.create_cloud(hdr, fields, pts)
            bag.write("/sonar_points", pc, stamp)
            next_sonar += 1.0 / SONAR_HZ

        t += DT
finally:
    bag.close()
    print(f"bag écrit : {OUT_BAG}")
```

**Notes d'adaptation (à vérifier sur TA version de HoloOcean) :**
- `env.agents[AGENT].teleport(location, rotation)` : si absent, utilise la méthode de
  ta version (certaines exposent `env.teleport(agent, loc, rot)` ou un mode de contrôle
  position dans `env.step`). Le reste ne change pas.
- `state["SonarFin"]` : la clé = `sensor_name`. L'image ImagingSonar HoloOcean est
  normalisée [0,1] (sinon adapte `thresh`).
- La convention `rotation` de teleport est en **degrés [roll, pitch, yaw]** — vérifie
  l'ordre dans ta doc si le robot part de travers (symptôme : hélice cabrée).
- Ratio 12:1 : 10 min simulées ≈ **2 h de rendu**. Lance en `nohup`, le script est
  autonome. Réduire `RangeBins/AzimuthBins` à 256 accélère si besoin.

---

## 5. Topics du bag (contrat d'interface avec le SLAM)

| Topic | Type | Contenu / convention |
|---|---|---|
| `/sonar` | sensor_msgs/Image (32FC1, R×Az) | image polaire du SonarFin (lignes = range 0.5→40 m, colonnes = azimut −60→+60°) |
| `/sonar_points` | sensor_msgs/PointCloud2 (x,y,z,intensity) | **points 3D MONDE, z RÉEL** (c'est le livrable 3D) |
| `/ground_truth` | nav_msgs/Odometry | pose 6-DOF exacte, repère **ENU monde** (x est, y nord, **z vers le HAUT** — sous l'eau z<0), quaternion xyzw |
| `/imu` | sensor_msgs/Imu | orientation absolue + gyro/accel (repère véhicule) |
| `/dvl` | geometry_msgs/TwistStamped | vitesse **repère véhicule** (vx avant, vy gauche, vz haut) |
| `/depth` | std_msgs/Float64 | profondeur **positive vers le bas** (= −z) |
| `/profiler_points` (stratégie 2) | sensor_msgs/PointCloud2 (x,y,z,intensity) | sections VERTICALES du ProfilerVert, points MONDE — la matière première de la reconstruction « façon grotte » |

**Conventions à NE PAS changer** (elles nous ont coûté 3 semaines sur Aracati) :
ENU + z haut ; quaternion xyzw ; stamps = temps simulé croissant ; mêmes noms de topics
que le bag actuel (drop-in côté SLAM).

---

## 5bis. ⚠ LE PIÈGE « 2.5D PLAQUÉ » (constaté sur le dataset réel caves, 07-06)

Sur le dataset de grotte réel, le sonar horizontal donne un nuage dont le z varie
(std 4.3 m)… mais **std(z) INTRA-scan = 0.0000 m** : chaque scan est une tranche
PLATE posée au z du véhicule — un ruban 2.5D, pas du volume. **Un std(z) global > 0.5
ne suffit donc PAS** : il peut venir uniquement de la profondeur de la trajectoire.

**La vraie 3D exige que le PLAN du sonar change d'orientation** (ou un capteur
vertical) :
- Stratégie 1 : c'est le RÔLE du roulis de l'hélice — le plan du SonarFin roule avec
  le robot → les z d'UN MÊME ping s'étalent. Si tu enlèves le roll, tu retombes
  exactement dans le piège.
- Stratégie 2 : le ProfilerVert fournit le volume, le sonar horizontal peut rester
  un ruban (c'est le montage du dataset caves réel).

**Test qui tranche (à ajouter à la checklist) : std(z) PAR MESSAGE de /sonar_points**
— sur des pings pris quand le robot roule, std(z) intra-message doit être > 0.5 m.
S'il est ~0 partout, le bag est du 2.5D plaqué, pas de la 3D.

**Note sur « faire tourner le sonar très vite »** (idée de Nathan — pas si folle !) :
c'est exactement le principe des sonars 3D à rotateur réels (ex. BlueView BV5000 :
un profiler monté sur un moteur pan/tilt) et des MSIS. MAIS le réalisme impose la
lenteur : un tour mécanique prend des secondes (le Micron réel : 8.6 s/tour), et
pendant ce temps le véhicule bouge → distorsion de balayage (vécue sur caves).
Les deux stratégies du guide obtiennent le même effet SANS moteur : le mouvement
du véhicule (roll) ou un second capteur fixe — plus simple et plus réaliste qu'un
sonar qui tournerait « très vite ». Si un jour on veut simuler un vrai rotateur :
stations stop-and-scan (véhicule immobile pendant le tour), pas de rotation rapide.

## 6. Checklist de validation AVANT d'envoyer le bag (5 min)

```bash
rosbag info test_3d.bag          # durée ~10 min, 6 topics, ~3000 pings /sonar
python3 - <<'EOF'
import rosbag, numpy as np
import sensor_msgs.point_cloud2 as pc2
bag = rosbag.Bag("test_3d.bag"); n=0
for _, msg, _ in bag.read_messages(topics=["/sonar_points"]):
    if n % 500 == 0:
        pts = np.array(list(pc2.read_points(msg, skip_nans=True)))
        if len(pts): print(f"msg {n}: z in [{pts[:,2].min():.1f}, {pts[:,2].max():.1f}], std={pts[:,2].std():.2f}")
    n += 1
EOF
```
- [ ] **std(z) > 0.5 m** sur /sonar_points (sinon la 3D n'est pas là — c'était le défaut du bag actuel)
- [ ] **std(z) INTRA-MESSAGE > 0.5 m** sur des pings où le robot roule (§5bis — le
      piège « 2.5D plaqué » : un std global élevé peut venir de la seule profondeur
      de la trajectoire, vérifié sur le dataset caves réel : intra-scan = 0.0000)
- [ ] une image /sonar SANS arcs (afficher une frame : les murs = lignes nettes, pas d'anneaux)
- [ ] /ground_truth : z oscille entre Z_AXIS−R_CYL et Z_AXIS+R_CYL (l'hélice se voit)
- [ ] le robot ne sort jamais des murs ni de l'eau (regarder les min/max de x,y,z GT)
- [ ] durée ≈ 2 tours (revisites → le SLAM pourra fermer des boucles)

**Checklist supplémentaire STRATÉGIE 2 :**
- [ ] ‖p(fin) − p(début)‖ < 2 m (boucle fermée)
- [ ] std(z) de la TRAJECTOIRE > 1 m (l'exploration verticale se voit)
- [ ] max |Δyaw|/Δt < 30°/s et |pitch| ≤ 20° sur toute la spline
- [ ] marges : distance spline→murs ≥ 1.5 m, surface/fond ≥ 1 m (asserts dans le script)
- [ ] `/profiler_points` : un message affiché en 3D dessine une SECTION verticale
      (arc/anneau), pas une ligne horizontale (sinon la rotation de montage est fausse)

## 7. Côté SLAM (déjà prêt, pour info)

La branche `holoocean` du dépôt consomme ce bag tel quel :
`./run_slam.sh holoocean` (2D, image) ou `./run_slam.sh holoocean 3D` (points 3D via
`sonar_points_bridge.py`). Les CSV exportent déjà une colonne z.

**Stratégie 2** : la reconstruction « profiler le long de la trajectoire » est DÉJÀ
écrite et validée côté Nathan sur le dataset réel de grotte (`analysis/caves_3d.py`,
branche caves — 23 000+ points de paroi, sections plafond/plancher correctes) ; son
adaptation holoocean lira `/profiler_points` directement (encore plus simple : les
points sont déjà en monde). Envoie le bag, c'est branché.

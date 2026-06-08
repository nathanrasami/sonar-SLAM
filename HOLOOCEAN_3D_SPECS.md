# Specs HoloOcean 3D — pour intégration dans sonar-SLAM (Bruce-SLAM)

Document à destination de la personne qui configure HoloOcean. Objectif : produire des données
permettant un SLAM **3D** (trajectoire avec profondeur + carte de points volumétrique), au lieu
du 2D actuel.

But final attendu côté résultat : une carte de points **3D** (par ex. un barrage où l'on voit
la **hauteur** et la **courbure**, pas une simple ligne plate) et une trajectoire avec **Z**.

---

## 1. Le point clé : le sonar doit mesurer l'ÉLÉVATION

Le pipeline actuel reçoit une image sonar **2D** (range × azimuth, FOV horizontal seulement).
L'élévation (angle vertical) est perdue → tous les points sont à z = 0 → carte plate.

**Pour la 3D, il faut une 3ᵉ dimension : l'élévation.** C'est le module à ajouter côté
HoloOcean.

---

## 2. Format sonar recommandé : nuage de points 3D direct

Le plus simple à intégrer côté SLAM : **publier directement un nuage de points 3D**, plutôt
qu'une image qu'il faudrait re-projeter.

- Le simulateur convertit lui-même (range, azimuth, élévation) → (x, y, z) cartésien
- On évite de réécrire toute la chaîne d'extraction 2D du SLAM
- Le pipeline consomme déjà du `PointCloud2` en interne → branchement direct

> Si tu préfères publier une image multi-plans d'élévation (un "cube" range×azimuth×élévation),
> c'est possible aussi mais ça demande plus de travail d'intégration de mon côté. **Le nuage de
> points 3D est l'option préférée.**

---

## 3. Topics à publier

| Topic | Type ROS | Contenu | Fréquence | Priorité |
|-------|----------|---------|-----------|----------|
| `/sonar_points` | `sensor_msgs/PointCloud2` | Points 3D détectés (x, y, z en mètres), frame capteur. Champ `intensity` optionnel. | 5–10 Hz (cadence ping) | **Indispensable** |
| `/ground_truth` | `nav_msgs/Odometry` | Pose vraie : **x, y, z** + orientation **quaternion complet** (roll, pitch, yaw). | ≥ 50 Hz | **Indispensable** (référence ATE) |
| `/imu` | `sensor_msgs/Imu` | Orientation 3D (quaternion), vitesse angulaire (3 axes), accélération linéaire (3 axes), avec covariances. | 100–200 Hz | Indispensable pour l'odométrie 6DOF |
| `/dvl` | `geometry_msgs/TwistStamped` | Vitesse linéaire 3D **vx, vy, vz** (frame véhicule). | 5–20 Hz | Indispensable (**vz** = profondeur) |
| `/depth` | `sensor_msgs/FluidPressure` ou `std_msgs/Float64` | Profondeur absolue (capteur de pression). | 5–20 Hz | **Recommandé** (z absolu sans dérive) |

### Remarques

- `/ground_truth` : aujourd'hui seul le **yaw** est exploité côté SLAM. Pour la 3D il faut que
  **z et l'orientation 3D complète** soient bien remplis.
- `/dvl` : le **vz** est ce qui permet d'estimer la profondeur. Sans lui, pas de Z odométrique.
- `/depth` : très conseillé — un capteur de pression donne un z absolu qui ne dérive pas, bien
  plus stable que l'intégration du vz.

---

## 4. Paramètres du sonar 3D à fixer (et à me communiquer)

| Paramètre | Valeur actuelle (2D) | À définir (3D) |
|-----------|----------------------|----------------|
| Range max | 40 m | à confirmer selon la scène |
| FOV horizontal (azimuth) | 120° | à confirmer |
| **FOV vertical (élévation)** | — (inexistant) | **NOUVEAU**, ex. 20–30° |
| **Nb de bins en élévation** | — | **NOUVEAU**, ex. 8–32 plans |
| Résolution range | — | m/bin |
| Résolution azimuth | — | degrés/beam |
| **Résolution élévation** | — | **NOUVEAU**, degrés/plan |

Communique-moi les valeurs choisies : je dois les reporter dans la config SLAM.

---

## 5. Conventions de repère — À FIGER ABSOLUMENT (source d'erreur n°1)

C'est le point le plus important. On doit être d'accord noir sur blanc, sinon tout est décalé.

- **Convention d'axes** : idéalement ROS **REP-103 / FLU** :
  - **x = avant**, **y = gauche**, **z = vers le HAUT**
  - Si HoloOcean utilise **NED** (z vers le bas), dis-le-moi explicitement → je ferai une
    conversion unique et propre.
- **Sens de z** : précise noir sur blanc **"z positif vers le haut"** ou **"vers le bas"**.
  Tout en dépend (profondeur, signe du vz DVL).
- **Unités** : mètres, radians, secondes.
- **frame_id** : capteur sonar = `sonar`, véhicule = `base_link`, monde = `odom` / `map`.
- **Timestamps** (`header.stamp`) : cohérents et synchronisés entre tous les topics (le SLAM
  fait une synchronisation temporelle stricte — un décalage casse l'association).

---

## 6. Checklist rapide pour toi

- [ ] Sonar mesure l'élévation (FOV vertical + bins) → publie `/sonar_points` (PointCloud2 3D)
- [ ] Le nuage montre bien un VOLUME (hauteur visible), pas une ligne plate
- [ ] `/ground_truth` avec x, y, **z** + quaternion complet
- [ ] `/imu` (orientation + gyro + accel)
- [ ] `/dvl` avec **vz**
- [ ] `/depth` (recommandé)
- [ ] Convention d'axes et **sens de z** actés et communiqués
- [ ] Timestamps synchronisés, unités SI

Quand tu as une première version, envoie-moi un petit bag de test (même court) et je vérifie la
compatibilité avant qu'on aille plus loin.

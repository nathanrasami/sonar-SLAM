# Suivi de Stage — Sonar-Inertial Odometry (Bruce-SLAM)

**Stagiaire :** Nathan RASAMIJAONA
**Sujet :** Topic 1 — Sonar-Inertial Odometry System
**Dépôt de référence :** [jake3991/sonar-SLAM](https://github.com/jake3991/sonar-SLAM)
**Article de référence :** `Bruce-SLAM_260511_085801.pdf` — *Virtual Maps for Autonomous Exploration of Cluttered Underwater Environments* (Wang et al., 2022)

---

## Contexte

Bruce-SLAM est un système de SLAM (Simultaneous Localization and Mapping) sous-marin à 3 degrés de liberté (x, y, cap θ, profondeur fixe). Il équipe un ROV (BlueROV) avec :
- un sonar imageur (Oculus M750d/M1200)
- un DVL (Doppler Velocity Log)
- une IMU (centrale inertielle)
- un capteur de pression (profondeur)

Le pipeline principal :
1. **Extraction de features** depuis l'image sonar (détecteur SOCA-CFAR)
2. **Scan matching** (ICP avec initialisation globale) pour estimer le déplacement
3. **Dead reckoning** DVL + IMU comme estimation initiale
4. **Graphe de poses** optimisé par iSAM2 (bibliothèque GTSAM)
5. **Détection de boucles** (loop closure) pour corriger la dérive
6. **Cartographie** par grille d'occupation (occupancy grid)
7. **Exploration EM** avec cartes virtuelles (landmarks virtuels)

---

## Jalons

### Jalon 0 — Prise en main du sujet
**Objectif :** Comprendre le contexte, lire l'article, explorer le code.

- [x] Lire entièrement l'article Bruce-SLAM (`Bruce-SLAM_260511_085801.pdf`)
- [x] Comprendre les notions fondamentales du SLAM (pose graph, facteur, iSAM2)

<details>
<summary>📖 Cours — Notions fondamentales du SLAM</summary>

### Le problème fondamental du SLAM

Imagine que tu es dans une pièce inconnue, les yeux bandés. Tu as un podomètre (qui compte tes pas) et un capteur qui mesure la distance aux murs. Comment construire un plan de la pièce et savoir où tu es dedans ?

**Deux problèmes liés :**
- Pour se localiser, il faut une carte
- Pour construire la carte, il faut se localiser

C'est l'œuf et la poule — le SLAM les résout **simultanément**.

---

### La dérive : le problème central

Ton podomètre accumule des erreurs à chaque pas. Après 100 pas, tu ne sais plus exactement où tu es — c'est la **dérive** (*drift*). Si tu ne te corriges jamais, ta carte devient de plus en plus fausse.

```
Vrai chemin :   A → B → C → D → A  (tu reviens à ton point de départ)
Chemin estimé : A → B → C → D → A' (A' ≠ A, erreur accumulée)
```

La correction vient quand tu **reconnais un endroit déjà visité** : tu sais alors que D→A' est faux et tu peux tout recaler. C'est une **loop closure** (fermeture de boucle).

---

### Les 3 ingrédients du SLAM

#### 1. Le modèle de mouvement

À chaque pas de temps, le robot met à jour sa position estimée à partir de ses capteurs de mouvement (ici DVL + IMU) :

$$\mathbf{x}_i = f(\mathbf{x}_{i-1}, \mathbf{u}_i) + \mathbf{w}_i \quad \mathbf{w}_i \sim \mathcal{N}(0, Q_i)$$

- $\mathbf{x}_i = [x, y, \theta]$ : pose du robot (position + cap)
- $\mathbf{u}_i$ : commande moteur ou mesure DVL
- $\mathbf{w}_i$ : bruit Gaussien — l'incertitude **croît** à chaque pas

#### 2. Le modèle de mesure

Le capteur (sonar) observe l'environnement et produit des mesures :

$$\mathbf{z}_{ij} = h(\mathbf{x}_i, \mathbf{l}_j) + \mathbf{v}_{ij}$$

- $\mathbf{z}_{ij}$ : mesure (ex: distance + angle vers un obstacle)
- $\mathbf{l}_j$ : position du landmark (obstacle) dans la carte
- Cette mesure **contraint** la relation entre pose et carte

#### 3. L'estimation MAP

On cherche la trajectoire et la carte les plus probables étant donnés toutes les mesures :

$$\mathcal{X}^*, \mathcal{L}^* = \underset{\mathcal{X}, \mathcal{L}}{\arg\max} \ P(\mathcal{Z}|\mathcal{X}, \mathcal{L}) \cdot P(\mathcal{X})$$

Après linéarisation, ça devient un **problème de moindres carrés** — minimiser l'erreur entre ce que le capteur a mesuré et ce que le modèle prédit.

---

### Le graphe de facteurs : la représentation clé

Tout le SLAM est encodé dans un **graphe** :

```
[x0] --odométrie-- [x1] --odométrie-- [x2] --odométrie-- [x3]
                     |                                      |
                  sonar(l1)                              sonar(l1)  ← loop closure !
```

- **Nœuds** = inconnues à estimer (poses $x_i$, landmarks $l_j$)
- **Arêtes** = contraintes de mesure (chaque mesure lie deux nœuds)
- Une **loop closure** ajoute une arête entre $x_3$ et $l_1$ déjà vu depuis $x_1$ → l'optimisation recale tout

---

### iSAM2 : l'optimiseur utilisé dans Bruce-SLAM

Optimiser tout le graphe depuis zéro à chaque nouvelle mesure serait trop lent. **iSAM2** (*incremental Smoothing and Mapping*) maintient une factorisation de Cholesky incrémentale : il ne recalcule que la partie du graphe affectée par la nouvelle mesure. C'est ce qui permet de tourner en temps réel.

---

### Résumé visuel du pipeline Bruce-SLAM

```
Sonar image
    ↓
[SOCA-CFAR] → features 2D  ──────────────────────────┐
                                                       ↓
DVL + IMU  → dead reckoning → pose initiale → [ICP scan matching] → contrainte de pose
                                                       ↓
                                              [Graphe de facteurs]
                                              optimisé par iSAM2
                                                       ↓
                                    Trajectoire corrigée + Carte d'occupation
```

---

### Ce que tu dois retenir absolument

| Concept | En une phrase |
|---|---|
| **Dérive** | Les capteurs de mouvement accumulent des erreurs → la carte devient fausse |
| **Loop closure** | Reconnaître un endroit déjà vu → contrainte qui recale tout |
| **Graphe de facteurs** | Représentation du problème SLAM : nœuds = inconnues, arêtes = mesures |
| **iSAM2** | Optimise le graphe incrémentalement en temps réel |
| **Pose SLAM** | Pas de landmarks explicites : les contraintes viennent du scan matching sonar |

</details>

- [x] Comprendre le principe du sonar imageur et du DVL

<details>
<summary>📖 Cours — Sonar imageur et DVL</summary>

### Le sonar imageur

#### Principe physique

Le sonar (*Sound Navigation And Ranging*) émet une impulsion acoustique et mesure l'écho renvoyé par les obstacles. Sous l'eau, le son se propage à ~1500 m/s (contre 343 m/s dans l'air) — c'est la seule onde utile à longue portée en milieu aquatique (la lumière est absorbée en quelques mètres).

Un **sonar imageur multibeam** (comme l'Oculus M750d utilisé dans Bruce-SLAM) émet simultanément des centaines de faisceaux (*beams*) dans un plan horizontal. Pour chaque beam, on mesure l'intensité du retour en fonction de la distance.

#### L'image sonar

Le résultat est une image en **coordonnées polaires** (range × bearing) :
```
bearing (angle)
←  -65°  ...  0°  ...  +65°  →
       ___________________
  0m  |                   |
      |  intensité du     |
      |  retour sonar     |
 30m  |___________________|
             range
```
- **Range** (distance) : de 0 à 30m dans Bruce-SLAM, résolution 4mm
- **Bearing** (angle) : de -65° à +65°, résolution 1°, 512 beams
- **Intensité** : forte si l'obstacle réfléchit bien (métal, béton), faible si peu réfléchissant

#### Particularité du sonar vs. lidar

| | Lidar | Sonar imageur |
|---|---|---|
| Milieu | Air | Eau |
| Portée | 100m+ | 30m |
| Résolution angulaire | ~0.1° | ~1° |
| Bruit | Faible | Élevé |
| Ouverture verticale | Très fine | Large (~20°) |

L'**ouverture verticale large** (20° vertical pour l'Oculus M750d) est une différence clé : le sonar peut détecter des objets derrière le premier plan. Cela complique le modèle de capteur — dans Bruce-SLAM, les cellules derrière le premier retour sont laissées *inconnues* (pas *libres* comme avec un lidar).

#### Extraction de features : SOCA-CFAR

L'image sonar brute est bruitée. On ne peut pas utiliser tous les pixels comme features. Le détecteur **SOCA-CFAR** (*Smallest-Of Cell-Averaging Constant False Alarm Rate*) identifie les retours significatifs de manière adaptive :

Pour chaque cellule testée, on compare son intensité à la moyenne de ses voisins proches avant et après dans le beam :
```
[voisins gauche | N cellules] [CUT] [voisins droite | N cellules]
       T_< = moyenne gauche          T_> = moyenne droite
       T = τ × min(T_<, T_>)   ← seuil adaptatif
```
Si `intensité(CUT) > T` → c'est un feature (retour sonar réel, pas du bruit).

Le seuil τ est calculé pour garantir un taux de fausse alarme $P_{fa}$ constant quelle que soit l'intensité du bruit ambiant. Les features détectés sont convertis en coordonnées cartésiennes 2D :
$$x = r\cos(\theta), \quad y = r\sin(\theta)$$

Ces points 2D forment un **nuage de points sonar** utilisé ensuite par le scan matching (ICP).

---

### Le DVL — Doppler Velocity Log

#### Principe physique

Le DVL mesure la **vitesse du robot** par effet Doppler acoustique. Il émet 4 faisceaux acoustiques vers le fond (ou les parois) et mesure le décalage de fréquence entre l'émission et le retour. Ce décalage est proportionnel à la vitesse relative entre le robot et le fond.

```
         [ROV]
        /  |  \
       /   |   \
      ↓    ↓    ↓   ← 4 faisceaux vers le fond
    fond marin
```

#### Ce que ça donne

Le DVL fournit la vitesse 3D du robot dans son référentiel : $[v_x, v_y, v_z]$ à 5 Hz dans Bruce-SLAM. C'est une mesure **directe de vitesse**, pas de position — bien plus fiable que d'intégrer l'accélération de l'IMU.

#### Limites du DVL

- Nécessite que les faisceaux atteignent un fond solide (inutilisable en pleine eau)
- En cas de perte de fond (*bottom lock lost*), on retombe sur l'IMU seule → dérive plus rapide
- Ne mesure pas le cap → combiné avec l'IMU pour avoir $\theta$

---

### Le Dead Reckoning DVL + IMU

Le **dead reckoning** est l'estimation de position par intégration des capteurs de mouvement, sans correction externe. Dans Bruce-SLAM :

```
DVL  →  vitesse [vx, vy, vz] à 5 Hz  ┐
                                       ├→ intégration → pose estimée [x, y, θ]
IMU  →  cap θ, vitesse angulaire      ┘
Pression → profondeur z (fixe à 1m)
```

C'est l'**initialisation** du scan matching ICP : sans une bonne pose initiale, l'ICP converge vers un mauvais minimum local. Le dead reckoning DVL+IMU est suffisamment précis sur de courtes distances pour initialiser l'ICP correctement.

La dérive s'accumule malgré tout sur de longues distances → corrigée par les loop closures du SLAM.

---

### Résumé

| Capteur | Ce qu'il mesure | Rôle dans Bruce-SLAM |
|---|---|---|
| **Sonar Oculus M750d** | Image acoustique 2D (512 beams, ±65°, 30m) | Source principale de features pour le scan matching et la cartographie |
| **DVL Rowe SeaPilot** | Vitesse 3D par effet Doppler | Dead reckoning, initialisation de l'ICP |
| **IMU VectorNav VN100** | Accélération, vitesse angulaire, cap | Cap θ, complément au DVL |
| **Capteur pression Bar30** | Profondeur | Maintien de la profondeur fixe à 1m |

</details>
- [x] Explorer l'architecture du dépôt (`bruce/`, `bruce_msgs/`, `bruce_slam/`)

<details>
<summary>📖 Description détaillée du package bruce_slam</summary>

Le package `bruce_slam/` contient l'intégralité du code du système SLAM. Il est écrit principalement en **Python 3**, avec deux modules C++ compilés via **pybind11** (binding Python/C++) pour les parties critiques en performance.

---

### Vue d'ensemble

```
bruce_slam/src/bruce_slam/
├── CFAR.py              ← Détection de features sonar (SOCA-CFAR)
├── feature_extraction.py← Nœud ROS front-end sonar
├── sonar.py             ← Modèle du sonar Oculus
├── dead_reckoning.py    ← Fusion DVL + IMU (dead reckoning)
├── gyro.py              ← Filtre cap IMU
├── kalman.py            ← Filtre de Kalman alternatif
├── slam_objects.py      ← Structures de données (Keyframe, ICPResult...)
├── slam.py              ← Cœur du SLAM (graphe de facteurs, iSAM2)
├── slam_ros.py          ← Nœud ROS wrappant slam.py
├── mapping.py           ← Cartographie par grille d'occupation (submaps)
├── cpp/
│   ├── cfar.cpp         ← SOCA-CFAR en C++ (rapide, appelé depuis CFAR.py)
│   └── pcl.cpp          ← ICP via libpointmatcher (appelé depuis slam.py)
└── utils/
    ├── conversions.py   ← Conversions de types (GTSAM ↔ numpy ↔ ROS)
    ├── topics.py        ← Noms des topics ROS (DVL, IMU, sonar, pression)
    ├── visualization.py ← Fonctions de visualisation matplotlib/ROS
    └── io.py            ← Lecture de bags, logs, timers
```

---

### Fichiers Python

#### `CFAR.py` — Classe `CFAR`
Implémentation Python du détecteur SOCA-CFAR. Calcule le seuil adaptatif à partir du taux de fausse alarme $P_{fa}$ souhaité et applique le filtre sur l'image sonar en coordonnées polaires. Utilisé par `feature_extraction.py`. La version C++ (`cfar.cpp`) est préférée en pratique pour la vitesse.

#### `feature_extraction.py` — Classe `FeatureExtraction`
**Nœud ROS du front-end sonar.** S'abonne au topic sonar (messages `OculusPing`), applique SOCA-CFAR pour détecter les features, convertit les features polaires en nuage de points cartésiens 2D, et publie le résultat pour le SLAM. Tourne à ~5 Hz.

#### `sonar.py` — Classes `OculusFireMsg`, `OculusProperty`
Modèle du sonar Oculus (M750d/M1200). Encapsule les propriétés du capteur (portée, nombre de beams, fréquence, ouverture angulaire) et la structure des messages sonar bruts. Fournit aussi le modèle inverse du sonar (conversion image polaire → coordonnées cartésiennes) utilisé pour la cartographie.

#### `dead_reckoning.py` — Classe `DeadReckoningNode`
**Nœud ROS de dead reckoning.** Fusionne les mesures DVL (vitesse 3D) + IMU (cap, vitesse angulaire) + pression (profondeur) pour produire une estimation de pose à haute fréquence (5 Hz). Cette pose sert d'initialisation à l'ICP dans le SLAM. Utilise GTSAM en interne pour la propagation d'incertitude.

#### `gyro.py` — Classe `GyroFilter`
Filtre dédié au cap (heading θ). Traite les messages d'odométrie et filtre le bruit gyroscopique de l'IMU. Utilisé en complément de `dead_reckoning.py` pour améliorer l'estimation du cap.

#### `kalman.py` — Classe `KalmanNode`
Implémentation d'un filtre de Kalman alternatif pour la fusion de capteurs. Variante de `dead_reckoning.py` utilisée dans certaines configurations ou pour des comparaisons.

#### `slam_objects.py` — Structures de données
Définit les types de données utilisés dans tout le système :
- `STATUS` (enum) : état d'une keyframe (WAITING, INITIALIZING, SLAM...)
- `Keyframe` : une pose de référence avec son scan sonar, ses features, sa pose estimée et sa covariance
- `ICPResult` : résultat d'un alignement ICP (transformation + score de convergence)
- `InitializationResult` : résultat de l'initialisation globale avant ICP
- `SMParams` : paramètres du scan matching (seuils, nombre d'itérations...)

#### `slam.py` — Classe `SLAM`
**Cœur algorithmique du système.** Contient toute la logique SLAM sans dépendance ROS (testable indépendamment) :
- Gestion des keyframes (décision d'ajout selon déplacement/rotation)
- Scan matching séquentiel et non-séquentiel (appel à `pcl.cpp` via pybind11)
- Construction et optimisation du graphe de facteurs (iSAM2 via GTSAM)
- Détection et validation des loop closures (PCM — Pairwise Consistent Measurement)
- Extraction des poses et covariances depuis iSAM2

#### `slam_ros.py` — Classe `SLAMNode(SLAM)`
**Nœud ROS wrappant `slam.py`.** Hérite de `SLAM` et ajoute tout ce qui est ROS : souscription aux topics (features sonar, dead reckoning), publication des résultats (trajectoire, carte, TF), gestion du threading et synchronisation des messages. C'est ce nœud qui est lancé par `slam.launch`.

#### `mapping.py` — Classes `Submap`, `Mapping`
Cartographie par grille d'occupation à base de sous-cartes :
- `Submap` : carte locale attachée à une keyframe. Construit la grille d'occupation à partir du modèle inverse du sonar pour un scan donné
- `Mapping` : gère l'ensemble des sous-cartes et fusionne en log-odds pour produire la carte globale. Supporte la mise à jour efficace quand une pose est corrigée par une loop closure (Eq. 40 de l'article)

---

### Fichiers C++ (via pybind11)

Ces fichiers sont compilés en `.so` (bibliothèque partagée) et importés directement en Python comme des modules normaux.

#### `cpp/cfar.cpp`
Version C++ haute performance du détecteur SOCA-CFAR. Utilise **Eigen** pour les opérations matricielles. Appelé depuis `CFAR.py` quand disponible — typiquement 10-20× plus rapide que la version Python pure pour traiter les 512 beams à 5 Hz.

#### `cpp/pcl.cpp`
Interface C++ vers **libpointmatcher** (ICP). Expose à Python les fonctions d'alignement de nuages de points :
- Scan matching séquentiel (entre deux keyframes consécutives)
- Scan matching non-séquentiel (loop closure, avec initialisation globale CONSAC)
Utilise **Eigen** pour les matrices de transformation et **pybind11** pour l'interface Python.

---

### Utilitaires (`utils/`)

#### `utils/conversions.py`
Fonctions de conversion entre les différents types de données du projet :
- `X(x)` : crée un symbole GTSAM pour une pose (convention de nommage du graphe)
- `n2g` / `g2n` : numpy ↔ GTSAM (Pose2, Pose3, matrices de covariance)
- `r2g` / `g2r` : messages ROS ↔ GTSAM
- `r2n` : message sonar `OculusPing` → tableau numpy
- `build_rgb_cloud` / `n2r` : numpy → PointCloud2 ROS (pour la visualisation RViz)

#### `utils/topics.py`
Centralise les noms et types des topics ROS utilisés : DVL (`rti_dvl/DVL`), IMU (`sensor_msgs/Imu`), profondeur (`bar30_depth/Depth`), sonar (`sonar_oculus/OculusPing`). Modifie ce fichier si tu changes le hardware.

#### `utils/io.py`
Outils divers : lecture de fichiers `.bag` ROS (`read_bag`), décorateurs de log colorés (`loginfo`, `logwarn`...), timer de performance (`CodeTimer`), parseur d'arguments CLI (`common_parser`).

#### `utils/visualization.py`
Fonctions matplotlib et ROS pour visualiser : trajectoires colorées par temps (`colorline`), ellipses de covariance (`plot_cov_ellipse`), polygones shapely, nuages de points ROS colorés. Utilisé pour générer les figures de debug et de résultats.

</details>
- [ ] Lire la documentation du README et du wiki

**Livrable :** Notes personnelles de compréhension du pipeline.

<details>
<summary>✅ Bilan Jalon 0 — Synthèse de prise en main</summary>

### Ce qu'est Bruce-SLAM en une phrase

Un système qui permet à un robot sous-marin (BlueROV2) de **se localiser et construire une carte** de son environnement de façon autonome, en utilisant uniquement un sonar, un DVL et une IMU — sans GPS, sans lumière, sans intervention humaine.

---

### Les capteurs et leur rôle

```
┌─────────────────────────────────────────────────────┐
│                     BlueROV2                        │
│                                                     │
│  Oculus M750d      DVL SeaPilot    IMU VN100        │
│  (sonar imageur)   (vitesse)       (cap, accél.)    │
│  512 beams         5 Hz            5 Hz             │
│  ±65°, 30m                                          │
│                         Bar30 (profondeur = 1m fixe)│
└─────────────────────────────────────────────────────┘
```

---

### Le pipeline complet

```
Image sonar (5 Hz)
       │
       ▼
  [SOCA-CFAR]          ← cfar.cpp + CFAR.py
  Détection features       Seuil adaptatif sur chaque beam
       │
       │  Nuage de points 2D
       ▼
  [ICP Scan Matching]  ← pcl.cpp (libpointmatcher)
  Aligner avec le scan précédent
  Init. globale (CONSAC) + raffinement local
       │
       │  Contrainte de pose relative
       ▼
┌─────────────────────────────────────────┐
│         Graphe de facteurs              │  ← slam.py + GTSAM
│                                         │
│  x0 ──── x1 ──── x2 ──── x3            │
│           │               │             │
│        sonar(l1)       sonar(l1) ← loop closure !
│                                         │
│    Optimisé par iSAM2 (incrémental)    │
└─────────────────────────────────────────┘
       │
       ├──→ Trajectoire corrigée    → slam_ros.py publie sur ROS
       │
       └──→ [Cartographie]          ← mapping.py
            Grille d'occupation
            par sous-cartes (submaps)
            1 submap par keyframe
```

**Dead reckoning** (DVL + IMU) tourne en parallèle à 5 Hz pour initialiser l'ICP à chaque keyframe.

---

### La logique de décision (exploration EM)

```
À chaque étape de planification :

  Carte virtuelle V  ←  grille d'occupation downsampleée (2m)
  (landmarks virtuels dans toutes les cellules occupées)
        │
        ▼
  Pour chaque chemin candidat :
    Calculer U_EM = -log det(Σ_pose) - Σ log det(Σ_landmark_virtuel) - α·distance
        │
        ▼
  Choisir le chemin qui minimise l'incertitude globale
  (exploration ET revisite équilibrés automatiquement)
```

---

### Architecture du code

| Fichier | Rôle | Langage |
|---|---|---|
| `feature_extraction.py` | Nœud ROS sonar → features | Python |
| `CFAR.py` + `cpp/cfar.cpp` | Détection SOCA-CFAR | Python + C++ |
| `dead_reckoning.py` | Fusion DVL+IMU | Python |
| `slam.py` | Cœur SLAM (graphe, ICP, loop closure) | Python |
| `slam_ros.py` | Interface ROS de slam.py | Python |
| `cpp/pcl.cpp` | ICP via libpointmatcher | C++ |
| `mapping.py` | Grille d'occupation par submaps | Python |
| `utils/conversions.py` | Conversions numpy/GTSAM/ROS | Python |

---

### Ce que tu sais maintenant faire

- Lire et comprendre l'article Bruce-SLAM dans sa totalité → `BRUCE_SLAM.md`
- Expliquer le principe du SLAM (dérive, loop closure, graphe de facteurs, iSAM2)
- Expliquer le rôle du sonar imageur, du DVL, de l'IMU dans le pipeline
- Naviguer dans le code source et identifier le rôle de chaque fichier

</details>

---

### Jalon 1 — Installation de l'environnement
**Objectif :** Avoir un environnement fonctionnel pour faire tourner Bruce-SLAM.

**Environnement : Fedora (pas Ubuntu) → RoboStack (micromamba/conda)**

ROS Noetic n'est pas supporté nativement sur Fedora. Solution retenue : **RoboStack** via micromamba, qui fournit des binaires conda de ROS Noetic fonctionnels sur Fedora sans Docker ni VM.

- [ ] Installer `micromamba` (gestionnaire de paquets conda léger)
  ```bash
  "${SHELL}" <(curl -L micro.mamba.pm/install.sh)
  ```
- [ ] Créer l'environnement ROS Noetic via RoboStack
  ```bash
  micromamba create -n ros_env -c conda-forge -c robostack-noetic ros-noetic-desktop python=3.8
  micromamba activate ros_env
  ```
- [ ] Installer catkin-tools et pybind11 dans l'env
  ```bash
  micromamba install -c conda-forge -c robostack-noetic ros-noetic-catkin python-catkin-tools
  ```
- [ ] Installer les dépendances Python 3 dans l'env conda
  ```bash
  micromamba install -c conda-forge gtsam opencv numpy scipy scikit-learn shapely tqdm pyyaml
  pip install rosbag  # si non disponible via conda
  ```
- [ ] Créer le workspace catkin : `mkdir -p ~/catkin_ws/src`
- [ ] [libnabo](https://github.com/ethz-asl/libnabo) (cloner dans `~/catkin_ws/src/`)
- [ ] [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) — version spécifique : commit `d478ef2`
  ```bash
  git checkout d478ef2eb33894d5f1fe84d8c62cec2fc6da818f
  ```
- [ ] [Argonaut](https://github.com/jake3991/Argonaut) (cloner dans `~/catkin_ws/src/`)
- [ ] Cloner ce dépôt dans `~/catkin_ws/src/`
- [ ] Compiler avec `catkin build` (PAS `catkin_make`)

> **Référence RoboStack :** https://robostack.github.io/GettingStarted.html

**Livrable :** Workspace catkin qui compile sans erreur.

---

### Jalon 2 — Reproduction des résultats
**Objectif :** Faire tourner Bruce-SLAM sur les données fournies et reproduire les résultats de l'article.

- [ ] Télécharger le fichier de données `sample_data.bag` (Google Drive)
- [ ] Lancer le mode offline : `roslaunch bruce_slam slam.launch file:=sample_data.bag`
- [ ] Visualiser le résultat dans RViz
- [ ] Lancer le mode online (rejouer le bag en temps réel)
- [ ] Comparer les trajectoires et cartes obtenues avec les figures de l'article
- [ ] Comprendre chaque paramètre dans le dossier `config/`

**Livrable :** Captures d'écran / vidéos des résultats reproduits. Rapport de comparaison.

---

### Jalon 3 — Compréhension approfondie du pipeline
**Objectif :** Maîtriser le pipeline de bout en bout en lisant et en instrumentant le code.

- [ ] **Front-end :**
  - [ ] Comprendre la détection SOCA-CFAR (`feature.yaml`, code de détection)
  - [ ] Comprendre le scan matching ICP (initialisation globale + raffinement local)
  - [ ] Comprendre la fusion DVL + IMU (dead reckoning)
- [ ] **Back-end :**
  - [ ] Comprendre la construction du graphe de facteurs
  - [ ] Comprendre l'optimisation iSAM2 (GTSAM)
  - [ ] Comprendre la détection de loop closure (PCM — Pairwise Consistent Measurement)
- [ ] **Cartographie :**
  - [ ] Comprendre la grille d'occupation par sous-cartes (submaps)
  - [ ] Comprendre les cartes virtuelles (virtual landmarks) pour l'exploration EM
- [ ] Ajouter des logs/visualisations pour observer l'état interne

**Livrable :** Schéma annoté du pipeline avec correspondances code/article.

---

### Jalon 4 — Analyse des limitations
**Objectif :** Identifier les points faibles du système actuel pour cibler les améliorations.

- [ ] Analyser la robustesse de SOCA-CFAR dans différentes conditions
- [ ] Étudier la dérive accumulée et l'impact des loop closures
- [ ] Identifier les goulets d'étranglement de performance (temps de calcul)
- [ ] Recenser les limitations mentionnées dans l'article (section Discussion/Conclusion)
- [ ] Tester le système dans des conditions dégradées (bruit, faible densité de features)

**Livrable :** Rapport d'analyse avec pistes d'amélioration priorisées.

---

### Jalon 5 — Amélioration et optimisation algorithmique
**Objectif :** Proposer et implémenter au moins une amélioration concrète du système.

Pistes possibles (à choisir/affiner avec l'encadrant) :
- [ ] Amélioration de l'extraction de features (alternative à SOCA-CFAR)
- [ ] Amélioration de l'initialisation du scan matching
- [ ] Fusion inertielle plus précise (filtre de Kalman étendu, pré-intégration IMU)
- [ ] Passage en 3D (lever l'hypothèse de profondeur fixe)
- [ ] Amélioration de la détection de loop closure
- [ ] Optimisation du temps de calcul (profiling + refactoring)

- [ ] Implémenter l'amélioration choisie
- [ ] Valider quantitativement (comparaison avec la baseline)

**Livrable :** Code versionné + rapport de résultats comparatifs.

---

### Jalon 6 — Rédaction du rapport de stage
**Objectif :** Produire le rapport final.

- [ ] Introduction : contexte, problématique, objectifs
- [ ] État de l'art : SLAM, sonar imageur, odométrie sonar-inertielle
- [ ] Présentation de Bruce-SLAM (pipeline, algorithmes clés)
- [ ] Expériences et résultats reproduced
- [ ] Contribution personnelle (amélioration algorithmique)
- [ ] Conclusion et perspectives

**Livrable :** Rapport de stage complet.

---

## Avancement global

| Jalon | Description                          | Statut      |
|-------|--------------------------------------|-------------|
| 0     | Prise en main du sujet               | En cours    |
| 1     | Installation de l'environnement      | Non démarré |
| 2     | Reproduction des résultats           | Non démarré |
| 3     | Compréhension approfondie            | Non démarré |
| 4     | Analyse des limitations              | Non démarré |
| 5     | Amélioration algorithmique           | Non démarré |
| 6     | Rédaction du rapport                 | Non démarré |

---

## Notes et ressources

### Ressources clés
- Article principal : `Bruce-SLAM_260511_085801.pdf`
- Dépôt GitHub : https://github.com/jake3991/sonar-SLAM
- Données de test : https://drive.google.com/file/d/1nmiFfyk8mVssLqgac7BOe4_RPBP6Wnc9/view
- GTSAM (backend d'optimisation) : https://gtsam.org/
- libpointmatcher (ICP) : https://github.com/ethz-asl/libpointmatcher

### Concepts SLAM à maîtriser
- Graphe de facteurs (factor graph) et inférence MAP
- iSAM2 — incrémental Smoothing And Mapping
- ICP — Iterative Closest Point (scan matching)
- Dead reckoning et dérive inertielle
- Loop closure et cohérence globale de la carte
- Grille d'occupation (occupancy grid, Bayes filter)
- CFAR — Constant False Alarm Rate (détection sonar)

### Journal de bord
> Utiliser cette section pour noter les décisions importantes, blocages rencontrés et solutions trouvées.

- **2026-05-11** — Initialisation du suivi de stage. Lecture partielle de l'article (pages 1-10). Exploration de la structure du dépôt.

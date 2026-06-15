# Suivi de Stage — Sonar-Inertial Odometry (Bruce-SLAM)

**Stagiaire :** Nathan RASAMIJAONA
**Sujet :** Topic 1 — Sonar-Inertial Odometry System
**Dépôt de référence :** [jake3991/sonar-SLAM](https://github.com/jake3991/sonar-SLAM)
**Article de référence :** `Bruce-SLAM_260511_085801.pdf` — *Virtual Maps for Autonomous Exploration of Cluttered Underwater Environments* (Wang et al., 2022)
**Durée du stage :** 4 mois (~17 semaines) — Mai à Septembre 2026
**Présentations :** mini-présentation hebdomadaire + diapo de compréhension du sujet (semaine 3)

---

## Timeline sur 4 mois

L'objectif est de **diluer** le travail sur toute la durée du stage, même si la compréhension technique avance vite. Chaque semaine doit produire quelque chose de présentable.

| Semaine | Période | Activité principale | Livrable hebdo |
|---------|---------|---------------------|----------------|
| S1 | 12–16 mai | Prise en main : lire l'article, explorer le code | Résumé oral du sujet |
| S2 | 19–23 mai | Installation environnement (VM Ubuntu + ROS) | Env fonctionnel, premier roslaunch |
| S3 | 26–30 mai | **Diapo présentation sujet** + reproduction résultats | Diaporama + RViz qui tourne |
| S4 | 02–06 juin | Analyse des résultats obtenus vs article | Comparaison cartes/trajectoires |
| S5 | 09–13 juin | Compréhension front-end : SOCA-CFAR en détail | Notes annotées sur CFAR.py |
| S6 | 16–20 juin | Compréhension front-end : ICP et scan matching | Notes annotées sur pcl.cpp |
| S7 | 23–27 juin | Compréhension back-end : graphe de facteurs + iSAM2 | Schéma annoté slam.py |
| S8 | 30 juin–04 juil | Compréhension back-end : loop closure (PCM) | Schéma annoté loop closure |
| S9 | 07–11 juil | Compréhension cartographie : submaps + occupancy grid | Notes annotées mapping.py |
| S10 | 14–18 juil | Analyse des limitations du système | Rapport d'analyse |
| S11 | 21–25 juil | Recherche bibliographique : pistes d'amélioration | Liste de pistes priorisées |
| S12 | 28 juil–01 août | Choix + prototypage de l'amélioration | Prototype initial |
| S13 | 04–08 août | Implémentation de l'amélioration | Code fonctionnel |
| S14 | 11–15 août | Tests et validation quantitative | Résultats comparatifs |
| S15 | 18–22 août | Rédaction rapport : intro + état de l'art | Chapitres 1-2 rédigés |
| S16 | 25–29 août | Rédaction rapport : résultats + contribution | Chapitres 3-4 rédigés |
| S17 | 01–05 sept | Finalisation rapport + préparation soutenance | Rapport final |

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

<details>
<summary>📄 Résumé de l'article Bruce-SLAM — voir détail complet dans <a href="BRUCE_SLAM.md">BRUCE_SLAM.md</a></summary>

**Titre :** *Virtual Maps for Autonomous Exploration of Cluttered Underwater Environments*
**Auteurs :** Jinkun Wang et al. — Stevens Institute of Technology, 2022
**Publié :** IEEE Journal of Oceanic Engineering — arXiv:2202.08359

---

### Problème central
Explorer un environnement sous-marin inconnu de façon autonome, sans GPS, sans vision, en gérant le compromis entre :
- Explorer (couvrir du terrain)
- Revisiter (fermer des boucles pour corriger la dérive)

---

### Contribution principale : cartes virtuelles + algorithme EM
- **Carte virtuelle** : landmarks hypothétiques dans les zones non encore visitées
- **EM** : choisit le chemin qui minimise `det(Σ)` (incertitude globale) en anticipant les observations futures
- Résultat : EM atteint 90% de couverture en **30% moins de distance** que l'approche heuristique

---

### Pipeline SLAM (Section IV)
| Étape | Méthode | Fréquence |
|-------|---------|-----------|
| Feature extraction | SOCA-CFAR | 5 Hz |
| Scan matching séquentiel | ICP + CONSAC init | 5 Hz |
| Dead reckoning | DVL + IMU + pression | 5 Hz |
| Optimisation graphe | iSAM2 (GTSAM) | 0.2 Hz |
| Loop closure | NSSM + PCM | 0.2 Hz |
| Cartographie | Grille d'occupation (log-odds) | 0.2 Hz |

---

### Capteurs (BlueROV2 réel — marina King's Point NY)
- Sonar : Oculus M750d, 512 beams, ±65°, 30m
- DVL : Rowe SeaPilot
- IMU : VectorNav VN100
- Pression : Bar30

---

### Résultats expérimentaux
- **4 algos comparés :** NF, NBV, Heuristic, EM
- **EM retenu** : meilleur compromis pose uncertainty / map error / couverture
- **Simulation** (45 et 60 essais) + **ROV réel** (12 runs)
- Map error à 400m : EM=1.05, NF=1.13, NBV=1.12, Heuristic=1.03

---

### Conclusion
Premier exemple d'exploration autonome réelle d'un port avec un ROV intégrant son propre SLAM dans chaque décision de planification.
**Perspective :** extension 3D (SE(2) → SE(3)).

📖 **Résumé complet section par section :** [BRUCE_SLAM.md](BRUCE_SLAM.md)

</details>

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

### Jalon 1 — Installation de l'environnement ✅
**Objectif :** Avoir un environnement fonctionnel pour faire tourner Bruce-SLAM.
**Semaines :** S2 (19–23 mai 2026)

**Environnement : Fedora → VM Ubuntu 20.04 (KVM/QEMU)**

RoboStack (conda) s'est avéré cassé avec conda-forge actuel (conflit numpy ≥1.26 introuvable). Solution retenue : **VM KVM** avec Ubuntu 20.04 — ROS Noetic officiellement supporté, zéro surprise pour une démo.

- [x] Vérifier la virtualisation AMD/SVM dans le BIOS Asus TUF (`grep -c svm /proc/cpuinfo` → 16)
- [x] Installer KVM + virt-manager sur Fedora
- [x] Créer une VM Ubuntu 20.04 (8 CPU, 8 GB RAM, 50 GB disque)
- [x] Installer ROS Noetic Desktop Full dans la VM
- [x] Configurer le workspace catkin (`~/catkin_ws/`)
- [x] Compiler libnabo depuis les sources (`make -j2` — RAM limitée en VM)
- [x] Compiler libpointmatcher commit `d478ef2` (`make -j2`)
- [x] Cloner sonar-SLAM et Argonaut dans `~/catkin_ws/src/`
- [x] Corriger `CMakeLists.txt` : `-std=c++11` → `-std=c++14` (requis par PCL 1.10)
- [x] Installer dépendances ROS : `pybind11-catkin`, `nav-core`, `navigation`, `ros-numpy`
- [x] Installer dépendances Python : `gtsam==4.1.1`, `numpy==1.23.5`, `tqdm`, `shapely`, `scikit-learn`
- [x] `catkin build` → tous les packages compilent

**Livrable :** Workspace catkin qui compile sans erreur. ✅

---

### Jalon 2 — Reproduction des résultats
**Objectif :** Faire tourner Bruce-SLAM sur les données fournies et reproduire les résultats de l'article.
**Semaines :** S3–S4 (26 mai – 06 juin 2026)

- [x] Télécharger le fichier de données `sample_data.bag` (Google Drive)
- [x] Lancer le mode offline : `roslaunch bruce_slam slam.launch file:=sample_data.bag`
- [x] Obtenir RViz qui affiche carte + trajectoire complètes sans erreur
- [ ] Lancer le mode online (rejouer le bag en temps réel avec `rosbag play`)
- [x] Vérifier la reproductibilité : 3 runs identiques → même résultat ✅
- [ ] Faire des captures d'écran des résultats et les comparer aux figures de l'article
- [x] Comprendre chaque paramètre dans le dossier `config/`

<details>
<summary>📖 Explication des fichiers de configuration</summary>

---

### `slam.yaml` — Paramètres du SLAM

#### Keyframes
```yaml
keyframe_duration: 1.0       # délai minimum (s) entre deux keyframes
keyframe_translation: 3.0    # distance (m) à parcourir pour créer une keyframe
keyframe_rotation: deg(30)   # rotation (°) pour créer une keyframe
```
Une keyframe est ajoutée si le robot s'est déplacé de **3m OU tourné de 30°** depuis la dernière. Valeurs recommandées : 1–4m selon la densité souhaitée du graphe.

#### Modèles de bruit (sigmas = écart-type de l'incertitude)
```yaml
prior_sigmas:    [0.1, 0.1, 0.01]   # incertitude initiale sur x(m), y(m), θ(rad)
odom_sigmas:     [0.2, 0.2, 0.02]   # incertitude du dead reckoning DVL+IMU
icp_odom_sigmas: [0.1, 0.1, 0.01]   # incertitude d'une contrainte ICP
```
Ces valeurs définissent la confiance accordée à chaque source de mesure dans le graphe de facteurs. Un sigma plus grand = moins de confiance = la contrainte pèse moins dans l'optimisation.

#### Sequential Scan Matching (SSM) — contraintes entre keyframes consécutives
```yaml
ssm:
  enable: True        # activer/désactiver le SSM
  min_points: 50      # nombre minimum de features pour tenter un SSM
  max_translation: 3.0   # déplacement max accepté entre deux keyframes (m)
  max_rotation: deg(30)  # rotation max acceptée (°)
  target_frames: 3    # comparer la keyframe courante aux 3 précédentes
```
Si l'ICP donne un résultat qui dépasse `max_translation` ou `max_rotation`, il est rejeté (trop irréaliste).

#### Non-Sequential Scan Matching (NSSM) — détection de loop closures
```yaml
nssm:
  enable: True        # activer/désactiver les loop closures
  min_st_sep: 8       # zone d'exclusion : ignorer les 8 keyframes les plus récentes
  min_points: 50      # nombre minimum de features
  max_translation: 10.0  # distance max pour tenter une loop closure (m)
  max_rotation: deg(60)  # rotation max (°)
  source_frames: 5    # agréger 5 frames pour augmenter la densité de points
  cov_samples: 30     # échantillons pour estimer la covariance de la loop closure
```
`min_st_sep: 8` évite de détecter des "fausses" loop closures avec des keyframes trop récentes (qui sont déjà liées par le SSM).

#### PCM — rejet des loop closures aberrantes
```yaml
pcm_queue_size: 5   # fenêtre glissante : analyser les 5 dernières loop closures
min_pcm: 2          # garder une loop closure seulement si au moins 2 autres dans la fenêtre sont cohérentes avec elle
```
Si une loop closure est incohérente avec ses voisines temporelles, elle est rejetée comme aberrante.

---

### `feature.yaml` — Paramètres du détecteur SOCA-CFAR

#### Détecteur CFAR
```yaml
CFAR:
  Ntc: 40      # nombre de cellules d'entraînement de chaque côté du CUT
  Ngc: 10      # nombre de cellules de garde (exclues du calcul de bruit)
  Pfa: 0.1     # taux de fausse alarme cible (10% → seuil τ calculé en conséquence)
  rank: 10     # rang de la matrice (stabilisation numérique)
  alg: 'SOCA' # algorithme : Smallest-Of Cell-Averaging (le plus robuste)
```
La fenêtre CFAR autour d'un pixel : `[Ngc | Ntc | CUT | Ntc | Ngc]`. Plus `Ntc` est grand, meilleure est l'estimation du bruit ambiant. `Pfa = 0.1` est assez permissif — on accepte 10% de fausses alarmes pour ne pas rater de vrais retours.

#### Filtrage du nuage de points
```yaml
filter:
  threshold: 65    # intensité minimale pour être gardé après CFAR (filtre dur)
  resolution: 0.5  # résolution du voxel downsampling (m) — réduit la densité
  radius: 1.0      # rayon (m) pour le filtre de rejet d'outliers
  min_points: 5    # nombre minimum de voisins dans le rayon pour garder un point
  skip: 1          # traiter 1 scan sur skip (1 = tous les scans)
```
Le pipeline de filtrage est : CFAR → seuil dur → voxel downsampling → rejet d'outliers isolés. Le résultat est un nuage de points 2D propre transmis à l'ICP.

---

### `icp.yaml` — Paramètres de l'alignement ICP (libpointmatcher)

#### Filtres de données
```yaml
readingDataPointsFilters:    # filtres appliqués au scan source (vide = aucun)
referenceDataPointsFilters:  # filtres appliqués au scan cible (vide = aucun)
```
Les filtres sont déjà appliqués en amont dans `feature.yaml`, donc vides ici.

#### Correspondances
```yaml
matcher:
  KDTreeMatcher:
    knn: 1         # 1 voisin le plus proche (nearest neighbor)
    epsilon: 0     # précision exacte (pas d'approximation)
    maxDist: 10.0  # distance maximale pour une correspondance valide (m)
```
Utilise un **KD-tree** pour trouver efficacement le point le plus proche dans le scan de référence.

#### Filtres d'outliers
```yaml
outlierFilters:
  - MaxDistOutlierFilter:
      maxDist: 3.0    # rejeter les correspondances avec distance > 3m
  - TrimmedDistOutlierFilter:
      ratio: 0.8      # garder seulement les 80% de correspondances les plus proches
```
Double filtre : d'abord rejeter les correspondances trop éloignées, puis ne garder que les 80% les meilleures. Robuste aux scans bruités.

#### Minimiseur d'erreur
```yaml
errorMinimizer:
  PointToPointErrorMinimizer   # minimise ||R·p_source + t - p_cible||²
```
Point-à-point classique. L'alternative commentée `PointToPlaneErrorMinimizer` est plus précise mais nécessite des normales de surface.

#### Critères d'arrêt
```yaml
transformationCheckers:
  - CounterTransformationChecker:
      maxIterationCount: 40     # arrêter après 40 itérations max
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.01       # arrêter si la rotation change de moins de 0.01 rad
      minDiffTransErr: 0.1      # arrêter si la translation change de moins de 0.1 m
      smoothLength: 4           # moyenner sur 4 itérations pour éviter les oscillations
```
L'ICP s'arrête dès qu'un des deux critères est atteint : convergence ou nombre max d'itérations.

---

### `dead_reckoning.yaml` — Fusion DVL + IMU

```yaml
dvl_max_velocity: 0.5      # vitesse max DVL acceptée (m/s) — au-delà = rejeté comme aberrant
imu_pose: [0,0,0,deg(-90),0,0]  # offset de montage de l'IMU (x,y,z,roll,pitch,yaw)
                                 # -90° en roll = IMU montée sur le côté sur ce ROV
keyframe_duration: 1.0     # délai minimum entre deux keyframes de dead reckoning
keyframe_translation: 4.0  # distance (m) pour créer une keyframe DR (légèrement > slam.yaml)
keyframe_rotation: deg(30) # rotation (°) pour créer une keyframe DR
use_gyro: False            # False = pas de FOG (Fiber Optic Gyroscope) sur ce robot
imu_version: 1             # version du driver IMU (1 = VectorNav lourd, 2 = MKII)
```

`imu_pose` est crucial : il compense l'orientation physique de l'IMU dans le châssis du ROV. Si l'IMU est montée de travers, sans correction toutes les mesures angulaires seraient fausses.

---

### `mapping.yaml` — Cartographie par grille d'occupation

#### Dimensions de la carte
```yaml
origin: [-100.0, -100.0]   # coin bas-gauche de la carte initiale (m)
size: [200.0, 200.0]       # taille initiale : 200m × 200m
resolution: 0.2            # résolution : 1 cellule = 0.2m × 0.2m
inc: 50.0                  # agrandissement automatique de 50m si le robot sort des bords
use_slam_traj: true        # utiliser la trajectoire SLAM (corrigée) plutôt que le dead reckoning
```

#### Modèle d'occupation (Bayes filter)
```yaml
pub_occupancy1: true
hit_prob: 0.8       # probabilité d'occupation si un retour sonar est détecté (P(occupé|hit))
miss_prob: 0.3      # probabilité d'occupation si le beam passe sans retour (P(occupé|miss))
inflation_angle: 0.04  # demi-angle d'ouverture d'un beam pour l'inflation (rad)
inflation_range: 0.4   # distance d'inflation autour d'un point de contact (m)
```
`hit_prob = 0.8` et `miss_prob = 0.3` sont convertis en log-odds et accumulés cellule par cellule. Une cellule devient "occupée" quand son log-odds dépasse un seuil.

#### Filtrage des outliers de carte
```yaml
outlier_filter_radius: 5.0      # rayon de recherche de voisins (m)
outlier_filter_min_points: 20   # nombre minimum de voisins pour garder un point
min_translation: 0.5            # déplacement minimum (m) avant de mettre à jour la carte
min_rotation: 0.015             # rotation minimum (rad) avant de mettre à jour la carte
```

---

### `gyro.yaml` — Filtre gyroscope (FOG)

```yaml
latitude: 40.70594689371728  # latitude de la marina USMMA (King's Point, NY)
                              # utilisée pour compenser la rotation terrestre
sensor_rate: 250             # fréquence du FOG (250 Hz)
offset:
  x: 0.
  y: 0.
  z: 45.                     # décalage angulaire de 45° entre le FOG et le sonar
```

> ℹ️ Ce fichier n'est utilisé que si `use_gyro: True` dans `dead_reckoning.yaml`. Le FOG (*Fiber Optic Gyroscope*) est un capteur de cap très précis, non présent sur tous les ROV. Le sample_data.bag a été enregistré **sans** FOG.

---

### `kalman.yaml` — Filtre de Kalman étendu (alternatif)

Ce fichier configure une **alternative** au dead reckoning simple — un filtre de Kalman complet à 12 états. Non utilisé par défaut dans Bruce-SLAM (remplacé par `dead_reckoning.py`), mais présent pour des expérimentations.

**Vecteur d'état (12 dimensions) :**
```
[x, y, z, roll, pitch, yaw, ẋ, ẏ, ż, roll̇, pitcḣ, yaẇ]
 ←── position ──→  ←── orientation ──→  ←── vitesses ──→
```

**Matrices clés :**
- `Q` — bruit de processus : incertitude du modèle de mouvement. Les valeurs faibles (0.0001) sur x signifient qu'on fait très confiance au modèle pour x, moins pour roll/yaw (0.1).
- `A_imu` — matrice de transition d'état à dt=0.005s (200 Hz IMU) : $x_{k+1} = A \cdot x_k$. Les termes `0.005` sur la diagonale position/vitesse encodent $x = x + \dot{x} \cdot dt$.
- `R_dvl`, `R_imu`, `R_depth`, `R_gyro` — matrices de bruit de mesure de chaque capteur. Plus la valeur est petite, plus on fait confiance au capteur.
- `H_dvl`, `H_imu`, `H_depth`, `H_gyro` — matrices d'observation : sélectionnent quelles dimensions du vecteur d'état sont observées par chaque capteur. Ex: `H_dvl` observe les vitesses (lignes 7-9 du vecteur d'état).

```yaml
imu_offset: 180   # offset de montage IMU pour ce dataset (USMMA) en degrés
dt_dvl: 0.2       # période DVL (5 Hz)
dt_imu: 0.005     # période IMU (200 Hz)
dt_depth: 0.25    # période capteur pression (4 Hz)
dt_gyro: 0.004    # période FOG (250 Hz)
```

</details>

**Livrable :** Captures d'écran / vidéos des résultats reproduits. Rapport de comparaison.

<details>
<summary>🧪 Tests de paramètres — voir détail complet dans <a href="TESTS.md">TESTS.md</a></summary>

Tests réalisés sur `sample_data.bag` en modifiant les paramètres un par un.
Images des résultats dans `TESTS_image/`.

**Paramètres testés (`feature.yaml`) :**
| Paramètre | Baseline | Valeurs testées | Impact principal |
|-----------|---------|----------------|-----------------|
| Pfa | 0.1 | 0.01, 0.001 | Densité points — peu d'impact trajectoire |
| threshold | 65 | 30, 90 | Fort impact : drift à 30, moins de LC à 90 |
| resolution | 0.5 | 0.1, 2.0 | Géométrie et loop closures |
| radius | 1.0 | 0.3, 3.0 | Drift si mal réglé |

**Paramètres testés (`icp.yaml`) :**
| Paramètre | Baseline | Valeurs testées | Impact principal |
|-----------|---------|----------------|-----------------|
| MaxDistOutlier | 3.0 | 1.0, 6.0 | Plus de LC à 1.0 |
| TrimmedDist ratio | 0.8 | 0.5, 0.95 | Peu d'impact |

**Conclusion générale :** `threshold` et `resolution` sont les paramètres les plus sensibles pour changer de dataset.

📊 **Détail complet avec images :** [TESTS.md](TESTS.md)

</details>

---

### Jalon 3 — Compréhension approfondie du pipeline
**Objectif :** Maîtriser le pipeline de bout en bout en lisant et en instrumentant le code.
**Semaines :** S5–S9 (09 juin – 11 juillet 2026)

- [x] **Front-end (S5–S6) :**
  - [x] Comprendre la détection SOCA-CFAR (`CFAR.py`, `cfar.cpp`, `config/feature.yaml`)
  - [x] Comprendre le scan matching ICP (initialisation globale CONSAC + raffinement, `pcl.cpp`)
  - [x] Comprendre la fusion DVL + IMU (`dead_reckoning.py`)
- [x] **Back-end (S7–S8) :**
  - [x] Comprendre la construction du graphe de facteurs (`slam.py`)
  - [x] Comprendre l'optimisation iSAM2 via GTSAM
  - [x] Comprendre la détection de loop closure — PCM (`slam.py`)
- [x] **Cartographie (S9) :**
  - [x] Comprendre la grille d'occupation par sous-cartes (`mapping.py`)
  - [x] Comprendre les cartes virtuelles (virtual landmarks) pour l'exploration EM (code non publié)
- [x] Ajouter des logs/visualisations pour observer l'état interne (via RViz + tests paramètres TESTS.md)

**Livrable :** Schéma annoté du pipeline avec correspondances code/article.

<details>
<summary>📖 Analogie — La boucle SLAM complète (du sonar au EM)</summary>

**Situation : tu explores un bâtiment inconnu dans le noir, avec une lampe torche qui éclaire juste devant toi.**

---

**1. Sonar → image brute**
- Vie réelle : ta lampe torche éclaire une zone, tu vois des formes floues
- SLAM : le sonar envoie 512 faisceaux, reçoit les échos → image en intensité

**2. CFAR → feature extraction**
- Vie réelle : tu ignores les ombres et zones floues, tu retiens seulement les objets nets (un mur, une colonne)
- SLAM : CFAR compare chaque pixel à ses voisins, garde seulement les réflexions fortes → nuage de points

**3. DVL + IMU → dead reckoning**
- Vie réelle : tu comptes tes pas et tu sais dans quelle direction tu marches → "j'ai avancé de 3m vers le nord"
- SLAM : DVL mesure la vitesse sol, IMU mesure l'orientation → estimation de la pose courante

**4. ICP → scan matching**
- Vie réelle : tu compares ce que tu vois maintenant avec ce que tu as vu 3 secondes avant → tu affines ta position
- SLAM : ICP aligne le scan actuel sur le scan précédent, le dead reckoning fournit le point de départ (guess)

**5. Vérification SSM**
- Vie réelle : si ICP dit que tu as bougé de 10m en 1 seconde → impossible → on rejette
- SLAM : si la transformation ICP s'écarte trop du dead reckoning → résultat rejeté, on garde juste le DR

**6. BetweenFactor → graphe iSAM2**
- Vie réelle : tu notes dans un carnet "depuis le point A jusqu'au point B, j'ai fait X pas dans cette direction"
- SLAM : `BetweenFactorPose2` = contrainte entre deux poses dans le graphe

**7. iSAM2 → optimisation globale**
- Vie réelle : à la fin de la journée tu relis ton carnet, tu corriges les incohérences → tu redresses ta carte mentale
- SLAM : iSAM2 optimise toutes les poses simultanément en minimisant l'erreur globale du graphe

**8. NSSM → loop closure detection**
- Vie réelle : tu reconnais une colonne que tu as déjà vue il y a 10 minutes → tu sais où tu es !
- SLAM : ICP compare le submap actuel (5 keyframes fusionnées) avec toutes les keyframes passées

**9. PCM → validation du loop closure**
- Vie réelle : tu demandes confirmation à 2 autres personnes qui explorent le même bâtiment → si elles confirment, c'est vrai
- SLAM : PCM vérifie que plusieurs loop closures candidats sont mutuellement cohérents → min_pcm validés → ajout au graphe

**10. Occupancy map → carte finale**
- Vie réelle : tu dessines le plan du bâtiment sur papier au fur et à mesure
- SLAM : chaque scan validé remplit la grille d'occupation (hit_prob=0.8 si objet, miss_prob=0.3 sinon)

**11. EM → exploration**
- Vie réelle : tu choisis toujours la prochaine pièce à explorer en priorité celle qui réduira le plus ton incertitude sur le plan global
- SLAM : EM place des virtual landmarks dans les zones inexplorées, choisit le chemin qui minimise det(Σ)

</details>

---

### Jalon 4 — Analyse des limitations
**Objectif :** Identifier les points faibles du système actuel pour cibler les améliorations.
**Semaines :** S10–S11 (14–25 juillet 2026)

- [ ] Analyser la robustesse de SOCA-CFAR dans différentes conditions
- [ ] Étudier la dérive accumulée et l'impact des loop closures
- [ ] Identifier les goulets d'étranglement de performance (temps de calcul)
- [x] Recenser les limitations mentionnées dans l'article (section VI)
- [ ] Recherche bibliographique sur les pistes d'amélioration existantes

<details>
<summary>📋 Limitations identifiées — article + code</summary>

### Limitations mentionnées dans l'article (Section VI)

**1. Hypothèse 3DOF (profondeur fixe)**
- Le système opère en SE(2) : x, y, θ uniquement
- La profondeur z est fournie par le capteur Bar30, pas estimée
- Pas de gestion du roulis/tangage dans le SLAM
- → Inutilisable si le robot change de profondeur ou dans un environnement 3D

**2. Code d'exploration EM non publié**
- Le pipeline SLAM est open-source mais l'exploration autonome (EM) ne l'est pas
- Impossible de reproduire les expériences d'exploration autonome du papier
- → Seul le mode bag (données enregistrées) est reproductible

**3. Pas de vérité terrain pour les runs réels**
- Les expériences en vrai port (King's Point NY) n'ont pas de ground truth
- Évaluation qualitative uniquement sur les cartes finales
- → Impossible de calculer ATE ou map error sur données réelles

**4. Dépendance aux structures réfléchissantes**
- CFAR détecte les échos forts → nécessite des obstacles solides bien réfléchissants
- Environnements ouverts (fond sableux, eau libre) → peu de features → ICP instable
- → Robustesse limitée dans des environnements non encombrés

**5. Paramètres non adaptatifs**
- Tous les paramètres (Pfa, threshold, keyframe_translation...) sont fixes
- Pas d'adaptation automatique à l'environnement ou aux conditions sonar
- Observé lors de nos tests : threshold et resolution très sensibles au dataset

### Limitations identifiées lors de nos tests (TESTS.md)

- **threshold trop bas** → drift visible par faux positifs CFAR
- **resolution trop haute** → perte de loop closures → trajectoire moins précise
- **radius outlier mal réglé** → drift même avec loop closures actifs
- **Pas de dataset alternatif compatible** → validation limitée à sample_data.bag

</details>

**Livrable :** Rapport d'analyse avec pistes d'amélioration priorisées.

---

### Jalon 5 — Amélioration et optimisation algorithmique
**Objectif :** Proposer et implémenter au moins une amélioration concrète du système.
**Semaines :** S12–S14 (28 juillet – 15 août 2026)

Pistes possibles (à choisir/affiner avec l'encadrant) :
- [ ] Amélioration de l'extraction de features (alternative à SOCA-CFAR)
- [ ] Amélioration de l'initialisation du scan matching
- [ ] Fusion inertielle plus précise (filtre de Kalman étendu, pré-intégration IMU)
- [ ] Passage en 3D (lever l'hypothèse de profondeur fixe)
- [ ] Amélioration de la détection de loop closure
- [ ] Optimisation du temps de calcul (profiling + refactoring)

- [ ] Prototyper l'amélioration choisie (S12)
- [ ] Implémenter l'amélioration (S13)
- [ ] Valider quantitativement — comparaison avec la baseline (S14)

**Livrable :** Code versionné + rapport de résultats comparatifs.

---

### Jalon 6 — Rédaction du rapport de stage
**Objectif :** Produire le rapport final.
**Semaines :** S15–S17 (18 août – 05 septembre 2026)

- [ ] Introduction : contexte, problématique, objectifs (S15)
- [ ] État de l'art : SLAM, sonar imageur, odométrie sonar-inertielle (S15)
- [ ] Présentation de Bruce-SLAM (pipeline, algorithmes clés) (S16)
- [ ] Expériences et résultats reproduits (S16)
- [ ] Contribution personnelle (amélioration algorithmique) (S16)
- [ ] Conclusion et perspectives (S17)
- [ ] Relecture + mise en forme finale (S17)

**Livrable :** Rapport de stage complet.

---

## Avancement global

| Jalon | Description                      | Semaines   | Statut       |
|-------|----------------------------------|------------|--------------|
| 0     | Prise en main du sujet           | S1         | ✅ Terminé   |
| 1     | Installation de l'environnement  | S2         | ✅ Terminé   |
| 2     | Reproduction des résultats       | S3–S4      | ✅ Terminé   |
| 3     | Compréhension approfondie        | S5–S9      | ✅ Terminé   |
| 4     | Analyse des limitations          | S10–S11    | ✅ Terminé (DISO + FABLE.md) |
| 5     | Amélioration algorithmique       | S12–S14    | 🔄 En cours (DISO intégré ✅, Sonar Context 1/5) |
| 6     | Rédaction du rapport             | S15–S17    | ⏳ À venir   |

> ⚡ **Avance sur le planning.** La compréhension technique a avancé plus vite que la timeline
> initiale : DISO est déjà intégré (amélioration n°1, jalon 5) et Sonar Context est entamé. Le
> travail réel a divergé du plan théorique — voir le **Journal de bord** pour l'état exact.

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
- **2026-05-13** — Jalon 0 terminé : article lu en entier (BRUCE_SLAM.md), notions SLAM/sonar/DVL comprises, architecture du code explorée.
- **2026-05-13** — Jalon 1 terminé : RoboStack abandonné (numpy cassé), VM Ubuntu 20.04 via KVM installée, workspace catkin compilé après corrections (C++14, GTSAM 4.1.1, numpy 1.23.5).
- **2026-05-13** — Jalon 2 démarré : `roslaunch bruce_slam slam.launch` tourne, RViz s'ouvre, images sonar visibles. Erreur GTSAM `quaternion()` résolue par downgrade à 4.1.1.
- **2026-05-20** — Jalon 2 avancé : simulation offline reproduite (3 runs identiques ✅). Tests de paramètres CFAR + ICP + slam.yaml documentés dans TESTS.md (12 tests, images dans TESTS_image/).
- **2026-05-20** — Jalon 3 terminé : deep dive complet du pipeline (CFAR, ICP, dead reckoning, iSAM2, loop closure PCM, mapping). Code EM non publié → coché comme non disponible.
- **2026-05-20** — Jalon 4 démarré : limitations recensées (5DOF, EM non publié, pas de ground truth réel, dépendance aux structures réfléchissantes, paramètres fixes).
- **2026-05-20** — Présentation semaine 1 préparée (Canva, 10 min) : pipeline Bruce-SLAM + résultats papier + ce qui a été fait.
- **2026-05-22** — Intégration dataset Aracati2017 dans bruce_slam : nœud `odom_bridge.py` (bridge `/odom_pose` → `nav_msgs/Odometry`), mode `cartesian_mode` dans `feature_extraction.py` pour images PNG BlueView P900-130, launch file `aracati.launch`, configs `feature_aracati.yaml` et `slam_aracati.yaml`.
- **2026-05-22** — Tests Aracati2017 : odométrie pure → trajectoire cohérente (forme proche GPS ground truth, drift attendu). SSM → dégradation (ICP non calibré pour BlueView). NSSM → faux loop closures. Conclusion : ICP à recalibrer pour ce sonar.
- **2026-05-22** — Export CSV ajouté (`slam_ros.py`) : `trajectory.csv` + `pointcloud.csv` générés dans `~/` au shutdown.
- **2026-05-27** — Bug critique corrigé : `_publish_features_stamped` mettait y=0 sur tous les points (ordre des axes x,zeros,y au lieu de x,y,zeros). ICP ne pouvait pas recaler en 2D. Fix : swap y et z dans np.c_[]. Résultat : lignes droites visibles dans la carte point cloud.
- **2026-05-27** — Constat : SSM (ICP) diverge sur Aracati2017 même après correction y=0. Confirmé par papier DISO (ICRA 2024) : BlueROV SLAM (= ICP) donne 16.25% translation error vs 8.69% pour DISO sur Aracati2017. ICP fondamentalement limité sur ce sonar (basse résolution + sensibilité aux conditions initiales).
- **2026-05-27** — Plan d'action : intégrer DISO (github.com/SenseRoboticsLab/DISO) dans bruce_slam, remplacer SSM par odométrie directe DISO. Conserver back-end iSAM2 de bruce_slam pour loop closures. Prochaine présentation : papier DISO.
- **2026-06-03** — **DISO intégré et fonctionnel.** DISO publie `/direct_sonar/pose` → `odom_bridge.py` convertit en `nav_msgs/Odometry` sur `LOCALIZATION_ODOM_TOPIC` → consommé par bruce_slam comme dead-reckoning. SSM désactivé (`ssm.enable: False`). Run DISO standalone : ATE 2.1–3.0 m (non déterministe, le timing temps-réel change la trajectoire). Présentation DISO faite → résumé complet dans `Paper/Sonar/DISO.md`.
- **2026-06-05** — **Refonte de l'évaluation de trajectoire** (`traj_eval.py`). Découverte du bug racine des plots : on mélangeait des CSV de runs différents (bases de temps incompatibles : temps simu 0–71 s vs temps Unix 2017) → `np.interp` clampait tout sur GT[0] → alignement faux. L'ancien « bricolage » flip-Y + rotation 45° magique masquait ce bug. Remplacé par : (1) garde-fou dans `associer_par_temps` (erreur explicite si les plages de temps ne se recouvrent pas), (2) alignement **Umeyama** (SVD, standard TUM/evo) à la place du bricolage manuel. ATE non faussé (translation cosmétique au départ appliquée APRÈS le calcul).
- **2026-06-06** — **Wrapper `run_slam.sh`** : chaque run dans un dossier horodaté `results/run_<type>_<date>/` (évite le mélange de CSV) ; `*.csv` gitignorés. Mode `diso` lance DISO + joue le bag (le launch DISO ne jouait pas le bag → cause du « RViz vide »). Export CSV cohérent par run : `groundtruth.csv`, `trajectory.csv`, `odometry.csv`, `pointcloud.csv`. Correction RLException dans `aracati.launch` (`optenv` ne peut pas imbriquer `$(find)`).
- **2026-06-10** — **Diagnostic loop closure NSSM natif sur Aracati.** `skip: 5` affamait le NSSM (69 % de keyframes vides → 0 boucle). En `skip: 1` le NSSM détecte des boucles mais elles sont **FAUSSES** (corr_y avec GT -0.88 → -0.12, forme cassée, ATE 11.3 m avec min_pcm 4). En les filtrant (min_pcm 6) : 0 boucle, ATE 5.2 m = **baseline à battre**. Conclusion : le problème est la *qualité* de la détection (features+ICP sur sonar basse résolution), pas le réglage. → motive Sonar Context.
- **2026-06-10** — **Ajout odométrie pure** dans `analyze_drift.py` : `/cmd_vel` intégré (modèle unicycle), ATE 14.7 m. C'est l'odométrie native d'Aracati que la doctorante affichait (≠ DISO, qui est mon ajout). Plot final : GT, odom pure, odom DISO, DISO standalone, Bruce-SLAM.
- **2026-06-10** — **Hiérarchie ATE mesurée sur Aracati** : odom pure 14.7 m → DISO standalone 2.1–3.0 m → odom DISO en run combiné 3.9 m → DISO+Bruce (NSSM off) 5.4 m → NSSM min_pcm6 5.2 m. Récit : DISO améliore l'odométrie ~4× vs dead-reckoning natif.
- **2026-06-10** — **Organisation en branches Git** (le projet a plusieurs variantes) : `main` (DISO+Bruce, éval Umeyama, docs), `feature/sonar-context` (Sonar Context, étape 1/5), `feature/slam-3d` (vide, migration 2D→3D future), `experiments/holoocean`. Workflow : Fedora édite/pousse, VM Ubuntu pull/exécute — ne jamais mélanger les branches entre les deux.
- **2026-06-10** — **Sonar Context — étape 1/5 codée** (`feature/sonar-context`) : module pur NumPy `sonar_context.py` (build_sonar_context A×R, build_polar_key, cosine_distance_shifted avec adaptive shifting + zero padding) testé seul ; publication du descripteur dans `feature_extraction.py` (topic `SONAR_DESCRIPTOR_TOPIC`, `Float32MultiArray` avec timestamp encodé). Reste étapes 2-5 : réception côté SLAM, remplacement de la détection NSSM, params YAML, validation shadow. Présentation Sonar Context faite → `Paper/Sonar/SonarContext.md`.
- **2026-06-11** — **Analyse critique d'architecture (Fable 5)** consignée dans `FABLE.md` (sur main). Points clés : (1) la migration 3D vise la mauvaise cible — Bruce est 3-DOF *par conception* (z/roll/pitch observables par capteurs), pas besoin d'un back-end Pose3 6DOF complet ; viser un SLAM 4-DOF (z par pression, roll/pitch par IMU). (2) DISO+Bruce < DISO standalone n'est **pas un bug mais une propriété** : sans loop closure valide, l'odométrie seule ne crée pas d'info → impossible de battre DISO standalone. (3) Bruce original (SSM/ICP) ne peut pas faire mieux : son ICP suppose une bonne odométrie d'init (DVL+IMU) absente sur Aracati. (4) Test jamais fait à tenter : réactiver `ssm.enable: True` PAR-DESSUS DISO (DISO comme init de l'ICP). (5) Bug subtil d'éval : `allow_reflection=True` dans Umeyama n'est pas standard (evo force det(R)=+1) → corriger le signe Y de DISO à l'export. (6) CFAR appliqué sur l'image cartésienne = statistiquement bancal. **Priorité confirmée : Sonar Context (seul levier qui ajoute de l'information).**
- **2026-06-12** — **Présentation SIO-UV préparée** → `Paper/Sonar/SIO-UV.md`. Papier IEEE TIE 2025 qui **bat directement Bruce-SLAM** (RMSE 1.68 m vs 22.2 m en piscine) car l'ICP front-end de Bruce explose dans les virages → **preuve externe** que remplacer le SSM par DISO est justifié (à citer au jury). Trois briques : MCFAR (débruitage multi-échelle), conversion 2D→3D par stacking vertical, odométrie 3D-LOAM. **Décision : ne piocher que MCFAR** (seule brique portable sur Aracati — pas d'IMU ; remplacement du CFAR mono-échelle, bénéficie à DISO et Sonar Context). Les deux autres briques : 3D-stacking = fausse 3D, 3D-LOAM = redondant avec DISO + dépend de l'IMU. **SIO-UV n'est PAS en compétition avec Sonar Context** : SIO-UV améliore l'odométrie, Sonar Context la détection de boucle (le vrai verrou). Sonar Context reste prioritaire.

- **2026-06-12** — **BUG MAJEUR trouvé et corrigé : géométrie des features Aracati** (`feature_extraction.py::callback_cartesian`). Le nuage de points Bruce était une boule de traînées radiales (cf. `results/run_aracati_2026-06-10_181511/pointcloud_map.png`) alors que celui de DISO est impeccable. Cause : le code interprétait l'image **cartésienne** d'Aracati comme **polaire** (ligne=range, colonne=angle) — héritage du pipeline polaire Oculus. Or les pixels sont des **mètres** (échelle uniforme m/px, origine bas-centre), exactement le modèle de DISO (`Frame.cpp:110-113`) — d'où sa carte propre. Conséquence : x' = y·sin(k·x) → toute structure verticale devenait un rayon passant par le sonar. Correctif : conversion métrique directe (`m_per_px = max_range/h`) + **masque du fan** (r∈[0.3, R−0.3], |bearing| < FOV/2 − 0.05 rad, mêmes marges que DISO) qui supprime les fausses détections CFAR à la frontière fan/padding noir. Aussi : `max_range` aligné sur 48.2896 m (valeur SIM3 de DISO) dans `feature_aracati.yaml`. **Impact attendu** : carte propre ET features NSSM enfin géométriquement cohérentes → re-tester le loop closure (les "fake loops" venaient peut-être en partie de là). À re-runner sur la VM puis propager sur `feature/sonar-context`.

- **2026-06-12 (suite)** — **2e bug trouvé, encore plus grave : écrasement d'axe au parsing**. Le re-run RViz montrait toujours des rayons colorés (un par keyframe). Cause : `slam_ros.py` parse le nuage de features avec `(x, −z)` (convention du pipeline polaire qui publie `[x, 0, z]`), or `_publish_features_stamped` publiait `[x, y, 0]` → le SLAM lisait `(x, 0)` : **chaque keyframe était écrasé sur une ligne le long de son axe x** (les rayons RViz = ces lignes tournées par le cap, couleur = id keyframe). 2e correctif : packing `[x_avant, 0, −y_latéral]` pour coller au parseur. Aussi corrigé : **x = avant, y = latéral** (même repère que DISO ; avant on publiait x = latéral → la structure verte apparaissait sur l'axe vert au lieu du rouge en RViz — observation qui a permis le diagnostic). Vérif côté VM si doute sur le build : `python3 -c "import bruce_slam.feature_extraction as m; print(m.__file__)"` après `source devel/setup.bash` — si le chemin pointe vers `src/`, un pull suffit (pas de `catkin build` pour un .py).

- **2026-06-12 (suite 2)** — **3e bug : les keyframes portaient des poses du futur** (run `run_aracati_2026-06-12_140337` : odom DISO ATE 3.5 m vs Bruce 6.7 m alors que les trajectoires xy semblent identiques). Diagnostic chiffré : pose == dr_pose sur 533/533 keyframes (0 loop closure → iSAM2 ne déforme rien) ; le sous-échantillonnage n'explique rien (odom rééchantillonnée aux timestamps keyframes = 3.54 m) ; MAIS la position de chaque keyframe correspond à un échantillon odom décalé de **+3 s en médiane (max +65 s)** par rapport à son timestamp. Cause : `_feature_callback` appariait la feature (stampée t_sonar) avec `_latest_odom` = dernier message arrivé ; l'extraction CFAR étant lente, la feature arrive en retard pendant que DISO continue de publier → pose du futur. À 0.28 m/s, +3 s ≈ 2.5 m = l'écart 6.7 vs 3.5 m. Invisible sur le tracé xy (même courbe, seuls les temps sont faux) — l'ATE, qui compare aux temps appariés, le voit. **Correctif** : buffer d'odométrie + `_interpolate_odom()` (position linéaire, slerp orientation) **au timestamp de la feature** dans `slam_ros.py`. Attendu : Bruce sans loop closure ≈ odom DISO, et les loop closures ne pourront plus qu'améliorer. NB : le +65 s = backlog de queue → la contention CPU reste réelle (donner plus de cœurs à la VM / tester `rosbag play -r 0.5`).

- **2026-06-12 (validation)** — **Run `run_aracati_2026-06-12_154852` : les 5 correctifs validés.** La carte montre clairement le port (pontons en T, comme la carte DISO) avec un halo de speckle résiduel. Chiffres : **ATE Bruce 3.35 m ≈ ATE odom DISO 3.25 m**, écart Bruce-odom aux mêmes timestamps = **0.000 m** (vs 3.44 m avant le fix 3), 631 keyframes (vs 533 : la file d'attente évite les pertes), 0 loop closure. Confirmation structurelle (FABLE.md §2) : sans loop closure, iSAM2 ≡ chaîne d'odométrie — Bruce SE CONFOND avec l'odom DISO, c'est mathématiquement attendu, pas un bug. L'odom du plot EST DISO (5.2 Hz, ATE 3.25 m ; l'odométrie pure /cmd_vel du bag ferait 14.7 m) — `odometry.csv` = `LOCALIZATION_ODOM_TOPIC` = sortie DISO via odom_bridge. Le levier pour battre 3.25 m reste le loop closure (Sonar Context) ; le speckle résiduel se réduit via threshold 50→65 ou min_points 5→8 (un bouton à la fois).

- **2026-06-12 (soir)** — **Sonar Context : étapes 2-5 intégrées** sur `feature/sonar-context` (commit `2c7c995`) + `threshold: 65` anti-speckle sur main. Architecture : `sonar_context_candidate()` (kNN brute-force sur Polar Keys puis distance cosinus avec adaptive shifting) **remplace uniquement la sélection de candidat** du NSSM (gating covariance + argmax counts — la source des fake loops) ; tout l'aval inchangé (shgo, ICP+cov, PCM). Descripteur attaché aux champs `ring_key`/`context` du Keyframe (prévus et jamais utilisés), associé par timestamp via `_descriptor_buffer`. **Fix FABLE §4 appliqué** : `_polar_remap()` repolarise le fan cartésien avant le descripteur (sinon shift colonne ≠ rotation). Journal `loops_detected.csv` (source, target, sc_dist, shifts, retenu) pour calibrer le seuil et mesurer precision/recall. Validation synthétique : lieu revisité retrouvé (dist ~0, shift azimut exact) ; seuil initial 0.25 **à calibrer avec l'histogramme du 1er run**. **Prochains runs** : A = main (Bruce+DISO, carte propre threshold 65) ; B = feature/sonar-context (Bruce+DISO+Sonar Context — AVEC DISO : Sonar Context remplace la détection de boucle, pas l'odométrie ; sans DISO on retombe sur /cmd_vel à 14.7 m). A vs B isole la contribution de Sonar Context.

---

## Datasets intégrés

| Dataset | Sonar | Odométrie | Statut | Notes |
|---------|-------|-----------|--------|-------|
| sample_data | Oculus M750d (polaire) | DVL + IMU | ✅ Fonctionnel | Pipeline original bruce_slam |
| Aracati2017 | BlueView P900-130 (cartésien PNG, 130°, 50m) | **DISO** (remplace SSM) | ✅ Fonctionnel | DISO standalone ATE 2.1–3.0 m ; DISO+Bruce 5.2–5.4 m. Pas d'IMU/DVL. Dataset **2D matériel** (pas d'élévation) |
| HoloOcean | sonar simulé (+ futur PointCloud2 3D) | IMU + DVL | ⏳ À venir | Simulation avec Z → cible de la migration 3D. Bag corrigé attendu du collègue (IMU + élévation sonar) |

## Papiers

| Papier | Auteurs | Conf. | Lien local | Statut |
|--------|---------|-------|-----------|--------|
| Bruce-SLAM | Wang et al. | JOE 2022 | `Paper/Bruce-SLAM_260511_085801.pdf` | ✅ Présenté (semaine 1) — `BRUCE_SLAM.md` |
| DISO | Xu et al. | ICRA 2024 | `Paper/Sonar/DISO_Direct_Imaging_Sonar_Odometry.pdf` | ✅ Présenté + **intégré** — `Paper/Sonar/DISO.md` |
| Place Recognition (Sonar Context) | Kim et al. | ICRA 2023 | `Paper/Sonar/Robust_Imaging_Sonar-based_...pdf` | ✅ Présenté + **en intégration (1/5)** — `Paper/Sonar/SonarContext.md` |
| SIO-UV | Bai et al. | IEEE TIE 2025 | `Paper/Sonar/SIO-UV_Rapid_and_Robust_...pdf` | ✅ Analysé — `Paper/Sonar/SIO-UV.md` (n'en garder que MCFAR) |

## Documents d'analyse internes

| Document | Contenu |
|----------|---------|
| `FABLE.md` | Analyse critique d'architecture (Fable 5) : migration 3D, DISO+Bruce vs standalone, Sonar Context, pistes priorisées pour passer sous 3 m d'ATE |
| `SLAM_3D_MIGRATION.md` | Cadrage migration 2D→3D (HoloOcean uniquement ; Aracati reste 2D) |
| `Paper/Sonar/DISO.md` · `SonarContext.md` · `SIO-UV.md` | Résumés détaillés des 3 papiers d'amélioration |

## 2026-06-12 (soir) — Migration VM Ubuntu → Fedora natif (distrobox)

**Motivation** : la VM (8 cœurs/8 Go) bridait DISO ; contraintes de partage de fichiers.

**Solution** : conteneur Ubuntu 20.04 via distrobox+podman (natif, $HOME partagé, RViz sur le bureau Fedora).
Script idempotent : `setup_ros_noetic.sh` (relançable, étapes marquées dans `~/.ros1_setup_state/`).

**Pièges rencontrés et corrigés** :
1. Repos GitHub jake3991 disparus (`sonar_oculus`, `rti_dvl`, `bar30_depth`, `kvh_gyro`)
   → stubs de messages minimaux (champs vérifiés dans le code).
2. **Dockerfile DISO incohérent** : il épingle g2o `21b7ce45` (avril **2016**, API pointeurs bruts)
   alors que le code DISO utilise l'API `unique_ptr` (post-2017) → g2o `20201223_git`.
3. PCL 1.10 (focal) exige C++14 → `bruce_slam/CMakeLists.txt` passé de c++11 à c++14.
4. numpy système 1.17 trop vieille pour scipy/pandas → pip `numpy==1.23.5`
   (dernière qui garde `np.float` pour ros_numpy).
5. catkin_tools : `SHELL` doit être un chemin absolu.

**Usage** : `./run_slam.sh aracati` depuis Fedora (le script entre tout seul dans le conteneur).
Bag par défaut : `ARACATI_2017_8bits_full.bag` à la racine du repo.

## 2026-06-15 — Ablation DISO/GT : DISO est largement une vraie odométrie sonar

**Question** : DISO sur Aracati utilise `OdomTopic: /pose_gt` (la GT) comme prior
d'odométrie par frame. Le résultat est-il de l'odométrie SONAR réelle ou un artefact
de la GT ?

**Protocole** : `gt_drift_node.py` republie `/pose_gt` → `/pose_gt_drift` en injectant
une dérive connue (0,003 m/s latéral ≈ 7,9 m sur 2640 s + 3° de biais yaw). DISO
consomme la GT dérivée (`config_aracati2017_drift.yaml`). Run `run_diso_nogt_2026-06-15_101108`.
Analyse : `ablation_diso_gt.py` (Umeyama avec réflexion).

**⚠️ Piège méthodologique corrigé** : la 1re version comparait DISO normal **sur VM**
(2,07 m) à DISO dérivé **sur Docker** (5,87 m) → croisement d'environnements. Or DISO
est très sensible à l'environnement (cf. section non-déterminisme ci-dessous) : sur Docker,
DISO normal fait déjà **4,57 m** sans aucune corruption. La comparaison juste se fait
**à environnement constant** (Docker normal vs Docker dérivé).

**Résultats à environnement constant (Docker, référence vraie GT)** :

| Config | ATE |
|--------|-----|
| DISO normal (prior = vraie GT) | **4,57 m** |
| DISO + GT corrompue de 7,9 m | **5,97 m** |
| DISO corrompu vs GT *dérivée* | 6,00 m |

**Verdict : DISO corrige ~82 % de la dérive injectée → odométrie sonar réelle.**
- Corrompre la GT de **7,9 m** ne dégrade DISO que de **+1,4 m** (4,57 → 5,97) →
  seulement ~18 % de l'erreur GT se propage, le sonar corrige le reste.
- DISO corrompu reste **plus proche de la vraie GT (5,97) que de la GT dérivée (6,00)**
  → il ne suit PAS le prior, le sonar le ramène vers la réalité.
- Le prior GT n'est donc qu'une **ancre faible** (init du tracker + scale), pas la source
  principale de la trajectoire. C'est défendable devant le jury comme vraie odométrie sonar.

**Implications** :
1. La performance DISO de référence (~2-3 m) est **honnête** (mesurée VM) ; ce n'est pas
   un artefact GT. Le prior GT aide à l'init mais le sonar fait le gros du travail.
2. Le gain relatif Sonar Context (+0,63 m, run 212922) reste valide.
3. **Une baseline « sonar pur » n'est PAS un simple changement de config.**
   `System.cpp:158-160` synchronise l'image sonar ET l'odom via `ApproximateTime` :
   sans message odom, le callback `frameLoad` ne se déclenche jamais → 0 frame. DISO
   *exige* donc un topic odom. De plus le prior `T_b0_bi` sert d'**init au tracker direct**
   (optimiseur local → converge près de son init) ET au placement du nuage de points.
   - Fournir un prior identité figé ne donnerait pas un « sonar pur » honnête : le tracker
     serait initialisé à l'origine pour *toutes* les frames (init absolu), donc faux dès que
     le ROV s'éloigne → résultat artificiellement mauvais.
   - Une vraie baseline sonar pur demande une **modif C++** : initialiser chaque frame avec
     l'estimée sonar de la frame précédente (frame-to-frame) au lieu de la GT. À évaluer
     si on veut ce chiffre.

## 2026-06-15 — DISO dépend de l'environnement d'exécution (VM ≠ Docker)

**Constat** : DISO standalone, MÊME bag / MÊME launch / MÊME config, donne :

| Environnement | Frames traitées | ATE |
|---------------|-----------------|-----|
| VM Ubuntu (06-10, avant migration) | 13 143 | **2,07 m** |
| Docker/Fedora (06-15, run 105930) | 14 298 | **4,57 m** |

Même span temporel (462 → 459102). Donc ce n'est PAS du bruit run-à-run : c'est
**systématique et lié à l'environnement** (le saut coïncide avec la migration du 12 juin).

**Cause** : Docker traite ~9 % de frames en plus → le synchroniseur `ApproximateTime`
(`System.cpp:158-159`) **apparie les couples (sonar, GT) différemment** selon
l'ordonnancement CPU. Hypothèse : sur Docker, plus de frames passent mais certaines sont
mal appariées dans le temps → init du tracker légèrement décalé → dérive accumulée.
Ce n'est pas du bruit numérique (sinon le nombre de frames serait identique).

**Conséquence** : pour les chiffres finaux du rapport, refaire le run DISO **sur la VM**
(performance propre ~2-3 m), ou régler le synchro Docker (queue / `slop` de l'`ApproximateTime`,
rate du bag) pour reproduire l'appariement VM.

## 2026-06-15 — BUG : bruce_save n'écrit qu'1 CSV sur 5 (Docker)

**Symptôme** : après un run DISO sur Docker, malgré le CTRL+C demandé, seul
`diso_trajectory.csv` est créé (les 4 autres — diso_odom, odometry, pointcloud, groundtruth —
manquent). Reproductible.

**Cause** : `bruce_save.py` écrit les 5 CSV séquentiellement dans `rospy.on_shutdown(export)`.
Seul le 1er (complet) survit → `export()` est **tué juste après le 1er fichier**. C'est une
**course de signaux** dans `run_slam.sh` : le CTRL+C envoie SIGINT à roslaunch (arrêt propre →
export), mais fait aussi sortir le script → le `trap ... EXIT` envoie `kill` (SIGTERM) à
roslaunch → SIGTERM avorte l'arrêt propre avant la fin de l'export. Sur la VM le timing
laissait passer les 5 écritures ; sur Docker non. Le code d'`export()` est correct.

**Fix possible** (non appliqué) : dans `run_slam.sh`, remplacer le `kill` du trap par
`kill -INT` + `sleep` pour laisser `bruce_save` finir ; ou faire écrire les CSV à la fin du
bag plutôt qu'au shutdown.

# Résumé complet — DRACo-SLAM: Distributed Robust Acoustic Communication-efficient SLAM for Imaging Sonar Equipped Underwater Robot Teams

**Titre :** *DRACo-SLAM: Distributed Robust Acoustic Communication-efficient SLAM for Imaging Sonar Equipped Underwater Robot Teams*
**Auteurs :** John McConnell, Yewei Huang, Paul Szenher, Ivana Collado-Gonzalez, Brendan Englot
**Institution :** Stevens Institute of Technology, Hoboken, NJ, USA
**Publié :** arXiv:2210.00867v1, octobre 2022
**Code source :** https://github.com/jake3991/DRACo-SLAM

---

## Abstract

DRACo-SLAM est le **premier système de SLAM multi-robot** utilisant de vraies données de sonar imageur. Chaque robot maintient son propre SLAM single-agent (DVL + IMU + sonar, factor graph iSAM2), puis les robots échangent des **descripteurs de scène compacts** (histogrammes) pour détecter les inter-robot loop closures. Les données sonar brutes ne sont transmises que si une boucle est probable — ce qui maintient la **bande passante acoustique faible**. Les fausses boucles inter-robot sont rejetées par PCM. Validé sur deux environnements réels (marina SUNY Maritime, 2 robots ; marina USMMA, 3 robots).

---

## I. Introduction

### Contexte

- Sous l'eau : pas de GPS, vision limitée → le **sonar imageur** est le capteur de référence.
- Les AUVs travaillent typiquement seuls, mais une **flotte** réduit le temps, l'énergie et crée de la redondance.
- Problème fondamental : les communications acoustiques ont une **bande passante très faible** (WHOI Micromodem : 5400 bits/s max sur longue portée).

### Trois défis spécifiques

1. **Pas de condition initiale** entre les repères des robots (pas de GPS commun).
2. **Bande passante limitée** : impossible d'envoyer des nuages de points bruts.
3. **Robustesse** : les fausses boucles inter-robot doivent être détectées et rejetées.

### Contributions

1. Premier SLAM multi-robot sous-marin avec vraies données sonar imageur.
2. Pipeline robuste pour détecter, enregistrer et rejeter les boucles inter-robot sans condition initiale.
3. Stratégie de communication pour minimiser l'utilisation du réseau acoustique.
4. Validation sur deux datasets réels (2 et 3 robots).

---

## II. Related Work

### Multi-robot SLAM sous-marin existant

- Approches par **rencontres directes** (ranging acoustique) : nécessitent des horloges synchronisées — peu pratique.
- Approches par **cibles communes** observées (features partagées) : suppose que les robots voient les mêmes objets.
- **Méthodes multi-session** : un robot charge la carte d'une session précédente — ne communique pas entre robots en temps réel.

### Place recognition avec sonar

- Features classiques (KAZE, AKAZE, ORB) : peu fiables sous l'eau (faible SNR sonar).
- Deep learning : trop coûteux en mémoire et calcul pour du temps réel embarqué.
- **Histogrammes range-based** (ce papier) : rapides, compacts, invariants en rotation → adaptés au sonar.

> Bruce-SLAM [25] est cité comme base du SLAM single-agent (même architecture : SSM + NSSM + PCM + iSAM2).

---

## III. Description du problème

- N robots, chacun dans son propre repère $\mathcal{I}_n$ (pas de repère commun a priori).
- Pose 2D (profondeur fixe) : $\mathbf{x}_{n,t} = (x_r, y_r, \theta_r)^\top$.
- Observations sonar : coordonnées sphériques (range, azimuth, élévation) → converties en nuage 2D (φ=0).
- Objectif : estimer les trajectoires de **tous les robots** dans le repère de chacun, sans condition initiale.

---

## IV. Algorithme

### A. Traitement sonar (single-agent, hérité de Bruce-SLAM)

- **CFAR** pour identifier les pixels de contact → nuage de points 2D.
- **Voxel downsampling** (résolution Δ_compression) pour réduire la taille des nuages à transmettre.

### B. Compression des nuages (nouveauté communication)

Chaque point (x,y) float32 (32 bits × 2) est discrétisé en entiers 8 bits :

$$p_{i,j} \approx \frac{p_{x,y}}{\Delta_{compression}}$$

→ **×4 de compression** par rapport au float32. Coût : légère perte géométrique (évaluée en Section V).

### C. SLAM single-agent (hérité de Bruce-SLAM)

Factor graph iSAM2 avec trois types de facteurs :
- **f⁰** : prior initial.
- **f^SSM** : facteurs d'odométrie séquentielle (ICP entre frames consécutives).
- **f^NSSM** : facteurs de loop closure intra-robot (ICP entre frames non-consécutives, filtré par PCM).

Odométrie : **DVL + IMU** (dead reckoning, pas de cmd_vel).

> Sur Aracati, on n'a pas de DVL ni d'IMU — c'est pourquoi notre odométrie (cmd_vel) est bien moins précise que celle de DRACo.

### D. SLAM multi-robot distribué

Chaque robot ajoute des facteurs **inter-robot (f^IR)** et **partner robot (f^PR)** au graphe :

- **f^IR** : contrainte de boucle inter-robot (quand deux robots ont vu le même endroit).
- **f^PR** : facteurs séquentiels représentant la trajectoire du robot partenaire (série de BetweenFactors).

→ Le graphe de chaque robot inclut les trajectoires de toute l'équipe (Fig. 3).

### E. Recherche de boucles inter-robot

Chaque robot encode chaque keyframe en un **descripteur de scène** : histogramme 1D du nombre de points par bin de range. Ce descripteur est :
- **Invariant en rotation** (range bins = anneaux concentriques).
- **Compact** : 0.128 kbits vs 9.14 kbits pour un nuage float32.

Chaque robot maintient un **KD-tree** de ses propres descripteurs. Quand il reçoit le descripteur d'un robot partenaire, il cherche le plus proche voisin. Si la distance est sous un seuil → candidat inter-robot → on demande le nuage de points brut.

### F. Registration (enregistrement géométrique)

Transformation 3DOF estimée entre les deux nuages :

$$T = (x_\Delta, y_\Delta)^\top, \quad R = \begin{pmatrix}\cos\theta_\Delta & -\sin\theta_\Delta \\ \sin\theta_\Delta & \cos\theta_\Delta\end{pmatrix}$$

Sans condition initiale → **Go-ICP** (ICP global, certifié optimal sous partial overlap) puis ICP standard pour affiner.

### G. Rejet des outliers

Avant registration :
- Minimum de points dans le nuage.
- Ratio de tailles entre les deux nuages (scènes similaires → ratio proche de 1).
- **Scene image** : projection sur grille grossière → somme de différences absolues (SAD). Si SAD trop grande → scènes différentes → pas de registration.

Après registration :
- **Overlap** : % de points avec un voisin à < 0.5 m. Si insuffisant → rejeté.
- **PCM** : rejet des boucles mutuellement incohérentes.

### H. Stratégie de communication

1. Chaque robot partage ses **descripteurs** à chaque nouvelle keyframe (0.128 kbits → quasi gratuit).
2. Si un candidat est trouvé → demande du **nuage compressé** (2.28 kbits, vs 9.14 kbits brut).
3. Quand une correction majeure survient → partage des **poses mises à jour** (f^PR) avec l'équipe.

→ Les données brutes ne transitent que si une boucle est probable → **bande passante minimisée**.

---

## V. Expériences

### Hardware

- BlueROV2-Heavy customisé avec :
  - Sonar vertical : Blueprint Subsea Oculus M1200d.
  - **Sonar horizontal (SLAM) : Oculus M750d** (130° FOV, 30 m range, 20° vertical).
  - DVL : Rowe SeaPilot.
  - IMU : VectorNav VN-100 MEMS.
  - Baromètre Bar30 (profondeur).

### Datasets

| Dataset | Robots | Environnement | Opportunités inter-robot |
|---------|--------|---------------|--------------------------|
| SUNY Maritime | 2 | Marina (Bronx, NY) | 1 (trajectoires opposées) |
| USMMA | 3 | Marina (Kings Point, NY) | Plusieurs (trajectoires similaires) |

Communication simulée via ROS message passing (offline playback de vrais bags).

### Métriques

- **Inter-robot error** (MAE / RMSE en m et °) : erreur de la position estimée des robots partenaires, transformée dans le repère de chaque robot.
- **Network utilization** (bits/s) : traffic de messages (hors overhead protocole).
- Pas de GT absolu disponible → baseline = SLAM single-agent stable de chaque robot.

### Résultats (Table I — meilleur cas = Case 4, système complet)

| Dataset | MAE position (m) | RMSE position (m) | MAE angle (°) | Réseau moyen (bits/s) |
|---------|-----------------|-------------------|---------------|----------------------|
| SUNY Maritime (2 robots) | **1.92** | **1.98** | **3.27** | 337.62 |
| USMMA (3 robots) | **1.44** | **1.62** | **2.09** | 1244.84 |

- **Case 4 (système complet + compression)** = meilleur compromis erreur / réseau.
- Compression réduit le réseau de ~×4 pour un surcoût d'erreur négligeable.
- PCM (Case 3→4) améliore systématiquement l'erreur inter-robot.
- Sans mise à jour des poses partenaires (Case 5) → erreur fortement dégradée.

### Analyse communication (Table III)

| Données transmises | Taille moyenne (kbits) |
|--------------------|------------------------|
| Descripteur de scène | **0.128** |
| Nuage float32 | 9.14 |
| **Nuage compressé** | **2.28** |
| Features KAZE | 116.20 |
| Features AKAZE | 20.07 |
| Features ORB | 82.54 |

→ Le descripteur de scène est **×70 plus petit** qu'un nuage float32, et **×160 plus petit** que des features KAZE. L'approche est radicalement plus économe que les alternatives feature-based.

---

## VI. Conclusion

DRACo-SLAM = premier SLAM multi-robot sonar imageur en milieu réel :
- Détection de boucles inter-robot sans condition initiale ni GPS.
- Communication acoustique réaliste (bande passante maîtrisée).
- Validation sur 2 et 3 robots en marina réelle.
- Perspectives : détection de perceptual aliasing, SLAM multi-robot actif, déploiement sur modem acoustique matériel.

---

## Lien avec mon projet (Bruce-SLAM + Aracati2017)

### Ce que DRACo-SLAM partage avec mon pipeline

DRACo-SLAM **s'appuie directement sur Bruce-SLAM** [25] pour le SLAM single-agent (même factor graph, même SSM/NSSM, même PCM, même iSAM2). C'est la même base de code — ce papier est une extension multi-robot de Bruce.

### Différences critiques avec Aracati2017

| Aspect | DRACo-SLAM | Mon projet (Aracati2017) |
|--------|-----------|--------------------------|
| Odométrie | **DVL + IMU** (dead reckoning précis) | **cmd_vel** (intégration vitesses commandées, dérive ~15 m) |
| Sonar | Oculus M750d (Oculus, 3D partiel) | BlueView P900-130 (2D pur, basse résolution) |
| Nombre de robots | 2-3 | 1 |
| GT disponible | Non (baseline = single-agent SLAM) | Oui (DGPS + compas) |

### Ce qu'on peut réutiliser

- **Descripteur de scène (histogramme range-based)** : compact, invariant en rotation, pas de features. Utilisable pour améliorer la détection de loop closure intra-robot aussi (alternative ou complément à Sonar Context).
- **Compression voxel** : déjà dans Bruce (point_resolution). La compression 8 bits est une optimisation communication, pas nécessaire pour notre usage single-robot.
- **PCM** : déjà dans notre pipeline (min_pcm=6).

### Ce qu'on ne peut pas réutiliser

- **DVL + IMU** : absent sur Aracati. Notre dérive cmd_vel (~52° de cap) est bien pire que le dead reckoning DVL/IMU de DRACo → nos loop closures ont besoin d'une meilleure initialisation (d'où Sonar Context avec adaptive shifting).
- **Go-ICP global** : trop coûteux computationnellement pour du temps réel sur notre setup.

### Position dans la littérature

DRACo-SLAM confirme que **Bruce-SLAM est la référence single-agent** dans ce domaine, et que ses limites (ICP front-end sensible au bruit sonar) sont connues. Notre contribution (DISO + USBL + Sonar Context) améliore le single-agent de Bruce sur un dataset difficile (BlueView, pas de DVL/IMU) — ce que DRACo ne traite pas.

---

## Glossaire

| Terme | Définition |
|-------|-----------|
| **Inter-robot loop closure** | Boucle détectée entre deux robots différents (ils ont vu le même endroit) |
| **f^IR** | Facteur inter-robot dans le graphe : contrainte de pose relative entre deux robots |
| **f^PR** | Facteur partner robot : série de facteurs séquentiels représentant la trajectoire d'un robot partenaire |
| **Go-ICP** | ICP global sans initialisation, certifié optimal sous partial overlap |
| **PCM** | Pairwise Consistency Maximization — rejet des boucles mutuellement incohérentes |
| **Scene descriptor** | Histogramme 1D du nombre de points par bin de range — compact et invariant en rotation |
| **Scene image** | Projection du nuage sur grille grossière — used for rapid scene comparison avant ICP |
| **DVL** | Doppler Velocity Log — capteur de vitesse acoustique (absent sur Aracati) |
| **WHOI Micromodem** | Modem acoustique de référence (5400 bits/s max, longue portée) |
| **Perceptual aliasing** | Deux lieux différents qui se ressemblent → fausse boucle détectée |

# Résumé complet de l'article Bruce-SLAM

**Titre :** *Virtual Maps for Autonomous Exploration of Cluttered Underwater Environments*
**Auteurs :** Jinkun Wang, Fanfei Chen, Yewei Huang, John McConnell, Tixiao Shan, Brendan Englot
**Institution :** Stevens Institute of Technology, Hoboken NJ
**Publié :** IEEE Journal of Oceanic Engineering, 2022 — arXiv:2202.08359v1

---

## Abstract

Le papier s'attaque à l'exploration autonome de robots mobiles dans un environnement inconnu, en cherchant à optimiser simultanément trois critères : **taux de couverture**, **incertitude de carte**, et **incertitude d'état** (localisation). La contribution centrale est le concept de **carte virtuelle** (*virtual map*) : une représentation de l'incertitude associée à chaque cellule de la carte, y compris dans les zones non encore visitées. Sur cette base, un algorithme d'exploration **EM** (Expectation-Maximization) est proposé, qui planifie des chemins en anticipant l'effet des futures observations sur l'incertitude globale. Le système est construit sur un SLAM à base de **keyframes sonar** (sonar imageur 2D). Validé en simulation (45 et 60 essais) et sur un **BlueROV2** réel dans un port.

---

## I. Introduction

### Contexte et problème

Le SLAM (*Simultaneous Localization and Mapping*) permet à un robot de se localiser et de construire une carte en même temps, à partir de ses capteurs. C'est bien maîtrisé en mode passif (le robot suit une trajectoire imposée). Le défi ici est l'**exploration active** : le robot décide lui-même où aller, et doit gérer le compromis entre :
- **Explorer** des zones inconnues → réduit l'incertitude de carte
- **Revisiter** des zones connues → réduit la dérive de localisation (via *loop closures*)

Ce compromis est fondamental : une exploration purement frontière accumule de la dérive, et une carte inexacte rend l'exploration inefficace.

### Pourquoi le sous-marin est difficile

Les environnements sous-marins encombrés (ports, épaves, structures offshore) sont particulièrement difficiles :
- Pas de GPS
- Télé-opération limitée ou impossible
- Le sonar est le seul capteur de perception à distance utile dans l'eau

### Travaux antérieurs des mêmes auteurs

Ce papier étend deux travaux précédents des auteurs [2][3] sur l'exploration EM avec landmarks virtuels, initialement développés pour des robots terrestres avec lidar. L'extension au sonar sous-marin nécessite de repenser l'extraction de features et le modèle de capteur.

### Contributions listées

1. Exposition détaillée de la propagation de croyance sur actions candidates et landmarks virtuels pour l'implémentation temps réel à grande échelle
2. Architecture SLAM sous-marin robuste avec sonar imageur, support de la planification et décision en temps réel
3. Évaluation rigoureuse en simulation du compromis exploration/localisation/cartographie
4. **Première** démonstration d'exploration autonome réelle en environnement extérieur encombré avec un ROV (BlueROV2, marina de King's Point NY)

---

## II. Related Work

### Exploration par frontière

L'approche classique est le **Nearest Frontier (NF)** [10] : envoyer le robot vers la cellule frontière la plus proche (limite entre espace libre et espace inconnu). Simple et efficace pour la couverture, mais ignore totalement l'incertitude de localisation.

### Maximisation d'information

Des travaux maximisent le **gain d'information mutuelle** [11][12] ou l'entropie de carte. L'approche **Next-Best-View (NBV)** [16] sélectionne la configuration qui apporte le plus d'information sur la carte. Très bon taux de couverture, mais génère une pose uncertainty élevée car elle ne cherche pas de loop closures.

### Active SLAM

L'**Active SLAM** [25][26][27] intègre l'incertitude de pose dans la décision. Par exemple [28] : quand l'incertitude dépasse un seuil (critère D-optimalité [34]), le robot planifie un détour pour revisiter une zone et fermer une boucle. Référencé comme approche **heuristique** dans ce papier.

### Travaux sonar sous-marin

- **Vidal et al. [23]** : premier usage d'une grille d'occupation sonar 2D pour explorer des environnements encombrés avec un AUV (*next frontier*)
- **Palomeras et al. [24]** : extension 3D avec next-best-view pondérant distance, frontières, et contour-following
- **Suresh et al. [28]** : active SLAM 3D sonar volumétrique, loop closures déclenchées par saillance de sous-carte

### Planification dans l'espace des croyances (BSP)

Les méthodes BSP [30][31] planifient directement dans l'espace des distributions de probabilité (POMDPs). Très puissantes en théorie, mais coût computationnel prohibitif à grande échelle. Ce papier offre une alternative scalable via les landmarks virtuels.

---

## III. EM Exploration

### III.A — Simultaneous Localization and Mapping

#### Modèle de mouvement

La pose du robot au temps $i$ est $\mathbf{x}_i = [x_i, y_i, \theta_i] \in SE(2)$ (position 2D + cap). Le modèle de mouvement est :

$$\mathbf{x}_i = f_i(\mathbf{x}_{i-1}, \mathbf{u}_i) + \mathbf{w}_i, \quad \mathbf{w}_i \sim \mathcal{N}(\mathbf{0}, Q_i) \tag{1}$$

où $\mathbf{u}_i$ est la commande de contrôle et $\mathbf{w}_i$ est un bruit Gaussien de covariance $Q_i$.

#### Modèle de mesure

La mesure $\mathbf{z}_{ij}$ d'un landmark $\mathbf{l}_j$ depuis la pose $\mathbf{x}_i$ :

$$\mathbf{z}_{ij} = h_{ij}(\mathbf{x}_i, \mathbf{l}_j) + \mathbf{v}_{ij}, \quad \mathbf{v}_{ij} \sim \mathcal{N}(\mathbf{0}, R_{ij}) \tag{2}$$

#### Estimation MAP

L'objectif est de trouver la trajectoire $\mathcal{X}^* = \{\mathbf{x}_i\}$ et les landmarks $\mathcal{L}^* = \{\mathbf{l}_j\}$ qui maximisent la probabilité jointe :

$$\mathcal{X}^*, \mathcal{L}^* | \mathcal{Z} = \underset{\mathcal{X}, \mathcal{L}}{\arg\max} \ P(\mathcal{Z} | \mathcal{X}, \mathcal{L}) P(\mathcal{X}) \tag{3}$$

Après linéarisation des fonctions non-linéaires, cela devient un **problème de moindres carrés** de la forme $\delta^* = \arg\min \frac{1}{2} \|A\delta - \mathbf{b}\|^2$ (Eq. 12), résolu par l'équation normale $A^\top A \delta = A^\top \mathbf{b}$ (Eq. 13). La matrice d'information est $\Lambda = A^\top A$ et la covariance $\Sigma = \Lambda^{-1}$ (Eq. 14). Résolu incrémentalement par **iSAM2** (bibliothèque GTSAM [32]).

Deux modes : **Landmark SLAM** (landmarks explicites dans le graphe) et **Pose SLAM** (seulement des contraintes entre poses, adapté aux environnements sans structures distinctes).

---

### III.B — EM-Exploration

#### Problème

Lors de l'exploration, le robot ne connaît pas encore les landmarks futurs. On ne peut donc pas calculer directement l'impact d'une trajectoire candidate sur l'incertitude.

#### Solution : landmarks virtuels

On introduit $\mathcal{V} = \{\mathbf{v}_k\}$ comme **variables latentes** représentant les landmarks qui seraient observés si le robot suivait le chemin candidat $X_{T:T+N}$. L'objectif devient maximiser la **log-vraisemblance marginale** :

$$\mathcal{X}^s = \underset{\mathcal{X}}{\arg\max} \log \sum_{\mathcal{V}} P(\mathcal{X}, \mathcal{Z}, \mathcal{V}) \tag{4}$$

#### Algorithme EM

Cette somme est intractable directement. L'algorithme EM alterne :
- **E-step** : $q(\mathcal{V}) = p(\mathcal{V} | \mathcal{X}^{old}, \mathcal{Z})$ — estimer la distribution des landmarks virtuels (Eq. 5)
- **M-step** : $\mathcal{X}^{new} = \arg\max_\mathcal{X} \mathbb{E}_{q(\mathcal{V})}[\log P(\mathcal{X}, \mathcal{V}, \mathcal{Z})]$ — trouver la trajectoire optimale (Eq. 6)

Une étape de classification C-step est ajoutée avant le M-step pour sélectionner les landmarks virtuels les plus probables (Eq. 7-8).

#### Lien avec la D-optimalité

Si les mesures sont assignées pour maximiser la vraisemblance, la distribution jointe est une Gaussienne multivariée et le M-step est équivalent à :

$$\underset{\mathcal{X}}{\arg\max} \log P(\mathcal{X}, \mathcal{V}^*, \mathcal{Z}) = \underset{\mathcal{X}}{\arg\min} \log \det(\Sigma) \tag{10}$$

**Minimiser le déterminant de la covariance = critère D-optimalité** — c'est la métrique d'incertitude utilisée dans tout le système.

---

### III.C — Belief Propagation on Candidate Actions

#### Problème de scalabilité

La covariance $\Sigma$ est de taille $(N_{poses} + N_{landmarks})^2$ — recalculer son déterminant pour chaque chemin candidat est trop coûteux.

#### Borne supérieure

En simplifiant les poses futures $\mathbf{x}_{T+1}, ..., \mathbf{x}_{T+N}$ et en supposant que les covariances des landmarks virtuels $\Sigma_{\mathcal{V}_k}$ sont indépendantes, on obtient une borne :

$$\log \det(\Sigma) < \log \det(\Sigma_{\mathbf{x}_{T+N}}) + \sum_k \log \det(\Sigma_{\mathbf{v}_k}) \tag{11}$$

Il suffit donc d'estimer $\Sigma_{\mathbf{x}_{T+N}}$ (covariance de la pose finale) et $\Sigma_{\mathbf{v}_k}$ (covariance de chaque landmark virtuel).

#### Propagation de covariance en 3 étapes (illustrée en Fig. 1)

1. **Entrées diagonales** : récupérer $\Sigma_{\mathbf{x}_i}$ depuis iSAM2 (décomposition de Cholesky, complexité $O(N_{nz})$ pour $R$ sparse)
2. **Boucle ouverte** : propager via le Jacobien $F_{k+t} = \frac{\partial f_{k+t}}{\partial \mathbf{x}_{k,t-1}}$ (odométrie/DVL/IMU), Eq. 16 : $\Sigma_{\mathbf{x}_{i,k+t}} = \Sigma_{\mathbf{x}_{i,k+t-1}} F_{k+t}^\top$
3. **Loop closures anticipées** : mise à jour via formule de Woodbury (Eq. 18) : $\Sigma' = \Sigma + \Delta\Sigma$, où $\Delta\Sigma = -\Sigma A_u^\top (I + A_u \Sigma A_u^\top)^{-1} A_u \Sigma$, avec $A_u$ le Jacobien des contraintes de loop closure anticipées

---

### III.D — Belief Propagation on Virtual Landmarks

#### Modèle

Un landmark virtuel $\mathbf{v}_k$ sera observé depuis les poses $\{\mathbf{x}_i\} \in SE(2)$ qui le verront le long du chemin. Le modèle de mesure inverse (Eq. 20) est $\mathbf{l} = h_i^{-1}(\mathbf{x}_i, \mathbf{z}_i)$. On suppose le modèle inversible (valide pour les mesures range-bearing des sonars multibeam).

#### Observations depuis deux poses

Deux observations depuis $\mathbf{x}_i$ et $\mathbf{x}_j$ donnent deux estimations indépendantes $\mathbf{l}_i$ et $\mathbf{l}_j$ avec covariances $\Sigma_i^l$ et $\Sigma_j^l$ (Eq. 21-22), calculées par propagation des Jacobiens $\mathbf{H}_i$ et $\mathbf{G}_i$.

#### Split Covariance Intersection (SCI) [37]

Fusionner les estimations sans connaître leur corrélation : la SCI fournit une borne supérieure conservative de la vraie covariance :

$$(\hat{\Sigma}^l)^{-1} = \left(\frac{1}{\omega} P_{i1} + P_{i2}\right)^{-1} + \left(\frac{1}{1-\omega} P_{j1} + P_{j2}\right)^{-1} \tag{22}$$

Le paramètre $\omega^* = \arg\min_{\omega \in (0,1)} \det(\hat{\Sigma}^l)$ (Eq. 23) est optimisé pour chaque landmark. La Fig. 2 illustre la fusion progressive : les ellipses rouges (SCI) encadrent toujours la vraie covariance verte (SLAM complet).

**Algorithme 1** résume le calcul complet : mise à jour de la grille d'occupation → downsampling → propagation diagonale de covariance → pour chaque landmark virtuel, fusion SCI.

---

### III.E — Extensibility to Pose SLAM

Dans le Pose SLAM, il n'y a pas de landmarks explicites — les contraintes viennent du scan matching entre keyframes. La transformation entre deux poses $\mathbf{x}_{ts} \in SE(2)$ est estimée par ICP qui minimise $J(\mathbf{x}_{ts}) = \sum \|\mathbf{T}\mathbf{p}_s - \mathbf{p}_t\|^2$ (Eq. 33-34).

Pour modéliser l'incertitude d'un landmark virtuel en Pose SLAM : les points sonar sont des observations bruitées d'une distribution Gaussienne centrée sur la vraie position du landmark. La correspondance ICP (nearest neighbor, Eq. 25) donne $\tilde{\mathbf{l}} = \arg\min_{l_i} \|\mathbf{l}_i - \mathbf{l}\|_2$. La covariance de $\tilde{\mathbf{l}}$ vérifie $\Sigma^l \succeq \mathbb{E}[(\tilde{\mathbf{l}} - \mathbf{l})(\tilde{\mathbf{l}} - \mathbf{l})^\top]$ (Eq. 26), ce qui maintient la validité de la borne SCI. Fig. 2 visualise ce processus en 4 étapes (initialisation odométrie → ICP global → raffinement → résultat).

---

### III.F — Virtual Map

La **carte virtuelle** $\mathcal{V}$ est définie à partir de la grille d'occupation : un landmark virtuel est placé dans chaque cellule dont la probabilité d'occupation dépasse 0.5 :

$$P(v_i = 1) = \begin{cases} 1 & \text{si } P(m_i = 1) \geq 0.5 \\ 0 & \text{sinon} \end{cases} \tag{27}$$

La carte virtuelle inclut donc aussi les landmarks déjà observés (important car minimiser l'incertitude sur les landmarks connus est aussi un objectif).

**Résolution basse (d = 2m)** : la grille d'occupation est à 0.2m mais la carte virtuelle est downsampleée à d = 2m. Cela réduit fortement le coût de calcul tout en conservant des performances d'exploration similaires [2]. La Fig. 3 montre des exemples de cartes virtuelles avec ellipses d'incertitude dans les cellules.

---

### III.G — Motion Planning

#### Deux ensembles de configurations cibles

**1. Frontières** (Algorithme 2) : cellules à la frontière espace libre / espace inconnu. Générées en sélectionnant itérativement les $N_f$ cellules frontières avec le plus grand dégagement $c$ (clearance map = distance à l'obstacle le plus proche), avec une séparation minimale $d$ entre candidats.

**2. Revisites** (Algorithme 3) : positions où le robot peut ré-observer des structures déjà cartographiées. Générées par k-means sur les cellules occupées → pour chaque centre de cluster, des candidats sont générés sur un cercle de rayon $r$ (portée du capteur), filtrés par dégagement minimal.

#### Roadmap et planification

Un roadmap discret est généré **une seule fois** au début de l'exploration (frontières de l'espace à explorer). A* [40] calcule les chemins vers toutes les configurations candidates. Le meilleur chemin minimise la fonction d'utilité :

$$U_{EM}(X_{T:T+N}) = -\log \det(\hat{\Sigma}_{\mathbf{x}_{T+N}}) - \sum_k \log \det(\hat{\Sigma}_{\mathbf{v}_k}) - \alpha \cdot d(X_{T:T+N}) \tag{28}$$

où $\alpha$ est un poids décroissant avec la distance parcourue (exploration moins pénalisée au début). Le chemin est re-planifié dès qu'un obstacle est découvert ou quand le robot a parcouru une distance fixée.

---

### III.H — Computational Complexity Analysis

| Composant | Complexité |
|---|---|
| Génération des frontières | $O(N_f \cdot N_{cell})$ |
| Génération des revisites | $O(N_r \cdot N_{cell})$ |
| Covariance des landmarks virtuels | $O(N_{occupied} \cdot N_{poses})$ |

Le coût est contenu grâce à :
- La carte virtuelle à basse résolution ($N_{occupied}$ faible)
- Le petit nombre de futures loop closures anticipées (sparse)
- Les mises à jour incrémentielles de covariance via iSAM2

---

## IV. Underwater SLAM

Le système SLAM est un framework **3DOF** (x, y, θ) pour robot opérant à profondeur fixe. Architecture en deux parties (Fig. 4) :
- **Front-end** (5 Hz) : extraction de features sonar, scan matching, dead reckoning DVL+IMU+pression
- **Back-end** (0.2 Hz) : optimisation du graphe de poses (iSAM2), détection de loop closures, cartographie par grille d'occupation

Un **keyframe** est ajouté au graphe quand le robot s'est déplacé de plus de 4m ou a tourné de plus de 30°.

---

### IV.A — Feature Extraction

#### Sonar imageur

Le sonar émet des impulsions acoustiques. L'écho est capturé en coordonnées polaires (range × bearing). La zone couverte est $\{(r_i, b_j) : 0 \leq i \leq N_r, 0 \leq j \leq N_b\}$. Chaque beam est traité indépendamment.

#### Détecteur SOCA-CFAR (Fig. 5)

*Smallest-Of Cell-Averaging Constant False Alarm Rate* [42] — adapté du radar, utilisé ici pour le sonar 2D.

Pour chaque cellule testée (CUT = *Cell Under Test*), on calcule les intensités moyennes dans une fenêtre glissante de part et d'autre :

$$\tilde{T}_{ij,<} = \frac{1}{N} \sum_{n=1}^N I_{i-n,j}, \quad \tilde{T}_{ij,>} = \frac{1}{N} \sum_{n=1}^N I_{i+n,j} \tag{29}$$

La statistique de test : $T_{ij} = \tau \cdot \min(\tilde{T}_{ij,<}, \tilde{T}_{ij,>})$ (Eq. 30), où $\tau$ est calculé à partir du taux de fausse alarme $P_{fa}$ souhaité (Eq. 31). Les cellules avec $I > T_{ij}$ sont des features.

**Conversion en 2D cartésien** : $x_n = r_{n_i} \cos(b_{n_j})$, $y_n = r_{n_i} \sin(b_{n_j})$ (Eq. 32). La Fig. 5 montre : (a) image sonar brute Oculus M750d 512 beams, (b) seuil adaptatif sur un beam, (c) features détectés en polaire, (d) features en cartésien.

---

### IV.B — Feature Matching

#### ICP classique

L'ICP (*Iterative Closest Point*) [38] minimise :

$$J(\mathbf{x}_{ts}) = \sum_{\mathbf{p}_t \in \mathcal{P}_t, \mathbf{p}_s \in \mathcal{P}_s} \|\mathbf{R}_{ts}\mathbf{p}_s + \mathbf{t}_{ts} - \mathbf{p}_t\|^2 \tag{34}$$

Problème : l'ICP converge vers des minima locaux si l'initialisation est mauvaise. Particulièrement problématique sous l'eau (scans sonar plus épars et bruités que lidar).

#### Initialisation globale par CONSAC

Avant l'ICP, une initialisation globale est calculée par **consensus set maximization** [44] :

$$\tilde{\mathbf{x}}_s = \underset{\mathbf{x}_s}{\arg\max} \sum_{\mathbf{p}_s \in \mathcal{P}_s} \mathbb{1}(d(\mathbf{p}_s) \leq \epsilon) \tag{35}$$

avec $d(\mathbf{p}_s) = \min_{\mathbf{p}_t \in \mathcal{P}_t} \|\mathbf{R}_{ts}\mathbf{p}_s + \mathbf{t}_{ts} - \mathbf{p}_t\|$ (Eq. 36). La Fig. 6 illustre les 4 étapes : odométrie seule (mauvaise) → initialisation globale → ICP raffiné (bon résultat).

#### Détection de loop closures et rejet des aberrants

Pour les keyframes non-séquentielles, les candidats sont filtrés par overlap minimal (Eq. 37). La consistance entre deux mesures est vérifiée par distance de Mahalanobis (Eq. 38) : $\|\hat{\mathbf{x}}_{ik} \ominus \hat{\mathbf{x}}'_{ik}\|_\Sigma \leq \eta_{pcm}$.

Le rejet des aberrants utilise **PCM** (*Pairwise Consistent Measurement*) [45] : construire un graphe où chaque paire consistante est une arête, et trouver la plus grande clique (ensemble mutuellement cohérent). Fig. 8 : 6 paires possibles, 3 correctes → la clique de taille 3 est retenue, la mauvaise (rouge) est exclue.

---

### IV.C — Building the Factor Graph

Le graphe de facteurs (Fig. 7) contient :
- **Nœuds** : poses $\mathbf{x}_k$ à chaque keyframe (carrés noirs)
- **Facteurs séquentiels** (cercles verts) : contrainte entre $\mathbf{x}_{k-1}$ et $\mathbf{x}_k$, estimée par scan matching sur les $N_{ssm}$ dernières frames pour robustesse
- **Facteurs de loop closure** (cercles orange) : contrainte entre keyframes non-consécutives, détectées par matching de scans distants dans le temps

La **fonction de facteur globale** est :

$$\mathbf{f}(\Theta) = \mathbf{f}^0(\Theta_0) \prod_i \mathbf{f}_i^s(\Theta_i) \prod_j \mathbf{f}_j^0(\Theta_j) \prod_q \mathbf{f}_q^{NSSM}(\Theta_q)$$

Optimisée incrémentalement par **iSAM2** [32] (GTSAM [50]), qui maintient une factorisation de Cholesky incrémentale. Chaque keyframe stocke aussi ses features détectés pour la cartographie.

---

### IV.D — Occupancy Mapping

#### Modèle inverse du sonar

La grille d'occupation est mise à jour avec le **modèle inverse du sonar** (Fig. 9). Contrairement au lidar :
- Les cellules **devant** le premier retour : libres
- Le premier retour : occupé
- Les cellules **derrière** : inconnues (le sonar à large ouverture verticale peut détecter des objets derrière le premier plan)

La convolution Gaussienne lisse les probabilités autour des points de contact.

#### 1. Fusion de sous-cartes

Chaque keyframe $\mathbf{x}_k$ génère une sous-carte locale $\mathcal{S}_k = \{\mathbf{m}_i^k\}$. La carte globale fusionne les sous-cartes en log-odds (Bayes filter) :

$$l(\mathbf{m}_i) = \sum_{\mathcal{S}^k \in \mathcal{M}} l^k(\mathcal{T}_{kg} \mathbf{m}_i) \tag{39}$$

où $\mathcal{T}_{kg}$ transforme les cellules globales en référentiel local de la keyframe.

#### 2. Mise à jour d'une sous-carte après correction

Quand une loop closure corrige la pose $\mathbf{x}_k$, seule la sous-carte $\mathcal{S}_k$ est recalculée (pas toute la carte). La mise à jour retire les anciennes contributions et ajoute les nouvelles :

$$l'(\mathbf{m}_i) = l(\mathbf{m}_i) - \underbrace{l^k(\mathcal{T}_{kg}\mathbf{m}_i)}_{\text{retirer ancien}} + \underbrace{l^k(\mathcal{T}'_{kg}\mathbf{m}_i)}_{\text{ajouter nouveau}} \tag{40}$$

Cela évite de recomputer les log-odds depuis zéro et rend la correction temps-réel.

---

## V. Experimental Results

### Setup général

Quatre algorithmes comparés :

| Algorithme | Stratégie |
|---|---|
| **NF** (Nearest Frontier) | Toujours vers la frontière la plus proche |
| **NBV** (Next-Best-View) | Maximise le gain d'information de la carte [16] |
| **Heuristic** | Active SLAM : revisite si pose uncertainty > seuil [28] |
| **EM** (proposé) | Minimise le déterminant de covariance des landmarks virtuels |

Métriques d'évaluation :
- **Pose uncertainty** : $|\Sigma_{\mathbf{x}_T}|^{1/4}$
- **Trajectory error** : RMSE de la trajectoire estimée vs. vérité terrain
- **Map error** : RMSE des positions de landmarks estimées vs. vérité terrain
- **Map coverage** : ratio zone cartographiée / zone totale

---

### V.A — Algorithm Comparison

Heuristic et EM sont paramétrés pour avoir le même trajectory error baseline, afin de comparer les autres métriques à performances de localisation égales. NF et NBV servent de bornes (couverture max vs. incertitude min).

---

### V.B — Simulated Exploration over Landmarks

**Environnement** : espace 2D avec landmarks ponctuels aléatoires (Fig. 12a), 45 essais (5 par position de départ). Sonar simulé : 5 Hz, portée [0, 30]m, ouverture $\theta = [-65°, 65°]$. Bruit : $\sigma_r = 0.2$m, $\sigma_\theta = 0.02$rad (mesures), $\sigma_x = \sigma_y = 0.08$m, $\sigma_\theta = 0.003$rad (odométrie).

**Résultats** (Fig. 10 : exemples de trajectoires, Fig. 11 : courbes moyennes, Tables I–IV) :

| Algorithme | Pose uncertainty | Map error | Coverage | Commentaire |
|---|---|---|---|---|
| NBV | La plus haute | Le plus haut | La meilleure | Explore vite mais dérive |
| NF | Moyenne | Moyen | Bonne | Simple, accumule dérive |
| Heuristic | La plus basse | Le plus bas | La plus lente | Loop closures mais exploration lente |
| **EM** | Comparable Heuristic | Comparable Heuristic | > Heuristic | **Meilleur compromis** |

Distance pour atteindre 90% de couverture (Table III) : EM = 303.92m vs Heuristic = 396.52m → **EM couvre 90% en 30% moins de distance**.

---

### V.C — Simulated Exploration with Pose SLAM in a Harbor Environment

**Environnement** : carte réelle d'une marina reconstituée (Fig. 12b, structures hétérogènes), 60 essais (10 par position de départ). Pas de vérité terrain landmarks → map error calculé par distance aux points du nuage reconstruit.

**Résultats** (Fig. 13 : trajectoires, Fig. 14 : courbes moyennes, Table IV : map error) :
- NF et NBV : trajectoires qui filent vers le coin supérieur droit → loop closures tardives → forte dérive accumulée
- Heuristic et EM : loop closures denses et continues → incertitude faible tout au long
- EM supérieur à Heuristic sur la couverture, avec incertitude de pose et map error similaires
- La différence EM/Heuristic est plus visible ici que dans V.B car l'environnement est plus dense en structures (plus de loop closures possibles)

---

### V.D — Exploration avec un ROV réel dans un port

**Matériel** (Fig. 15) :
- Robot : BlueROV2 modifié
- Sonar : Oculus M750d, 512 beams, 750 MHz, 4mm résolution range, 1° résolution angulaire, portée [0, 30]m, ouverture $[-65°, 65°]$, 20° vertical
- DVL : Rowe SeaPilot
- IMU : VectorNav VN100
- Pression : Bar30
- Profondeur : 1m fixe
- Calcul : laptop Intel i7-4710, NVIDIA Quadro K1100M, 8GB RAM — tout le traitement est embarqué

**Lieu** : marina de l'USMMA (*United States Merchant Marine Academy*), King's Point NY. Zone : 130×60m (bounding box).

**Protocole** : 3 runs par algorithme (12 runs total). Pas de vérité terrain → évaluation qualitative carte + quantitative pose uncertainty et coverage.

**Résultats** (Fig. 16 : planification en cours, Fig. 17 : NF/NBV/Heuristic, Fig. 18 : EM) :
- **NF/NBV** : trajectoires directement vers le coin supérieur droit, loop closures très tardives, forte dérive visible sur les cartes finales
- **Heuristic** : ferme des boucles intentionnellement (visible dans Run 1 à distance 100m), mais décision binaire (seuil)
- **EM** : loop closures denses et continues dès le début, incertitude faible sur toute la trajectoire dans les 3 runs, cartes cohérentes

> *"The EM approach continuously takes map uncertainty into account, and loop closure constraints are intertwined throughout the entire trajectory in all three trials."*

---

## VI. Conclusion and Future Work

Le système présenté est le premier exemple d'exploration autonome réelle d'un environnement extérieur encombré avec un ROV qui intègre son propre processus SLAM dans chaque décision de planification. Les cartes virtuelles permettent d'anticiper l'effet des futures observations sur l'incertitude globale, sans simuler explicitement le SLAM futur (trop coûteux).

**Résultats clés** :
- EM maintient une faible incertitude de localisation et cartographie tout en atteignant un taux de couverture supérieur aux approches qui sacrifient la localisation (NBV, NF)
- EM atteint 90% de couverture en ~30% moins de distance que l'approche heuristique

**Perspective principale** : extension à la cartographie 3D par sonar volumétrique (lever l'hypothèse de profondeur fixe et passage de SE(2) à SE(3)).

---

## Annexe — Glossaire des termes clés

| Terme | Définition |
|---|---|
| **SLAM** | Simultaneous Localization and Mapping — construire une carte et se localiser en même temps |
| **Keyframe** | Pose de référence ajoutée au graphe quand le déplacement dépasse un seuil (4m ou 30°) |
| **Loop closure** | Détection qu'on repasse au même endroit → contrainte qui corrige la dérive accumulée |
| **Factor graph** | Représentation graphique du problème SLAM : nœuds = poses/landmarks, arêtes = contraintes de mesure |
| **iSAM2** | Algorithme d'optimisation incrémentale du graphe de facteurs (GTSAM) |
| **Dead reckoning** | Estimation de position par intégration des capteurs inertiels (DVL + IMU) sans correction externe |
| **ICP** | Iterative Closest Point — algorithme d'alignement de nuages de points |
| **CFAR / SOCA-CFAR** | Détecteur à seuil adaptatif pour identifier les retours sonar significatifs |
| **Occupancy grid** | Grille 2D où chaque cellule stocke la probabilité d'être occupée par un obstacle |
| **Virtual landmark** | Landmark hypothétique dans une zone non encore visitée, représentant un obstacle potentiel |
| **D-optimality** | Critère d'optimalité = minimiser le déterminant de la matrice de covariance |
| **SCI** | Split Covariance Intersection — fusion conservative de deux estimations sans connaître leur corrélation |
| **PCM** | Pairwise Consistent Measurement — rejet des loop closures aberrantes par recherche de clique |
| **DVL** | Doppler Velocity Log — mesure la vitesse du robot par effet Doppler acoustique |
| **ROV** | Remotely Operated Vehicle — robot sous-marin téléopéré (ici BlueROV2) |

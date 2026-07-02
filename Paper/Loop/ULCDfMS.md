# Résumé complet — PHD-LCD: Underwater Loop-Closure Detection for Mechanical Scanning Imaging Sonar by Filtering the Similarity Matrix with Probability Hypothesis Density Filter

**Titre :** *Underwater Loop-Closure Detection for Mechanical Scanning Imaging Sonar by Filtering the Similarity Matrix with Probability Hypothesis Density Filter*
**Auteurs :** Min Jiang, Sanming Song, J. Michael Herrmann, Ji-Hong Li, Yiping Li, Zhiqiang Hu, Zhigang Li, Jian Liu, Shuo Li, Xisheng Feng
**Institutions :** State Key Laboratory of Robotics, Shenyang Institute of Automation (CAS, Chine) ; Institutes for Robotics and Intelligent Manufacturing (CAS, Chine) ; University of Chinese Academy of Sciences (Chine) ; University of Edinburgh (Royaume-Uni) ; Korea Institute of Robot and Convergence (Corée du Sud)
**Publié :** IEEE Access, 2019 (DOI 10.1109/ACCESS.2019.2952445)
**Code source :** non mentionné / non publié

---

## Abstract

Ce travail présente une méthode robuste de détection de fermeture de boucle pour le SLAM acoustique sous-marin utilisant un sonar à balayage mécanique (MSIS). L'approche repose d'abord sur la construction d'une matrice de similarité globale via l'extraction de deux caractéristiques originales : les histogrammes de projection d'intensité (dédiés à la translation) et une matrice de gradient polaire (dédiés à la rotation). Pour éliminer le bruit relationnel accidentel induit par la faible résolution acoustique, la détection des trajectoires de fermeture de boucle est modélisée comme un problème de suivi multi-cibles virtuelles à l'aide d'un filtre à densité d'hypothèse de probabilité (PHD). Les contraintes spatiales extraites sont ensuite intégrées dans un algorithme GraphSLAM afin de corriger les erreurs cumulées de l'estime du véhicule et de générer une carte panoramique cohérente. Les performances de la méthode sont validées sur des jeux de données réels collectés en milieux structurés (marina) et non structurés (grotte sous-marine).

---

## I. Introduction

### Contexte
La construction de cartes panoramiques cohérentes par le biais du SLAM est cruciale pour permettre aux véhicules sous-marins autonomes (UUV) de naviguer et d'intervenir de manière autonome. En raison de l'atténuation rapide des ondes électromagnétiques sous l'eau, les signaux GPS sont inaccessibles, contraignant les robots à s'appuyer sur des systèmes de navigation inertielle dont la dérive de l'estime (dead-reckoning) croît de façon temporelle. Bien que des systèmes acoustiques externes (LBL, USBL) permettent de recaler la position absolue, ces infrastructures sont fréquemment indisponibles. La détection de fermeture de boucle (reconnaissance de lieux déjà visités) s'impose donc comme une stratégie incontournable pour borner l'erreur globale accumulée.

### Approches existantes / Limites
Les caméras optiques souffrent d'une portée extrêmement réduite en eaux turbides, ce qui généralise l'usage des sonars imageurs comme capteurs d'exteroception longue distance. Néanmoins, l'utilisation des sonars MSIS introduit des verrous techniques sévères :
* **Faible résolution temporelle :** Le transducteur balaie l'environnement secteur par secteur de manière séquentielle, induisant de longues périodes d'acquisition qui provoquent des distorsions géométriques majeures lorsque le robot est en mouvement.
* **Faible résolution spatiale et bruit :** Les images acoustiques possèdent une faible résolution spatiale et intègrent un fort bruit de tavelure (speckle noise), rendant l'extraction de descripteurs géométriques classiques (lignes, blobs) peu fiable, en particulier dans les milieux naturels non structurés.
* **Ambiguïtés et faux positifs :** L'évaluation de similarité directe entre scans individuels engendre une forte incertitude et des faux positifs en raison des corrélations accidentelles entre scènes naturelles topologiquement distinctes mais acoustiquement similaires.

### Idée centrale
Pour contourner la fragilité des matchings d'images isolées, ce papier propose une méthode robuste fondée sur l'appariement de séquences de scans sonars. L'apparition de fermetures de boucle successives au cours du déplacement génère des segments de trajectoires distincts au sein d'une matrice de similarité globale. L'originalité de l'approche consiste à assimiler ces trajectoires de fermeture de boucle aux trajectoires cinématiques de multiples cibles virtuelles. Un filtre PHD (Probability Hypothesis Density) est mis en œuvre pour suivre ces cibles dont le nombre est inconnu et variable, ce qui permet d'extraire simultanément les vraies contraintes topologiques tout en s'affranchissant du bruit de fond sans requérir d'association de données explicite lors de la mise à jour.

---

## II. Méthodologie

### A. Matrice de similarité
La matrice de similarité est définie par $M = \{m_{ij}\}_{i,j=1,...,N}$, où $N$ représente le nombre total de scans accumulés. Chaque score de similarité $m_{ij}$ entre un scan courant (floating scan $I_i$) et un scan historique (reference scan $I_j$) correspond au niveau de chevauchement spatial des intensités acoustiques une fois le recalage appliqué :

$$m_{ij} = g(f(I_i, \Psi), I_j)$$

Le vecteur de transformation $\Psi = \{\Delta x^*, \Delta y^*, \Delta\theta^*\}$ applique les paramètres de translation ($\Delta x^*, \Delta y^*$) et de rotation ($\Delta\theta^*$) requis pour aligner le scan courant sur le repère du scan de référence. Afin de limiter l'impact du bruit de fond, seuls les scores inférieurs à un seuil prédéfini sont conservés pour les étapes ultérieures.

### B. Descripteurs et Recalage Grossier

#### 1. Estimation de la rotation (Polar Gradient Matrix)
Pour modéliser la distribution d'énergie acoustique selon l'angle azimutal indépendamment de la distance, l'image sonar est discrétisée sous forme de cellules en coordonnées polaires définies par un pas angulaire $d\theta$ et un pas de distance $dD$. La dimension de la matrice de caractéristiques de rotation résultante $R$ est de $2C_{N_d}^2 \times N_a$, où $N_a = \frac{360}{d\theta}$ et $N_d = \frac{D}{dD}$. 
Pour chaque cellule, l'intensité moyenne et le gradient d'intensité moyen sont calculés afin de saisir les tendances structurelles de la scène. Inspiré du descripteur BRIEF, un tuple binaire encode les différences de gradient et de courbure entre toutes les paires de cellules $i$ et $j$ appartenant à un même secteur angulaire $k$ :

$$T_{l,k} = [H(I_{i,k} - I_{j,k}), H(I'_{i,k} - I'_{j,k})]^T$$

où $H(\cdot)$ est la fonction de Heaviside. Le paramètre de rotation optimal $\Delta\theta^*$ est obtenu en minimisant la fonction de corrélation croisée basée sur une opération OU exclusif (XOR) circulaire entre les matrices de caractéristiques des deux scans :

$$\Delta\theta^* = \arg\min_{\Delta\theta} \sum_{k=1}^{N_a} \text{XOR}(R_1(k), R_2(k + \Delta\theta))$$

#### 2. Estimation de la translation (Intensity Projection Histogram)
Afin d'obtenir des fonctions de corrélation dotées de pics nets et discriminants, les pixels de l'image sonar subissent une projection d'intensité le long d'un vecteur unitaire orthogonal. L'angle de projection initial $\theta_g$ est déterminé via une stratégie heuristique de minimisation d'entropie appliquée aux secteurs de l'image pour maximiser la concentration structurelle. Les paramètres de translation le long des axes orthogonaux sont résolus en identifiant le maximum global de la fonction de corrélation croisée des histogrammes de projection respectifs :

$$\Delta x_{\theta}^* = \arg\max_{\Delta x} \sum_{i=-N_h/2}^{N_h/2} H_1(i) \cdot H_2(i - \Delta x)$$

$$\Delta y_{\theta}^* = \arg\max_{\Delta y} \sum_{i=-N_h/2}^{N_h/2} H_1^\perp(i) \cdot H_2^\perp(i - \Delta y)$$

Ce recalage grossier fournit une excellente initialisation géométrique, qui est affinée ultérieurement par l'algorithme ICP (Iterative Closest Point) pour établir la contrainte finale.

### C. Détection de boucles par filtre GM-PHD
Lorsque le véhicule navigue en sens inverse sur une trajectoire déjà cartographiée, la corrélation séquentielle génère une ligne continue perpendiculaire à la diagonale principale au sein de la matrice de similarité. Le vecteur d'état associé à chaque cible virtuelle intègre sa position et sa vitesse sur les axes de la matrice : $x_k = (p_{x,k}, \dot{p}_{x,k}, p_{y,k}, \dot{p}_{y,k})^T$, avec des vitesses contraintes à $\dot{p}_{x,k} = 1$ et $\dot{p}_{y,k} = -1$ frame/étape.

Le framework implémente un filtre PHD sous forme de mélange de gaussiennes (GM-PHD) pour modéliser analytiquement les étapes de prédiction et de mise à jour :
* **Modèle dynamique (Prédiction) :** Les mouvements des cibles virtuelles sont décrits par un modèle linéaire gaussien avec un bruit de processus additif $Q_k$. À chaque pas temporel, l'intensité des cibles naissantes (spontaneous birth $\gamma_k$) est initialisée à partir des coordonnées des 5 scans historiques affichant la plus forte corrélation avec le scan actuel.
* **Modèle d'observation (Mise à jour) :** Les observations correspondent aux coordonnées des points de forte similarité extraits de la matrice. La variance du bruit de mesure associée à l'axe $y$ est paramétrée pour être plus faible que celle de l'axe $x$ afin de compenser les incertitudes locales d'appariement.
* **Gestion des trajectoires :** Une étiquette unique (identity label) est assignée à chaque composante gaussienne. La trajectoire complète d'une cible virtuelle est extraite de manière rétroactive en remontant l'arbre des parents à partir de la couche terminale dès lors que le poids d'un nœud franchit un seuil critique. Des étapes d'élagage (pruning) et de fusion (merging) maintiennent le nombre maximal de gaussiennes sous un plafond $J_{max} = 200$ pour préserver le temps réel.

### D. Validation des candidats et Ajustement global
Pour écarter les fausses fermetures de boucle, trois filtres heuristiques sont appliqués :
1. **Seuil de distorsion :** Les scans présentant un saut ou un gap angulaire supérieur à $90^\circ$ après compensation cinématique sont éliminés car ils altèrent la convergence géométrique de l'ICP.
2. **Seuil d'erreur résiduelle :** Le candidat est rejeté si l'erreur quadratique de l'ICP ne descend pas sous un seuil critique après affinement.
3. **Cohérence spatio-temporelle :** L'écart de déplacement topologique avant et après optimisation de graphe est analysé pour exclure les fausses contraintes.

Les facteurs de fermeture de boucle validés forment des arêtes non consécutives injectées dans un formalisme GraphSLAM. L'optimisation minimise l'erreur quadratique globale entre les contraintes de recalage finies issues de l'ICP ($z_{ij}$) et les estimations de l'estime ($\hat{z}_{ij}$) via un solveur iSAM2, permettant d'aligner la trajectoire et d'assembler la carte finale par recalage d'images.

---

## III. Évaluation expérimentale

### A. Dataset de la Marina (Milieu man-made structuré)
Ce jeu de données a été acquis à l'aide d'un sonar Tritech Miniking (portée 50 m, résolution 0,1 m) tracté par un AUV au sein de la marina désaffectée de Fluvia Nautic (Espagne) sur une trajectoire d'environ 400 m intégrant une boucle fermée et un canal linéaire. Un système DGPS embarqué sur une bouée fournit la trajectoire de référence (ground truth) avec une précision de 1,22 m. 

Le filtre GM-PHD extrait avec succès 6 segments de trajectoire distincts, validés comme de véritables fermetures de boucle par les coordonnées GPS. Les trajectoires formées simultanément lors des intervalles temporels avancés (ex. index 154-164) reflètent les passages multiples du véhicule dans la même zone géographique, confirmant la capacité du filtre PHD à traquer un nombre variable de boucles simultanées grâce à la portée étendue du sonar.

*Tableau 1 : Erreurs d'estimation de la position (m) par rapport à la vérité terrain GPS — Marina*

| Méthode | Modules intégrés | Erreur Moyenne (m) | Écart-Type (m) | Erreur Min (m) | Erreur Max (m) |
| :--- | :--- | :---: | :---: | :---: | :---: |
| Estime seule (DR) | DR | 17.46 | 11.47 | 0.09 | 45.42 |
| MSISPIC | DR + SR | 3.99 | 2.11 | 0.03 | 8.49 |
| Feature-based SLAM | DR + SR + LCD | 1.91 | 1.32 | 0.01 | 5.08 |
| Pose-based SLAM | DR + SR + LCD | 1.90 | 1.09 | 0.01 | 4.93 |
| **Méthode proposée** | DR + LCD | **1.58** | **0.98** | 0.09 | 5.85 |

*(Légende des modules : DR = Dead Reckoning (Estime) ; SR = Scan Registration (Recalage local entre frames consécutives) ; LCD = Loop-Closure Detection).*

La méthode proposée surpasse les frameworks alternatifs en précision moyenne bien qu'elle n'intègre pas de module de recalage local entre images consécutives (DR+LCD uniquement). L'évaluation quantitative de la cohérence de la carte (crispness) affiche un score de 11359 (murs minces et nets) contre 12159 pour l'estime seule (marquée par de forts effets de ghosting textuels).

### B. Dataset de la Grotte (Milieu naturel non structuré)
Ce second essai évalue la robustesse de l'algorithme dans une grotte sous-marine naturelle non structurée située à L'Escala (Espagne) sur un parcours de 500 m. En l'absence de signal GPS utilisable sous voûte rocheuse, six cônes de signalisation ont été immergés à des positions fixes survolées à deux reprises par le robot. Une caméra vidéo classique orientée vers le nadir valide visuellement les passages au-dessus de ces repères.

En environnement naturel, la matrice de similarité intègre un niveau de bruit de fond considérablement supérieur à celui de la marina à cause des corrélations acoustiques fortuites du fond rocheux. Malgré ce bruit, le filtre GM-PHD isole efficacement les trajectoires associées aux repères fixes. Seul le cône numéro 4 n'est pas détecté en raison d'une rupture drastique de l'intensité acoustique liée au changement d'angle d'incidence du sonar lors du second passage.

*Tableau 2 : Erreurs de localisation spatiale des cônes de signalisation (m) — Grotte*

| Repère Target | Estime seule (DR) (m) | Méthode de Mallios et al. (m) | Méthode proposée (m) |
| :--- | :---: | :---: | :---: |
| **Cône 1** | 6.60 | 2.20 | **1.22** |
| **Cône 2** | 3.84 | 1.68 | **0.33** |
| **Cône 3** | 2.81 | **0.28** | 1.42 |
| **Cône 4** | 3.53 | **0.49** | 2.89 (Hors contrainte) |
| **Cône 5** | 2.44 | 1.40 | **0.93** |
| **Cône 6** | 4.37 | 2.55 | **2.08** |

L'introduction des contraintes de fermeture de boucle réduit fortement les erreurs géométriques par rapport à l'estime seule sur quatre des cinq cônes mesurés. L'indice de netteté global de la carte de la grotte s'établit à 5926 avec la méthode de filtrage PHD contre 6348 pour la cartographie brute par estime, éliminant les doublons de parois rocheuses visibles sur les tracés non corrigés.

---

## IV. Discussions

### Pourquoi cette approche fonctionne
1. **Indépendance vis-à-vis de l'estime :** Contrairement aux méthodes classiques (dont celles de Ribas ou Mallios) qui restreignent la recherche de boucles aux voisinages géométriques incertains fournis par la centrale inertielle, la génération de candidats dans cette pipeline s'appuie *uniquement* sur le contenu des images sonars. Cela évite d'exclure de vrais candidats lorsque la dérive de l'estime devient critique.
2. **Robustesse du filtre PHD face au bruit acoustique :** Modéliser la détection de trajectoires à l'aide d'un filtre PHD permet de traiter le bruit de la matrice de similarité comme un bruit de clutter aléatoire fini. Les fausses associations n'ayant pas de cohérence temporelle ou cinématique linéaire sont rejetées naturellement au fil des itérations.

### Limitations
* **Complexité calculatoire quadratique ($N \times N$) :** L'évaluation systématique de chaque nouveau scan sonar par rapport à l'intégralité de l'historique pour alimenter la matrice de similarité génère un coût computationnel lourd qui croît de manière quadratique à mesure que la mission s'allonge, limitant le déploiement sur de très longues distances sans optimisation structurelle.
* **Sensibilité au seuillage binaire :** Le passage par une matrice binarisée simplifie le suivi cinématique mais introduit un risque de non-détection (miss-detection) si deux scènes affichent un score de similarité légèrement inférieur au seuil fixe en raison de fortes variations de point de vue ou d'intensité.

---

## V. Conclusion

Le papier démontre la faisabilité d'une architecture de détection de fermeture de boucle sous-marine fondée exclusivement sur la topologie intrinsèque des données d'un sonar à balayage mécanique (MSIS). Les descripteurs d'histogrammes de projection et de gradients polaires capturent efficacement les structures des images acoustiques basse résolution. Le recours au filtrage GM-PHD permet d'extraire des trajectoires multi-boucles cohérentes en milieu bruité non structuré. Les travaux futurs s'orienteront vers la sélection adaptative de scans clés (key-scans) pour réduire la dimension de la matrice, le traitement direct des similarités en niveaux de gris et la fusion de données multi-capteurs.

---

## Glossaire

| Terme | Définition |
| :--- | :--- |
| **MSIS** | *Mechanical Scanning Imaging Sonar* — Sonar imageur à balayage mécanique sectoriel pas-à-pas. |
| **Filtre PHD** | *Probability Hypothesis Density Filter* — Filtre bayésien s'appuyant sur les ensembles finis aléatoires (RFS) pour suivre un nombre variable de cibles sans association explicite de données. |
| **Matrice de gradient polaire** | Descripteur modélisant les variations locales d'intensité acoustique et de courbure au sein de secteurs angulaires polaires pour résoudre la rotation. |
| **Histogramme de projection** | Descripteur basé sur la somme cumulée des intensités des pixels projetés le long d'axes orthogonaux pour résoudre la translation. |
| **Crispness** | Métrique quantitative de netteté mesurant la coherence géométrique d'une carte par le décompte de voxels occupés (un score plus bas indique une meilleure compacité). |
| **GraphSLAM** | Algorithme de SLAM modélisant la trajectoire sous forme de graphe de poses optimisé par minimisation d'erreur quadratique sous contraintes. |
# Résumé complet — ISOPoT: Imaging Sonar Odometry by Point Tracking

**Titre :** *ISOPoT: Imaging Sonar Odometry by Point Tracking*
**Auteurs :** Jaša Samec, Vid Rijavec, Marko Peljhan, Aleksander Grm, Andrej Androjna, Danijel Skočaj, Matej Dobrevski
**Institutions :** University of Ljubljana (Faculty of Computer and Information Science), University of Ljubljana (Faculty of Maritime Studies and Transport), OZON Research Group, MAT Systemics (UC Santa Barbara)
**Publié :** arXiv:2606.23006v1, 22 juin 2026 (projet European Defense Agency, SABUVIS II)
**Code source :** non publié à ce jour

---

## Abstract

ISOPoT est une **odométrie sonar** qui remplace le matching de keypoints par paires (DISO, SONIC)
par un **suivi de points multi-frames** (point tracking) issu des modèles deep modernes "Track Any
Point" (TAP). La correspondance primaire n'est plus une paire de keypoints entre 2 images, mais une
**trajectoire de points** sur une fenêtre glissante de plusieurs images sonar, augmentée
d'optimisations légères. Estime le mouvement planaire **SE(2)**. Évalué sur **Aracati 2017** et un
nouveau dataset **Portoroz 2025**, ISOPoT **surpasse SONIC et DISO** en sonar seul comme en
configuration multi-capteurs.

---

## I. Introduction

### Contexte

Le sonar imageur frontal (FLS) est le capteur de référence sous l'eau (l'optique est limitée à
quelques mètres par la turbidité). On veut estimer le mouvement planaire **SE(2)** depuis les images
FLS. Mais les images sonar ont des statistiques très différentes des images optiques : **speckle
fort**, artefacts capteur, **texture locale faible**, et fortes variations d'apparence selon l'angle
de vue et d'insonification. → la détection et le matching de keypoints locaux y sont **peu fiables**.

### Limite commune des méthodes existantes

Le point clé du papier : **DISO, SONIC et la plupart des méthodes formulent la correspondance comme
un matching de keypoints LOCAUX entre deux frames consécutives.** Ce paradigme est fragile sur le
sonar :
- les patchs locaux sont **ambigus**,
- les correspondances appariées indépendamment ne protègent pas contre un mouvement **globalement
  incohérent**,
- résultat : estimation bruitée, échec brutal dans les zones pauvres en structure ou riches en
  artefacts.

Beaucoup compensent avec une odométrie auxiliaire (init du mouvement, contrainte de recherche), mais
ça ne résout pas le **mismatch fondamental** entre l'imagerie sonar et le paradigme de matching par
paires.

### Idée centrale

Puisque les patchs locaux sont ambigus, une approche qui considère les images **globalement** et
exploite la **cohérence temporelle sur plusieurs frames** est plus adaptée. Les modèles récents de
tracking (AllTracker, CoTracker3, TAPNext) sont conçus pour ça. Bien qu'entraînés sur vidéo optique,
les auteurs constatent qu'ils **transfèrent efficacement** au sonar frontal. D'où **ISOPoT** : une
odométrie sonar par suivi de points, à partir d'une fenêtre glissante, **avec TAPNext comme
backbone**.

---

## II. Background et modèle

### A. Formation de l'image sonar et modèle de mouvement

Sonar multifaisceaux frontal → image 2D d'intensité acoustique (range × azimuth). Un pixel **p**
fournit un couple **range-bearing** $(r(\mathbf{p}), \theta(\mathbf{p}))$ mais **PAS l'élévation**
(perdue dans l'ouverture du faisceau ; l'intensité peut agréger plusieurs réflecteurs au même
range/bearing — cf. Fig. 2). Point cartésien dans le plan capteur (éq. 1) :

$$\mathbf{x}(\mathbf{p}) = \begin{bmatrix} r(\mathbf{p})\cos\theta(\mathbf{p}) \\ r(\mathbf{p})\sin\theta(\mathbf{p}) \end{bmatrix}$$

Sous les hypothèses standard (scène statique, peu de pitch/roll, mouvement planaire), un point
observé sur deux frames consécutives satisfait (éq. 2-3) :

$$\mathbf{x}_{k+1} \approx \mathbf{R}(\Delta\psi_k)\mathbf{x}_k + \mathbf{t}_k, \qquad
T_{k+1\leftarrow k} = \begin{bmatrix} \mathbf{R}(\Delta\psi_k) & \mathbf{t}_k \\ \mathbf{0}^T & 1 \end{bmatrix} \in SE(2)$$

où $\Delta\psi_k$ est le **yaw relatif** et $\mathbf{t}_k$ la translation planaire entre frames $k$
et $k+1$. Relation **approximative** : l'élévation n'est pas observable, et les phénomènes sonar
(speckle, ombres acoustiques, multipath) créent un **processus de visibilité stochastique** — des
points apparaissent/disparaissent indépendamment du mouvement. Le problème clé est donc de récupérer
un ensemble de correspondances **conjointement cohérentes avec un mouvement planaire dominant**,
plutôt que de se fier à des matchs locaux isolés.

### B. Odométrie sonar et SLAM existants

- **Géométriques** : alignent des structures extraites (ICP sur features/cartes locales) [Li 2018].
- **Keypoints + descripteurs** : matching dans des régions contraintes par une odométrie init.
- **DISO** : dépend d'une **odométrie externe** pour l'alignement grossier, puis raffine en
  minimisant les différences d'intensité entre keypoints matchés, suivi d'une optimisation par
  factor graph.
- **SONIC** : retire la dépendance aux priors de mouvement, fait du matching sonar-only par features
  CNN avec corrélation coarse-to-fine.

→ Tous partagent l'hypothèse "mouvement = correspondances locales par paires entre frames
consécutives", la cohérence globale n'étant imposée qu'**après** (RANSAC, pose-graph, factor graph,
odométrie auxiliaire). Fragile sur le sonar. **Ce constat motive le passage au tracking de points
multi-frames.**

### C. Point Tracking pour la correspondance sonar

Les méthodes **TAP (Tracking Any Point)** estiment les trajectoires de points arbitraires sur une
séquence vidéo, en exploitant le **contexte temporel**, le raisonnement sur la **visibilité** et le
raffinement itératif sur de longues fenêtres (TAP-Net, PIPs, TAPIR, CoTracker, AllTracker, TAPNext).
C'est particulièrement pertinent pour le sonar, qui viole les hypothèses de constance d'apparence
plus sévèrement que la vidéo RGB — mais dont l'**évolution temporelle garde un signal géométrique
fort**. Parmi les trackers évalués, **TAPNext** donne les pistes les plus fiables (inférence causale
en ligne, sortie de visibilité explicite, robustesse au matching local fragile) → choisi comme
backbone.

---

## III. Méthode

### Vue d'ensemble (Fig. 3)

À chaque itération, ISOPoT traite une fenêtre glissante $\mathcal{W} = \{F_1, ..., F_W\}$ : les $B =
W-N$ premières frames recouvrent la fenêtre précédente, les $N$ dernières sont nouvelles. $W$ = la
fréquence sonar (une fenêtre ≈ 1 seconde de données), $B = 3$ (période de "warm-up" : sur le sonar
les premières prédictions de TAPNext sont moins fiables, le recouvrement supprime ces erreurs
transitoires).

### A. ISOPoT — pipeline en deux étages

**a) Initialisation des points de requête.** L'ensemble de requête $Q = \{\mathbf{q}_j\}$ sur la
première frame $F_1$ combine : (i) nouveaux points détectés par un **détecteur de Sobel**
(gradient > 98e percentile, plus stables que AKAZE/SuperPoint sur le sonar), (ii) points ayant
survécu comme inliers fiables dans la partie recouvrante de la fenêtre précédente.

**b) Grid Point Manager (GPM).** Grille **16×16**, max **5 points par cellule**. Maintient une
couverture spatiale (RANSAC est moins stable si les points sont concentrés dans une petite région).
Fusionne nouveaux points Sobel + points conservés, applique la limite par cellule (excédent retiré
au hasard). Un point est retiré s'il est : (i) outlier RANSAC, (ii) prédit non-visible par TAPNext,
(iii) hors de la zone de scan sonar valide.

**c) Tracking.** TAPNext suit chaque point de requête $\mathbf{q}_j$ sur la fenêtre → trajectoires
$\{\mathbf{q}_{j,t}\}_{t=1}^W$ + drapeaux de visibilité $\{v_{j,t}\}$.

**d) Estimation grossière du mouvement.** Pour chaque frame cible $F_t$, on estime une transformée
euclidienne 2D **vers la frame de référence commune $F_1$** (plus robuste que frame-à-frame : le
déplacement vs $F_1$ est plus grand → points dérivants/stationnaires plus faciles à identifier),
par **RANSAC** (seuil inlier 3 pixels, fournit aussi la partition inlier/outlier) — éq. 7. Les
incréments consécutifs sont ensuite recomposés (éq. 8).

**e) Match Refinement (Fig. 4).** TAPNext est précis mais prédit chaque point **indépendamment** →
petites erreurs par point. Étage de raffinement : les points de $F_1$ sont propagés vers $F_t$ via
la transformée grossière ; descripteurs **ResNet50** (pré-entraîné ImageNet) extraits sur une
fenêtre de référence et une région de recherche → corrélation → une **heatmap par point**. Un
optimiseur global **maximise la corrélation sommée** sur tous les points fiables pour une **seule**
transformée $SE(2)$ (éq. 9, montée de gradient). Réduit l'influence des matchs locaux bruités tout
en imposant la cohérence avec le modèle de mouvement planaire.

**f) Conversion en pose robot.** Les transformées image sont converties en incréments de pose
robot via la géométrie sonar (§II). Seuls les incréments des $N$ nouvelles frames sont ajoutés à la
trajectoire de sortie ; les frames recouvrantes ne servent qu'à stabiliser tracking et raffinement.

### B. Correction par capteur auxiliaire (optionnelle) — **le point clé sur le cap**

ISOPoT est sonar-only par défaut. Pour une comparaison équitable avec les méthodes assistées, un
étage de correction optionnel combine ISOPoT avec une **odométrie externe** et, si dispo, un
**magnétomètre**.

Soit $(\hat{\mathbf{t}}_t^{iso}, \hat{\psi}_t^{iso})$ la translation et le yaw prédits par ISOPoT, et
$(\hat{\mathbf{t}}_t^{odo}, \hat{\psi}_t^{odo})$ ceux d'une odométrie externe. Un **poids de
confiance** $w_t$ est dérivé du ratio d'inliers RANSAC $\rho_t = M_t^{in}/M_t^{trk}$ (éq. 11-12,
sigmoïde). La translation est interpolée (éq. 13) :

$$\hat{\mathbf{t}}_t = w_t\hat{\mathbf{t}}_t^{iso} + (1-w_t)\hat{\mathbf{t}}_t^{odo}$$

**Pour le cap, ils ne fusionnent pas — ils REMPLACENT** (éq. 14) :

$$\hat{\psi}_t = \hat{\psi}_t^{mag}$$

> **Citation directe :** *"In our experiments, yaw drift is the dominant long-term failure mode when
> only sonar is used. Therefore, when a bounded-error heading source such as a magnetometer is
> available, we simply replace the predicted yaw with the measured heading."* Cette simple
> correction s'est révélée **plus fiable que de mélanger deux estimées d'orientation dérivantes**.

---

## IV. Évaluation

### A. Aracati 2017 (dataset de mon stage)

Particulièrement difficile à cause d'**artefacts récurrents** (Fig. 5) : grandes **bandes (stripes)**
persistantes qui rendent des régions inexploitables ; **artefacts de réflexion** — *copies miroir des
pieux du quai (pier poles) qui se déplacent dans le sens OPPOSÉ à la scène physique* ; grandes zones
**surexposées** qui masquent la structure subtile du fond. Ces effets violent le modèle de mouvement
planaire dominant. AKAZE y trouve 248 à 2933 keypoints/frame.

> **Le capteur de cap est monté au-dessus de la ligne de flottaison** → proche du heading GT → en
> configuration assistée, toute méthode qui adopte ce heading atteint une **erreur de rotation
> quasi-nulle**. La comparaison signifiante en assisté porte donc sur la **translation et l'ATE**.

### B. Portoroz 2025 (nouveau dataset)

Marina de la Faculty of Maritime Studies (Ljubljana), véhicule autonome RAS-HA-X25 (projet SABUVIS
II), sonar **Oculus 750d**. Deux séquences : *Boot* (1452 images) et *Lawnmower* (4080 images),
1220×610. Moins de landmarks isolés contrastés, **dominé par le fond** (structure faible mais
spatialement étendue). AKAZE : max 342 keypoints, parfois **0** → met en lumière la faiblesse des
détecteurs locaux épars (l'info utile est mal représentée, pas absente).

### C. Protocole

Pour Aracati : protocole d'évaluation de **DISO**, même toolbox, séquence en 3 sections (S1, S2, S3).
Métriques : **ATE** (alignée au 1er pose seulement → reflète la dérive long-horizon) et **Relative
Error (RE)** sur des segments de 10% (moins sensible aux outliers isolés, plus informatif sur la
précision odométrique locale).

### D. Résultats Aracati (Table I, plus bas = mieux)

| Aux | Méthode | ATE S1/S2/S3 | Trans. (%) | Rot. (°/m) |
|-----|---------|--------------|-----------|-----------|
| Aucun | SONIC | 36.6 / 113.3 / 69.8 | 137.65 | 2.45 |
| Aucun | **ISOPoT** | **8.8 / 12.7 / 16.7** | **22.76** | **0.99** |
| Odom+Mag | / (sans raffinement sonar) | 5.8 / 12.5 / 6.5 | 19.07 | 0.0 |
| Odom+Mag | DISO | 5.3 / 6.1 / 10.9 | 13.90 | 0.44 |
| Odom+Mag | SONIC | 7.0 / 11.2 / 13.7 | 22.83 | 0.0 |
| Odom+Mag | **ISOPoT** | **3.2 / 3.5 / 4.6** | **9.69** | 0.0 |

ISOPoT bat SONIC largement (sonar seul) **et** DISO (assisté), avec la plus basse ATE et la plus
basse erreur de translation relative. Faiblesse résiduelle : une petite portion finale de
trajectoire où quelques mauvaises prédictions font monter l'ATE (Fig. 7).

### E. Résultats Portoroz (Table II)

| Aux | Méthode | ATE Boot/Lawn | Trans. (%) | Rot. (°/m) |
|-----|---------|---------------|-----------|-----------|
| Aucun | SONIC | 78.5 / 68.8 | 171.28 | 2.47 |
| Aucun | **ISOPoT** | **6.5 / 16.18** | **16.18** | **0.61** |
| Odom+Mag | DISO | 25.5 / 11.4 | 52.75 | 3.54 |

**ISOPoT sonar-only bat DISO assisté (odom+mag)** sur les deux séquences. SONIC échoue (scènes
dominées par le fond).

### F. Ablations

**Trackers (Table III, Portoroz Boot) :** AllTracker ATE 50.3, TrackOn2 16.2, **TAPNext 6.5** →
TAPNext donne le champ de mouvement le plus cohérent → justifie le choix du backbone.

**Sous-modules (Table IV, Aracati) :** retirer le **Match Refiner** = plus grosse dégradation
(TAPNext seul insuffisant, le raffinement est nécessaire pour convertir des pistes grossières en un
mouvement globalement cohérent). Retirer l'overlap buffer améliore légèrement la translation locale
mais dégrade l'ATE (cohérence long-terme). Retirer le GPM dégrade là où il y a beaucoup de candidats
(la couverture spatiale stabilise RANSAC). Les trois modules adressent des modes d'échec différents.

---

## V. Conclusion

ISOPoT = odométrie sonar par **suivi de points multi-frames** (TAPNext + GPM + raffinement par
corrélation), répondant à la fragilité du matching de keypoints. Bat les approches antérieures (SONIC,
DISO) sur Aracati 2017 et Portoroz 2025, surtout en scènes à keypoints rares/peu fiables. Les capteurs
auxiliaires améliorent encore, mais le sonar seul reste performant. Perspectives : contraintes
physiques plus fortes, extension vers un SLAM complet.

---

## Lien avec mon projet (Bruce_SLAM + cap GT-free) — **le papier le plus pertinent**

### 1. Il valide DIRECTEMENT le diagnostic du cap

Tout mon travail a convergé sur un constat : **le cap (yaw) est le verrou, et il ne se récupère pas
depuis le sonar seul**. ISOPoT le dit explicitement (§III-B, éq. 14) : *"yaw drift is the dominant
long-term failure mode when only sonar is used"* → ils **remplacent le yaw par le magnétomètre**.
C'est exactement mon approche (cap = compas via `cmd_vel`). **L'état de l'art post-DISO confirme que
récupérer le cap du sonar n'est pas résolu**, et que la solution est un capteur de cap externe.
→ À citer en priorité pour légitimer mon choix devant le jury.

### 2. Attention au piège de la comparaison ATE

ISOPoT (odom+mag) : ATE Aracati **3.2 / 3.5 / 4.6 m**. Notre Bruce-SLAM : **~1.4 m**. **NE PAS en
conclure qu'on fait mieux** : ce n'est pas la même métrique ni le même système.

| | Nous (Bruce-SLAM) | ISOPoT |
|---|---|---|
| Type | **SLAM** (USBL + loops, ancrage absolu) | **Odométrie** pure |
| ATE | full-seq, **Umeyama** (best-fit) | par section, **alignée 1er pose** (mesure la dérive) |

Notre 1.4 m vient de l'**USBL** (positionnement acoustique absolu) qu'ISOPoT n'utilise pas. ISOPoT
est un **front-end** (comme DISO) ; il ne concurrence pas notre pipeline complet — il pourrait le
**nourrir**.

### 3. Il décrit MON artefact de réflexion

Fig. 5 mentionne les *"reflected pier poles — mirrored copies that move in the opposite direction"*.
C'est exactement le **miroir du cap** qu'on a trouvé (`compass ≈ -theta`). Preuve externe que cet
artefact de réflexion est un phénomène connu d'Aracati, pas un bug de notre pipeline.

### 4. Piste future : remplacer DISO par du point tracking

ISOPoT bat DISO en remplaçant le scan-matching ICP par du **suivi de points multi-frames** (TAPNext).
Or notre limite des "doigts du quai" vient du **cap local par scan** que l'ICP de DISO ne donne pas
GT-free (4.6° de bruit). Un tracking multi-frames pourrait fournir un cap local plus précis →
**piste la plus crédible pour récupérer les doigts GT-free**. Coût : intégrer un modèle deep
(TAPNext) comme nouveau front-end = gros chantier (remplace le front-end, pas le back-end gtsam).

### Positionnement vs les autres papiers

| Papier | Cible pipeline | Verrou attaqué | Testé Aracati |
|--------|----------------|----------------|---------------|
| **Sonar Context** | Loop closure (NSSM) | Détection de boucles | ✅ |
| **DISO** | Odométrie front-end | Matching par intensité | ✅ |
| **SIO-UV** | Odométrie + débruitage | ICP front-end (besoin IMU) | ❌ |
| **ISOPoT** | Odométrie front-end | **Matching par paires → tracking multi-frames** | ✅ |

ISOPoT et SIO-UV répondent au même diagnostic que DISO (front-end faible) par deux voies
différentes ; ISOPoT est la plus moderne et la seule **sonar-only compétitive** sans IMU → pertinente
pour Aracati (pas d'IMU).

---

## Glossaire

| Terme | Définition |
|-------|-----------|
| **Point tracking / TAP** | Suivi de la trajectoire de points arbitraires sur une séquence (Track Any Point) — exploite le contexte temporel multi-frames |
| **TAPNext** | Modèle de tracking causal en ligne (prédiction de tokens de trajectoire), backbone d'ISOPoT |
| **Matching par paires** | Paradigme classique (DISO, SONIC) : apparier des keypoints entre 2 frames consécutives — fragile sur sonar |
| **Fenêtre glissante** | Ensemble de W frames traité conjointement ; B frames recouvrent la fenêtre précédente (warm-up) |
| **Grid Point Manager (GPM)** | Gestionnaire de points sur grille 16×16, max 5/cellule → couverture spatiale pour stabiliser RANSAC |
| **Match Refinement** | Raffinement par corrélation de descripteurs ResNet → une seule transformée SE(2) globale |
| **SE(2)** | Mouvement planaire (x, y, yaw) — seul observable par un sonar 2D (élévation perdue) |
| **Visibilité** | Drapeau par point indiquant s'il est visible dans une frame (gère speckle/ombres/multipath) |
| **ATE / RE** | Absolute Trajectory Error (dérive globale) / Relative Error (précision locale, segments 10%) |
| **Reflected pier poles** | Copies miroir des pieux du quai dans l'image sonar (artefact de réflexion d'Aracati) |
| **Portoroz 2025** | Nouveau dataset des auteurs (sonar Oculus 750d), dominé par le fond, séquences Boot et Lawnmower |

# Résumé complet — Robust Imaging Sonar-based Place Recognition and Localization

**Titre :** *Robust Imaging Sonar-based Place Recognition and Localization in Underwater Environments*
**Auteurs :** Hogyun Kim, Gilhwan Kang, Seokhwan Jeong, Seungjun Ma, Younggun Cho
**Institution :** Inha University, Incheon, Corée du Sud (SPARO Lab)
**Publié :** IEEE ICRA 2023, Londres, UK
**Code source :** https://github.com/sparolab/sonar_context.git

---

## Abstract

Méthode de **place recognition** (reconnaissance de lieux déjà visités) pour le SLAM sonar sous-marin. Elle encode l'information géométrique directement à partir des caractéristiques des images sonar brutes, **sans connaissance préalable ni entraînement** (pas de deep learning).

Trois contributions :
1. **SONAR Context** — descripteur global (coarse = *polar key*, fine = *SONAR context*)
2. **Adaptive shifting** — robustesse aux changements de rotation et translation
3. Estimation de la **pose initiale pour l'ICP** → meilleure fermeture de boucle

Testé en simulation (HOLOOCEAN), bassin réel (KRISO) et environnement réel (**Aracati2017**).

---

## I. Introduction

### Problème

La place recognition est essentielle pour le loop closure, qui corrige la dérive accumulée du SLAM. Sous l'eau :
- Pas de GPS fiable (atténuation électromagnétique)
- Vision optique limitée (turbidité de l'eau)
- → le **sonar imageur** est le capteur privilégié

### Limites des approches existantes

**Feature-based (ORB, SIFT, AKAZE) :**
- Cherchent des correspondances locales entre images sonar
- Problème : faible précision du loop detection à cause du manque d'information géométrique/structurelle sous l'eau

**Learning-based (CNN, Triplet networks) :**
- Bonnes performances mais **problèmes de mémoire** et **temps de calcul** → pas temps-réel
- Nécessitent un entraînement

**Opti-acoustique (sonar + caméra) :**
- Nécessite une caméra optique en plus → matching difficile (disparité d'échelle énorme entre sonar et caméra)

### Idée centrale

Inspiré de **Scan Context** (méthode LiDAR connue) mais adapté au sonar. Encode l'image sonar telle quelle (range + azimuth + intensité) en un descripteur global compact, sans features ni apprentissage → **rapide et temps-réel**.

---

## II. Background — Représentation SONAR

Une mesure sonar $p_s$ en coordonnées sphériques (range $r$, azimuth $\theta$, élévation $\phi$) :

$$p_s = \begin{bmatrix} x_s \\ y_s \\ z_s \end{bmatrix} = \begin{bmatrix} r\cos\theta\cos\phi \\ r\sin\theta\cos\phi \\ r\sin\phi \end{bmatrix}$$

> Comme DISO : l'**élévation $\phi$ est perdue** dans l'image sonar 2D → on travaille en range/azimuth uniquement.

L'image polaire encodée occupe $\mathcal{W} \times \mathcal{H}$ (largeur = azimuth, hauteur = range).

---

## III. Méthode proposée

Pipeline (Fig. 3) : **Place Description** + **Point Processing** en parallèle, puis **Place Recognition**, puis **Pose-Graph SLAM**.

### A. Place Description : SONAR Context + Polar Key

**SONAR Context (descripteur fin) :**
- Divise l'image en patches $\mathcal{P}_{ij}$ de taille $p_w \times p_h$ (ici 4×4)
- Pour chaque patch, garde l'**intensité maximale** comme représentant :

$$\mathcal{M}(\mathcal{P}_{ij}) = \max_{p \in \mathcal{P}_{ij}} i(p)$$

- Le contexte $\mathcal{I} \in \mathbb{R}^{A \times R}$ occupe l'espace azimuth × range
- Logique : forte intensité = structure (objet réfléchissant) → encode la géométrie de l'environnement

**Polar Key (descripteur grossier) :**
- Vecteur 1D résumant chaque contexte : moyenne d'intensité de chaque ligne (range)

$$\mathfrak{P}_j = \mathcal{F}(\mathbf{p}_{1j}, ..., \mathbf{p}_{Aj}), \quad j = 1, ..., R$$

- → vecteur de dimension $R$, sert à la recherche rapide

### B. Place Recognition (détection de candidats loop closure)

1. **Recherche rapide via Polar Key + KD-tree** : on compare le polar key de la frame courante à ceux des frames passées (distance euclidienne) → 1er voisin = candidat loop closure
2. **Vérification fine via SONAR Context** : distance cosinus colonne par colonne entre query et candidat

$$\mathcal{D}_a(\mathcal{I}^q, \mathcal{I}^c) = \frac{1}{A}\sum_{j=1}^{R}\left(1 - \frac{c_j^q \cdot c_j^c}{\|c_j^q\|\|c_j^c\|}\right)$$

### C. Adaptive Shifting (le cœur de la robustesse)

Sous l'eau, l'AUV peut revoir un lieu **sous un angle différent** → l'image est décalée → matching raté. Solution : shifter le contexte.

**Bounded-column shifting** (robustesse rotation) :
$$\mathcal{S}_a(\mathcal{I}^q, \mathcal{I}^c) = \min_{n \in [-\frac{A}{2}, \frac{A}{2}]} \mathcal{D}_a(\mathcal{I}^q, \mathcal{I}^c_{n \times \mu})$$

**Bounded-row shifting** (robustesse translation) : shift des lignes (range).

**Zero padding** : tient compte du FOV limité du sonar → on met à zéro les colonnes/lignes qui sortent du champ au lieu de faire un shift circulaire.

Le décalage optimal trouvé donne aussi la **pose relative initiale** pour l'ICP.

### C bis. Point Processing (en parallèle)

Pour le nuage de points utilisé par l'ICP :
- **Median filter** → réduit le bruit speckle
- **Otsu binarization** → extrait les points fiables
- → SONAR frame $S = (\mathfrak{P}, \mathcal{I}, \mathcal{C})$

### D. Loop factors pour Pose-Graph SLAM

Pose graph minimisant la dérive :

$$X^* = \arg\min_X \sum_t \|f(x_t, x_{t+1}) - z_{t,t+1}\|^2_{\Sigma_t} + \sum_{i,j \in LC}\|f(x_i, x_j) - z_{i,j}\|^2_{\Sigma_{i,j}}$$

- 1er terme : contraintes d'odométrie (6-DOF relatif)
- 2e terme : contraintes de loop closure $z_{i,j}$ (3-DOF XYH via ICP 2D)
- La pose initiale issue de l'adaptive shifting améliore la convergence de l'ICP

---

## IV. Résultats expérimentaux

### Datasets

| Dataset | Type | Capteur |
|---------|------|---------|
| HOLOOCEAN | Simulation (OpenWater, 2km) | sonar simulé |
| KRISO | Bassin réel 7×7m | DIDSON |
| **Aracati2017** | Marina réelle (Brésil) | BlueView P900-130 |

Comparé à : **Scan Context** (LiDAR original), **AKAZE** (features), **AKAZE+polar key**.

### Précision-Recall (Fig. 4)

- AKAZE (feature-based) → mauvaises performances partout (matching instable sous l'eau)
- Le polar key améliore déjà AKAZE → preuve de son utilité
- **Méthode proposée surpasse tous les autres**, surtout sur Aracati pour la robustesse rotation/translation

### Robustesse partielle (Fig. 5)

- Capte jusqu'à **~40° de rotation** et **5m de translation** avec **80% de précision**
- Bien meilleur que Scan Context (13.3%), AKAZE (10.2%), AKAZE+p (35.8%) → **82.1%**

### Localisation globale avec loop closure (Fig. 8)

- Sur Aracati : la trajectoire corrigée par loop closure suit bien la référence
- L'erreur de localisation **chute nettement** par rapport à l'odométrie seule
- L'estimation de pose initiale (Fig. 7) améliore l'alignement ICP avant registration

---

## V. Conclusion

Descripteur global **SONAR Context** pour place recognition robuste :
- Encode géométrie + intensité de l'image sonar brute, **sans apprentissage**
- **Adaptive shifting** → robuste aux rotations/translations
- Estime la pose initiale pour l'ICP → meilleur loop closure
- Surpasse Scan Context, AKAZE et AKAZE+p sur 3 environnements

**Perspectives :** extension à d'autres sonars (side-scan, profiling), SLAM multi-session, gestion de cartes.

---

## Lien avec mon projet (Bruce_SLAM + DISO)

| Module Bruce_SLAM | Actuellement | Apport de ce papier |
|-------------------|--------------|---------------------|
| Front-end odométrie | DISO (remplace SSM/ICP) | — |
| **Place recognition / loop closure (NSSM)** | **inopérant** (0 boucle détectée) | **SONAR Context** = détection de boucles robuste |
| Back-end | iSAM2 | — |

**Pertinence directe :** ce papier propose exactement une **meilleure détection de loop closure**
(SONAR Context + adaptive shifting), testée sur Aracati2017. C'est une **amélioration d'un
module existant** de Bruce_SLAM, pas un remplacement complet → conforme à mon objectif.

### Motivation expérimentale (mes runs sur Aracati2017)

J'ai testé le NSSM natif de Bruce_SLAM (détection par features + ICP) sur ce dataset. Le
diagnostic s'est fait en deux temps.

**1. Le NSSM était d'abord affamé de features.** Avec le paramètre `skip: 5` (1 scan sonar
traité sur 5), 69 % des keyframes étaient vides → le NSSM ne trouvait **aucune boucle** :

| Run DISO + Bruce_SLAM | ATE (Umeyama) | Loop closures |
|-----------------------|---------------|---------------|
| NSSM off | 5.4 m | 0 |
| NSSM on, seuils stricts | 6.2 m | 0 |
| NSSM on, seuils relâchés | 5.5 m | 0 |

**2. En passant à `skip: 1` (tous les scans traités), le NSSM détecte enfin des boucles —
mais elles sont FAUSSES.** Mesuré via la corrélation de forme avec le GT :

| Trajectoire | corr_y avec GT | Loop closures | ATE |
|-------------|----------------|---------------|-----|
| DISO standalone | -0.99 | — | **3.0 m** |
| Bruce AVANT correction NSSM | -0.88 | — | — |
| Bruce APRÈS 8 boucles (min_pcm 4) | **-0.12 (forme cassée)** | 8 | 11.3 m |
| Bruce, boucles filtrées (min_pcm 6) | -0.98 (forme OK) | 0 | **5.2 m** |

→ Les boucles proposées par le NSSM natif **dégradent** la trajectoire (corr_y -0.88 → -0.12).
Soit on les garde (forme cassée, ATE 11.3 m), soit on les filtre par PCM (0 boucle, ATE 5.2 m
mais aucun gain de loop closure). **Dans les deux cas, aucun bénéfice réel.**

Le problème n'est donc pas le réglage ni la quantité de features, mais la **qualité de la
détection** : features + ICP produisent de fausses correspondances sur les images sonar
BlueView basse résolution. C'est précisément le constat de ce papier (Fig. 4 : AKAZE/features
très mauvais sous l'eau), et SONAR Context y répond sans ICP ni features locales.

**Baseline à battre : ATE 5.2 m** (skip=1, min_pcm=6, 0 fausse boucle).

**Prochaine étape :** remplacer la détection de boucles features+ICP du NSSM par SONAR Context,
puis réactiver le NSSM → on attend de **vraies** boucles robustes (validables par PCM) et un
ATE Bruce **inférieur** à DISO standalone (le loop closure corrige enfin la dérive).

---

## Glossaire

| Terme | Définition |
|-------|-----------|
| **Place recognition** | Reconnaître un lieu déjà visité (≠ matching frame-à-frame) → déclenche le loop closure |
| **Loop closure** | Contrainte ajoutée au graphe quand le robot revient sur ses pas → corrige la dérive |
| **SONAR Context** | Descripteur global 2D (azimuth × range) encodant l'intensité max par patch |
| **Polar Key** | Vecteur 1D résumant le SONAR Context → recherche rapide par KD-tree |
| **Adaptive shifting** | Décalage du contexte pour matcher malgré rotation/translation du robot |
| **Scan Context** | Descripteur global LiDAR (méthode source d'inspiration) |
| **AKAZE** | Détecteur de features classique utilisé comme baseline comparative |
| **Zero padding** | Remplissage par zéros pour gérer le FOV limité du sonar (≠ shift circulaire) |
| **Blind Traversal** | Distance parcourue sans loop closure (plus court = meilleur) |

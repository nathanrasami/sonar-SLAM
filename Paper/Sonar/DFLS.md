# Résumé Complet détaillé — DFLSO: Direct Forward-Looking Sonar Odometry

**Titre :** Direct Forward-Looking Sonar Odometry: A Two-Stage Odometry for Underwater Robot Localization
**Auteurs :** Wenhao Xu, Jianmin Yang, Jinghang Mao, Haining Lu, Changyu Lu, Xinran Liu
**Institutions :** Shanghai Jiao Tong University (Chine)
**Publié :** Remote Sensing (MDPI), Juin 2025

---

## Abstract

DFLSO (Direct Forward-Looking Sonar Odometry) est une solution front-end d'odométrie légère conçue pour fournir une localisation rapide et précise aux robots sous-marins. L'algorithme repose sur une approche en **deux étapes** (two-stage). Il résout les limites des méthodes classiques (qui accumulent de la dérive) en :
1. Extrayant rapidement un **nuage de points 3D** à partir d'images sonar 2D en exploitant les ombres acoustiques.
2. Effectuant une double mise en correspondance : **image-à-image** (pour la pose relative instantanée) et **image-à-carte** (via des sous-cartes de keyframes) pour éliminer l'accumulation d'erreurs, sans attendre de fermeture de boucle globale. 
Validé en simulation et sur un ROV réel en mer.

---

## I. Introduction

Les robots sous-marins (inspection benthique, miniers) nécessitent une estimation de position extrêmement précise. Le FLS (Forward-Looking Sonar) est privilégié car il offre une imagerie acoustique en temps réel (jusqu'à 15 Hz) sur de longues distances (jusqu'à 100 m).

**Le problème fondamental :** La projection de l'environnement 3D sur une image 2D entraîne la **perte de l'angle d'élévation (φ)**, introduisant des ambiguïtés géométriques majeures. 

**Les limites actuelles :** L'odométrie pure (matching d'images adjacentes) accumule progressivement des erreurs. Le SLAM classique utilise la détection de fermeture de boucle (loop closure) pour atténuer la dérive à long terme, mais l'optimisation globale est retardée tant qu'aucune boucle n'est détectée. 
**L'approche DFLSO :** S'inspirer de l'odométrie LiDAR (LO) en reconstruisant une 3D pour effectuer un matching direct sur une sous-carte locale (scan-to-map), garantissant une localisation précise en temps réel.

---

## II. Related Work (Travaux connexes)

L'article identifie quatre approches historiques et leurs limites pour l'odométrie FLS :
* **Features 2D (Points/Régions) :** Utilisation de points (Harris, SVD, PnP) ou de régions couplée à des modèles de projection linéarisés. Ces méthodes sont très vulnérables au faible rapport signal/bruit (SNR) et à la basse résolution du sonar.
* **Analyse spectrale :** Utilisation de la transformée de Fourier spatiale. Requièrent des hypothèses fortes (altitude constante, roulis/tangage négligeables), limitant leur applicabilité.
* **Deep Learning :** Modèles (CNN, LSTM) estimant directement le mouvement. Dépendent massivement de données synthétiques, soulevant des problèmes de généralisation dans le monde réel.
* **Correction globale (Loop Closure) :** Dépendent fortement de la précision de la détection des boucles, négligeant le problème d'accumulation d'erreurs de dérive entre deux boucles (ce que DFLSO cherche à corriger).

---

## III. Méthodologie : L'algorithme DFLSO

### A. Modèle de projection FLS
Le FLS projette un point 3D polaire (r, φ, θ) sur un plan 2D en écrasant l'angle d'élévation (φ = 0). L'image brute polaire (beam-bin) est interpolée en un plan de pixels cartésiens P'(u_c, v_c) via des facteurs d'échelle.

### B. Traitement d'image et Extraction 3D
L'extraction du nuage de points s'appuie sur la dynamique des échos acoustiques : un terrain plat = intensité modérée ; un obstacle = forte intensité (highlight) suivie d'une ombre (shadow) ; une dépression = ombre isolée.

1.  **Pipeline de traitement d'image :**
    * **Filtre médian :** Suppression du bruit "poivre et sel".
    * **Compensation de gain :** Compense l'atténuation acoustique liée à la distance de propagation en moyennant les valeurs de pixels.
    * **Segmentation K-means (K=3) :** Classe les pixels en Highlight, Shadow, et Background.
    * **Morphologie mathématique :** Érosion (nettoyage) puis dilatation (consolidation des formes endommagées).
2.  **Estimation géométrique de l'élévation :**
    * **Obstacles (Saillies) :** Calculée trigonométriquement grâce à l'altitude connue du FLS (h_o), le début de l'obstacle (fin du highlight) et la fin de l'ombre portée.
    * **Tranchées (Dépressions) :** Élévation négative calculée à partir du point d'entrée et du point de sortie de l'ombre isolée.
3.  **Fast Point Cloud Extraction :**
    Pour garantir le temps réel, l'algorithme ne reconstruit pas la surface itérativement pixel par pixel. Il attribue l'élévation calculée au pixel frontière à **l'intégralité** des pixels de la zone correspondante.

### C. Étape 1 : Matching Image-to-Image (Pose Relative)
Le nuage de points extrait P_k est aligné avec le nuage précédent P_{k-1} par l'algorithme **ICP (Iterative Closest Point)**, fournissant la pose relative instantanée.

### D. Étape 2 : Matching Image-to-Map (Pose Globale)
Pour bloquer la dérive accumulée au Stage 1, le nuage courant est ré-aligné via ICP contre une sous-carte locale (Submap).

1.  **Sélection des Keyframes :**
    Recherche KNN (K-Nearest Neighbors) basée sur le **centre géométrique du champ de vision (FoV)** des keyframes (moitié de la portée maximale). Garantit une forte co-visibilité spatiale.
2.  **Adaptive Keyframing (Seuils d'insertion) :**
    Le seuil de distance pour insérer une nouvelle keyframe varie dynamiquement selon la distance médiane (m_k) des points détectés :
    * m_k > 40 m : Seuil de **15 m**
    * 30 m < m_k <= 40 m : Seuil de **10 m**
    * 10 m < m_k <= 30 m : Seuil de **5 m**
    * 5 m < m_k <= 10 m : Seuil de **2 m**
    * m_k <= 5 m : Seuil de **1 m**
    *(Le seuil angulaire est fixe à 1/6 de l'ouverture horizontale).*
3.  **Fast Submapping (Rastérisation anti-ghosting) :**
    Superposer brutalement les nuages engendre des artefacts de dédoublement ("ghosting") dus aux approximations d'élévation. DFLSO applique une grille de discrétisation 2D horizontale et **moyenne** la position et l'élévation des points dans chaque cellule.

---

## IV. Expérimentations et Résultats

Le papier compare trois méthodes d'odométrie : (1) Features 2D, (2) Nuages de points 3D (sans sous-carte), et (3) DFLSO (Framework complet à deux étapes).

### A. Validation en Simulation (Gazebo / UUV-Simulator)
* **Setup :** FLS simulé (10 Hz, portée 8m) à altitude fixe de 2m sur fond sableux/rocheux.
* **Temps de calcul :** DFLSO prend **32.7 ms / image** sur un CPU Intel Ultra 9 (16 cœurs) (compatible temps réel).
* **Comparatif Erreur de Position Absolue (RMSE) :**
    * 2-D Features : 2.16 m
    * 3-D Point Clouds : 0.57 m
    * **DFLSO : 0.28 m** (la seconde étape améliore drastiquement la cohérence).

### B. Essais Réels en Mer (Sea Trials)
* **Setup :** Crawler benthique testé à **2 000 mètres de profondeur** (Juillet 2024). Sonar BlueView M900 (15 Hz, portée 50m, FOV 120°).
* **Vérité terrain :** Obtenue par recalage manuel d'images FLS sur 4 points de référence.
* **Temps de calcul :** **52.6 ms / image** en moyenne sur le terrain.
* **Métriques Statistiques d'Erreur (RMSE) :**
    * 2-D Features : 1.5 m
    * 3-D Point Clouds : 0.9 m
    * **DFLSO : 0.5 m**

---

## V. Conclusions

DFLSO démontre qu'une méthode géométrique légère, capable d'extraire rapidement des nuages de points 3D de fonds marins à partir d'images sonar 2D, permet d'éviter les erreurs de pose liées aux ambiguïtés projectives. En couplant cela à une architecture à deux étapes (Scan-to-Scan puis Scan-to-Submap basé sur des keyframes), le système garantit une haute précision temps réel et atténue la dérive **sans dépendre d'un bouclage strict (loop closure)**.

---

## 📌 Comparaison contextuelle (DFLSO vs Bruce-SLAM & SIO-UV)

* **Face à Bruce-SLAM :** Bruce-SLAM s'appuie sur une odométrie front-end géométrique 2D (SSM/ICP) vulnérable en milieu peu texturé, et compte sur les *loop closures* globales (back-end) pour rattraper la dérive. DFLSO attaque le problème à la racine (au niveau du front-end) en lissant la dérive en continu via sa sous-carte locale 3D rastérisée, ce qui le rend beaucoup plus robuste avant même toute considération de fermeture de boucle.
* **Face à SIO-UV :** SIO-UV densifie aussi l'information via une "pseudo-3D" (stacking vertical) et un débruitage poussé (MCFAR), mais dépend vitalement d'une pré-intégration IMU de haute précision. DFLSO propose une alternative purement acoustique et géométrique : il recrée une vraie approximation topographique 3D par l'analyse géométrique des ombres, offrant une solution robuste de correction locale (Scan-to-Submap) pour les systèmes où l'IMU n'est pas prédominant ou absent.
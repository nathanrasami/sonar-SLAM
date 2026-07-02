# Résumé complet — SONIC: Sonar Image Correspondence using Pose Supervised Learning for Imaging Sonars

**Titre :** *SONIC: Sonar Image Correspondence using Pose Supervised Learning for Imaging Sonars*
**Auteurs :** Samiran Gode, Akshay Hinduja, Michael Kaess
**Institution :** Robotics Institute, Carnegie Mellon University (Robot Perception Lab — le labo
de Kaess, la lignée directe de l'ICP sonar utilisé par Bruce-SLAM)
**Publié :** arXiv:2310.15023 (oct. 2023, révisé mai 2024) — financé ONR
**Code :** https://github.com/rpl-cmu/sonic (code + datasets simulés et réels publics)

---

## Abstract

L'association de données est LE problème dur du SLAM sonar : une même scène vue de deux points
de vue différents produit des images sonar très différentes (projection acoustique, perte de
l'élévation) → les correspondances de features classiques échouent. SONIC est un réseau
**supervisé par la pose** (pas par des correspondances vérité-terrain, introuvables en sonar)
qui apprend des correspondances denses **robustes au changement de point de vue**, pour des
contraintes de loop closure plus précises et de la place recognition.

---

## I. Le problème : ambiguïté d'élévation et association de données

Une caméra pinhole perd la profondeur ; un sonar imageur perd **l'élévation φ**. Un pixel sonar
encode (range r, bearing θ) et projette tout l'arc d'élévation sur le plan φ=0 :
p = r·[cos θ, sin θ]ᵀ. Conséquence : deux vues du même objet ne sont PAS liées par une
homographie/épipolaire classique — les méthodes features caméra (SIFT/SuperPoint/LightGlue)
se dégradent fortement (le warping polaire n'est pas dans leur distribution d'entraînement).

## II. L'idée clé : géométrie épipolaire SONAR comme supervision

Pas de vérité-terrain de correspondances nécessaire. Étant donnée la pose relative (R, t) entre
deux frames (connue en simu / par gantry) :
1. un point p₁ = (r, θ) de l'image 1 définit un **arc d'élévation** en 3D (tous les φ possibles) ;
2. cet arc, transformé par (R, t) et reprojeté en polaire dans l'image 2, donne un
   **contour épipolaire** ;
3. le réseau est entraîné à placer sa correspondance prédite SUR ce contour.

**Losses :** L = 0.7·L_épipolaire (distance min au contour, en polaire) + 0.3·L_cyclique
(aller-retour 1→2→1 doit revenir au point de départ).

## III. Architecture

- Encodeur-décodeur **ResNet-34** + couche de matching différentiable, entrée mono-canal.
- Travail en **espace POLAIRE** (range × bearing) — préserve la géométrie sonar (même leçon
  que notre fix Sonar Context : le shift-colonne ↔ rotation ne vaut qu'en polaire).
- Matching **coarse-to-fine** : niveau grossier sur toute l'image, raffinement local ensuite.
- Inférence : corrélation du descripteur du keypoint avec toute l'image 2 → softmax →
  correspondance = espérance de la distribution, avec **pondération par incertitude** pour
  filtrer les matches douteux.

## IV. Entraînement : 100 % simulation HoloOcean (!)

- **HoloOcean** (notre simulateur du chantier 2 !) : ~300 000 paires d'entraînement + 30 000 de
  validation, 10 scènes, placement d'objets randomisé.
- Sonar simulé : Blueprint **M1200d** low-freq — 130° FOV azimutal, 20° d'élévation, 10 m de
  portée, 512×512 bins range-bearing.
- Test réel : bassin de 7 m, capteur sur portique 6-DOF, vérité Leica Total Station.

## V. Résultats (inlier ratio = matches cohérents avec la pose GT)

| Méthode | Simu (petites variations) | Simu (±40°, ±7 m) | Bassin réel |
|---|---|---|---|
| AKAZE (handcrafted) | 24.2 % | 10.2 % | 40.1 % |
| LightGlue (SuperPoint, caméra) | 39.1 % | 11.2 % | 51.9 % |
| **SONIC** | **49.4 %** | **23.6 %** | **74.5 %** |

Erreur de pose planaire (RANSAC sur les matches, simu) : SONIC 0.88 m / 0.25 rad
vs LightGlue 3.62 m / 0.97 rad vs AKAZE 4.24 m / 1.56 rad.

## VI. ⚠ SONIC sur Aracati (évalué par ISOPoT, pas par les auteurs)

Le papier ISOPoT (cf. `ISOPoT.md` §D) a évalué SONIC **en odométrie frame-à-frame** sur
Aracati 2017 : ATE 36.6/113.3/69.8 m par section, 137 % d'erreur de translation — mauvais.
**Lecture correcte** : en odométrie pure sur un port épars (eau libre sans features), TOUTE
méthode d'appariement échoue (même leçon que notre diagnostic DISO GT-free). Ce n'est PAS le
cas d'usage de SONIC : son terrain, c'est l'appariement **à la revisite** (deux vues d'une même
structure avec grand changement de point de vue) — exactement le maillon loop-closure.

## VII. Limites (avouées par les auteurs)

- Entraîné 100 % simu : gap sim→réel à combler (tests en eau libre nécessaires).
- Bassin réel : variations de pose limitées, clutter des réflexions métalliques.
- Un modèle par mode de fréquence sonar ; le cross-sonar matching = travail futur.

---

## Ce que ça apporte à MON stage (pourquoi ce papier)

1. **Il attaque NOTRE goulot chiffré.** Post-fix miroir, la chaîne loop est : Sonar Context
   détecte 122 candidats (tous vrais, 0 faux) → **l'ICP n'en convertit que 82 en contraintes**
   (diagnostic 1.1). Le maillon faible n'est plus la détection mais **l'association géométrique
   au point de revisite** — précisément ce que SONIC remplace : correspondances robustes au
   point de vue → transform + inliers, à la place de shgo+ICP point-à-point.
2. **Synergie HoloOcean.** SONIC est entraîné sur HoloOcean avec un sonar 130° : notre chantier
   2 utilise le même simulateur — on pourrait générer des paires d'entraînement avec la config
   sonar du collègue (et le pipeline polaire qu'on a déjà).
3. **Lignée cohérente pour le récit du stage :** Kim ICRA23 (Sonar Context, détection) →
   SONIC (association) → facteurs gtsam (back-end Bruce). Chaque maillon remplacé par l'état
   de l'art, avec une mesure à l'appui.
4. **Code public** (github.com/rpl-cmu/sonic) — intégrable en post-traitement offline d'abord
   (rejouer les 122 paires candidates du run 141223 et comparer transform SONIC vs ICP vs GT),
   AVANT tout branchement online. C'est le test discriminant bon marché.

**Pièges à déclarer si on l'intègre :** (a) P900 Aracati = images cartésiennes → repasser en
polaire (on a déjà `_polar_remap` côté Sonar Context) ; (b) modèle entraîné pour un M1200d
10 m de portée → réentraînement ou fine-tuning probablement nécessaire pour le P900 48 m
(HoloOcean peut générer les données) ; (c) réseau au runtime = CPU/GPU en plus (leçon
contention DISO : tester à rate 0.5 ou en post-traitement).

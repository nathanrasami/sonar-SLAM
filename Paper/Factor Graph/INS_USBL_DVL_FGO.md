# Résumé complet — A Robust INS/USBL/DVL Integrated Navigation Algorithm Using Graph Optimization

**Titre :** *A Robust INS/USBL/DVL Integrated Navigation Algorithm Using Graph Optimization*
**Auteurs :** Peijuan Li, Yiting Liu, Tingwu Yan, Shutao Yang, Rui Li — Nanjing Institute of Technology
**Publié :** *Sensors* 2023, 23, 916 (MDPI), doi 10.3390/s23020916
**Code :** non publié (solveur : Ceres, dérivation automatique des jacobiennes)

---

## Abstract

Fusion multi-capteurs pour la navigation AUV : INS (dérive cumulative), USBL (position
acoustique absolue, bruitée), DVL (vitesse Doppler). Deux propositions : (1) remplacer le
filtre de Kalman traditionnel par une **optimisation de graphe de facteurs (FGO)** —
itérations multiples + optimisation conjointe de l'historique ; (2) un module **robuste aux
outliers** fondé sur le **critère de correntropie maximale** (MCC), intégré au FGO, pour le
bruit non-gaussien de l'USBL/DVL. Gains terrain vs Kalman : **5.3 / 9.1 / 5.1 %** (E/N/haut).

## I. Contexte

- INS : autonome mais diverge (biais gyro/accéléro intégrés).
- **USBL** : position absolue par distance oblique r (temps aller-retour) + angles d'arrivée
  (α, β) vers un transpondeur de position connue — mais sujette aux **outliers**
  (multipath, pings ratés) → bruit **non-gaussien**.
- DVL : vitesse dans le repère véhicule (effet Doppler sur le fond), pas de position.
- Tradition = Kalman (KF) et variantes robustes (Huber = HKF). Limite : linéarisation
  unique, état courant seulement.

## II. Modèles capteurs (§2)

- USBL : position du transpondeur dans le repère u : t^u = [r·cosα, r·cosβ,
  −√(r²−(r cosα)²−(r cosβ)²)] (Eq. 1) ; position AUV via les matrices de passage (Eq. 2-3),
  erreur d'installation u→b calibrée d'avance.
- DVL : vitesse b-frame par décalage Doppler f_d (Eq. 4-6).

## III. Le graphe de facteurs (§3, Fig. 2)

État X = position, vitesse, attitude (+ biais IMU). Quatre types de facteurs :
1. **Prior** (état initial).
2. **IMU pré-intégration** (Eq. 11-20) : α (position), β (vitesse), γ (attitude,
   quaternions) pré-intégrés entre keyframes, avec correction au 1er ordre des biais
   (jacobiennes J) — le standard VINS/GTSAM.
3. **USBL** (Eq. 21-23) : résidu dans **l'espace MESURE** — [r̃−r, α̃−α, β̃−β] — pas en
   position reconstruite. (≠ mon implémentation Bruce : prior unaire en position x,y.)
4. **DVL** (Eq. 24-25) : résidu de vitesse C_n^b·V^n − V^b.

MAP total (Eq. 26) = somme des ‖résidus‖²_Σ → résolu par Ceres.

## IV. Le module robuste : correntropie maximale (§3.4) — LE cœur du papier

- Correntropie V(X,Y) = E[κ(X,Y)] avec noyau gaussien G_σ(e) = exp(e²/2σ²) (Eq. 27-29).
- Idée : au lieu de MINIMISER un carré (hypersensible aux outliers), on MAXIMISE la
  correntropie des résidus → un résidu énorme sature le noyau et **perd son influence**.
- Mise en œuvre élégante (Eq. 30-36) : tout se ramène à des **covariances repondérées
  itérativement** : Σ̃ = Σ^{-T/2}·ψ·Σ^{-1/2} avec ψ = diag(G_σ(e_i)) — chaque mesure
  USBL/DVL reçoit un poids ∈ (0,1] selon son résidu courant. C'est de l'IRLS
  (iteratively reweighted least squares) avec le noyau de Welsch — cousin du
  **Cauchy** que Bruce utilise déjà sur ses facteurs USBL.

## V. Résultats

**Simulation** (trajectoire carrée 2 m/s, transpondeur central, 5 % d'outliers injectés
×100-200 sur les covariances, Eq. 37 ; IMU 200 Hz, USBL 0.5 Hz, DVL 1 Hz) — RMSE (m) :

| Méthode | PE (est) | PN (nord) | PU (haut) |
|---|---|---|---|
| KF | 0.79 | 0.96 | 0.34 |
| HKF (Huber) | 0.34 | 0.56 | 0.34 |
| FGO (non robuste) | 1.14 | 1.12 | 0.36 |
| **RFGO (proposé)** | **0.30** | **0.36** | **0.26** |

⚠ Le twist : le **FGO nu est PIRE que le Kalman** (mécanisation INS simplifiée + zéro
robustesse) — c'est le module robuste qui fait gagner le graphe. Monte-Carlo 50 tirages :
même hiérarchie.

**Test terrain** (fleuve Yangtze, bateau ; USBL 0.1 m+1 % r à 0.5 Hz, DVL ±0.5 %, IMU
200 Hz ; vérité = RTK GPS + PHINS 2-5 cm) — RMSE (m) :

| Méthode | PE | PN | PU |
|---|---|---|---|
| KF | 1.33 | 1.54 | 2.57 |
| HKF | 1.32 | 1.46 | 2.54 |
| FGO | 1.59 | 1.72 | 2.60 |
| **RFGO** | **1.26** | **1.40** | **2.44** |

Gains terrain **modestes** (5.3/9.1/5.1 % vs KF). Coût : le RFGO est le plus cher et son
temps CROÎT avec la durée (lissage complet, pas de marginalisation — Fig. 9).

## VI. Limites (lecture critique)

- Gains terrain faibles vs la complexité ajoutée ; pas de marginalisation/fenêtre glissante
  (pas temps-réel long terme) — là où Bruce utilise iSAM2 (incrémental).
- σ du noyau MCC non discuté (le hyperparamètre sensible du papier).
- Pas de comparaison à Cauchy/Huber DANS le graphe (seulement Huber dans le Kalman).
- Code non publié ; capteurs riches (IMU 200 Hz + DVL) — Aracati n'a ni l'un ni l'autre.

---

## Ce que ça apporte à MON stage (pourquoi ce papier)

1. **C'est exactement mon back-end.** Bruce+USBL = un graphe de facteurs avec des priors
   USBL — ce papier est la version « navigation » du même problème, et il VALIDE mes
   choix : facteurs USBL robustes (mon Cauchy ≈ leur MCC, même famille IRLS) + gate
   d'outliers (mes sauts ~73 m = les pings ratés qu'ils modélisent en Eq. 37).
2. **Il éclaire ma saga du sigma** (ablation A/B/B′, runs 1.3/1.4) : j'ai mesuré qu'une
   ancre RAIDE (σ=1.0-1.4) casse la cohérence scan et qu'une ancre DOUCE la préserve.
   Leur réponse est plus élégante : **ne pas choisir un σ fixe — repondérer chaque fix
   par son résidu** (ψ = G_σ(e)). Piste concrète : transformer mon sigma USBL fixe en
   repondération adaptative par fix.
3. **Le facteur USBL en espace mesure** (r, α, β) plutôt qu'en position : plus juste
   physiquement (le bruit est gaussien en r/angles, pas en x,y — l'erreur position croît
   avec la distance au transpondeur : 0.1 m + 1 % r). Sur Aracati je n'ai que
   /usbl_point (position déjà reconstruite) — à noter comme limite de données.
4. **La leçon anti-hype** : un graphe SANS robustesse perd contre un Kalman — le récit
   de mon stage (fix de chiralité, gates, PCM, noyaux) est du même bois : la robustesse
   est le vrai ingrédient, pas l'outil d'optimisation.

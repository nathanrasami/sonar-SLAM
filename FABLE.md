# FABLE.md — Analyse critique de l'architecture (Claude Fable 5, 2026-06-11)

Analyse demandée après le handoff de compaction. Couvre : migration 3D, pourquoi
DISO+Bruce < DISO standalone, Bruce original vs Bruce+DISO, Sonar Context
(remplacement ou ajout), et pistes priorisées pour passer sous 3 m d'ATE sur Aracati.

Fichiers lus pour cette analyse : `SLAM_3D_MIGRATION.md`, `ENSUITE.md`,
`bruce_slam/config/slam_aracati.yaml`, `bruce_slam/config/feature_aracati.yaml`,
`bruce_slam/config/icp.yaml`, `bruce_slam/src/bruce_slam/slam.py` (NSSM ~850-1010),
`bruce_slam/src/bruce_slam/slam_objects.py` (Keyframe), `bruce_slam/src/bruce_slam/odom_bridge.py`,
`traj_eval.py`, `analyze_drift.py`, `run_slam.sh`, et sur la branche `feature/sonar-context` :
`sonar_context.py` + diff de `feature_extraction.py`.

---

## 1. Migration 3D : faisable, mais le plan vise la mauvaise cible (et c'est une bonne nouvelle)

Le plan `SLAM_3D_MIGRATION.md` est honnête sur le point dur (ICP 3D), mais il rate
**l'observation qui change tout** : Bruce est 3-DOF *par conception*, pas par limitation.
Sous l'eau, z, roll et pitch sont **directement observables** par capteurs :

- capteur de pression → z absolu, SANS dérive ;
- IMU → roll/pitch absolus (référence gravité), SANS dérive.

C'est exactement pourquoi l'auteur de Bruce n'optimise que x, y, yaw. Le code le
confirme : `Keyframe.update()` (`slam_objects.py:137-155`) prend x, y, yaw du SLAM et
**garde z/roll/pitch du dead-reckoning** — ce n'est pas un échafaudage incomplet,
c'est l'architecture voulue.

**Conséquence** : pour la demande de la tutrice (trajectoire avec Z + carte
volumétrique), il n'y a PAS besoin de migrer le graphe en Pose3. Il faut :

1. z depuis le capteur de pression HoloOcean, roll/pitch depuis l'IMU → injectés dans
   `dr_pose3` ; `pose3` sort alors déjà complet, sans toucher au back-end ;
2. des points 3D réels au lieu de `np.zeros` pour l'élévation → couvert par le
   PointCloud2 3D publié par le simulateur (court-circuite l'extraction polaire 2D) ;
3. l'export CSV + ATE 3D (étapes 1-2 du plan, qui sont correctes :
   généraliser `umeyama` à `np.eye(d)`, interpoler z dans `associer_par_temps`).

Ça correspond aux étapes 0-2 du plan. **L'étape 3 (back-end Pose3 6DOF) a un ROI très
discutable** : un graphe Pose3 où z/roll/pitch ne sont quasi pas contraints par le
sonar (un sonar imageur ne contraint pas l'élévation) est numériquement plus fragile
que le 3-DOF actuel, pour un gain quasi nul puisque ces dimensions sont mieux mesurées
par les capteurs. C'est l'état de l'art en SLAM sous-marin : on parle de SLAM
"4-DOF" (x, y, z, yaw) où z vient de la pression et roll/pitch de l'IMU.

Si l'étape 3 est quand même faite un jour :
- le piège de l'ordre des sigmas gtsam Pose3 `[roll, pitch, yaw, x, y, z]` est réel ;
- il y a plus de 2D câblé en dur que le plan ne le dit : tout le code NSSM
  (`slam.py:884-895`) manipule `cov[:2,:2]`, `cov[2,2]`, des bearings `arctan2` 2D —
  la covariance 3×3 est partout (gating, PCM, échantillonnage shgo).

**Verdict** : faisable, oui — mais redéfinir l'objectif comme « carte et trajectoire
3D avec dimensions observables par capteurs » (étapes 0-2 + `dr_pose3` alimenté par
pression/IMU), pas « SLAM 6DOF complet ». ~10× moins de travail, et défendable
scientifiquement. L'étape 5 (ICP 3D) ne se justifie que avec des nuages 3D denses et
beaucoup de temps.

⚠️ **HOLOOCEAN_3D_SPECS.md n'existe pas sur main** (seul `ENSUITE.md`, mémo de
4 lignes, existe). Les specs pour le collègue n'ont apparemment jamais été commitées —
à retrouver/réécrire (le contenu est résumé dans le plan : PointCloud2 3D, GT avec z,
IMU, DVL avec vz, depth, conventions de repère à figer, surtout le sens de z).

---

## 2. DISO+Bruce < DISO standalone : l'analyse utilisateur est aux 2/3 correcte

### (a) Contention CPU : VALIDÉE — et c'est la mesure elle-même qui le prouve

L'odométrie DISO *dans le run combiné* fait 3.9 m vs 2.1 m standalone. DISO est une
méthode directe : si des frames sont droppées, la baseline inter-frames grandit et
l'optimisation photométrique converge moins bien. **C'est la cause dominante.**

Test discriminant trivial : rejouer le run combiné avec `rosbag play -r 0.5`
(tout est en `use_sim_time`, donc rien d'autre ne change). Si l'odom DISO du run
combiné redescend vers 2.1 m → confirmé. (Modifier le `-r 1.0` dans
`aracati.launch:26` ou passer par `run_slam.sh`.)

### (b) Sous-échantillonnage (600 keyframes vs 14000 poses) : artefact d'ÉVALUATION, pas dégradation réelle

Un graphe iSAM2 avec uniquement un prior + des facteurs d'odométrie a pour solution
optimale **exactement la chaîne d'odométrie** : Bruce ne peut pas « dégrader » DISO de
3.9 → 5.4 m algorithmiquement. La différence vient de :

1. l'ATE calculé sur 189 keyframes vs 12468 points — Umeyama ajuste sur des
   distributions d'échantillons différentes, le RMSE pondère différemment les zones ;
2. l'association temporelle odométrie↔keyframe (interpolation du cache odom au stamp
   des features).

Test discriminant : sous-échantillonner `odometry.csv` aux timestamps de
`trajectory.csv` et recalculer son ATE. Prédiction : il remonte vers ~5 m. Si oui,
Bruce ne perd presque rien — il *paraît* pire à cause de la métrique.

### (c) « Overhead iSAM2 » : à moitié vrai

L'overhead CPU contribue à (a), mais iSAM2 ne corrompt pas l'estimée. Reformulation
correcte : iSAM2 sans loop closure valide est un **coût sans bénéfice**, pas une
source d'erreur.

### Le point manquant : ce n'est pas un bug, c'est une propriété

Sans loop closure acceptée, Bruce+DISO = DISO ré-échantillonné + bruit d'association,
moins du CPU. Il est **structurellement impossible** que Bruce+DISO batte DISO
standalone dans cette configuration. Toute la proposition de valeur du stage repose
sur Sonar Context : c'est le seul mécanisme qui injecte de l'information nouvelle
dans le graphe.

---

## 3. Bruce original (SSM/ICP) mieux que Bruce+DISO ? Non — raison architecturale, pas de tuning

**Le point clé : dans le Bruce original, le SSM n'est PAS l'odométrie.** L'odométrie
de Bruce vient de la fusion DVL+IMU (le `LOCALIZATION_ODOM_TOPIC`), et le SSM/ICP ne
fait que la *raffiner*, avec cette bonne odométrie comme initialisation.

Or Aracati n'a **ni DVL ni IMU** : la seule odométrie native est /cmd_vel
(ATE 14.7 m). « Bruce original sur Aracati » = ICP initialisé par une odométrie qui
dérive de 14.7 m. L'ICP est une optimisation locale : sans bonne init, il diverge.
C'est exactement ce qui a été observé. **Aucun tuning de CFAR/ICP ne répare une
initialisation à 14 m de la vérité.** Le remplacement de l'odométrie par DISO est
donc architecturalement sain.

Faiblesses de fond qui s'ajoutent :
- le CFAR est appliqué **sur l'image cartésienne** (`feature_extraction.py:285-287`)
  alors que le CFAR suppose un bruit homogène par cellule — l'interpolation
  polaire→cartésien corrèle les pixels et la densité varie avec la distance →
  détections statistiquement bancales ;
- le P900 est basse résolution ;
- `Pfa: 0.1` est très permissif (10 % de fausses alarmes par cellule testée).

### MAIS : un test bon marché et prometteur jamais fait

Remettre `ssm.enable: True` **par-dessus DISO**. C'est la vraie architecture de
Bruce : dead-reckoning de qualité (rôle joué ici par DISO) + raffinement ICP
séquentiel. Avec DISO comme init (erreur inter-keyframe de quelques cm), l'ICP du SSM
a toutes les chances de converger, et il ajoute des contraintes **indépendantes** de
DISO dans le graphe. C'est la seule configuration où Bruce+DISO peut battre DISO
*sans* loop closure.

Paramètres de départ : `icp.yaml` tel quel ; `Pfa: 0.05` ; surveiller le taux de
rejet du SSM ; `ssm.max_translation` peut être serré de 3.0 → 1.5 vu la qualité de
DISO.

---

## 4. Sonar Context : c'est un REMPLACEMENT de la détection — approche correcte, avec un défaut réel à corriger

### Clarification remplacement vs ajout

Dans le papier de Kim (ICRA 2023), SONAR Context est un descripteur de *place
recognition*. Il remplace **la proposition de candidats de boucle** — chez Bruce, le
bloc « détection » du NSSM : recherche par recouvrement de FOV gatée par covariance +
`np.argmax(counts)` (`slam.py:913-915`). Il ne remplace **pas** le recalage
géométrique : le papier fait toujours une registration derrière pour la contrainte.

→ L'architecture choisie (remplacer la détection NSSM, garder l'ICP de contrainte)
est **exactement la bonne**.

**Bonus sous-exploité** : le `best_azimuth_shift` retourné par
`cosine_distance_shifted` est une estimation de la rotation relative → utilisable
comme **initialisation de l'ICP de contrainte, à la place du shgo**
(`slam.py:950-963`), qui est l'étape la plus coûteuse et fragile du NSSM.
Double gain : meilleure détection + init quasi gratuite.

### Critique 1 — DÉFAUT RÉEL : le contexte est construit sur l'image cartésienne

Le commentaire de `build_sonar_context` dit « colonnes ~ azimuth » — **c'est faux en
cartésien** : les colonnes sont des positions x, pas des gisements. Conséquence : le
« shift azimuth » de l'adaptive shifting ne correspond plus à une rotation du
véhicule. L'équivalence shift↔rotation, cœur du papier de Kim, ne tient qu'en
**polaire**. Ça marche approximativement à longue portée et petites rotations, mais
dégrade précision/rappel.

**Correctif** : re-projeter l'image cartésienne en polaire avant
`build_sonar_context` (`cv2.warpPolar`, ou inversion de la géométrie fov=130°,
range=50 m déjà codée dans `feature_extraction.py`). À faire AVANT l'étape 5 de
validation, sinon la validation se fera sur des bases faussées.

### Critique 2 — Keyframe a déjà les champs prévus

`Keyframe` a **déjà** des champs `ring_key` et `context` inutilisés
(`slam_objects.py`, vestiges du Bruce multi-robot). Pour l'étape 2, ranger le
descripteur là : c'est l'emplacement prévu par l'auteur. Le `Float32MultiArray` avec
timestamp encodé en tête fonctionne (pas de header sur ce type de message — choix
acceptable), garder ce transport.

### Étapes 2-5 — comment finir

- **Étape 2** : cache côté SLAM (`slam_ros.py`), association par stamp →
  `kf.context` / `kf.ring_key`.
- **Étape 3** : recherche rapide par `scipy.spatial.cKDTree` sur les Polar Keys,
  reconstruit toutes les N keyframes (volumes petits, pas d'optimisation nécessaire).
  Candidats → `cosine_distance_shifted` → seuil. Garder `min_st_sep`. **Garder PCM**
  comme deuxième barrière (détection par apparence + vérification géométrique =
  ceinture et bretelles).
- **Étape 4** : params YAML (`sonar_context/enable`, `num_azimuth`, `num_range`,
  `max_col_shift`, `max_row_shift`, seuil de distance).
- **Étape 5 — validation SHADOW d'abord** : loguer les candidats détectés avec leur
  distance GT réelle (loops_detected.csv), mesurer précision/rappel, choisir le seuil
  de distance cosinus sur la courbe, ENSUITE seulement activer dans le graphe.

---

## 5. Pistes priorisées pour passer sous 3 m sur Aracati

| # | Action | Coût | Gain attendu |
|---|--------|------|--------------|
| 1 | **Contention CPU** : `rosbag play -r 0.5` dans le run combiné | ~1 h | 5.4 → ~3.5 m. Préalable à tout : inutile d'évaluer des loops sur une odométrie dégradée. Légitime (limite de la VM, pas de l'algo) |
| 2 | **Finir Sonar Context** (avec correctif polaire, §4) | étapes 2-5 | Le SEUL levier pour passer SOUS DISO standalone — seul ajout d'information. Aracati a des revisites → quelques vraies boucles valent tous les tunings |
| 3 | **SSM réactivé sur DISO** (§3) | ~1 jour | Contraintes ICP indépendantes de DISO, cheap à tester |
| 4 | **Shift Sonar Context comme init ICP** au lieu de shgo | petit | Robustesse + CPU (renforce #1) |
| 5 | **Hygiène d'évaluation** (voir ci-dessous) | quelques heures | Comparaisons honnêtes, présentation défendable |

### Hygiène d'évaluation (point 5 détaillé)

- (a) Comparer les ATE **à timestamps identiques** (cf. §2b) — sinon on se compare à
  soi-même avec deux règles différentes.
- (b) Ajouter le **RPE** (Relative Pose Error, erreur relative par mètre parcouru) en
  plus de l'ATE : c'est le RPE qui montre l'apport des boucles ; l'ATE seul peut le
  masquer.
- (c) **Umeyama : légitime, mais corriger `allow_reflection`.** L'alignement Umeyama
  est 100 % standard (TUM/evo, Horn/Umeyama 1991) et la translation cosmétique
  post-ATE est propre. MAIS `allow_reflection=True` (`traj_eval.py:77`) n'est PAS
  standard — evo force `det(R)=+1`. Une réflexion dans l'alignement peut masquer un
  vrai bug de miroir d'un estimateur et flatter l'ATE. **Correctif** : inverser le
  signe de l'axe Y de DISO à l'export (une ligne, convention documentée), puis
  repasser `allow_reflection=False`. Devant un jury, c'est le genre de détail qu'on
  sortira.

### Hiérarchie attendue si tout fonctionne

odom pure 14.7 m → DISO seul 2-3 m → DISO+SSM ~2 m → DISO+SSM+Sonar Context **< 2 m**

### Récit de stage

Solide : remplacement des deux maillons faibles de Bruce sur un dataset sans DVL/IMU —
odométrie par DISO (méthode directe, ICRA 2024), détection de boucle par apparence
(SONAR Context, ICRA 2023) — chaque choix justifié par une mesure.

---

## Rappels d'état (mesures de référence)

| Configuration | ATE |
|---------------|-----|
| Odométrie pure (/cmd_vel intégré) | 14.7 m |
| DISO standalone | 2.1–3.0 m (non déterministe, dépend du timing) |
| Odom DISO dans run combiné | 3.9 m (preuve de la contention CPU) |
| DISO + Bruce (NSSM off) | ~5.4 m (artefact d'échantillonnage, cf. §2b) |
| DISO + Bruce (NSSM on, min_pcm 4) | 11.3 m (8 boucles FAUSSES) |
| DISO + Bruce (NSSM on, min_pcm 6) | 5.2 m (0 boucle — baseline à battre) |

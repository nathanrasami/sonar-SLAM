# Presentation 4 — SIO-UV (paper) + my Aracati simulation results

Slide script for Canva. One slide = one `##` block.
Each block: **CANVA** = what goes on the slide / **ORAL** = what you say.
Two parts: **Part 1 = the SIO-UV paper**, **Part 2 = my current simulation results**.

═══════════════════════════════════════════════════════════
# PART 1 — The SIO-UV method (paper review)
═══════════════════════════════════════════════════════════

## Slide 1 — Context: what is SIO-UV?

**CANVA**
- Title: *SIO-UV — Sonar-Inertial Odometry for Underwater Vehicles* (IEEE T-IE 2025)
- 3 short bullets (no detail yet):
  - A **complete underwater SLAM system** — estimates trajectory + builds a map
  - Inputs: **a sonar (FLS) + an IMU**, fused in a **factor graph**
  - **A different system from Bruce-SLAM** — not an extension, built from scratch
- Small icon: sonar + IMU → trajectory

**ORAL**
> "Je présente SIO-UV, un papier de 2025. C'est un système de SLAM sous-marin complet, qui fusionne un sonar et une centrale inertielle dans un factor graph. Le point important : ce n'est **pas** une amélioration de Bruce-SLAM, c'est une méthode concurrente, indépendante. Je la présente parce qu'elle **bat Bruce-SLAM** et qu'elle éclaire ce qui manque à Bruce. On ne rentre pas dans les détails ici — juste le contexte."

---

## Slide 2 — Overview: follow the data (Fig. 1)

**CANVA**
- **Fig. 1 du papier** en grand
- 2 entrées à gauche + 1 bloc calibration + flèche vers « 2 threads parallèles » :
  - **Input — IMU** : accélération linéaire + vitesse angulaire (~100 Hz)
  - **Input — FLS** : image sonar 2D (fan horizontal, ~5 Hz)
  - **Time calibration** : aligne les horodatages IMU/sonar (l'IMU est 20× plus rapide) → pour chaque image sonar, on sait quel intervalle IMU lui correspond
- Encadrer les 2 chemins parallèles : *IMU thread* et *FLS thread*

**ORAL**
> "On va suivre le chemin des données. Deux entrées : l'IMU, qui donne l'accélération linéaire et la vitesse angulaire à 100 Hz, et le sonar FLS, qui donne une image 2D à 5 Hz. Première étape commune : la calibration temporelle. L'IMU est 20 fois plus rapide que le sonar ; cette étape aligne les horloges pour qu'à chaque image sonar on associe le bon intervalle d'IMU. Ensuite les données partent dans deux chaînes parallèles que je détaille une par une — d'abord l'IMU, puis le sonar, puis le back-end."

---

## Slide 3 — Thread 1: IMU pre-integration

**CANVA**
- Titre : *IMU thread — fast initial guess*
- **IN** : mesures IMU brutes (accél. + vitesse ang.) sur l'intervalle entre 2 images sonar
- **PROCESS** : pré-intégration (Forster) = on **intègre deux fois** les n mesures IMU — accél. → vitesse → position — pour obtenir le déplacement relatif entre les 2 images sonar. Le résultat est stocké comme **un seul terme** (ΔP, ΔV, ΔR) qu'on ne recalcule pas à chaque itération d'optimisation
- **OUT** : `T_imu` = transformation initiale (pose relative) entre 2 keyframes sonar
- État estimé : `x = [position, rotation, vitesse, biais_accel, biais_gyro]`

**ORAL**
> "Premier thread, l'IMU. En entrée : les mesures brutes d'accéléromètre et de gyroscope, sur l'intervalle entre deux images sonar. Le process s'appelle la pré-intégration : au lieu de réintégrer toutes les mesures IMU à chaque itération d'optimisation — ce qui serait très coûteux — on les condense en un seul terme de déplacement relatif. En sortie : T_imu, une estimation initiale du mouvement entre deux keyframes. C'est rapide mais ça dérive — d'où le besoin du sonar pour corriger."

---

## Slide 4 — Thread 2: FLS reverse regression mapping (3 blocks)

**CANVA**
- Titre : *Sonar thread — from 2D image to 3D features*
- **IN** : 2 images sonar 2D consécutives
- **3 blocs en chaîne** :
  1. **MCFAR** — débruitage (sépare cible / bruit) *(zoom slide suivant)*
  2. **3D point cloud** — reconstruit un nuage 3D à partir de l'image 2D *(zoom slide d'après)*
  3. **Feature extraction** — extrait arêtes / surfaces / features horizontales (façon LOAM)
- **OUT** : keyframes sonar + nuage 3D + features → vers le back-end

**ORAL**
> "Deuxième thread, le sonar. En entrée : les images sonar 2D. Elles traversent trois blocs en série. Un : MCFAR, qui débruite et sépare les vraies cibles du bruit acoustique. Deux : la reconstruction d'un nuage 3D à partir de l'image 2D. Trois : l'extraction de features, où on classe les points en arêtes et surfaces, comme en LiDAR. En sortie : des keyframes avec leur nuage 3D et leurs features, prêts pour le back-end. Je zoome sur les deux premiers blocs."

---

## Slide 5 — Zoom block 1: MCFAR denoising  [optional]

**CANVA**
- **Fig. 2 du papier** (3 fenêtres de tailles différentes sur le fan sonar)
- Formule compacte : `T_final = min(T₁, T₂, T₃)`
- CFAR = seuil adaptatif (cible si l'intensité dépasse le bruit local)
- **MCFAR** = 3 tailles de fenêtre → seuil **minimum** → aucune petite cible manquée

**ORAL**
> "Zoom sur MCFAR. CFAR, c'est un seuil adaptatif : un pixel est une cible s'il ressort du bruit local. Bruce utilise une seule taille de fenêtre. SIO-UV en utilise trois et garde le seuil minimum : une cible vue à une seule échelle survit. Moins de détections manquées. C'est aussi la seule brique que je pourrais réutiliser chez moi."

---

## Slide 6 — Zoom block 2: 2D → 3D point cloud  [optional]

**CANVA**
- **Fig. 3 du papier** (fan horizontal → empilement vertical → cylindre 3D)
- Formule : `P = R·[cosθcosφ, sinθcosφ, sinφ]ᵀ`
- ⚠️ Note honnête : *élévation φ **supposée**, pas mesurée*

**ORAL**
> "Zoom sur la reconstruction 3D. L'image sonar perd l'angle d'élévation — tout est aplati dans un plan. SIO-UV suppose cet angle petit et empile le scan horizontal pour fabriquer un nuage 3D. Les auteurs le reconnaissent : c'est une hypothèse, pas une vraie mesure verticale. Mais ça densifie le nuage, donc le matching est plus stable."

---

## Slide 7 — Back-end: the forward path

**CANVA**
- Titre : *BackEnd — from features to optimized pose*
- Chemin (gauche → droite) :
  - **IN** : keyframes + nuage 3D + features (du sonar) **et** `T_imu` (de l'IMU)
  - **Feature Match** : features de la keyframe ↔ **sous-carte locale**, initialisé par `T_imu`
    → produit `T_sonar = w₁·T_h + w₂·T_c` (2 matchings fusionnés : horizontal + arêtes/surfaces)
  - **Factor Graph** : nœuds = poses ; facteurs = IMU (`T_imu`) + odométrie sonar (`T_sonar`) + boucle
    → optimisation MAP (iSAM2) sur fenêtre glissante → **poses optimisées**

**ORAL**
> "Le back-end, dans le sens de la marche. En entrée il reçoit deux choses : du sonar les keyframes et leurs features, de l'IMU la pose initiale T_imu. Premier traitement, le feature match : on aligne les features de la keyframe courante avec la sous-carte locale, en partant de T_imu comme initialisation. Ça produit T_sonar, l'odométrie sonar — ici deux matchings fusionnés, un sur les features horizontales, un sur les arêtes et surfaces. Ensuite tout entre dans le factor graph : les poses sont les nœuds, et on a trois types de contraintes — IMU, odométrie sonar, et boucles. iSAM2 optimise le tout et sort les poses corrigées."

---

## Slide 8 — Back-end: the two feedback loops (bidirectional)

**CANVA**
- Titre : *Why two arrows go backwards*
- **Boucle 1 — Loop closure (bidirectionnel)** :
  - poses optimisées → **Loop Detection** (cherche keyframes proches + valide par ICP) → **Loop Factor** → **réinjecté dans le factor graph** → ré-optimisation globale
  - *bidirectionnel* car : le graphe fournit la pose pour **trouver** les candidats, et la boucle trouvée **corrige** le graphe en retour
- **Boucle 2 — IMU bias calibration** :
  - poses optimisées → renvoyées à la pré-intégration IMU → **corrige les biais** accél./gyro
- Schéma : 2 flèches retour sur la Fig. 1

**ORAL**
> "Et il y a deux flèches qui repartent en arrière — c'est le point clé. Première boucle, la fermeture de boucle. Les poses optimisées vont vers la détection de boucle, qui cherche des keyframes spatialement proches et les valide par ICP, ce qui crée une contrainte de boucle réinjectée dans le graphe. C'est bidirectionnel : il faut la pose courante du graphe pour savoir quels lieux passés sont proches, et la boucle trouvée corrige ensuite tout le graphe. Deuxième boucle : les poses optimisées repartent vers la pré-intégration IMU pour recalibrer ses biais. L'IMU aide le graphe vers l'avant, le graphe corrige l'IMU vers l'arrière. C'est ce couplage croisé qui rend SIO-UV robuste."

---

## Slide 9 — Results in numbers

**CANVA**
- Tableau (RMSE, plus bas = mieux) :

| Scene | **SIO-UV** | Bruce-SLAM |
|-------|-----------|-----------|
| Sim 40×40 (clear) | **1.79 m** | 8.48 m |
| Sim 30×30 (degraded) | **1.86 m** | 9.30 m |
| Real pool 25×25 | **1.68 m** | **22.2 m** |

- Note : *chiffres du papier — pas de code public, non reproductible sur Aracati (pas d'IMU, sonar 2D)*

**ORAL**
> "Les chiffres du papier. En simulation Bruce fait 8 à 9 m d'erreur. En piscine réelle il s'effondre à 22 m. SIO-UV tient sous 2 m partout. Précision : ce sont leurs chiffres, je n'ai pas pu les reproduire — pas de code public, et SIO-UV exige un IMU et de la vraie 3D que le dataset Aracati n'a pas."

---

## Slide 10 — SIO-UV vs Bruce-SLAM (the method)

**CANVA**
- Titre : *Same diagnosis, different fix*
- Tableau conceptuel :

| | Bruce-SLAM | SIO-UV |
|---|---|---|
| Odometry front-end | single 2D-ICP (weak) | MCFAR + 3D + LOAM |
| Sensors | sonar (+EKF) | sonar **+ IMU** |
| Why Bruce fails | ICP explodes in turns / low texture | — |

**ORAL**
> "La comparaison de fond. SIO-UV et Bruce partent du même problème : l'odométrie front-end de Bruce, un simple ICP 2D, est le maillon faible — elle explose dans les virages et les scènes peu texturées. SIO-UV y répond avec du MCFAR, de la 3D et un IMU. Ce diagnostic, c'est exactement celui de mon stage — et ça justifie mon choix d'avoir remplacé l'ICP de Bruce."

---

## Slide 11 — What I can borrow (short)

**CANVA**
- Une seule ligne forte : **Only MCFAR is portable to Aracati** (drop-in CFAR, no IMU needed)
- Les deux autres : ❌ 3D-stacking (faux 3D, Aracati est 2D) · ❌ LOAM (besoin IMU)
- Petit encart : *Je suis encore en intégration de Sonar Context → pas de plan d'action figé*

**ORAL**
> "Concrètement, sur Aracati, la seule brique réutilisable c'est MCFAR — un simple remplacement du CFAR existant. Le reste bloque sur l'absence d'IMU. Je note ça pour plus tard ; pour l'instant je suis encore sur l'intégration de Sonar Context, donc pas de plan d'action définitif là-dessus."

═══════════════════════════════════════════════════════════
# PART 2 — My current Aracati simulation results
═══════════════════════════════════════════════════════════

## Slide 12 — Part 2: my own simulations

**CANVA**
- Titre : *Part 2 — Where my Bruce-SLAM stands today (Aracati2017)*
- Setup :
  - Dataset **Aracati2017** (ROV réel, sonar BlueView, GPS = ground truth)
  - Ma pipeline : **DISO (odométrie) → Bruce-SLAM (iSAM2) → Sonar Context (loop closure)**
  - Question mesurée : **les fermetures de boucle améliorent-elles la trajectoire ?**

**ORAL**
> "Deuxième partie, indépendante du papier : où en est concrètement mon Bruce-SLAM sur le dataset Aracati. Ma pipeline c'est DISO pour l'odométrie, Bruce et iSAM2 pour le back-end, et Sonar Context que j'intègre pour les fermetures de boucle. La question que je mesure : est-ce que les boucles améliorent vraiment la trajectoire par rapport à l'odométrie seule ?"

---

## Slide 13 — Methodology: trustworthy ATE

**CANVA**
- Titre : *Making the comparison fair*
- 3 points (avec ✅) :
  - **Common origin** : toutes les trajectoires partent de (0,0) — le CSV le confirme ; le plot les y ramène (translation **cosmétique, après calcul de l'ATE** → ne fausse rien)
  - **Fixed alignment bug** : avant, la traj Bruce était tournée de ~45° pour coller à la GT (absurde). Cause : alignement manuel fragile (flip-Y + centroïde). **Corrigé** → alignement rigide optimal (Umeyama)
  - **ATE** = on aligne (rotation+translation, réflexion Y autorisée) puis **RMSE des distances à la GT**
- Optionnel : avant/après du plot (traj tournée vs alignée)

**ORAL**
> "Avant de montrer les résultats, la rigueur de la mesure. Trois points. Un : toutes les trajectoires partent de zéro-zéro — le CSV le dit, et le plot les y ramène par une translation purement cosmétique appliquée **après** le calcul de l'erreur, donc ça ne triche pas. Deux : il y avait un bug — la trajectoire de Bruce était tournée de 45° pour coller à la vérité terrain, ce qui n'a aucun sens physique. La cause était un alignement manuel bricolé. On l'a remplacé par un alignement rigide optimal, l'algorithme d'Umeyama, qui n'absorbe que le flip d'axe Y légitime de DISO, pas une rotation inventée. Trois : l'ATE, c'est cet alignement puis la moyenne quadratique des distances à la vérité terrain. Donc le chiffre est fiable."

---

## Slide 14 — Result: loop closures finally help

**CANVA**
- Run : `run_aracati_2026-06-14_212922`
- Tableau :

| Configuration | ATE |
|---------------|-----|
| Odom DISO seule (équitable, aux keyframes) | 5.18 m |
| **Bruce-SLAM + Sonar Context** | **4.54 m** |
| **Gain des fermetures de boucle** | **+0.63 m** ✅ |

- Plot trajectoire du run (GT vs odom vs SLAM)

**ORAL**
> "Le résultat actuel. L'odométrie DISO seule, comparée équitablement aux mêmes instants, fait 5,18 m. Avec Bruce et Sonar Context, on descend à 4,54 m — soit 0,63 m de mieux. C'est la première fois que les fermetures de boucle **améliorent** la trajectoire au lieu de la dégrader : avant, les boucles étaient fausses et cassaient tout. Là, elles aident. C'est le résultat clé de mon intégration de Sonar Context."

---

## Slide 15 — Takeaways

**CANVA**
1. **SIO-UV** confirms (from outside) that **Bruce's ICP front-end is the weak link**
2. Only **MCFAR** is portable to Aracati (no IMU) — noted for later
3. **My current Bruce-SLAM**: loop closures now **help** (+0.63 m) with a **trustworthy ATE**
4. Next: keep improving Bruce-SLAM itself

**ORAL**
> "Pour conclure. Un, SIO-UV confirme de l'extérieur que le front-end de Bruce est le problème central. Deux, seule la brique MCFAR est réutilisable chez moi, je la garde pour plus tard. Trois, et c'est l'essentiel : mon Bruce-SLAM actuel montre enfin des fermetures de boucle qui améliorent la trajectoire, avec une métrique d'erreur fiable. La suite, c'est de continuer à améliorer Bruce lui-même."

---

## Quick glossary (for questions)

- **FLS**: Forward-Looking Sonar — sonar imageur multifaisceaux
- **CFAR / MCFAR**: détecteur à seuil adaptatif (MCFAR = multi-échelle, seuil = min)
- **IMU pre-integration**: accumulation IMU entre keyframes (Forster et al.)
- **LOAM / PL-ICP**: odométrie par features de courbure / ICP point-to-line
- **iSAM2**: solveur incrémental de factor graph (utilisé par Bruce ET SIO-UV)
- **DISO**: Direct Sonar Odometry — mon front-end d'odométrie (remplace l'ICP de Bruce)
- **Umeyama**: alignement rigide optimal entre deux trajectoires (avant calcul d'ATE)
- **ATE**: Absolute Trajectory Error — RMSE des distances à la GT après alignement

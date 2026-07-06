# COMPARE.md — Nos runs Aracati2017 vs les méthodes présentées dans `Paper/`

Uniquement des tableaux (protocole détaillé : `MINI_PAPIER.md` §VI, `analysis/paper_eval.py`).
Runs source : `TESTS_image/run_aracati_2026-07-04_233119_Bruce_USBL_1` (**Bruce**, σ2.5 USBL
soft-anchor, sans Sonar Context) et `TESTS_image/run_aracati_2026-07-04_201541_Bruce_Sonar_USBL_1`
(**Bruce_Sonar_USBL**, champion répété, σ1.4 + Sonar Context). Recalculés le 07-06 avec
`paper_eval.py` (mêmes formules que le mini-papier, pas de nouveaux runs).

---

## 1. Seule métrique dont NOUS disposons pour toutes nos méthodes : ATE Umeyama SE(2), par section

Alignement rigide optimal (position+cap, sans échelle) sur toute la trajectoire, puis re-aligné
indépendamment sur chaque tiers temporel (S1/S2/S3, ~15 min chacun). **Ni DISO ni ISOPoT ne
publient cette variante** (ils utilisent l'alignement 1ʳᵉ-pose par section, cf. tableau 2) — ce
tableau ne compare donc que nos propres méthodes entre elles.

| Méthode | ATE um. global (m) | S1 (m) | S2 (m) | S3 (m) | Cap méd. (°) |
|---|---|---|---|---|---|
| Odométrie pure (`/cmd_vel`) | 10.61 | — | — | — | — |
| **Bruce** (σ2.5, sans Sonar Context) | 1.74 | 2.04 | 1.62 | 1.41 | 2.0 |
| **Bruce_Sonar_USBL** (champion, σ1.4 + SC) | **1.45** | **1.78** | **1.40** | **1.08** | 3.0 |
| Référence assistée GT (borne, DISO+GT-seed) | 0.89 | 0.49 | 1.18 | 0.57 | 1.6 |

- Bruce_Sonar_USBL gagne S2/S3 nettement (loops Sonar Context, qui n'existent pas sur la branche
  Bruce pure) mais perd S1 (5.13 m en 1ʳᵉ-pose, cf. tableau 2 — début de mission, peu de boucles
  encore détectées).
- Chiffres de continuité avec le mini-papier (run `003823`, config identique, plus ancien) :
  ATE 1.50 m, S1/S2/S3 1.81/1.36/1.27 — cohérent avec la répétabilité déjà mesurée
  (1.45-1.60 m, méd. 1.5±0.1, cf. `TESTS.md` §2.4).

---

## 2. Convention comparable à ISOPoT/DISO : ATE 1ʳᵉ-pose par section

C'est la métrique que publient DISO et ISOPoT : alignement sur le **départ de chaque section
seulement** (pas de best-fit) → mesure la dérive long-horizon, pas la précision moyenne. Beaucoup
plus sévère que l'Umeyama du tableau 1 (cf. `MINI_PAPIER.md` §6.2b) — **ne pas comparer les deux
tableaux entre eux**, seulement ligne à ligne dans un même tableau.

| Aux | Méthode | ATE S1/S2/S3 (m) | Trans. (%) | Rot. (°/m) |
|---|---|---|---|---|
| Aucun | SONIC (Aracati, ISOPoT Table I) | 36.6 / 113.3 / 69.8 | 137.65 | 2.45 |
| Aucun | ISOPoT (sonar seul) | **8.8 / 12.7 / 16.7** | 22.76 | 0.99 |
| Odom+Mag | Odométrie seule (ISOPoT Table I) | 5.8 / 12.5 / 6.5 | 19.07 | 0.0 |
| Odom+Mag | DISO (re-testé par ISOPoT) | 5.3 / 6.1 / 10.9 | 13.90 | 0.44 |
| Odom+Mag | SONIC (Odom+Mag) | 7.0 / 11.2 / 13.7 | 22.83 | 0.0 |
| Odom+Mag | ISOPoT (Odom+Mag) | **3.2 / 3.5 / 4.6** | 9.69 | 0.0 |
| USBL+loops | **Bruce** (nous, σ2.5) | 4.79 / 3.55 / 1.75 | 23.63¹ | 0.057 |
| USBL+loops+SC | **Bruce_Sonar_USBL** (nous, champion) | **5.13 / 2.38 / 4.42** | 5.00 | 0.090 |
| — | Odométrie pure (nous, `/cmd_vel`) | — | 32.6 | 0.00 |

¹ Trans. 23.6 % de ce run précis est un artefact du seed de cap USBL (β 112° au lieu des ~94°
habituels) qui gonfle la RE (déplacements exprimés dans le repère de chaque pose) sans affecter
l'ATE ni la carte — cf. `TESTS.md` §2.4 note 4. Les autres runs Bruce mesurés : ~9-12 %.

**Piège à ne pas reproduire en slide** (déjà noté dans `ISOPoT.md` §"Attention au piège") : nos
méthodes sont des **SLAM** (ancrage absolu USBL + fermetures de boucle) contre des **odométries
pures/frontales** (DISO, ISOPoT, SONIC) — même sur le même dataset, ce n'est pas le même problème
résolu. Une odométrie qui bat notre SLAM en 1ʳᵉ-pose ne serait pas surprenante ; ce n'est pas le
cas ici mais l'inverse ne prouve pas grand-chose non plus dans l'autre sens.

DISO publie aussi ses propres chiffres Aracati dans sa Table II, en **RE% par section** (pas en
ATE — protocole différent de sa ré-évaluation par ISOPoT) : Trans. 5.91/9.08/7.28 → 8.69 %
overall, Rot. 0.17/0.19/0.16 → 0.25°/m overall. Les deux jeux de chiffres DISO (ici et dans le
tableau ci-dessus) **ne sont pas le même run/toolbox** — ne pas les fusionner.

---

## 3. Erreur relative globale (RE, segments 10 %) — résumé simple à l'oral

| Méthode | Trans. (%) | Rot. (°/m) | Type |
|---|---|---|---|
| SONIC sonar-only | 137.65 | 2.45 | odométrie front-end pure |
| ISOPoT sonar-only | 22.76 | 0.99 | odométrie front-end pure |
| Odométrie /cmd_vel (Aracati, tous) | ~19-33 | **0.00** | odométrie du bord (compas) |
| DISO (assisté) | 13.90 | 0.44 | odométrie front-end + odom ext. |
| SONIC (assisté) | 22.83 | 0.00 | odométrie front-end + odom ext. |
| ISOPoT (assisté) | **9.69** | 0.00 | odométrie front-end + odom ext. |
| **Bruce_Sonar_USBL** (nous) | **5.00** | 0.090 | **SLAM** (USBL absolu + loops SC) |
| **Bruce** (nous, run cité) | 23.63¹ | 0.057 | **SLAM** (USBL absolu, sans SC) |

¹ voir note du tableau 2 — artefact de ce run précis, pas de la méthode (autres runs Bruce : ~9-12 %).

---

## 4. Le cap à 0.00°/m : nuance magnétomètre — réponse à ta question

Tous les « Odom+Mag » ci-dessus affichent une erreur de rotation **quasi nulle**, ce qui ressemble
à de la triche. Ce qu'on sait avec certitude (audit du bag Aracati2017, `FABLE.md` §2) :

- Le bag Aracati2017 lui-même **n'expose que 8 topics**, aucun `/imu` ni `/compass` brut —
  seulement `/cmd_vel`, dont le README du dataset précise (verbatim) : *« the angular velocity
  in Z (heading) is estimated from the vehicle compass »*. C'est donc bien un compas **embarqué
  sur le véhicule** (le ROV LBV300-5), **pas** un instrument du bateau porteur du DGPS — le README
  dit explicitement « vehicle compass », et le bateau ne porte que le DGPS de vérité terrain.
- Notre propre odométrie `/cmd_vel` (même source, aucune GT) a une RE rotation mesurée de
  **0.00-0.003 °/m** sur tous nos runs (tableaux ci-dessus) — **identique** aux lignes « Odom+Mag »
  de DISO/SONIC/ISOPoT. C'est une coïncidence trop precise pour ne pas être le même signal : il n'y
  a qu'**une seule** source de cap publiée dans ce dataset, donc « leur » magnétomètre et « notre »
  compas `/cmd_vel` sont très probablement **le même topic**, pas un capteur supplémentaire qu'ils
  auraient eux-mêmes rajouté.
- Point que je n'ai **pas** vérifié formellement : le nom exact du topic ROS que DISO/ISOPoT
  remappent en interne pour leur bloc « Odom+Mag » (leur code de chargement Aracati n'est pas
  inspecté ligne à ligne ici). DISO publie son code (https://github.com/SenseRoboticsLab/DISO) —
  à ouvrir si tu veux la certitude à 100 % plutôt qu'une très forte présomption.

**Conclusion pour la slide** : ce n'est pas une triche isolée d'ISOPoT — c'est la même
convention que nous (cap = compas du bord, légitime sur un UV réel), et probablement le **même
topic physique** que notre `/cmd_vel`. Le tableau 3 le confirme numériquement : nos RE rotation
(0.057-0.090°/m) sont même légèrement supérieures à 0.00 parce que le SLAM *touche* au cap via les
contraintes de boucle/graphe — une odométrie pure ne le fait jamais, d'où le 0.00 exact partout.

---

## 5. Méthodes de `Paper/` non comparables numériquement sur Aracati2017

Pas testées sur ce dataset (ou pas en ATE/RE) — citées comme inspiration de conception, pas comme
concurrentes chiffrées :

| Papier | Testé sur Aracati ? | Pourquoi non comparable | Rôle dans notre pipeline |
|---|---|---|---|
| **Sonar Context** (Kim, ICRA 2023) | ✅ mais en Précision-Recall (PR-curve loop closure), pas ATE | métrique différente | base du détecteur de boucles par apparence (§III mini-papier) |
| **DRACo-SLAM** | ❌ (multi-robot, DVL+IMU requis) | dataset et capteurs différents (Aracati n'a ni DVL ni IMU) | inspiration architecture pose-graph multi-robot, non utilisée telle quelle |
| **ULCDfMS** (Loop) | ❌ | sonar mécanique rotatif (branche `caves`, pas Aracati) | inspiration compensation de balayage lent (piste ouverte, caves) |
| **Factor Graph INS/USBL/DVL** | ❌ (code non publié, IMU 200Hz+DVL requis) | capteurs absents d'Aracati | inspiration facteur USBL robuste (Cauchy) dans notre back-end (§2.4) |

---

*Sources numériques : `analysis/paper_eval.py` (nos runs, recalculé 07-06) ; `Paper/Sonar/DISO.md`
Table II ; `Paper/Sonar/ISOPoT.md` Table I ; `Paper/MiniPapier/MINI_PAPIER.md` §6.3
(référence GT, continuité 1.2a).*

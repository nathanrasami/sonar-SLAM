# FABLE.md — Investigations (Claude Fable 5) — v2, 2026-07-02

Remplace la v1 du 2026-06-11 (obsolète : DISO/SC intégrés depuis, ATE 5.2 m → 1.5 m).
Deux chantiers : **Aracati** (améliorer, viser ATE < 1 m, caractériser) et **HoloOcean** (branche
`holoocean`, SLAM 2D puis 3D). Suivi quotidien : `PROGRESS.md`. Historique : `STAGE.md`.

---

## 1. RÉSOLU — le point-cloud « tourbillon » était un bug de miroir

**Cause racine** : en mode cmd_vel, `feature_extraction.py` extrait le y latéral en convention
image (+droite, « même signe que DISO » = repère réfléchi), alors que l'odométrie cmd_vel/USBL
est en repère propre (θ ≈ cap monde ENU, Umeyama det=+1). Chaque scan était donc peint **en
miroir de son cap** → en virage tout se smear en arcs. Le pipeline DISO (poses en repère
lui-même réfléchi, det=−1) compensait accidentellement → la carte nette du run réf `011733`.

**Preuves** (décomposition offline, scripts scratchpad) :
- Poses GT parfaites (position ET cap) sur le cloud de `150034` : NN **inchangé** 0.365→0.365
  → ni le cap ni la position n'étaient le goulot.
- Clouds locaux 100 % identiques entre `011733` et `150959` sur les frames communes
  → pas les détections non plus.
- Rendu des deux runs au **même** jeu de poses : sans miroir 12 655 cellules ≈ réf (12 243) ;
  avec miroir 24 990. CQFD.

**Fix** (non commité, branche de travail) :
- `bruce_slam/src/bruce_slam/feature_extraction.py` : param `cartesian/flip_bearing`, flip
  appliqué dans le packing `_publish_features_stamped` (⚠ APRÈS la relecture d'intensité —
  un 1er essai avant la relecture donnait un cloud famélique de 4 325 pts, run `111934` invalide).
- `bruce_slam/config/feature_aracati.yaml` : `flip_bearing: True` (**mettre False si
  `odom_source=diso`**, son repère est réfléchi).
- Post-traitement des runs existants : `fix_mirror_cloud.py` → `pointcloud_demiroir.csv/png`
  (⚠ pas pour `111746` : rendu use_compass_cap, theta ≠ cap de rendu).

**Validation** (run natif `run_aracati_2026-07-02_141223`, 100 % GT-free) :

| | avant (150034) | après fix (141223) | réf GT (011733) |
|---|---|---|---|
| NN médian cloud | 0.365 | **0.203** | 0.199 |
| Aire (cellules 0,5 m) | 24 223 | **11 098** | ~19 800* |
| ATE Umeyama | 1.46 m | 1.53 m | 0.89 m |
| Cap méd / RMS | 3.6° / 6.1° | 3.4° / 5.8° | 1.7° / 2.9° |
| Loops NSSM acceptées | ~122 | 82 | — |

*011733 non filtré (toutes intensités) — pas comparable cellule à cellule avec un cloud CFAR-255.
Le quai en T est visible (fig. `bilan_run.png` du run). La hiérarchie des rendus est enfin saine :
poses GT (0.138) < hybrides < estimé (0.203) → il reste **~30 % de netteté à gagner via les poses**.

**Décodeur de conventions** (à connaître pour toute analyse de cap) :
`groundtruth.csv:theta` = cap compas **NED** : gθ ≈ −θ_map + 90.7° (résidu 3.6° méd).
`cmd_vel.angular.z` = −d(gθ)/dt → intégration = cap **ENU** propre. D'où tous les « s=−1 »,
« offset 90°/162° », « compas ~50° tourné » des analyses passées.

**Retombée** : la divergence DISO GT-free (ATE 22 m) s'explique probablement par le même
mismatch de chiralité — prior cmd_vel à cap ENU donné à un tracker en repère réfléchi : après
un virage de 90°, le prior est ~180° faux → « too few inlier ». Test bon marché : DISO +
prior cmd_vel avec **wz inversé**. Si ça marche, on récupère une odométrie DISO GT-free.

---

## 2. Audit GT-free (fait, verdict : ✅ conforme, une nuance)

- `/pose_gt` n'entre **jamais** dans l'estimation : seul subscriber côté SLAM =
  `slam_ros.py:190` → `_gt_callback` → `export_csv` (groundtruth.csv). En mode
  `seed_from_usbl` (défaut), `cmd_vel_odom` ne crée même pas le subscriber GT (chaîne elif).
- Entrées de l'estimation : `/son/compressed`, `/cmd_vel`, `/usbl_point`. Topics vérifiés
  au `rosbag info` (8 topics, pas d'IMU/compas/DVL/profondeur exposés).
- ⚠ **Nuance à déclarer** (README aracati2017, verbatim) : « */cmd_vel — the angular velocity
  in Z (heading) is estimated from the vehicle compass* ». Le wz intégré EST dérivé du compas
  embarqué. C'est un capteur réaliste sur UV (tout ROV a un compas), mais si la contrainte
  « pas de compas » est stricte, il n'existe AUCUNE source de cap dans ce bag — même le nœud
  odom original d'aracati2017 utilise ce wz. À assumer explicitement dans le rapport.
- `/usbl` : « when the ping fails the transponder position is published » (README) → les
  glitches ~73 m sont documentés ; le gate de vitesse est la bonne parade.
- Question « groundtruth.csv suffit-il pour le cap ? » : **oui** — bit-identique entre tous
  les runs (même flux `/pose_gt`), un « run GT parfait » ne changerait que l'estimée, pas la
  référence. Erreur de cap véridique : fit circulaire s·θ+β, wrap, offset retiré (bilan_run.py).

---

## 3. « Bruce pur (branche `Bruce`) peut-il battre le bricolage ? »

État des lieux (runs existants, ATE Umeyama recalculés) :

| Run | Config | ATE |
|---|---|---|
| `Bruce_2026-06-26_102700/112326` | Bruce pur, SSM/NSSM/USBL off (partiels ~250 KF) | 5.4 / 3.7 m (= DR pur) |
| `Bruce_2026-06-26_135507` | + test USBL front-end (partiel) | 5.8 m (DR 10.6) |
| `Bruce_DIO_Sonar_USBL_2026-06-27` | complet, 652 KF | 4.55 m (DR 59 !) |
| **Bricolage** (`Bruce_Sonar_USBL`) | cmd_vel + USBL back-end + Sonar Context | **1.43–1.53 m** |

**Le point qui change la donne : tous les verdicts « SSM diverge » (05-27, ATE 14 m) et
« loops NSSM natives fausses » (06-10, ATE 11.3 m) datent d'AVANT le fix miroir.** SSM et NSSM
font de l'ICP scan-contre-scan initialisé par l'odométrie : avec des scans miroités face à une
odométrie propre, l'init était systématiquement à contre-sens en rotation — échec structurel,
pas algorithmique. Même chose pour les facteurs de loop : transforms ICP mirror-conjugués
(rotation de signe inversé) injectés dans un graphe propre.

**Réponse honnête** : « Bruce pur » strict (cmd_vel + SSM + NSSM, sans USBL ni SC) n'a aucune
ancre globale — sur 44 min avec un DR compas-based, l'ATE dépendra entièrement des loops.
Possible qu'il s'approche du bricolage, improbable qu'il le batte. MAIS le test n'a jamais été
fait avec des scans corrects, et il est bon marché. **Plan d'ablation post-fix (3 runs × 45 min,
branche `Bruce` + fix porté)** :
1. **A** : Bruce pur (SSM on, NSSM natif on, USBL off, SC off) → mesure la vraie valeur des
   modules natifs réparés.
2. **B** : A + USBL back-end (USBL = capteur du bag, pas une « méthode papier ») → le candidat
   minimal défendable.
3. **C** (référence) : bricolage actuel post-fix.
Verdict par ATE + NN + nombre de loops saines. Si B ≈ C, le récit du stage se simplifie
énormément (« Bruce réparé suffit ») ; si C garde 0.3–0.5 m d'avance, le bricolage est justifié
par les chiffres.

### Post-ablation (07-02 soir) — anatomie des 1.95 m de A, et le « miracle » possible

Ablation FAITE : **A = 1.95 m** (champion Bruce pur), **B = 2.03 m** (l'ancre USBL sigma 1.0
DÉGRADE tout — murs doublés), C = 1.53 m. Verdict complet : `ABLATION.md`.

Décomposition offline du résidu de A :
- **Pas un problème de loops** : zones très revisitées 1.28 m ≈ zones peu revisitées 1.43 m
  (revisites abondantes partout, 33-49 passages proches des points à fort résidu).
- Profil = gauchissement quasi UNIFORME ~1.3-1.4 m + **UN événement local 5.2-5.7 m à
  t≈14-15 min** (vitesse GT normale → vrai décrochage SLAM, pas un artefact DGPS ;
  cap local 3.5° vs 2.3° global).
- Cloud de A au plafond de ses poses (0.204 → 0.190 avec poses parfaites, 7 %).

**Le seul « miracle » crédible restant pour Bruce pur : B′ = USBL back-end à sigma RELÂCHÉ
(2.5-3.0)** — une ancre douce rabat le warp uniforme ET l'excursion t=14-15, sans la raideur
qui a cassé B (sigma 1.0). 1 run : `usbl/sigma: 2.5` dans slam_aracati.yaml (branche Bruce)
puis `SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true ./run_slam.sh`.
Secondaire : le prochain run exporte `nssm_constraints` → vérifier si les loops de t=14-15
sont rejetées par PCM (alors min_pcm 6→5 ciblé). Le reste (densité keyframes, icp.yaml) :
coût > gain attendu — « run A déjà satisfaisant ».

---

## 4. Pistes vers ATE < 1 m — configurations à tester, PAR BRANCHE, DANS L'ORDRE

Règles : **un changement par run** ; après chaque run → `python3 analysis/bilan_run.py results/run_...`
(1 image + 1 ligne de chiffres) ; reporter le résultat dans PROGRESS.md.
**Recettes d'implémentation détaillées : [CONFIGS.md](CONFIGS.md)** (une ancre par piste).
**Pièges à lire avant toute modif : [PIEGES.md](PIEGES.md).**
**Référence à battre : 141223 = ATE 1.53 m / NN 0.203 / cap 3.4°** (réf GT 011733 : 0.89 m / 0.199).

> **État (2026-07-02 soir)** : 1.1 ✅ FAIT — les « 122 loops » historiques étaient les
> candidats retenus ; les constraints PCM réelles sont passées de **6 (pré-fix) à 82
> (post-fix)**. Marge : +108 vrais candidats (0 faux) entre sc_dist 0.60 et 0.70
> → **1.2a = dist_threshold 0.70** ([CONFIGS.md](CONFIGS.md#12a-dist-threshold)).
> 2.1 ✅ FAIT — run A `194559` : **ATE 1.95 m** (DR pur 10.55 → ÷5.4 sans aucune ancre),
> cap 2.3° (meilleur que C !), erreur PLATE dans le temps → l'ancre USBL (B) doit
> encore réduire. Plafond cloud de A par poses parfaites : 0.204→0.190 (7 %)
> → le cloud est limité par les DÉTECTIONS, plus par les poses.

### Phase 1 — branche `Bruce_Sonar_USBL` (pipeline cmd_vel + USBL back-end + Sonar Context)
Tout existe déjà : ce ne sont que des paramètres.

| Ordre | Config | Quoi changer | Existant ? | Coût | Succès si |
|---|---|---|---|---|---|
| 1.1 | ✅ FAIT — diagnostic loops (cf. État ci-dessus) | — | ✅ | 0 run | fait : cause = PCM cassé pré-fix (6→82) ; marge = +108 vrais à 0.60-0.70 |
| 1.2a | dist_threshold 0.60→0.70 — [CONFIGS.md](CONFIGS.md#12a-dist-threshold) | `slam_aracati.yaml sonar_context/dist_threshold` | ✅ | 1 run | constraints 82→150+, ATE < 1.4 m |
| 1.3 | SSM réactivé — [CONFIGS.md](CONFIGS.md#13-ssm) | `slam_aracati.yaml ssm/enable: True` + `max_translation: 1.5` | ✅ | 1 run | cap < 3° (A le prouve : 2.3°), ATE ≤ 1.4 m |
| 1.4 | Meilleur combo 1.2a + 1.3 — [CONFIGS.md](CONFIGS.md#14-combo) | les deux réglages gagnants | ✅ | 1 run | ATE < 1.2 m |
| 1.5 | USBL sigma — [CONFIGS.md](CONFIGS.md#15-usbl-sigma) | offline d'abord (résidu fixes vs GT), puis `usbl/sigma` | ✅ | 0-1 run | ATE ↓ (l'ancre globale domine l'ATE Umeyama) |

### Phase 2 — branche `Bruce` (pipeline Bruce pur : cmd_vel + SSM + NSSM natif, sans SC)
Préparé clé en main : **suivre `ABLATION.md` sur la branche `Bruce`** (fix miroir porté,
SSM/NSSM/USBL par variables d'env, s'arrête tout seul à la fin du bag).

| Ordre | Config | Commande | Succès si |
|---|---|---|---|
| 2.1 | ✅ FAIT — **A — Bruce pur** (run `194559`) | `SSM=true NSSM=true USBL=false ./run_slam.sh` | **ATE 1.95 m**, cap 2.3°, NN 0.204 (seuil 65) — modules natifs ressuscités |
| 2.2 | ✅ FAIT — **B** (run `204329`) : **2.03 m**, cap 2.9°, NN 0.218 — PIRE que A partout : l'ancre USBL raide (sigma 1.0) casse la cohérence scan (murs doublés). Verdict : champion Bruce = **A (1.95)** ; C garde 0.42 m d'avance ATE → SC justifié. Option B' sigma relâché notée (ABLATION.md). | | |

### Phase 3 — à CRÉER (après les phases 1-2)

| Ordre | Config | Branche / pipeline | À créer | Coût | Succès si |
|---|---|---|---|---|---|
| 3.1 | DISO GT-free « wz inversé » — [CONFIGS.md](CONFIGS.md#31-diso-wz) | `Bruce_Sonar_USBL`, `ODOM_SOURCE=diso DISO_PRIOR=cmd_vel RATE=0.5` | param `invert_wz` dans `cmd_vel_odom.py` (1 signe) + `flip_bearing: False` pour ce mode | petit + 1 run | l'odom DISO brute ne diverge plus (< 5 m vs 22 m) → ouvre DISO+SC+USBL vers < 1 m |
| 3.2 | MCFAR (SIO-UV) — [CONFIGS.md](CONFIGS.md#32-mcfar) | `Bruce_Sonar_USBL`, feature_extraction | débruitage multi-échelle avant CFAR | moyen + 1 run | NN ↓ ET loops ↑ (le cloud est désormais limité par les détections) |
| 3.3 | SONIC (association loops) — [CONFIGS.md](CONFIGS.md#sonic-offline) + `Paper/Sonar/SONIC.md` | test OFFLINE d'abord (122 paires du run 141223) | pipeline de rejeu + inférence (code public rpl-cmu/sonic) | moyen, 0 run | transform SONIC > ICP sur les 40/122 candidats non convertis |
| — | ISOPoT (arXiv 2606.23006) | biblio seulement | rien (code non publié) | — | citation rapport. ⚠ leur ATE 3.2–4.6 m = par section, alignée 1re pose — PAS comparable à notre Umeyama full-seq |

---

## 5. Caractérisation d'un run : `bilan_run.py` (nouveau)

`python3 analysis/bilan_run.py results/run_X [results/run_avec_theta]` → **1 image** `bilan_run.png` :
trajectoire alignée + ATE ; pointcloud + NN ; **erreur de cap véridique dans le temps** (fit
s·θ+β, offset retiré — le transitoire initial ~29° = convergence du seed USBL course-over-ground).
Console : 1 ligne de chiffres. À appeler depuis `analyse.sh` (ajout d'une ligne). C'est le
« autre aspect » demandé (cap + cloud), sans multiplier les sorties.

---

## 6. HoloOcean (branche `holoocean`, ex-slam3-d) — après Aracati

Bag : `test.bag` (le dernier). Objectif : Bruce-SLAM dessus, RViz + CSV (gt, traj,
pointcloud, loops), en **2D** d'abord, **3D** ensuite. Cf. `SLAM_3D_MIGRATION.md` (mis à jour) :
- **2D** : pipeline actuel (features cartésiennes ou polaires selon le format sonar du bag).
- **« 2.5D » honnête** : z (pression) + roll/pitch (IMU) injectés dans `dr_pose3` → trajectoire
  et carte 3D SANS back-end Pose3 (architecture voulue par Bruce). Projection verticale du
  sonar OK si l'élévation est fixe.
- **Vraie 3D — proposition pour le collègue HoloOcean** (§ dédié dans SLAM_3D_MIGRATION.md) :
  faire osciller le tilt du sonar OU publier le PointCloud2 3D du simulateur, + IMU/pression
  dans le bag. C'est ce qui débloque une carte volumétrique réelle.

## 7. Mini-papier (à la fin d'Aracati — noté)

Quand tu seras satisfait du meilleur run : mini-papier (4-6 pages type workshop) sur la
méthode retenue. Squelette proposé : problème (SLAM FLS GT-free sans DVL/IMU) → système
(Bruce + USBL back-end + Sonar Context + fix de chiralité) → l'histoire du bug de miroir comme
étude de cas (les conventions de repère en SLAM sonar) → résultats (ATE Umeyama, NN, ablations
A/B/C du §3) → limites. Les figures existent déjà (bilan_run, avant/après miroir).

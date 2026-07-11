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

> ## 🏁 RÉSULTAT FINAL (07-03) — la comparaison champion vs champion est BOUCLÉE
>
> | Champion | Run | ATE | Cap méd | Cloud NN | Loops |
> |---|---|---|---|---|---|
> | **Bruce_New = 1.2a** (SC 0.70 + USBL σ1.4) | `003823` | **1.50 m** | 2.6° | 0.204 (I≥255) | 116 SC |
> | **Bruce pur = B′** (SSM+NSSM + USBL σ2.5) | `120352-1` | **1.88 m** | 2.6° | 0.205 (seuil 65) | 130 natives |
> | variante « champion cloud » = 1.3 (SSM+σ1.4) | `015742` | 2.14 m | 4.3° | **0.173** record | 103 |
> | réf GT-assistée (pas GT-free) | `011733` | 0.89 m | 1.7° | 0.199 | — |
>
> **Écart final : 0.38 m en faveur de la contribution (Sonar Context)** — les deux champions
> partagent les mêmes capteurs (cmd_vel + sonar + USBL) ; la différence méthodologique = la
> détection de loops par apparence. Configs FIGÉES dans les yaml des deux branches.
> Essais rejetés par les chiffres : B (σ1.0 raide) 2.03 ; 1.4 (SSM+σ2.5) 3.13 ;
> **loterie DISO wz inversé : CLOSE** (odom brute 39.2 m — la chiralité du prior était
> nécessaire mais pas suffisante, l'association DISO frame-à-frame reste le mur sur FLS épars).
> Leçon transversale (3× mesurée) : **chaque pipeline a son σ d'ancre optimal** — raide avec
> loops SC (1.4), douce avec SSM/NSSM natifs (2.5) ; la version principielle est le σ adaptatif
> par fix (papier INS/USBL/DVL FGO, présentation 6).
> Mini-papier ✅ RÉDIGÉ (§7) · papier branche Bruce ✅ (`BRUCE_SLAM.md`, branche Bruce) ·
> **phase ULTIME lancée** (§8, branche `Bruce_Ultime`) · HoloOcean en attente (bag 3D du collègue).

### Phase 1 — branche `Bruce_Sonar_USBL` (pipeline cmd_vel + USBL back-end + Sonar Context)
Tout existe déjà : ce ne sont que des paramètres.

| Ordre | Config | Quoi changer | Existant ? | Coût | Succès si |
|---|---|---|---|---|---|
| 1.1 | ✅ FAIT — diagnostic loops (cf. État ci-dessus) | — | ✅ | 0 run | fait : cause = PCM cassé pré-fix (6→82) ; marge = +108 vrais à 0.60-0.70 |
| 1.2a | ✅ FAIT — **CHAMPION New** (run `003823`) | dist_threshold 0.70 | ✅ | fait | **ATE 1.50**, cap 2.6°, 230 retenus/116 constraints, NN 0.204 |
| 1.3 | ✅ FAIT — champion CLOUD (run `015742`) | SSM on + σ1.4 | ✅ | fait | NN **0.173** (record GT-free) mais ATE 2.14 → variante carte, pas champion traj |
| 1.4 | ✅ FAIT — REJETÉ (run `140908-2`) | SSM + σ2.5 | ✅ | fait | ATE 3.13 : relâcher l'ancre avec SC aggrave (l'inverse de Bruce pur) |
| 1.5 | CLOS de fait (couvert par 1.4/B/B′ : σ 1.0/1.4/2.5 testés sur les 2 pipelines) | — | ✅ | fait | verdict : σ optimal DÉPEND du pipeline (SC→1.4, natif→2.5) |

### Phase 2 — branche `Bruce` (pipeline Bruce pur : cmd_vel + SSM + NSSM natif, sans SC)
Préparé clé en main : **suivre `ABLATION.md` sur la branche `Bruce`** (fix miroir porté,
SSM/NSSM/USBL par variables d'env, s'arrête tout seul à la fin du bag).

| Ordre | Config | Commande | Succès si |
|---|---|---|---|
| 2.1 | ✅ FAIT — **A — Bruce pur** (run `194559`) | `SSM=true NSSM=true USBL=false ./run_slam.sh` | **ATE 1.95 m**, cap 2.3°, NN 0.204 (seuil 65) — modules natifs ressuscités |
| 2.2 | ✅ FAIT — **B** (run `204329`) : **2.03 m**, cap 2.9°, NN 0.218 — PIRE que A partout : l'ancre USBL raide (sigma 1.0) casse la cohérence scan (murs doublés). Puis **B′ (run `120352-1`, σ2.5) : 1.88 m, cap 2.6°, 130 loops — CHAMPION Bruce pur** (l'ancre douce marche avec les modules natifs). | | |

### Phase 3 — à CRÉER (après les phases 1-2)

| Ordre | Config | Branche / pipeline | À créer | Coût | Succès si |
|---|---|---|---|---|---|
| 3.1 | ✅ FAIT — **CLOSE** (run `151239-3`, branche archivée) | loterie DISO wz inversé | ✅ | fait | odom DISO brute **39.2 m** (pire que 22) : chiralité nécessaire mais pas suffisante — DISO GT-free définitivement clos |
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
- **⚠ vérifié 07-07** : sur les 2 runs holoocean actuels (`141231`, `135343`), `nssm_constraints`
  = 0 sur toutes les keyframes → back-end sans contrainte sonar acceptée → colonnes `x,y`
  (« SLAM ») et `dr_x,dr_y` (DR IMU+DVL) de `trajectory.csv` **identiques bit-à-bit** (diff
  max ~3e-14). `holoocean_report.py` (labels/colonnes séparés, correct dans le code) trace donc
  malgré lui deux courbes superposées dans `error_over_time.png`/`trajectory_plot.png` — pas un
  bug de script, attendu tant qu'aucune correction sonar n'est branchée (avant prépa 3D). Ne pas
  interpréter un futur écart nul SLAM/DR comme une régression du rapport : normal en l'état.
  **Suite (07-07 ap-m)** : cause = `ssm/nssm enable: False` dans slam_holoocean.yaml (l'ICP
  n'était jamais tenté). Upstream jake3991 vérifié = True/True → **défauts passés à ON**
  (parité Bruce original) dans yaml + launch (nouvel arg `ssm`) + run_slam.sh
  (`SSM=false NSSM=false` pour retrouver l'ancien état 0.13 m). Cf. PROGRESS.md §R3 07-07.
- **⚠ VERDICT parité (07-07 ap-m, run 162710 : ATE 4.79 m vs DR 0.12 m) — investigation
  complète, 7 runs** : **SSM = coupable** (seul → 4.79 m ; l'ICP séquentiel remplace le facteur
  DVL excellent par un recalage sonar biaisé, features simulateur → dérive dès KF1, cap −0.44 rad).
  NSSM sep8 non répétable (0.96/2.52 m, fausses loops court-terme = classe PIEGES §11) ;
  **fix figé : SSM off + NSSM on avec min_st_sep 25 → 0.13 m ×2 répétable** (0 loop sur ce bag
  court : neutre, machinerie prête pour un bag long). Rejeté : bug bridge/bearing (NSSM seul
  fonctionne, PCM gate). La parité Bruce reste testable : `SSM=true ./run_slam.sh holoocean`.

## 7. Mini-papier — ✅ RÉDIGÉ (07-04)

**`Paper/MiniPapier/MINI_PAPIER.md`** (+ `figs/`, généré par `analysis/paper_eval.py`).
Chapitres par branche (II=Bruce, III=Bruce_Sonar_USBL, IV=pistes closes, V=holoocean),
formules des contributions (chiralité R(θ)M=MR(−θ), facteur USBL Cauchy, descripteur SC
densité), protocole d'éval expliqué (§6.2 : Umeyama SE(2) sans échelle = primaire ; pourquoi
pas de rescale ; sections S1/S2/S3 façon DISO ; ATE première-pose instable — sensibilité
mesurée ; RE trans %/rot °/m comparables aux tables DISO/ISOPoT) + NOUVELLE métrique carte
(nuage vs re-rendu poses GT : 1.2a méd 0.11 m). Relecture/retouches par Nathan ensuite.

## 8. PHASE ULTIME — CLOSE ET FUSIONNÉE (07-05)

> **Branche `Bruce_Ultime` fusionnée dans `Bruce_Sonar_USBL` puis archivée**
> (tag `archive/Bruce_Ultime`) : la répétabilité (TESTS.md §2.4) a montré σ1.4 ≈ σ1.8
> (BSU méd 1.50 ≤ Ultime 1.54) et tout le reste était déjà partagé — aucun avantage
> propre. BSU récupère : ULTIME.md (historique), le code des expériences rejetées
> U4/U6 (off), les boutons USBL_SIGMA/LOOP_UNION/USBL_ADAPTIVE. **Le dépôt revient à
> 4 branches : main, Bruce, Bruce_Sonar_USBL (= LA contribution), holoocean.**

### (historique 07-04) RU1-RU4 : 🏆 CHAMPION RU1 (σ1.8) 1.47 m

Directive : partir des 2 bases (traj `Bruce_Sonar_USBL` + carte `Bruce`) pour une branche
« ultime ». Plan détaillé, verdicts et journal : **`ULTIME.md`** (branche `Bruce_Ultime`).

- **🏆 RU1 `125434-RU1` (SC 0.70 + USBL σ1.8) : ATE 1.47 m**, 125 constraints, et avec le
  rendu compas U1 : NN 0.172 (meilleur cloud du stage), carte vs vraie **0.075/0.413**
  (≈ borne GT 0.40). Config FIGÉE dans le yaml Ultime. Balayage σ : 1.4→1.50, 1.8→1.47,
  2.0→1.60 (RU2).
- **U1 ✅** rendu compas (0 run) ; **U2 ✅** intégré à analyse.sh (toutes branches).
- **U4 ❌ RU3 2.91 m** : l'union SC+natif réinjecte les faux positifs non gatés (583
  candidats natifs) + CPU saturé → PIEGES §12. **U5 ❌ RU4 17.17 m** : fenêtres NSSM en
  KEYFRAMES (min_st_sep 8 = 8 m après densification) → fausses loops court-terme dès
  t=3 min → PIEGES §11 ; rollback fait, B″-bis optionnelle (fenêtres ×3) dans ABLATION.md.
- **U6 ❌ RU5 `161907-RU5` : 1.62 m** — le σ adaptatif (proxy MAD pourtant corrélé 0.64
  au vrai bruit) perd contre le σ fixe 1.8 : sur-raidissement local (min 0.9) + le graphe
  moyenne déjà les fixes. Code laissé (défaut off). **PHASE ULTIME CLOSE sur RU1** —
  `./run_slam.sh` nu (branche Bruce_Ultime) = champion, carte fine = pointcloud_compass.
  (Épaisseur du nuage RViz en live = drift résiduel inter-passages ~8-40 cm superposé :
  attendu ; la carte livrable est le produit offline.)
- **U7 verdict** : ISOPoT infaisable (code non publié, tracker vidéo lourd) ; SONIC = test
  offline en réserve seulement (domain gap probable, SONIC déjà faible sur Aracati dans
  la table ISOPoT). MCFAR en réserve.
- Papier doctorante : **`BRUCE_SLAM.md`** (branche Bruce) — original vs modifications,
  ablation A/B/B′ (+ verdict B″ §6.1 à intégrer après RU5).

## 9. 2026-07-11 — « on ne détecte pas grand-chose » (carte 2D traj4 pauvre) : DIAGNOSTIQUÉ

**Symptôme** (Nathan) : sur la carte 2D du run `160434` (traj4), les quais sont à peine
visibles ; la 3D, qui s'appuie dessus, paraît pire. 16 105 pts / 758 KF ≈ **21 pts/KF**.

**Ce que la carte montre VRAIMENT (élucidé, rien de faux)** : tout ce qui existe est détecté —
quai OUEST = ligne fine (x_SLAM≈−63) ; quai EST = tirets (pilotis pleine hauteur + bavure
radiale) ; **le « blob bleu » (−57..−41, y≈15) = mur Γ** (monde : segment y=−645, x 464→485
+ retour x=485 vers le sud — géométrie de `pierharbor-geometrie-monde` au mètre près).
Le reste du bassin est RÉELLEMENT vide (fond muet à l'ImagingSonar en incidence rasante).

**Mécanisme (chaîne causale complète, mesures scratchpad `scan_sonar_intensity/thresholds.py`)** :
1. Nos bags v3/v4 sortent des images FAIBLES : max/ping méd **0.265** brut (p90 0.398,
   plafond 0.485 ≈ le « ≤0.47 » noté dans le bridge 07-08). Pilotis fins = peu de hits
   octree/bin. Pas un effet distance (méd stable 0.24–0.28 de 0 à 80 m).
2. Le TÉMOIN test.bag (collègue, champion 0.13 m) : max méd **0.776, sature à 1.0**,
   **15 679 px ≥ seuil 50**/ping (méd) contre **100** chez nous (×157) — images speckle
   pleines, autre monde (« Bruce_slam_nathan »/Charuco), autre dynamique.
3. `feature_holoocean.yaml filter.threshold: 50` a été calibré (07-03) sur CETTE dynamique-là
   → sur nos images il coupe la bande 26–50 où vivent la plupart des échos de pilotis :
   frame quai EST à 9,5 m : 37 px ≥50 vs **300 px ≥30** (×8) ; bateau ×6 ; méd bag ×3.2.
4. Bruit mesuré (frames quasi vides) : p99 image = 0.031–0.034 ≈ **8–9 mono8** → un seuil 30
   garde ×3 de marge ; le CFAR (contraste local) ne laissera pas passer le speckle de toute façon.

**Hypothèses rejetées** : « échos trop faibles car quais trop loin » (méd plate vs distance) ;
« mes modifs traj4 » (même plafond ~0.47 que les bags traj1-3, config sonar = briques v3
inchangées) ; « c'est le rendu/aval » (le CSV ne contient vraiment que ~21 pts/KF).

**Options (config CHAMPION FIGÉE — accord Nathan requis avant tout édit, R3)** :
- **A (recommandée)** : `filter.threshold 50→30` (1 variable) + re-run traj4 ×2 → attendu
  carte ×3 plus dense (quais nets), vérifier ATE (témoin 0.04 m) et NSSM.
- **B (zéro impact SLAM)** : carte 2D densifiée OFFLINE en projetant `/sonar_points`
  (seuil génération 0.10≈26, repère véhicule) le long des poses SLAM — livrable rendu only.
- **C (3D)** : la maigreur 3D est un problème de COUVERTURE (fan vertical étroit, « phare »),
  pas de seuil (`/sonar_vert_points` déjà à 0.10) → traj5 avec plus de sweeps, ou fusion
  patchs polaires (StereoFLS) déjà listée.
- NON recommandé : régénérer les bags avec la dynamique bruitée du collègue (du bruit pour
  faire plaisir aux seuils = non-sens ; mieux vaut calibrer les seuils sur NOS images propres).

### §9-bis — CORRECTION en cours d'investigation (même jour, 2 découvertes de plus)

**① Le miroir latéral de `/sonar_points` (bug générateur RÉEL, confirmé 2×)** : en émulant la
chaîne CFAR réelle sur le bag, les structures atterrissaient en miroir du cap (crochet Γ à
y≈53 = réflexion de y=15 par la jambe nord). Vérif analytique frame t=301 s (véhicule
(500,−626) cap OUEST) : l'arc fort à +37°/31 m = mur Γ qui est à BÂBORD → dans l'image
HoloOcean les colonnes hautes = GAUCHE, or `sonar_to_points3d_msg` (gen_bag_3d.py) fait
`y = −r·sin(a)` en croyant colonnes hautes = droite. **Fix = flip latéral, 1 ligne** —
c'était déjà le « fix racine miroir y » noté au PROGRESS 09-09 (§2.3quater), jamais fait ;
E1–E7 ne testent QUE le fan vertical (E3 = élévation), aucun check latéral horizontal.
- **Le fan VERTICAL est net-CORRECT tel quel** (fond à −19.4 dans carte_3d.npy, E3/E4 PASS,
  NN 0.083 vs carte SLAM mondialement juste) → NE PAS « corriger » l'appel vert sans re-passer
  E3/E4 : le miroir ne touche que le plan LATÉRAL du sonar horizontal.
- Impact : comblage horizontal de carte_3d (9 632 pts miroirs → pollue la 3D), fusion_plus
  (faible : gate ±6° quasi symétrique, y-erreur ≤ 0.1·r), verdict traj3 « blobs = artefacts »
  (partiellement faux : certains blobs = structures miroir). Le SLAM n'utilise PAS
  /sonar_points → ATE 0.04 intact. Réécriture des topics points = OFFLINE (images dans le
  bag, pas besoin d'UE).

**② La carte du run n'accumule que les KEYFRAMES (758/6486 pings)** : la chaîne réelle émulée
à seuil 50 sur 1 ping/3 donne déjà 50 017 pts (×3 vs 16 105) au même seuil — carte dense
possible SANS toucher à la config, en rejouant le détecteur sur tous les pings le long des
poses SLAM (GT-free).

**Émulation chaîne exacte (CFAR SOCA 20/4/0.1/10 + seuil + downsample 0.5 + outlier 1.0/5,
scripts scratchpad emul_pipeline.py, images `fable_emul_50_vs_30.png` du run 160434)** :
seuil 50 → 50 017 pts ; seuil 30 → 165 646 pts (×3.3) : quais = bandes évidentes, rangées de
pilotis lisibles ; contrepartie = bavure tangentielle accrue aux sweeps (le CFAR est
range-only, la bavure d'azimut ≥30 passe).

**Options RÉVISÉES pour Nathan** :
- **B′ (recommandée, zéro config)** : script analysis/ « carte 2D dense » = détecteur réel
  rejoué sur tous les pings × poses SLAM, seuil au choix (50 sobre / 30 dense) → règle
  « quais à peine visibles » sans toucher au pipeline figé.
- **A (pipeline)** : filter.threshold 50→30 + re-run ×2 (vérif ATE 0.04 et bavure) — utile
  si on veut aussi nourrir NSSM/loops, sinon B′ suffit.
- **D (bug racine)** : fix flip latéral dans sonar_to_points3d_msg pour le sonar HORIZONTAL
  seulement + réécriture offline des /sonar_points du bag + re-run carte_3d/fusion
  (le comblage 3D et la fusion en profitent) ; re-passer E1–E7 + ajouter un check E8
  « anti-miroir latéral horizontal » au guide.

### §9-ter — EXÉCUTION B′+D (décision Nathan, 07-11 soir) : FAIT, tout vérifié

**D — fix miroir (racine)** :
- `gen_bag_3d.py` : `y = +r·sin(a)` + `R_MOUNT_PROF` flippé (+90°→−90°) en même temps →
  projection nette du fan VERTICAL prouvée IDENTIQUE (diff numérique 0.0). Ne jamais
  flipper l'un sans l'autre.
- `check_traj4.py` : **E8 anti-miroir latéral** (reprojection GT vs structures connues,
  PASS si score(tel quel) > 2×score(miroir) et >15 %). Validé en DÉTECTEUR : bag miroir
  10.9/33.9 % → FAIL ✓ ; bag corrigé 45.2/13.6 % → PASS.
- Bags traj4 (complet 10.3 Go + court) RÉÉCRITS offline (/sonar_points recalculés depuis
  /sonar, reste verbatim, tailles à l'octet) — **E1–E8 TOUT PASS ×2 bags**. Anciens
  conservés `*_avant_fix_miroir.bag` (suppression = décision Nathan). ⚠ traj1-3 (bag/)
  PAS réécrits : archives, toujours miroir.
- Aval re-mesuré (run 160434, commandes d'origine) : carte structurelle vert IDENTIQUE
  (25 586 pts, NN 0.083/p90 0.693 — preuve de non-régression) ; comblage 9 632 pts bien
  placés ; **fusion M1 0.296→0.205 m méd PASS (−31 %)** ; M2 méd 0.066→0.052 m,
  ⚠ p90 0.286→0.709 NON investigué (doute ouvert ; M1 est le critère géométrique).

**B′ — carte 2D dense offline (`analysis/carte_2d_dense.py`, nouveau)** : détecteur RÉEL
(CFAR SOCA + seuil + downsample + outlier) rejoué sur TOUS les pings × poses SLAM (GT-free),
+ filtre anti-bavure (crête locale en azimut : la bavure passe le CFAR range-only et
dessine des arcs centrés sonar — la remarque de Nathan sur les « clouds circulaires »).
Run 160434 : seuil 30 → **33 740 cellules 0.2 m** (24 725 en persistance ≥2 pings) ·
seuil 50 → 13 126 · carte SLAM keyframes : 16 105 pts. Les deux quais + Γ + bateau lisibles.

**Leçon trajectoire (remarque Nathan, VALIDÉE par le témoin)** : le run traj3 du collègue
(161938, même monde/seuils/chaîne) sort 74 947 pts NN 0.098 avec les DEUX quais en échelles
nettes — son errance passe à ~6 m du quai OUEST (x≈468) vs notre carré à 28 m (x=490), cap
et z variant en continu. **La trajectoire domine la qualité de la carte 2D** ; le seuil est
le 2ᵉ facteur. → traj5 : reprendre une errance naturelle (rapprochements des 2 quais, z
continu) + garder les acquis traj4 (sweeps verticaux, approche bateau, phase A calibration).

## 10. 2026-07-11 (nuit) — traj5 « errance naturelle » : TOUTE la chaîne validée, la trajectoire gagne

**Demande Nathan** : passer près des structures comme le collègue + hauts/bas aléatoires naturels,
pas de carré robotique — puis tout lancer (holoocean → ros → analyse). FAIT de bout en bout :

**Pipeline exécuté** : gen_bag_3d_v5.py (errance PCHIP du collègue VERBATIM sur circuit médian
évitant Γ, phase A calibration conservée) → bag court E1–E8 TOUT PASS du 1er coup → bag complet
6957 pings/1391 s/11.1 Go E1–E8 TOUT PASS (E8 47.6 % vs 10.5 % miroir) → run SLAM 222233 →
analyse.sh + fusion_plus + carte_2d_dense.

**Résultats traj5 (run 222233) vs traj4 (160434, post-fix miroir)** :
| métrique | traj4 (carré) | traj5 (errance) |
|---|---|---|
| ATE Umeyama / cap RMS | 0.04 m / 0.1° | **0.05 m / 0.1°** (équivalent) |
| carte 3D structurelle | 25 586 pts, NN 0.083, p90 0.693 | **36 704 pts (+43 %), NN 0.064, p90 0.261** |
| fusion M1 méd/p90 | 0.205 / 0.402 | **0.122 / 0.268** |
| fusion M2 méd/p90 | 0.052 / 0.709 ⚠ | **0.034 / 0.112** |
| carte 2D dense s30 | 33 740 cell. (forte bavure) | 25 455 cell. MIEUX placées, 2 quais en échelles |

- Le doute M2 p90 de traj4 (0.709) s'est RÉSORBÉ avec la géométrie errance (0.112) → hypothèse
  (non prouvée) : le p90 venait des fusions à longue portée du carré, pas d'un bug de code.
- La bavure d'azimut chute (p90 3D 0.693→0.261) : passages proches = faisceaux mieux conditionnés.
- CQFD de la leçon §9-ter : **à chaîne identique, la trajectoire fait la carte**. Le SLAM, lui,
  est insensible (0.04↔0.05 m).

**Livrables** : bag `BAG_files/holoocean_3d_traj5.bag` (+ court) · run `222233` complet
(bilan, carte_3d, fusion_plus, carte_2d_dense_s30) · générateur `gen_bag_3d_v5.py` +
`gen_traj5.sh` (commit de66a1e).

### §10-bis — résidus 3D traj5 ÉLUCIDÉS + filtre + faisabilité traj6 (directives Nathan)

**Deux artefacts distincts, tous deux mesurés puis dégagés (carte_3d.py, opt-out `--brut`)** :
1. **Surface** (31 % de la carte, 11 201 pts) : miroir acoustique vu de près quand le robot
   monte à z −2.3 (traj4 : 8 % seulement, il restait à −4/−9.5). Coupe z > −1.8 (GT-free).
2. **« Coin » sous la traj à droite du quai OUEST = FANTÔME hors-plan** — chaîne complète :
   hypothèse « bruit faible gardé par per-beam-max » RÉFUTÉE (I méd 0.229 > quais 0.174) ;
   hypothèse « vraie structure jamais vue » RÉFUTÉE par capteur indépendant (profiler
   transverse traj3, 197 788 pts : 0 point dans z [−18,−4] à cet endroit, seulement le fond) ;
   mécanisme CONFIRMÉ analytiquement : depuis le corridor (485,−642) cap OUEST, la face du
   mur Γ (portée 13.1 m, élév −38°) est à ~17-20° HORS du plan du fan vertical et entre
   quand même → rabattue sur le plan → nappe fantôme (z≈−13, y≈−640, 111 pings).
   **L'ouverture hors-plan réelle du SonarVert est >> ±3° annoncés** (PIEGES #15).
**Filtre carte_3d** : surface (z>−1.8) + cohérence 2D (point vertical gardé si ≤2 m x-y d'un
point de pointcloud.csv, translation SLAM→GT gérée) + exemption FOND (z < p03+1.5, invisible
au fan horizontal). Run 222233 : 90 709 → 49 129 pts (−24 661 surface, −16 919 fantômes) →
carte 18 126 pts PROPRE (quais/Γ/bateau/fond intacts, vérif visuelle).
⚠ MÉTRIQUE HONNÊTE : NN carte passe de 0.064/0.261 à **0.107/0.733** — l'ancien chiffre était
FLATTÉ par les artefacts (grandes nappes cohérentes qui se matchent parfaitement entre les
versions GT-free et GT). Comparer les NN à CONTENU ÉGAL uniquement.

**traj6 « tout capter » (accord Nathan si couverture complète) — FAISABLE, testé** :
`ProfilingSonar` accepte **Azimuth=360** (image 512×720 rendue, test 30 ticks à P_A cap nord :
quai+fond visibles, 3/12 secteurs occupés = normal en port ouvert). Config cible : 3 capteurs =
/sonar (horizontal, devant G-D) + /sonar_vert (« + », devant H-B) + profiler TRANSVERSE 360°
rotation [90,0,90], RangeMax 20 m, 720 bins 0.5° (flancs/haut/bas au passage). Reste à coder :
gen v6 (v5 + capteur + /profiler_points via sonar_to_points3d_msg r_mount transverse — vérifier
le signe, E-check E9 dédié), carte_3d : FUSIONNER les sources structurelles vert+transverse
(aujourd'hui priorité exclusive au vert) + étendre le filtre anti-résidus au transverse.

## 11. 2026-07-12 — traj6 « tout capter » CODÉ : mount transverse MESURÉ (pas deviné), E9, fusion carte_3d

**Livrables (tout vérifié ce tour, avant tout run)** : `gen_bag_3d_v6.py` (= traj5 importée
verbatim + ProfilingSonar TRANSVERSE 360° [90,0,90], RangeMax 20 m, 512×720 @ 2 Hz →
/profiler_points repère véhicule + /profiler image 0.4 Hz debug) · `gen_traj6.sh` (clone
autonome de gen_traj5.sh, checks E1–E9) · `check_traj4.py` +E9 · `analysis/carte_3d.py`
fusion vert+transverse.

**Mount transverse : le candidat analytique était FAUX (PIEGES #16)** — probe statique
(525,−662,−5) cap sud, quai EST à 6.5 m bâbord, fond ~−18.5/−19.7 :
- Candidat « analogie du vertical » Rz(90)@Rx(−90) : mur du bon côté mais FOND AU-DESSUS
  (0 point sous −8 m ; flipZ gagnant avec 76.9 % dans la bande fond).
- **Mesuré et confirmé ×2 : R_MOUNT_TRANS = Rz(90)@Rx(+90)** (x_capteur→+y bâbord,
  y_capteur→+z haut) : mur x_med 532.1 (attendu 531.5, 55-75 % à <1.5 m), fond z_med −19.7
  (80.9 % dans [−21,−17]) ; flipY et flipZ échouent chacun sur leur discriminant.
**E9 (check_traj4.py)** : verrouille les 2 signes sur bag — (a) latéral : échos de la bande
z [−17,−2.5] collent aux structures connues vs miroir y (ratio>2, >10 %) ; (b) vertical :
pings z_rob>−6.5 (géométrie asymétrique — à z≈−9.7 fond/surface symétriques, test dégénéré),
échos >8 m sous le robot dans [−21,−17] vs flip z (ratio>2, >30 %). SKIP propre si topic
absent. Fenêtre z_rob>−6.5 vérifiée présente dès le bag --test 150 (24.3 % du temps, calcul
offline sur le profil PCHIP seed 42).
**carte_3d.py** : fusion des sources structurelles (vertical + transverse, fin de la
priorité exclusive) ; flip y du transverse RESTREINT aux bags v3 (frame_id=map, traj1-3
jamais réécrits) ; anti-résidus étendu à tout set structurel ; libellé FUSION.

**Régressions (rien de cassé par les édits)** : check_traj4 sur traj5_test.bag → E1–E8
PASS (valeurs identiques) + E9 SKIP · carte_3d sur run 222233 → IDENTIQUE au validé
(−24 661 surface, −16 919 fantômes, 18 126 pts, NN 0.107/0.733, comblage 10 144).

**Hypothèses rejetées** : mount = analogie Rz(90)@Rx(−90) (réfuté probe run 1) ·
« --test 150 trop court pour E9b » (réfuté : z max −2.33 atteint dès la 1re minute
d'errance). **Reste (runs Nathan)** : gen test → gen complet → run SLAM → analyses.

### §11-bis — bag test traj6 E1–E9 TOUT PASS + préparation handoff Opus (2026-07-12 soir)

**E9 mesuré sur bag réel** (--test 150, gen par Nathan) : latéral 55.8 % <1.5 m des
structures vs **0.0 %** en miroir (n=4553) · fond 95.4 % dans [−21,−17] vs **0.0 %** en
flip z (n=2251) → le mount mesuré au probe est confirmé sur bag, marges nettes. E1–E8
inchangés vs traj5 (mêmes valeurs : E8 36.2/12.5 %). Bag complet en génération (nuit du 12).
**Handoff Opus (Nathan n'a plus Fable demain)** : `TRAJ6_ANALYSE.md` (mode d'emploi + textes
d'interprétation + interdits + STOP) et `analysis/compare_traj6.py` (ΔATE, couverture
réf→run, apport transverse ; sans lecture de bag). Auto-test du script : 222233 vs lui-même
→ ATE 0.048 m / cap RMS 0.07° / Δ 0.000 / NN 0.00 — cohérent avec les 0.05 m / 0.1° publiés
(§10) ; l'alerte « apport 0 % » en self-compare est le comportement attendu.
**Nettoyage bags (décision Nathan)** : −34 Go vérifiés (df 372→339 ; btrfs libère en
asynchrone, ~20 s). Supprimés : traj1-3 (miroir), traj4 ×2, _avant_fix_miroir ×2,
traj3_test. Gardés : traj5 ×2, traj6 ×2, test.bag, caves, ARACATI.

### §11-ter — traj6 VALIDÉ bout en bout (run 005329, 2026-07-12) : les 3 verdicts au vert

**Bag complet E1–E9 TOUT PASS** (E9 : latéral 34.4 % vs 0.0 % miroir, n=46 075 · fond
83.5 % dans [−21,−17] vs 0.3 % flip z, n=42 311) — mount transverse verrouillé ×2
(bag test §11-bis + bag complet). **Run `run_holoocean_2026-07-12_005329`**, verdicts
compare_traj6.py :
- **[1] ΔATE = 0.000 m** (0.048 vs 0.048 traj5, cap RMS 0.05° vs 0.07°, 861 keyframes) —
  le SLAM est STRICTEMENT insensible au 3ᵉ capteur (attendu : mêmes topics d'entrée).
- **[2] couverture NN réf→run 0.05 m méd / 0.16 m p90** — la fusion contient tout traj5.
- **[3] apport transverse 59.4 %** des points à >1 m de la carte traj5 ; carte
  18 126 → **63 265 pts (×3.5)**, fond 24.8 → 51.0 % (fond continu hors nadir + flancs).
Verdict Nathan : « c'est exactement ça que je veux ». Le « + » devient un « ⊹ » complet :
horizontal (SLAM) + vertical avant (structures devant) + transverse 360° (tout au passage).
**Piste notée (Nathan, plus tard / traj7 potentielle)** : se rapprocher BEAUCOUP plus des
quais que les 4-6.5 m de traj6 (gain attendu : détails pilotis/échelles, faisceaux encore
mieux conditionnés — cf. leçon §9-ter « la trajectoire fait la carte »).

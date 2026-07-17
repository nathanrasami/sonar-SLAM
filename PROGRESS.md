# PROGRESS — état au 2026-07-17 (00h30) — REFONTE exécutée, BSU 2.02 validé, gates FAIL sur Bruce_U

## 🏗 REFONTE (16-07, branche `refonte`, session Ultracode) — commits 40872d0→45aa602
- **ÉTAPE 0 tranchée par mesure** : seed = (0,0), CAP FIXE 0 en dur (θ0 vrai −1.1/+2.5° ;
  route-fond naïve = 45-78° faux, le ROV tourne de −88° sur 30 m — etape0_seed_cap.py ×2).
- **⛔2 PROUVÉ** (gate ①) : traces dr des 4 runs finaux BIT-IDENTIQUES (0.0000°/0.00000 m,
  14 430+ pts) — l'odométrie ne touche plus jamais l'USBL (cmd_vel_odom purgé 259→75 l.).
- **Amendement 1** : 2 yamls champions FIGÉS (PIEGES #26) — NSSM natif sur yaml BSU =
  31 fausses loops/+190°/80.7 m. sc=false → slam_aracati_native.yaml ; sc=true → BSU.
- **Amendement 2** : Bruce_Sonar = FINDING hors verdict (SC détecte 154 vraies revisites,
  0 acceptée : correction méd 9.5 m > garde-fou 8 m sans ancre ; avec USBL 1.2 m → 65).
- **Analyse unifiée** : analyse.sh → paper_figs_origine (translation PURE) partout,
  labels programmatiques ; zéro fit 15 % (vérifié code + git + cross-check ×4 runs).

## 📊 RÉSULTATS (translation pure, PV = results/gates_2026-07-17_0020.txt)
| méthode | run | ATE origine | S1/S2/S3 | Umeyama | carte méd/p90 |
|---|---|---|---|---|---|
| **BSU** 🏆 | 213715 | **2.02** | 2.36/**1.89**/**3.69** | **1.38** (record stage) | **0.07/0.51** |
| Bruce_Sonar | 205255 | 19.24 (=odom, finding) | 6.06/12.50/8.64 | 10.64 | 0.25/1.73 |
| Bruce natif | 230354 | 4.99 ⚠ | 2.62/2.84/4.46 | 3.21 (arch. 1.88) | 0.13/0.95 |
| Bruce_U natif | 234801 | 9.38 ⚠ | 3.16/3.29/11.00 | 7.01 (arch. 1.74) | 0.29/1.60 |
- BSU devant ISOPoT (3.2/3.5/4.6) sur LES 3 sections, seed 100 % GT-free.
- **Gates : FAIL** (Bruce_U 9.38 > Bruce 4.99) → STOP mission sur chap 1.

## 📄 CHAP 1-2 CLOS (17-07 ~2h, option A actée) — commits holoocean 9d9c32d + 8d58950 + 801cf8e
- Chap 2 : tab 3 runs (2 seeds) + run FINAL 2.02 · tab:compare = run final (2.36/1.89/3.69,
  devant publiés ×3) · NOUVEAU §ablation SC-sans-ancre · protocole seed cap 0 mesuré ·
  preuve dr bit-identiques. Chap 1 inchangé (runs archivés, déjà en translation pure).
- Relecture Nathan tranchée par mesure (801cf8e) : écart S3 odométrie 8.44 vs 6.5 publié =
  SENSIBILITÉ AUX FRONTIÈRES de sections (enveloppe 6.1-9.4 m pour ±3 min de frontière ;
  effets de cap RÉFUTÉS : re-ancrage entrée section −1°, compas continu → S3 9.0-9.2).
  Repères archivés vérifiés : seed USBL = rotation RIGIDE +30.88/31.19° (résidu 0-2 mm),
  SLAM B_USBL partiellement redressé (24.7°/11.8° résiduel, variable) vs BSU 1.40° (SC).
- xelatex ×2 PASS, 21 p, 0 undef. PDF frais copié dans Paper/main.pdf (repo courant).
- Docs synchronisés sur holoocean (PIEGES #26, REFONTE_MISSION, PROGRESS).
- Bruce_USBL archivé (8-11 m origine) : insatisfaisant pour Nathan mais ASSUMÉ (convention
  pure sans re-fit ; l'artefact de seed est expliqué au papier ; options B/C sinon).

## ✅ CHAP 3 HOLOOCEAN ÉCRIT (17-07, périmètre simplifié par Nathan)
- Décisions Nathan : partie BRUIT ABANDONNÉE (papier trop long), méthode chap 2 SEULE
  (pas de comparaison Bruce), présentation détaillée du système 2.5D ; papier désormais
  SYNCHRONISÉ sur les 6 branches (554a9a7 holoocean + syncs) + règles ignore build LaTeX.
- Chiffres papier (run BS_9_1 = traj9 propre 15-07) : ATE Umeyama 1.14 m / 443 m (0.26 %),
  cap méd 0.7° RMS 1.1°, carte 3D 155 284 pts (voxel 0.2 m), NN méd 0.218 m / p90 1.69 m.
- MESURÉ et dit dans le papier : 0 loop acceptée sur ce run, estimé ≡ DR (écart < 0.1 mm,
  821/837 KF ≠ = bruit numérique iSAM2) → le chap 3 valide la CHAÎNE 3D, pas les loops.
- Figures : Holo_carte3d_BS9.png ASPECT RÉEL (set_box_aspect, fini le cube) + Holo_traj_BS9
  (anglais, convention Umeyama bilan_run répliquée 1.14 ✓) ; 3D interactif dans le PDF
  REJETÉ (media9 = Acrobat seul, incompatible xelatex) ; compil 25 p PASS (chap 3 = p 20-23).
- RESTE : dessin UV capteurs par Nathan (placeholder fig. 15) ; relecture globale Nathan
  (viser ~10 p, actuellement 25) — repasses de style possibles avec Sonnet (compréhension
  consignée ici + mémoire).

## 🚨 NON TRANCHÉ (R2) — Bruce/Bruce_U refonte SOUS les champions archivés
- Mesuré : 88/57 loops natives, torsions 25-30°, USBL AGGRAVE (7.01 vs 3.21 Umeyama).
- H2 ouverte : CODE divergent (752 lignes de diff slam.py/slam_ros.py vs branche Bruce) ;
  H variance (1 seul run, R3 exige ×2) ; archéologie launch Bruce incomplète (front-end
  usbl=true gain 0.4 par DÉFAUT chez eux, env TESTS.md le coupait).
- Options (Nathan tranche) : A) chap 1 = runs ARCHIVÉS branche Bruce (223959/233119),
  refonte porte BSU+BS+preuve odométrie — zéro run en plus · B) ×2 re-runs (1,5 h) ·
  C) archéologie code (long). Papier PAS touché (⛔ mission : EN DERNIER).

---


## 🧠 MÉMOIRE REFONDUE (07-06 soir) — claude-mem SUPPRIMÉ, natif en place
- claude-mem v13 installé puis **supprimé le soir même** : ses hooks bloquaient les réponses
  (worker port 37700 muet → CLI en attente infinie). Nettoyage TOTAL fait (machine + 5 branches).
  ⚠ Ne JAMAIS réinstaller d'outil mémoire tiers à hooks (leçon en auto-memory).
- Architecture finale : **CLAUDE.md v2** (41 l, IDENTIQUE 5 branches — resync si modif) +
  **`.claude/rules/branche.md`** par branche (~10 l, spécificités locales ; main : aucun) +
  **auto-memory native** (commune aux 5 branches et discussions, index 21/200 lignes).
- **Prochaine étape (Nathan, nouvelle discussion)** : ajouter une règle dédiée à l'ère
  Claude Opus (quand Fable 5 ne sera plus accessible) — candidat naturel : CLAUDE.md §dédié
  ou `.claude/rules/`, à synchroniser sur les 5 branches.

## 📊 PRÉSENTATION (07-06) — Paper/COMPARE.md (5 branches)
- Tableaux comparatifs pour la slide : ATE par section S1/S2/S3 en DEUX conventions
  (Umeyama = interne ; 1ʳᵉ-pose = comparable ISOPoT/DISO), RE trans/rot, méthodes non
  comparables listées. Runs recalculés via paper_eval.py : `233119_Bruce_USBL_1` (1.74) et
  `201541_BSU_1` (1.45).
- Nuance « magnéto » ISOPoT : README Aracati = *vehicle compass* (embarqué ROV, pas le bateau) ;
  une seule source de cap dans le bag → leur Odom+Mag ≈ notre /cmd_vel (rot 0.00°/m identique).
  Certitude 100 % nécessiterait d'ouvrir le code DISO (topic exact non vérifié).

## 📈 FIX doublon odométrie « jaune/violet » (07-06, 5 branches)
- analyze_drift/analyze_origine ne tracent la ré-intégration offline (« Odom pure », violette,
  θ0=0, divergence cap ~1.4°/min) QUE si odometry.csv absent (vieux runs). La JAUNE
  (odometry.csv = vraie entrée du SLAM) est la seule canonique.

## 🕳 CAVES (07-06) — chaîne VALIDÉE, runs de suite listés
- Bridge MSIS v2 OK (CFAR interne, PIEGES §13 FOV<180°) ; run Nathan : 227 KF, 494 m,
  fermeture 8.47 m (1.7 %, SLAM=odom sans loops) ; grotte 3D SeaKing (caves_3d.py, 69 854 pts,
  --with-map) ; ruban Micron = 2.5D mesuré (std z intra-scan 0). Doc : CAVES.md.
- Reste caves : ① run loops `NSSM=true ./run_slam.sh caves` (refermer les 8.47 m) ;
  ② `./run_slam.sh caves Bruce_Sonar_USBL` (recalibrer τ SC via loops_detected.csv) ;
  ③ optionnel : compensation balayage 8.6 s (ULCDfMS), aerial view (image à fournir).

## 📌 Reste global
- Relecture par Nathan : MINI_PAPIER.md, BRUCE_SLAM.md, COMPARE.md.
- HoloOcean 3D : bag du collègue (stratégies + garde-fous : HOLOOCEAN_3D_GUIDE.md, GARDE_FOU).
- Optionnel : SONIC offline, MCFAR.

# Historique — état au 2026-07-05 — ✅ STAGE ARACATI BOUCLÉ (runs finaux + audit GT-free)

## 🧭 SUITE HOLOOCEAN (07-05 soir)

- **Méthode `Bruce_Sonar_USBL` intégrée aux options holoocean** :
  `./run_slam.sh holoocean 2D Bruce_Sonar_USBL` (alias `bsu`) — descripteur SC calculé
  en POLAIRE natif (sans remap), nœud SLAM dédié (SC on + NSSM on, usbl off : pas de
  capteur), seuils SC à RECALIBRER sur le sonar holoocean (loops_detected.csv,
  cf. HOLOOCEAN_GARDE_FOU.md §7). Non testé en run — 1er run de validation à faire.
- **`HOLOOCEAN_GARDE_FOU.md`** (branche holoocean) : le manuel pour l'ère Opus —
  carte du pipeline, 8 invariants, recettes (format 3D différent / passage odométrie
  ±USBL / ajouter une méthode), checklists chiralité + run valide + table des pannes,
  calibration SC, GO/NO-GO Pose3.

## ✅ CLÔTURE ARACATI (07-05) — tout dans TESTS.md partie 2 (§2.4-2.6)

- **10 runs finaux faits et archivés** (TESTS_image/) : livrable robuste = **ATE 1.5 ± 0.1 m,
  carte compas 0.075/0.43** (6 runs SC cumulés ; σ1.4 ≈ σ1.8 dans la variance ICP) ;
  Bruce pur ~1.9 (cap record 1.8-2.2°) ; holoocean 0.13 ×2. Écart contribution : +0.4 m médian.
- **Audit GT-free VÉRIFIÉ dans le code** (TESTS.md §2.6) : /pose_gt → uniquement le CSV
  d'éval ; seed USBL par défaut ; chemins GT = modes diagnostic non-défaut ; nuance
  compas (wz cmd_vel) déclarée. Topics = sonar + cmd_vel + USBL (+dvl/imu/depth en simu).
- **Lexique des sorties** (TESTS.md §2.5) : compass/filtered/cloud_vs_gt expliqués.
- Mini-papier : légende des colonnes façon DISO/ISOPoT + §6.3bis répétabilité.
  BRUCE_SLAM.md : avant/après code des modifications (pour la doctorante).
- Il reste (hors périmètre actuel) : relecture des papiers par Nathan ; HoloOcean 3D
  (bag du collègue) ; optionnel SONIC offline/MCFAR (U7).

# Historique — état au 2026-07-04 (nuit) — 🏆 CHAMPION ULTIME RU1 : 1.47 m / carte 0.075-0.413

## 🏆 RÉSULTATS RU1-RU4 (07-04) — détail : ULTIME.md (branche Bruce_Ultime)

| Run | Teste | Verdict |
|---|---|---|
| RU1 `125434-RU1` | σ1.8 | **CHAMPION : ATE 1.47, carte compas 0.075/0.413, NN 0.172** — FIGÉ yaml |
| RU2 `134157-RU2` | σ2.0 | 1.60 → optimum ~1.8 (balayage 1.4/1.8/2.0 = 1.50/1.47/1.60) |
| RU3 `145924-RU3` | union détecteurs | ❌ 2.91 — faux positifs natifs non gatés (PIEGES §12) |
| RU4 `114439-RU4` | B″ KF 1.0 (Bruce) | ❌ 17.17 — fenêtres NSSM en keyframes (PIEGES §11), rollback fait |

- RU5 `161907-RU5` (σ adaptatif U6) : ❌ 1.62 → rejeté, **PHASE ULTIME CLOSE sur RU1**.
  `./run_slam.sh` nu (branche Bruce_Ultime) reproduit le champion ; carte fine =
  `pointcloud_compass` (l'épaisseur RViz live = drift résiduel superposé, attendu).
- Verdict pistes externes : ISOPoT infaisable (code non publié) ; SONIC test offline en
  réserve seulement ; MCFAR en réserve.
- **Reste : consolidation** — intégrer RU1 dans le mini-papier + BRUCE_SLAM.md ;
  §7 usage avec/sans USBL ajouté au papier Bruce (run A = sans USBL, cas doctorante).

## 🚀 Précédent (07-04 matin)

- **Mini-papier** : `Paper/MiniPapier/MINI_PAPIER.md` (toutes branches) — relecture Nathan.
- **Papier branche Bruce** (pour la doctorante) : `BRUCE_SLAM.md` **sur la branche Bruce** —
  original vs modifications, ablation A/B/B′, améliorations restantes (KF 3.0→1.0 = réponse
  à la « trajectoire géométrique » : 256 KF espacées de 3 m, réglage upstream).
- **Branche `Bruce_Ultime` créée** (base BSU) — plan : `ULTIME.md` (FABLE §8).
  **U1 validé offline (0 run)** : rendu carte au cap compas recalé (GT-free) → traj 1.2a
  garde ATE 1.50 ET carte méd 0.077 / p90 0.441 (mieux que B′ 0.09/0.74, = borne cap GT).
  Script : `analysis/render_compass_cloud.py`. Prochain : U2 (analyse.sh) puis U3 (σ 1.8).

# Historique — état au 2026-07-03 (soir) — ARACATI : comparaison finale BOUCLÉE

> Docs : **FABLE.md** (§4 = résultat final) · **CONFIGS.md** (réfs) · **PIEGES.md** ·
> **ABLATION.md** (branche Bruce) · STAGE.md (journal). Éval : `./analyse.sh <run>`.

## 🏁 CHAMPIONS (configs FIGÉES dans les yaml)

| Champion | Run | ATE | Cap | Cloud NN | Loops |
|---|---|---|---|---|---|
| **Bruce_New = 1.2a** (SC 0.70 + USBL σ1.4) — yaml BSU figé | `003823` | **1.50 m** | 2.6° | 0.204 | 116 |
| **Bruce pur = B′** (SSM+NSSM + USBL σ2.5) — yaml Bruce figé | `120352-1` | **1.88 m** | 2.6° | 0.205 | 130 |
| variante « champion cloud » = 1.3 (SC + SSM + σ1.4) | `015742` | 2.14 m | 4.3° | **0.173** | 103 |

- **Écart : +0.38 m pour la contribution (Sonar Context)** — mêmes capteurs des deux côtés.
- Rejetés par les chiffres : B (σ1.0) 2.03 ; 1.4 (SSM+σ2.5 sur BSU) 3.13 ;
  **loterie DISO wz inversé CLOSE** (odom brute 39.2 m — chiralité nécessaire mais pas
  suffisante ; branche archivée en tag `archive/Bruce_DISO_wz`).
- Leçon transversale (3× mesurée) : **le σ d'ancre optimal dépend du pipeline**
  (loops SC → σ1.4 raide ; SSM/NSSM natifs → σ2.5 doux). Version principielle :
  σ adaptatif par fix (papier INS/USBL/DVL FGO — présentation 6).
- Fondation de tout : le **fix de chiralité** (PIEGES §1) — loops PCM 6→82→116/130,
  quai en T GT-free, SSM ressuscité.

## 📌 Ce qui reste

1. **Mini-papier — RÉDIGÉ (07-04)** : `Paper/MiniPapier/MINI_PAPIER.md` + figs générées par
   `analysis/paper_eval.py` (protocole DISO/ISOPoT : sections S1-S3, RE, ATE 1ʳᵉ-pose ;
   nouvelle métrique carte vs poses GT). Reste : relecture de Nathan + retouches.
2. **HoloOcean** : re-copier le bag du collègue (`test.bag`, cf. PIEGES §10) ; bag 3D
   à venir avec `HOLOOCEAN_3D_GUIDE.md` (donné au collègue) ; côté SLAM tout est prêt
   (`sonar_source:=points3d`).
3. Optionnel si temps : SONIC offline (CONFIGS #sonic-offline) ; MCFAR (#32-mcfar) pour
   pousser le cloud (désormais limité par les détections).

## Présentations prêtes

SONIC (`Paper/Sonar/SONIC_Presentation.md`) · INS/USBL/DVL FGO
(`Paper/Factor Graph/INS_USBL_DVL_Presentation.md` + résumé `INS_USBL_DVL_FGO.md`).

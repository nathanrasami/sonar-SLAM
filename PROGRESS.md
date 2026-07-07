# PROGRESS — état au 2026-07-07 (ap-m) — dérive SSM corrigée + analyse unifiée

## 🔧 JOURNAL R3 (07-07 ap-m) — grosse dérive run 162710 : CORRIGÉE (SSM off, sep 25)
- Symptôme : run défauts-ON 162710 → ATE 4.79 m (DR 0.12 m), dérive dès KF1 (avant toute loop).
- Runs discriminants (test.bag, ATE Umeyama) : SSM+NSSM 4.79 · **SSM seul 4.79 (coupable)** ·
  NSSM seul sep8 : 0.96 puis 2.52 (NON répétable, fausses loops court-terme type PIEGES §11) ·
  NSSM sep25 : 0.13 ×2 (0 loop passée) · tout off 0.13.
- Mécanisme : SSM (ICP séquentiel) REMPLACE le facteur DVL (excellent) par un recalage sonar
  biaisé (features simulateur éparses/arcs) → biais de cap cumulé (θ −0.44 rad en fin).
  Rejeté : loops NSSM (dérive avant KF24) ; bug bridge/bearing (NSSM seul fonctionne).
- Modifs (avant → après) :
  - `slam_holoocean.yaml` : ssm.enable True→**False** ; nssm.min_st_sep 8→**25** (nssm reste True).
  - `holoocean.launch` : arg ssm true→**false** (nssm reste true) ; commentaires chiffrés.
  - `run_slam.sh` : `ssm:=${SSM:-false} nssm:=${NSSM:-true}`.
- Vérifié : `./run_slam.sh holoocean` nu ×2 → **ATE 0.13 m répétable** (164521, 164659).
  Parité Bruce original au besoin : `SSM=true` (et sep 8 à remettre dans le yaml).
- ⚠ nssm sep25 sur CE bag = 0 loop (les candidates fin de parcours seules < min_pcm 4) :
  neutre ici, la machinerie loops reste prête pour un bag long (3D collègue).

## 🧰 analyse.sh unifié (07-07 ap-m)
- `paper_eval.py` intégré à `./analyse.sh` (chaînes aracati ET holoocean ; sauté sur caves,
  GT continue requise) — vérifié sur 162710 : holoocean_report + paper_eval + bilan_run
  s'enchaînent. Les autres scripts hors chaîne = outils d'investigation (sc_bench, verify_
  fusion, fix_mirror_cloud, inspect_bag) ; analyze_holoocean.py = supersedé (07-05).

---
# (précédent) PROGRESS — état au 2026-07-07 — SSM/NSSM défaut ON (parité Bruce) sur holoocean

## ⚙ JOURNAL R3 (07-07) — holoocean : SSM/NSSM passent à TRUE par défaut
- Constat : runs 141231/135343 → `nssm_constraints=0` partout, colonnes SLAM ≡ DR IMU+DVL
  bit-à-bit dans trajectory.csv (holoocean_report.py hors de cause). Cause : ssm/nssm
  `enable: False` dans slam_holoocean.yaml + run_slam.sh qui ne passait `nssm:=false` que.
- Upstream vérifié (`git show upstream/main:bruce_slam/config/slam.yaml`) : Bruce original
  = ssm True + nssm True. Décision Nathan : parité → défaut ON, opt-out `SSM=false NSSM=false`.
- Modifs (avant → après) :
  - `slam_holoocean.yaml` : ssm.enable False→True ; nssm.enable False→True.
  - `holoocean.launch` : arg `nssm` false→true ; NOUVEL arg `ssm` (true) ; `ssm/enable`
    injecté dans les nœuds bruce ET bruce_sonar_usbl (nssm reste forcé true côté BSU).
  - `run_slam.sh` (holoocean) : passe `ssm:="${SSM:-true}" nssm:="${NSSM:-true}"`.
- ⚠ L'ancien état de réf (dvl ATE 0.13 m) était ssm+nssm OFF ; nssm seul avait donné 2.36 m
  sur test.bag (61 s). Reproduire l'ancien : `SSM=false NSSM=false ./run_slam.sh holoocean`.
  Premier run défaut-ON à évaluer (NON VÉRIFIÉ : aucun run lancé avec ces défauts).
- Rappel BSU/holoocean : PAS d'USBL dans les bags du collègue → méthode BSU = loops SC
  uniquement (usbl off, déjà le cas dans le launch).

---
# (précédent) PROGRESS — état au 2026-07-06 (soir) — mémoire native + COMPARE.md + caves validée

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

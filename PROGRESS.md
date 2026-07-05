# PROGRESS — état au 2026-07-05 — ✅ STAGE ARACATI BOUCLÉ (runs finaux + audit GT-free)

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

# CONFIGS.md — guide d'implémentation des prochaines configurations

> Pour l'agent (Opus/autre) et pour Nathan : CHAQUE piste de `FABLE.md` §4 pointe ici
> (ancres `#12a-…`). Recette exacte, fichiers/lignes, résultat attendu, comment évaluer.
> **Lire `PIEGES.md` AVANT toute modification.** Règles : 1 changement par run ;
> après chaque run `python3 bilan_run.py results/<run>` ; reporter dans PROGRESS.md.

**Références (2026-07-02 soir, ablation complète)** :
| Run | Config | ATE | NN | Cap méd | Loops |
|---|---|---|---|---|---|
| **A** `194559` | **champion branche Bruce** (SSM+NSSM, 0 USBL) | **1.95 m** | 0.204* | **2.3°** | n/e |
| B `204329` | A + USBL back-end (sigma 1.0) | 2.03 m | 0.218* | 2.9° | n/e (≫A) |
| **C** `141223` | bricolage (USBL+SC), branche Bruce_Sonar_USBL | **1.53 m** | 0.203 | 3.4° | 82 |
| DR pur | (colonnes dr_* de A) | 10.55 m | — | — | — |
| Réf GT `011733` | DISO+GT (PAS GT-free) | 0.89 m | 0.199 | 1.7° | — |

**Leçon d'ablation (cf. ABLATION.md verdict)** : l'ancre USBL raide (sigma 1.0) DÉGRADE
la solution scan-cohérente (B < A partout, murs doublés). Trade-off ancre↔cohérence.
Option B' non testée : sigma relâché 2.5-3.0. Le « n/e » loops est corrigé : la branche
Bruce exporte désormais `nssm_constraints` (slam_ros.py).

*cloud seuil 65 (A) vs seuil 255 (C) : NN non comparables entre eux, cf. PIEGES §8.

---

## 1.2a — dist_threshold 0.60 → 0.70 {#12a-dist-threshold}

- **Branche** : Bruce_Sonar_USBL. **Fichier** : `bruce_slam/config/slam_aracati.yaml`,
  clé `sonar_context/dist_threshold: 0.60` → `0.70`.
- **Justification (diag 1.1)** : les candidats non-retenus à sc_dist 0.60-0.70 sont
  +108, TOUS vrais (dist GT < 15 m), 0 faux — la porte géométrique 20 m filtre en amont.
  L'échec historique de 0.695 (ATE 5.96) était PRÉ-fix miroir (PCM cassé) : caduc.
- **Run** : `./run_slam.sh` (défauts). Arrêt : SIGINT à la fin du bag (PIEGES §5).
- **Attendu** : constraints (somme nssm_constraints de trajectory.csv) 82 → 150+ ;
  NN < 0.20 ; ATE ≤ 1.4 m. Si CPU sature (shgo par candidat) : `RATE=0.5`.
- **Si succès** : tenter 0.75 (encore +80 vrais). **Si ATE se dégrade** : PCM laisse
  passer des transforms ICP faibles → remonter min_pcm 6→7 plutôt que redescendre le seuil.

## 1.3 — SSM sur Bruce_Sonar_USBL {#13-ssm}

- **Fichier** : `slam_aracati.yaml` : `ssm/enable: True` + `ssm/max_translation: 1.5`.
- **Justification** : le run A (branche Bruce) prouve que SSM post-fix marche
  (cap 2.3° méd vs 3.4° pour C — SSM contraint le cap, l'USBL non).
- **Attendu** : cap < 3°, NN ↓. Risque faible (échecs pré-fix caducs).

## 1.4 — combo gagnant 1.2a + 1.3 {#14-combo}

Appliquer les deux réglages gagnants ensemble, 1 run. Cible : ATE < 1.2 m.

## 1.5 — USBL sigma {#15-usbl-sigma}

- **D'abord OFFLINE** (aucun run) : résidu des fixes `/usbl_point` vs GT par segment de
  mission (extraire les fixes du bag OU logger usbl_buffer). Si le bruit réel varie
  (multipath près du quai ?), un sigma fixe 1.4 est sous-optimal.
- **Puis** : `slam_aracati.yaml usbl/sigma` 1.0 ou 2.0 (1 run chacun). Le noyau Cauchy
  protège déjà des outliers ~73 m (pings ratés → position transpondeur, cf. README).

## 2 — Ablation branche Bruce {#2-ablation}

Voir **`ABLATION.md`** (branche Bruce). État : A = 1.95 m ✅ fait (run `194559`) ;
B lancé. **TODO code (petit)** : la branche Bruce n'exporte PAS les loops —
dans son `slam_ros.py` (writer trajectory.csv, ~ligne 199), ajouter la colonne
`nssm_constraints` = `len(kf.constraints)` comme sur Bruce_Sonar_USBL (export_csv).
Sans ça, impossible de savoir combien de loops NSSM natives sont actives.

## 3.1 — DISO GT-free « wz inversé » (chiralité du prior) {#31-diso-wz}

> ✅ **PRÉPARÉ (07-03) sur la branche `Bruce_DISO_wz`** — rien à éditer :
> ```bash
> git checkout Bruce_DISO_wz
> ODOM_SOURCE=diso DISO_PRIOR=cmd_vel RATE=0.5 ./run_slam.sh
> ```
> (RATE=0.5 : DISO = méthode directe, sensible à la contention CPU, PIEGES §3.
> flip_bearing passe automatiquement à False en mode diso — surcharge launch.)
> Verdict sur l'odométrie BRUTE (colonnes dr_* / odometry.csv) : < 5 m = jackpot.

- **Hypothèse** : DISO+prior cmd_vel divergeait (ATE 22 m) parce que le cap ENU du
  prior tourne À CONTRE-SENS du repère DISO (réfléchi, det=−1) — même famille de bug
  que le miroir des scans (PIEGES §1, §3).
- **Recette** (branche Bruce_Sonar_USBL) :
  1. `cmd_vel_odom.py` : ajouter param `~invert_wz` (défaut False) ; si True,
     `wz = -wz` dans le callback `/cmd_vel` (l'intégration produit alors un cap
     en convention réfléchie = celle que DISO attend).
  2. `aracati.launch`, bloc `odom_source==diso && diso_prior==cmd_vel` : ajouter
     `<param name="invert_wz" value="true"/>` au nœud cmd_vel_odom (celui qui publie
     `/cmd_vel/pose`).
  3. **`flip_bearing: False`** pour ce run (PIEGES §1) + `usbl/flip_y` déjà géré.
  4. Run : `ODOM_SOURCE=diso DISO_PRIOR=cmd_vel RATE=0.5 ./run_slam.sh`.
- **Attendu** : l'odométrie DISO brute (colonnes dr_* / odometry.csv) ne diverge plus
  (< 5 m vs 22 m). Si oui → DISO GT-free devient une odométrie candidate pour le run
  final (DISO + SC + USBL back-end). Si non → clore la piste DISO GT-free.

## 3.2 — MCFAR (SIO-UV) {#32-mcfar}

Débruitage multi-échelle AVANT le CFAR dans `feature_extraction.py::callback_cartesian`
(pyramide de médianes/bilatéral, cf. `Paper/Sonar/SIO-UV.md`). Moyen effort. Mesure :
NN ↓ ET nombre de loops ↑ à config égale. Ne pas combiner avec un autre changement.

## SONIC — test OFFLINE avant toute intégration {#sonic-offline}

- **Papier/résumé** : `Paper/Sonar/SONIC.md`. Code : https://github.com/rpl-cmu/sonic
- **Test discriminant bon marché (aucun run)** : rejouer les **122 paires candidates**
  du run C (`results/run_aracati_2026-07-02_141223/loops_detected.csv`, retenu=1) :
  extraire les 2 images sonar de chaque paire (dump du bag aux temps des keyframes),
  passer en polaire (réutiliser `_polar_remap`), inférer les correspondances SONIC,
  estimer la transform (RANSAC planaire) et comparer à (a) la transform ICP du NSSM,
  (b) la pose relative GT. Métrique : erreur de transform + taux de conversion.
- **Si SONIC > ICP** sur ces paires → remplacer l'étape ICP du NSSM (le goulot 1.1 :
  40/122 candidats non convertis). ⚠ modèle entraîné M1200d/10 m sur HoloOcean →
  fine-tuning probablement nécessaire pour le P900/48 m (HoloOcean du chantier 2 peut
  générer les paires d'entraînement, on a déjà le pipeline polaire).

## Objectif final : UN SEUL run {#run-final}

Quand les phases 1-3 ont désigné la config gagnante : la consolider sur UNE branche
(défauts du yaml/launch = config gagnante, plus aucune variable d'env nécessaire),
1 run complet + `bilan_run.py` + `analyse.sh` = le livrable.
**Plancher réaliste** : la « GT » est un DGPS sur planche flottante — viser ATE ~0 est
impossible contre cette référence ; < 1 m est le bon objectif (réf GT-assistée : 0.89 m).

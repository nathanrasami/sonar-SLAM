# PIEGES.md — À NE JAMAIS FAIRE (sinon on casse le run)

> Document PARTAGÉ sur toutes les branches (main, Bruce, Bruce_Sonar_USBL, holoocean).
> Chaque piège ci-dessous a RÉELLEMENT cassé quelque chose pendant le stage. À lire
> AVANT de modifier une config ou d'intégrer une méthode. Compagnon : `CONFIGS.md`
> (comment implémenter les prochaines pistes), `FABLE.md` (investigations).

## 1. ⚠⚠ CHIRALITÉ scans ↔ odométrie (LE piège du stage — 3 semaines perdues)

**Règle : `cartesian/flip_bearing` (feature_aracati.yaml) = `True` avec l'odométrie
cmd_vel/USBL, `False` avec l'odométrie DISO. Ne JAMAIS mélanger.**

- Pourquoi : DISO a un repère RÉFLÉCHI (det=−1 — son System.cpp:89 swappe x↔y du prior).
  Les features cartésiennes avaient été calées « même signe que DISO » → en odométrie
  cmd_vel (repère propre), chaque scan était peint EN MIROIR de son cap.
- Symptômes : carte « tourbillon » (arcs), PCM qui rejette presque toutes les loops
  (6 constraints au lieu de 82), SSM qui « diverge », loops NSSM « fausses ».
- Détection en 1 run court : `python3 analysis/bilan_run.py results/<run>` → si le cloud est en
  arcs/spirale au lieu de structures, suspecter la chiralité EN PREMIER.
- Historique : c'est cette inversion, héritée de DISO, qui a cassé le pointcloud de
  Bruce_Sonar_USBL pendant des semaines (« swirl = fond du sonar », « position = goulot »,
  « NN non-déterministe » : tous ces diagnostics étaient des symptômes du miroir).

## 2. ⚠ USBL : jamais de DOUBLE ancrage

- **Interdit** : fusion USBL front-end (gain > 0 dans cmd_vel_odom) ET facteurs USBL
  back-end (usbl/enable) en même temps → l'odométrie snappe sur chaque fix bruité
  → zigzag, ATE 1.45 → 4.66 m (runs 111133 vs 135228).
- Branche Bruce, mode B : `USBL_GAIN=0` OBLIGATOIRE avec `USBL_BACKEND=true`
  (le front-end ne sert alors qu'à seeder le repère).
- Branche Bruce_Sonar_USBL : laisser `usbl:=false` (launch) — l'ancrage est back-end.

## 3. ⚠ DISO : subtilités qui cassent tout

- Repère y RÉFLÉCHI (det=−1) → `usbl/flip_y=True` requis en mode DISO (le launch de
  Bruce_Sonar_USBL le force selon odom_source). Sans flip : ATE 13.9 m.
- DISO publie `/direct_sonar/pose` avec `ros::Time::now()` au lieu du stamp du scan
  → décalage temporel sous charge.
- Méthode DIRECTE = sensible à la contention CPU : en combiné, `RATE=0.5`
  (2.1 m standalone → 3.2-5.5 m à rate 1.0 combiné).
- Prior cmd_vel pour DISO : le cap ENU propre est À CONTRE-SENS du repère DISO →
  divergence (ATE 22 m). Piste de fix : wz inversé, cf. CONFIGS.md §3.1.

## 4. ⚠ Jamais deux runs SLAM en parallèle

La contention CPU fausse les résultats (frames droppées, files pleines) — même sur des
branches différentes. Un seul run à la fois, toujours.

## 5. Arrêt des runs (sinon AUCUN CSV)

- Les 3 branches de run (**Bruce**, **Bruce_Sonar_USBL** depuis le 07-03, **holoocean**) :
  rosbag_play a `required="true"` → le run s'arrête TOUT SEUL à la fin du bag et écrit
  les CSV. Ne rien faire.
- Pour interrompre un run AVANT la fin : **SIGINT** (Ctrl-C dans le terminal, ou
  `kill -INT <pid roslaunch>` dans le conteneur ros1). **JAMAIS kill -9** :
  les CSV ne sont écrits qu'au on_shutdown.

## 6. Ne pas toucher au dépôt pendant qu'un run tourne

Pas de `git checkout`, pas d'édition de config/launch/scripts pendant un run : les
paramètres sont chargés au démarrage mais certains scripts sont relus, et le run devient
non-reproductible. Analyses offline (results/, scratchpad) : OK.

## 7. Conventions de cap (sinon ~98° d'« erreur » fantôme)

- `groundtruth.csv:theta` = cap COMPAS en convention **NED** : gθ ≈ −θ_map + 90.7°.
  Toute comparaison naïve θ_SLAM vs gθ donne ~98° d'erreur médiane (faux).
- Toujours passer par `bilan_run.py` (fit circulaire s·θ+β, offset retiré, wrap).
- `cmd_vel.angular.z` = −d(gθ)/dt (dérivé du compas du véhicule, cf. README aracati2017)
  → l'odométrie « GT-free » utilise le compas embarqué : à assumer, pas à cacher.
- Ne PAS réactiver `cloud/use_compass_cap` ni `cap_offset_deg: 162` : hacks pré-fix.

## 8. Évaluation : les règles

- ATE = **Umeyama full-séquence** (`traj_eval.py` / `bilan_run.py`), rien d'autre.
- Ne JAMAIS comparer aux ATE des papiers « par section, alignés 1re pose » (ISOPoT :
  3.2-4.6 m ≠ notre 1.5 m — métriques différentes).
- NN médian du cloud : dépend du seuil d'intensité (cloud à 65 ≠ cloud filtré 255).
  Comparer à seuil ÉGAL. `map/threshold` (rendu carte) ≠ `filter/threshold` (features
  SLAM) : monter filter/threshold à 140 AFFAME les loops (traj 5.2 m) — ne pas confondre.
- Résultats pré-fix miroir (avant 2026-07-02) impliquant SSM/NSSM/ICP/PCM : **caducs**.
  Ex. « dist_threshold 0.695 → ATE 5.96 » testé avec PCM cassé — ne pas citer.

## 9. HoloOcean

- `holoocean_sonar_bridge.py` : `~range_m`/`~fov_deg` DOIVENT matcher la config du
  simulateur (défauts 40 m / 120°) — sinon échelle/bearing faux en silence.
- `OculusPingUncompressed` n'a PAS de champ `num_beams` ; bearings en **centi-degrés**
  (to_rad = b·π/18000). Ne pas « corriger » ça.
- Vérifier la chiralité au 1er run court (bilan_run) avant tout run long — leçon §1.

## 10. Divers qui ont déjà mordu

- `keyframe_translation` diffère entre branches (Bruce ≈ 2.4 m/KF → 255 KF ;
  Bruce_Sonar_USBL ≈ 0.9 m → 665 KF) : ne pas comparer les comptes de KF/points bruts.
- Branche Bruce : les loops ne sont PAS exportées (pas de nssm_constraints ni
  loops_detected.csv) — cf. CONFIGS.md pour l'ajout.
- Fichiers avec retour à la ligne dans le nom (Paper/) : cassent les scripts — renommer.
- ⚠⚠ **Fichiers IGNORÉS écrasés au checkout** (vécu, 07-03 : test.bag 714 Mo PERDU) :
  si un fichier est À LA FOIS ignoré (.gitignore `*.bag`) et SUIVI par une autre branche,
  `git checkout` l'ÉCRASE SANS AVERTIR (les ignorés sont « sacrifiables ») puis le
  supprime au retour. Règle : un gros fichier (bag…) ne doit être suivi sur AUCUNE
  branche (git rm --cached partout) — fait le 07-03. Garder une copie source des bags
  HORS du dépôt.
- Ne rien committer sans le demander à Nathan ; messages détaillés (chiffres + pourquoi).

## 11. Les fenêtres NSSM/PCM sont en KEYFRAMES, pas en mètres (vécu : RU4, ATE 17 m)

`min_st_sep`, `pcm_queue_size`, `source_frames` comptent des KEYFRAMES. Densifier les
keyframes (3.0 → 1.0 m, run B″ 07-04) SANS les rescaler a réduit l'exclusion de revisite
de ~24 m à ~8 m → auto-appariements court-terme validés comme « loops » (1re à t=3 min,
47 avant 8 min alors qu'aucune revisite n'existe) → 415 contraintes fausses, ATE 1.88 →
**17.17 m**. Règle : tout changement de `keyframe_translation` doit rescaler ces trois
fenêtres du même facteur (recette B″-bis : 1.0 m + min_st_sep 24, pcm_queue 15,
source_frames 15 — cf. ABLATION.md).

## 12. Un détecteur de loops SANS porte géométrique pollue le PCM (vécu : RU3, 2.91 m)

L'union SC + gating natif (07-04) a réinjecté 583 candidats natifs NON gatés → shgo/ICP
saturés (frames sonar perdues : 615 KF au lieu de 665) et contraintes fausses corrélées
acceptées par le PCM (210 constraints, S3 3.70 m). La porte géométrique 10 m de SC n'est
pas un détail : c'est elle qui tue les faux positifs AVANT l'ICP. Tout nouveau détecteur
de candidats doit passer par une porte équivalente.

## 13. La chaîne image de feature_extraction suppose un FOV < 180° (vécu : caves, 07-05)

La conversion cartésienne du chemin polaire (`generate_map_xy` + remap) calcule la
largeur du canvas via sin(FOV/2) : pour un MSIS 360°, sin(180°)≈0 → TOUTES les
coordonnées s'effondrent. Symptôme sournois : nuage minuscule et BIT-IDENTIQUE quels
que soient les seuils CFAR (672 pts sur 2 runs caves — aucun paramètre n'y change rien,
ce qui est LA signature : si régler un seuil ne change pas la sortie, le goulot est en
aval du seuil). Règle : sonar à couverture ≥180° → features calculées EN POLAIRE
(CFAR sur l'image polaire + r·cosθ/r·sinθ), cf. msis_scan_bridge.py branche caves.

# MISSION round 2 « noise » — exécutant : Opus · décideur : Nathan (2026-07-15)

**But (demande prof)** : régénérer les 2 bags scénarios avec **beaucoup plus de noise sur
sonar, IMU et DVL**, puis re-appliquer les 2 méthodes SLAM — comparer au round 1.
**Une seule variable-groupe vs round 1 : le noise.** Mêmes trajs (traj9 = tour complet
z const, sonar 20 m · traj5 = z variable + contourne bateau, sonar 40 m), mêmes seeds,
même résolution **1024×512** (PIEGES #22 : AzimuthBins ≤ 512 sinon stries).

## ✅ FAIT (Opus, 15-07 soir) — étapes ①-③ ; reste ④-⑤ (Nathan génère)
- **⛔ ① chiffré avec Nathan** : L1 sonar **×5** · L2 capteurs **×5** · L3 nav structuré **×2**.
  ⚠ mesure : **traj5 n'a AUCUN L3** (v5.py:354-395, nav parfaite) → le ×2 L3 = traj9 SEUL ;
  traj5 round 2 = L1+L2 ×5. Dérive DR traj9 ×2 mesurée à sec = 11.54 m (round 1 : 5.76).
- **② patch = module `noise_round2.py`** (gated `NOISE_ROUND2=1`, sinon round 1 byte-identique).
  L1 v5.py:287 + v6.py:61 · L2 gen_bag_3d.py:63 · L3 v8.py:49-52 · assert v8 `[1,8]→[1,30]` ·
  bags `_noise` (wrappers gen_traj{5,9}.sh + gen_2bags.sh). Vérif à froid PASS (2 états env).
- **③ à faire par Nathan** : `NOISE_ROUND2=1 ./gen_traj9.sh --test 150` (E1-E9) +
  `NOISE_ROUND2=1 ./gen_traj5.sh --test 150` (E1-E8). E-check FAIL sous gros noise = **STOP +
  montrer les chiffres** (peut être physiquement légitime — AddSigma 0.05 « noie » les échos).
- **④-⑤ ensuite** : `NOISE_ROUND2=1 ./gen_2bags.sh` → 4 runs SLAM _noise + analyse + rapport.

## État au moment de l'écriture (Fable, 15-07 soir)
- Round 1 : `./gen_2bags.sh` EN COURS (traj9 puis traj5, 1024×512, E-checks auto) ;
  ensuite Nathan lance `./slam_2bags.sh` (4 runs {traj9,traj5}×{Bruce,Bruce_Sonar},
  nssm=true parité, labels `results/*/scenario_label.txt`) puis analyse. Round 2 APRÈS.
- ⚠ Docs non commités (gen en cours → commit interdit) : PIEGES.md #22-23, ce fichier,
  PROGRESS.md § 15-07. **Première action Opus : `pgrep -af "gen_bag_3d|roslaunch"` →
  si créneau propre, committer ces docs** (branche `git branch --show-current` = holoocean).

## Leviers de noise — valeurs ACTUELLES mesurées (fichier:ligne, vérifiées 15-07)
1. **Image sonar (simulateur)** — `gen_bag_3d_v5.py` `make_cfg()` dict `common` :
   `AddSigma 0.01 · MultSigma 0.01 · RangeSigma 0 · MultiPath False`. Hérité par les
   DEUX bags (chaîne v10→v6→v5). (Le sonar vertical/profiler ont leurs propres cfg.)
2. **Bruit gaussien des messages capteurs** — `gen_bag_3d.py:63` :
   `SIGMA_GYRO 0.002 · SIGMA_ACC 0.02 · SIGMA_DVL 0.01 · SIGMA_DEPTH 0.02`
   (importés par v5 ; LIRE le code qui les applique avant de toucher).
3. **Erreurs nav STRUCTURÉES (DR réaliste)** — `gen_bag_3d_v8.py:49-52` :
   `PSI0_DEG 2.0 (biais compas) · RW_SIG_DEG 0.15 °/√s · DVL_SCALE 0.005 · DVL_MIS_DEG 0.5`.
   ⚠ figées après calibration (le 1er tirage 1.0/0.05 était « trop optimiste ») —
   les changer change la NATURE de l'expérience, pas juste le niveau.

## Étapes
① ⛔ **Chiffrer avec Nathan AVANT tout code** : quelles couches (1/2/3) et quels facteurs
   (« beaucoup » = ×5 ? ×10 ? bruit sonar en absolu ?). Proposer un tableau avant→après.
   Convention de nommage bags round 2 (suggestion : `holoocean_3d_traj{9,5}_noise.bag`)
   — ne JAMAIS écraser les bags round 1 (la comparaison en dépend).
② Patch minimal (variables groupées en tête de fichier, pattern des overrides v7/v10),
   vérif à froid des cfg (les 2 générateurs, pattern du 15-07 : import + inspection dict).
③ **Bag test 150 s + E-checks d'abord** (`./gen_traj9.sh --test 150`). ⚠ les E-checks
   sont calibrés au noise actuel (E6 I_MIN 0.15 ; E8/E9 fractions) : un FAIL sous gros
   noise peut être PHYSIQUEMENT légitime → **STOP + montrer les chiffres à Nathan**,
   jamais d'affaiblissement silencieux d'un check (et PIEGES #21 : les seuils SLAM aval
   — CFAR threshold 30 — sont calibrés sur la dynamique actuelle ; la dégradation du
   SLAM sous noise est probablement L'OBJET de l'expérience → ne rien « compenser »
   côté SLAM sans consigne explicite).
④ Bags complets (adapter gen_2bags.sh ou wrappers pour les noms _noise) puis 4 runs
   (adapter les SPECS de slam_2bags.sh vers les bags _noise) + analyse.
⑤ Rapport PROGRESS (tableau round 1 vs round 2 : ATE, loops, funnel) + commit détaillé.

## Ce qu'Opus ne fait PAS
Committer/modifier le dépôt pendant un gen ou un run (PIEGES, incident test.bag) ·
lancer 2 gens/runs en parallèle · toucher aux bags round 1 · affaiblir un E-check ou un
seuil SLAM pour « faire passer » · changer résolution/trajectoire/seed en même temps que
le noise. Arrêt d'un gen : PIEGES #23 (orphelins, shm, bag partiel à supprimer).

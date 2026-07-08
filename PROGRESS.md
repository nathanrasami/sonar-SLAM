# PROGRESS — 2026-07-08 — carte_3d UNIQUE (vraie 3D only) + traj3 validée

## ⚠→✅ 08-08 (Opus) : carte 3D = NUAGE DENSE COMPLET (filtre verticalité RÉVERTÉ)
- FAUSSE PISTE assumée : j'avais mis un filtre de verticalité (garder colonnes à grand z-span,
  retirer le fond) → 547k→15,7k pts. Nathan : « presque plus aucun point », analogie au
  sur-sharpening du pointcloud SLAM (piège connu). CORRECT : sur-filtrage, symptôme pire que
  l'original (R8 : mon fix a créé un défaut). Filtre + code retirés.
- État FIGÉ : `carte_3d.py` = nuage 3D COMPLET (fond + structures), juste dédoublonné par
  **voxel 0.2 m** (densité/perf, ne retire aucune structure). Vérifié run 143452 : 547k→
  **355k pts**, **NN 0.056 m**, html 12 Mo dense (nuage + trajectoire + départ/arrivée, accents
  échappés unicode = normal, PAS un bug d'affichage). C'est la carte honnête et dense.
- ⚠ Le « de dessus pas net comme pointcloud_map » reste inhérent (fond + arcs sonar) : NE PAS
  re-filtrer agressivement. pointcloud_map (features sonar horizontal) reste la vue de dessus
  propre ; carte_3d = le volume. Deux vues complémentaires assumées.

## ⚠ 08-07 (Opus) : Pose3 RÉFUTÉ comme fix de la carte 3D (test discriminant)
- Affirmation « il faut Pose3 pour la carte 3D » testée : composer profiler+sonar_points
  avec la trajectoire GT (= Pose3 PARFAITE) → `_test_gt_topview.png` : **bave quand même**
  vu de dessus (lames radiales profiler, arcs sonar). Pose3 hors-cause = géométrie capteur.
- Corroboré : ATE 0.04 m, carte déjà 0.042 m, 0 loop → back-end n'a rien à optimiser en simu.
- Pose3 reste un chantier réel mais pour données RÉELLES/bruitées (drift z/roll/pitch + loops)
  et comme fondation front-end tilt — PAS pour la carte de dessus. Détail : SLAM_3D_MIGRATION.md
  § VÉRIFICATION 08-07. Vrai fix carte présentable = reconstruction offline (fusion XY sonar
  horizontal + Z profiler), pas le back-end.

## 🔍 08-07 : pointcloud_map vs carte_3d vus de dessus (question Nathan) — EXPLIQUÉ
- Constat : vus de dessus, pointcloud_map (poteaux du quai NETS) ≠ carte_3d (empreinte
  brouillée, poteaux noyés). Ce sont 2 nuages de sources DIFFÉRENTES :
  - pointcloud_map.csv = features CFAR du **sonar HORIZONTAL** (balaye azimut ±60°,
    std y_veh 3.4 m) → empreinte XY nette → dessine les 2 quais + pilotis.
  - carte_3d = **profiler VERTICAL** (+sonar tilté). Profiler : std y_veh **0.00**, std z
    12.7 m = lame verticale dans l'axe robot → vu de dessus = traînée RADIALE (aucune
    largeur), rayonne selon le cap → brouille l'empreinte, poteaux noyés.
- Hypothèse REJETÉE : « le SLAM ignore le tilt → pointcloud_map distordu » — FAUX,
  pointcloud_map est au contraire le plus net des deux. La cause est l'inverse : le
  profiler est fait pour la HAUTEUR (z), pas pour l'empreinte XY.
- Conclusion : complémentaires, pas un bug. Sonar horizontal = OÙ (plan) ; profiler =
  HAUTEUR (coupe verticale). Une carte 3D à poteaux nets de dessus = fusionner XY(sonar
  horizontal) + Z(profiler) → c'est le vrai SLAM 3D (Pose3, reste à faire).

## ➕ 08-07 : trajectoire SLAM dans carte_3d (html + png)
- Demande Nathan : la carte n'avait que le nuage. Ajout de la trajectoire SLAM
  (traj x/y/z, même repère GT-free que le nuage) : ligne rouge + marqueurs
  départ/arrivée, dans le .html plotly (4 traces, légende) ET le .png (légende
  lower left pour ne pas chevaucher le titre). Vérifié à l'image (run 143452) :
  errance aléatoire v4 bien visible dans le nuage PierHarbor.

## 🔧 08-07 (suite) : carte_3d auto-réparante + plotly conteneur
- Nathan a nettoyé un run (garde les CSV seuls) → bag_source.txt supprimé → pas de
  carte et message trompeur. Fixes vérifiés (run 143452 nettoyé, commande nue) :
  - analyse.sh retrouve le bag : argument → bag_source.txt → **auto-détection par
    durée** (trajectory.csv vs rosbag info, match ±5 %) + self-heal (ré-écrit le txt) ;
  - message d'échec honnête (bag inconnu ≠ pas de vraie 3D) ;
  - ⚠ PIÈGE trouvé : **plotly absent du conteneur ros1** → les .html étaient sautés
    en silence (seuls .png/.npy). Installé (`pip3 install --user plotly`, persiste via
    $HOME) + print honnête dans carte_3d.py. `./analyse.sh 3D 143452` → html 11.8 Mo ✅.

## 🗺 08-07 soir : carte_3d.py = LA carte 3D unique d'un run (directive Nathan)
- Règle : UNE carte par run (`carte_3d.html/.png/.npy`), **VRAIE 3D uniquement** —
  gate par PING (std(z) intra > 0.5 m) : les tranches plates (pseudo-3D « 2D plaqué »)
  sont exclues ; s'il n'y a AUCUN ping volumique → REFUS explicite, pas de carte.
- `analysis/carte_3d.py` : auto-détection des sources (p90 sondage + gate/ping),
  gère v4 (frame auv0 → composition directe pose SLAM, GT-free natif) ET v3
  (frame map → dé-projection GT du stamp = décodage) ; métrique **Umeyama traj**
  (remplace première-pose). Pipeline : run_slam.sh écrit `bag_source.txt` →
  analyse.sh enchaîne carte_3d (conteneur) ; `./analyse.sh 3D` OUVRE carte_3d.html
  (view3d pseudo-3D débranché) ; caves : grotte_3d.html copié en carte_3d.html.
- **Tests PASS (3 formats)** : traj3 v4 → profiler+sonar tilté fusionnés, 547 k pts,
  **NN vs carte GT 0.042/0.076 m** (l'alignement première-pose causait les 0.32-0.59) ;
  traj2 v3 → profiler dé-projeté, sonar pseudo EXCLU, NN 0.020 m ; traj1 pseudo-only
  → refus, aucun fichier. Anciens scripts (view3d, profiler_3d, profiler_slam_3d,
  traj3_map_3d) = outils, plus des livrables.

# (précédent) PROGRESS — TRAJ3 PierHarbor VALIDÉE (0.04 m, cartes 3D GT-free natives)

## 🏆 08-07 ap-m : traj3 (PierHarbor, contrat v4) reçue et VALIDÉE
- Bag `bag/holoocean_3d_traj3.bag` (5.5 Go, 1057 s) : conforme v4 sur tout —
  `/sonar_tilt` ±15.0°, points en repère VÉHICULE (auv0), roll/pitch 0.0000,
  fermeture 0.00 m, échos méd 0.486 (1ᵉʳ ping 0.047 = face au large, normal).
- **SLAM 2D (run 143452, défauts nus)** : ATE 0.04 m, 665 KF, z −11.6→−2.4, 0 loop.
- **Cartes 3D GT-FREE NATIVES** (`analysis/traj3_map_3d.py`, plus de dé-projection GT —
  composition directe pose SLAM × points véhicule) :
  - `map3d_profiler` 237 k pts : quai + pilotis + treillis jusqu'à −30 m — NN vs
    carte GT **0.59/1.65 m** (méd/p90) ;
  - `map3d_sonar` (tilt) 554 k pts : NN **0.32/0.80 m** — la 3D vue par le SLAM.
  - ⚠ NN dominé par l'alignement première-pose (yaw t0 × bras de levier ~60 m) —
    amélioration possible : alignement Umeyama trajectoire avant comparaison.
- Reste : SC/loops sur traj3 (2 tours → revisites) ; exploiter /sonar_tilt dans la
  chaîne SLAM (aujourd'hui l'image est traitée comme plane — ok à ±15° mais info
  élévation perdue) ; Pose3.

# (précédent) PROGRESS — MEILLEUR RÉSULTAT figé : 0.03 m ×2 + carte 3D GT-free 0.02 m

## 🏆 07-08 (pleine capacité) — campagne « meilleur résultat » CLOSE
- **Arbitrage loops (bag1 complet, 2 tours, DR 0.03, 1 variable/run)** :
  NSSM natif 0.84 m (4 loops tour 2 nuisibles) · **NSSM off 0.03 m** · bs+SC 0.98 :
  0.03 m, 218 candidates retenues (toutes vraies revisites), 0 acceptée par l'ICP
  aval, 0 dégradation → gates SC SÛRS, ICP simu = le verrou (partout).
- **Défauts FIGÉS** (répétés ×2 par la commande nue, runs 124548/131322 = 0.03 m) :
  run_slam.sh + launch → `ssm:=false nssm:=false` ; SC dist_threshold 0.98 (yaml).
  Le plancher = la DR (quasi parfaite en simu) : les loops n'apportent rien ICI —
  elles serviront sur données réelles (drift) une fois l'ICP fiable.
- **🏆 LIVRABLE PHARE : carte 3D 100 % GT-free** = sections /profiler_points
  re-projetées sur la traj SLAM (`analysis/profiler_slam_3d.py`) :
  **NN médian 0.021 m / p90 0.041 m vs carte GT** (135 802 pts). Piège résolu :
  aligner repère SLAM→monde avant la métrique (Umeyama, comme l'ATE) — sans ça
  on mesure le décalage de repère (6.1 m), pas la carte. Sorties png/html/npy
  dans run 105410.
- **Guide §8 (demande v4 au collègue)** : vraie 3D par le sonar PRINCIPAL (tilt
  oscillant, critère std(z) intra > 0.5 m) + points en repère VÉHICULE (auv0),
  pas monde (la transformation GT cachée dans les bags actuels oblige la
  dé-projection de profiler_slam_3d.py).
- Reste : Pose3/iSAM2 (avec le futur bag tilt) ; ICP simu (verrou des loops) ;
  maillage/densification de la carte GT-free.

# (précédent) PROGRESS — 2026-07-08 — bags 3D VALIDÉS (0.03 m, vraie 3D prouvée) + ablations en cours

## ✅ 07-08 : bags 3D validés + prochaines étapes lancées (fin de session Fable)
- traj1 (arrêt 471 s) : ATE 0.03 m, z porté, cloud NN 0.027 — pseudo-3D OK.
- traj2 (arrêt 404 s) : ATE 0.03 m + **profiler_3d.png/.html** (murs VERTICAUX 0→−11 m,
  186 852 pts) = vraie 3D par le sonar PROUVÉE. Fix : import mplot3d (conteneur).
- **`suite_next.sh` en nohup** (résultats → `results/suite_next.log`, commit auto) :
  RUN A = SSM=true sur rendu propre (réf ancien rendu : 4.79 m ; témoin nu : 0.03 m) ·
  RUN B = méthode bs (SC) sur traj1 2 tours (lire loops_detected.csv, seuil 0.87 à
  recalibrer si besoin). 1 variable/run.
- **VERDICTS ablations (07-08, témoin 0.03 m/0 loop)** :
  - RUN A SSM=true (110806) : **1.45 m** — mieux que l'ancien rendu (4.79) mais 48× pire
    que la DR → **SSM définitivement OFF en simu** (l'ICP séquentiel ne bat jamais la DVL).
  - RUN B bs/SC (111933) : 0.03 m, 0 loop acceptée MAIS SC détecte de **VRAIES revisites
    tour2↔tour1** (191↔0, 198-202↔5, sc_dist 0.95-0.99) rejetées par le seuil 0.87.
- **Run 115143 (bag1 COMPLET 676 s, défauts nus)** : 4 loops NSSM natives acceptées au
  tour 2 (KF331-336) → **ATE 0.84 m** vs 0.03 (partiel sans loops). Les loops qui
  passent PCM DÉGRADENT (ICP simu peu fiable) — cohérent avec tous les verdicts.
- Reste (session suivante) : arbitrer NSSM natif off vs SC seuil ~0.98 (les deux voies
  butent sur l'ICP simu) ; vérifier l'ICP aval ; reconstruction profiler raffinée
  (fusion sections×traj SLAM, maillage) ; Pose3 (SLAM_3D_MIGRATION.md).

# (précédent) PROGRESS — état au 2026-07-07 (soir) — BAGS 3D LIVRÉS, runs à faire (session suivante)

## 🤿 07-07 SOIR (session Fable close à 91%) — SUITE 3D AUTONOME LANCÉE
- Bags copiés dans `bag/`. Run traj1 lancé ; 1er échec diagnostiqué : /sonar = 32FC1
  [0,1] → applyColorMap crash (feature_extraction:271) + CFAR aveugle (seuil 95/255).
  FIX : bridge convertit 32FC1→mono8 ×255 (holoocean_sonar_bridge.py). Run relancé,
  Nathan confirme À L'ÉCRAN : traj + sonar OK, « bon résultat ».
- **`suite_3d.sh` tourne en nohup** : attend fin traj1 → analyse → run traj2 →
  analyse → profiler_3d.py (nuage vraie-3D de /profiler_points, z<0) → commit+push
  auto. **Tout se lit dans `results/suite_3d.log`** + 2 derniers run_holoocean_*.
- Session suivante : lire suite_3d.log, vérifier ATE traj1/traj2 + profiler_3d.png,
  puis reprendre le plan ci-dessous (SC/loops sur bag long, SSM re-test rendu propre).

## 🚀 REPRISE ICI (nouvelle discussion) — runs des bags 3D du collègue
Contexte : guide v3 = `HOLOOCEAN_3D_GUIDE.md` (écrit par le Fable du collègue, bags
GÉNÉRÉS et PASS toutes checklists). Bags PAS ENCORE SUR CETTE MACHINE :
`Scripts_HoloOcean/BAG_files/holoocean_3d_traj{1,2}.bag` (2 × 3.6 Go, 676 s, même
trajectoire ; traj2 = + `/profiler_points`). À copier d'abord (Nathan).

Plan de runs (UN à la fois, rien committer pendant) :
1. `BAG_HOLO=<chemin>/holoocean_3d_traj1.bag ./run_slam.sh holoocean` (2D, ~11 min)
   → `./analyse.sh <run>` ; attendu : DR bon (bruits faibles), SLAM=DR si 0 loop.
2. Si OK : idem traj2 (SLAM identique) + reconstruction `/profiler_points` offline
   (adapter analysis/caves_3d.py — lecture directe du topic, points déjà en MONDE).
3. Option : mode 3D (`./run_slam.sh holoocean 3D` = /sonar_points, z porté).

⚠ PIÈGES CONNUS AVANT LE 1er RUN (vérifiés dans le code ce soir) :
- **Échelle d'intensité** : nouveaux bags = float [0,1], échos murs ≤ 0.47 (guide §3.5) ;
  le bridge transmet l'image BRUTE et `feature_holoocean.yaml` a `intensity_threshold: 95`
  (échelle 0-255 de l'ancien bag) → CFAR aveugle. FIX PROPOSÉ (à faire + vérifier au
  1er run) : dans holoocean_sonar_bridge.py callback, si `max(img) <= 1.0` → ×255 avant
  publication (garde les seuils yaml valides). Vérif : compter les features du 1er ping.
- SC `dist_threshold: 0.87` calibré sur l'ANCIEN rendu → recalibrer via loops_detected.csv
  (procédure éprouvée : viser retenus tous vrais).
- NSSM min_st_sep 25 OK pour 676 s (revisites tour 2 ≈ 338 KF d'écart) ; SSM reste off
  (re-testable SSM=true sur le NOUVEAU rendu propre — l'ICP échouait sur l'ancien).
- IMU sans accélérations propres (gravité seule, guide §3.2) ; stamps +1 s ; échos ≤0.47.
- `/profiler_points` traj2 : z>0 possibles (murs émergés) → filtre z<0 (guide §4.1).

## (fait 07-07 ap-m/soir, sessions précédentes)
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

## 🔬 JOURNAL R3 (07-07 soir) — Bruce_Sonar (ex-BSU) testée sur holoocean : 1er essai
- Renommage : méthode holoocean `bruce_sonar_usbl` → **`bruce_sonar`** (alias bs ; bsu
  accepté en legacy) — pas d'USBL simulé, le U était mensonger. Fichiers : run_slam.sh,
  holoocean.launch, README, yaml/feature commentaires.
- Runs bs (3) : ATE 0.13 m = DR à chaque fois (aucune loop au graphe). Chaîne SC :
  - ✅ détection par apparence : les SEULES candidates émises sont VRAIES (fin↔départ,
    KF56-59 ↔ KF0-2) — le descripteur SC marche même sur rendu simulateur ;
  - ✅ rétention : dist_threshold 0.70 (Aracati) rejetait tout (sc_dist 0.83-0.85) →
    recalibré **0.87** (yaml, prévu par le commentaire d'origine) → 3/3 retenues ;
  - ❌ acceptation : 0 au graphe, même avec min_pcm 4→3 (remis à 4) → le blocage est
    le RECALAGE ICP/PCM en aval — même cause racine que le SSM à 4.79 m : features
    simulateur trop pauvres. Piste close jusqu'au fix rendu sonar (guide 3D §0).
- 💡 à garder en tête (Nathan) : si on revient à la config native Bruce (IMU+DVL),
  explorer des alternatives à Sonar Context pour le front (place recognition) et le
  back-end — SC n'est peut-être pas optimal hors Aracati.

## 📐 HOLOOCEAN_3D_GUIDE v2 (07-07 ap-m) — respécifié sur directives Nathan
- v1 (558 l, code détaillé) → v2 (~120 l, OBJECTIFS + critères PASS/FAIL) : le collègue
  a Fable 5, pas besoin du code. v1 récupérable via git (commit précédent).
- Nouvelles specs : fix rendu sonar AVANT tout (arcs + « fuite » mur GAUCHE parallèle au
  robot, suspect réflexion/multipath) ; départ DANS la structure ; **« grande route » :
  roll=0 permanent, le bas regarde le bas** (fini l'hélice roll=φ de v1) ;
  **Traj 1 = pseudo-3D** (spirale en profondeur, robot à plat — 2.5D assumé, intra-msg ~0
  normal) ; **Traj 2 = vraie 3D par le SONAR** (profiler vertical recommandé, ou tilt) —
  preuve = std(z) intra-message > 0.5 m.
- Préparation vérifiée pour bags longs 2D/3D : défauts SSM off/NSSM sep25 compatibles
  (revisites > 25 s passent) ; mode 3D branché (sonar_points_bridge) ; ⚠ bridge
  RANGE_M=40/FOV=120 codés en dur ; graphe encore Pose2 (2.5D).

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

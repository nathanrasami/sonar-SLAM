# PROGRESS — 2026-07-11 — HoloOcean v1.0.0 INSTALLÉ EN LOCAL et VÉRIFIÉ (collègue indispo → autonomie)

> ⚠ **Dates** : les tags `08-08`, `09-08`, `09-09` de ce fichier valent respectivement 8, 9 et
> 9 juillet 2026. Le format `JJ-MM` s'est corrompu par recopie (preuve : `git log -S` sur les
> en-têtes). **Écrire les nouvelles sections en ISO `AAAA-MM-JJ`.**
> ⚠ **`§2.3quinquies` n'existe plus** : HOLOOCEAN_3D_GUIDE.md a été réécrit lean (129 lignes).
> La spec du sonar vertical est désormais son **§1** ; les checks sont **E1–E4** (§2).

## 🔜 REPRISE ICI — 2026-07-12 : traj6 VALIDÉ bout en bout (run 005329) — passer à la SUITE DU SLAM

**Run `run_holoocean_2026-07-12_005329` — les 3 verdicts au vert (FABLE §11-ter)** ;
bag complet **E1–E9 TOUT PASS** (E9 : lat. 34.4 % vs 0.0 % miroir · fond 83.5 % vs 0.3 %) :
- **[1] ΔATE = 0.000 m** vs traj5 (0.048 m, cap 0.05°) — SLAM insensible au 3ᵉ capteur ✓
- **[2] couverture NN 0.05/0.16 m** — la fusion contient tout ce que traj5 voyait ✓
- **[3] apport transverse 59.4 %** — carte 18 126 → **63 265 pts (×3.5)**, fond 24.8→51 % ✓
Verdict Nathan : « exactement ça que je veux ». TRAJ6_ANALYSE.md reste le mode d'emploi
pour re-analyser/comparer tout nouveau run.
**💡 À FAIRE PLUS TARD (demande Nathan 2026-07-12) : se rapprocher BEAUCOUP plus des quais**
(traj6 serre à 4-6.5 m) — traj7 potentielle, gain attendu : pilotis/échelles détaillés
(leçon §9-ter : la trajectoire fait la carte).
**Suite du SLAM (nouvelle discussion recommandée)** : loops SC traj5/traj6 · fusion patchs
polaires (StereoFLS) · threshold 50→30 (réserve) · ménage possible : traj6_test.bag (1.2 Go).

## (clos) 2026-07-12 (soir) : traj6 prêt à analyser — HANDOFF OPUS (plus d'accès Fable)

**État** : traj6 codé et vérifié (FABLE §11 ; PIEGES #16 : mount transverse MESURÉ
Rz(90)@Rx(+90), figé). **Bag TEST : E1–E9 TOUT PASS** (E9 : latéral 55.8 % vs 0.0 % miroir,
n=4553 · fond 95.4 % dans [−21,−17] vs 0.0 % flip z, n=2251 — marges nettes). **Bag COMPLET :
génération lancée nuit du 12** (gen_traj6.log ; notification « traj6 : TOUT PASS ✅ »
attendue en fin — sinon STOP).
**Nettoyage bags (demande Nathan, −34 Go, 372→339 Go)** : SUPPRIMÉS traj1-3 de bag/ (MIROIR
jamais réécrits, PIEGES #14), traj4 + traj4_test, les 2 `_avant_fix_miroir`, traj3_test.
GARDÉS : traj5 (réf du run 222233), traj5_test (témoin régression), traj6 + traj6_test,
test.bag, caves.bag, ARACATI. ⚠ Les runs ≤ 160434 ne sont plus re-analysables (artefacts
archivés seulement).
**Pour OPUS — tout est prêt, suivre `TRAJ6_ANALYSE.md`** (étapes exactes §2, références
chiffrées §3, textes d'interprétation §4, interdits §5, critères STOP §6, consignation §7) :
0. vérifier le verdict E1–E9 du bag complet ;
1. `BAG_HOLO=$PWD/BAG_files/holoocean_3d_traj6.bag ./run_slam.sh holoocean` ;
2. `./analyse.sh 3D <run>` ;
3. `python3 analysis/compare_traj6.py results/<run>` (verdicts [1] ΔATE / [2] couverture /
   [3] apport transverse — script AUTO-TESTÉ : self-compare 222233 → ATE 0.048 m,
   cap 0.07°, Δ 0.000, NN 0.00 ✓).
Puis : loops SC traj5 · fusion patchs polaires (StereoFLS) · threshold 50→30 (en réserve).

## (clos) 2026-07-11 (nuit) : traj5 ERRANCE VALIDÉE bout en bout — la trajectoire fait la carte

**Décision Nathan (nuit)** : errance naturelle façon collègue (près des structures, hauts/bas
aléatoires) puis tout lancer. **FAIT — chaîne complète holoocean→ros→analyse (FABLE §10)** :
- `gen_bag_3d_v5.py` + `gen_traj5.sh` (commit de66a1e) : errance PCHIP du collègue VERBATIM
  (±1.2 m lat / z aléatoire [−12,−2] tous les 8 m / padding cyclique) sur circuit médian
  serrant quai E 6.5 m → bateau 4 m → Γ 5 m → quai O 5.5 m SANS traverser Γ ; phase A
  calibration conservée (mêmes fenêtres → check_traj4.py tel quel).
- Bags court + complet (6957 pings/1391 s/11.1 Go) : **E1–E8 TOUT PASS** (E8 47.6 %/10.5 %).
- **Run `222233`** : ATE 0.05 m (=traj4), cap RMS 0.1° ; **carte 3D 36 704 pts (+43 %),
  NN 0.064, p90 0.693→0.261** ; fusion **M1 0.122/0.268, M2 0.034/0.112** (le doute M2 p90
  de traj4 s'est résorbé → venait de la géométrie carré, hypothèse non prouvée) ;
  carte_2d_dense_s30 : 25 455 cellules, **2 quais en échelles complètes** (le rendu voulu).
**Suite de soirée (directives Nathan, FABLE §10-bis)** : résidus 3D ÉLUCIDÉS et DÉGAGÉS —
① surface 31 % (robot à z −2.3, miroir acoustique) ; ② « coin » = FANTÔME du mur Γ rabattu
(ouverture hors-plan réelle du SonarVert ~±20° vs ±3° annoncés, PIEGES #15 ; réfuté comme
structure par le profiler traj3). Filtre anti-résidus dans `carte_3d.py` (surface + cohérence
2D + exemption fond, opt-out `--brut`) : carte 222233 = 18 126 pts PROPRE. ⚠ NN honnête
0.107/0.733 (l'ancien 0.064/0.261 était flatté par les artefacts — ne comparer qu'à contenu
égal). **traj6 « tout capter » (accord Nathan) : FAISABLE, testé** — ProfilingSonar
Azimuth=360 rend (512×720) ; cible = 3 capteurs : /sonar + /sonar_vert + profiler TRANSVERSE
360° ([90,0,90], RangeMax 20, 720 bins).

**Reste (traj6 en tête, NOUVELLE session recommandée)** :
1. gen v6 = v5 + profiler transverse 360° + /profiler_points (⚠ vérifier le signe latéral,
   E9 dédié type E8) ; 2. carte_3d : FUSIONNER vert+transverse (aujourd'hui priorité
   exclusive) + étendre l'anti-résidus au transverse ; 3. chaîne complète gen→run→analyses.
Puis : décision suppression bags `_avant_fix_miroir` + traj4 (~34 Go) · loops SC traj5 ·
fusion patchs polaires (StereoFLS) · threshold 50→30 pipeline (en réserve).

## (clos soir) 2026-07-11 : B′+D EXÉCUTÉS (fix miroir + carte dense), tout PASS

**Décision Nathan (soir)** : B′+D. **FAIT et vérifié (détail FABLE §9-ter)** :
- **D (fix miroir, racine)** : `gen_bag_3d.py` corrigé (sin + R_MOUNT_PROF flippés ensemble,
  vert prouvé identique 0.0) · **E8 anti-miroir** dans `check_traj4.py` (FAIL sur bag miroir
  10.9/33.9 % ✓ → PASS sur bag corrigé 45.2/13.6 %) · bags traj4 complet+court RÉÉCRITS
  offline, **E1–E8 TOUT PASS ×2**, anciens en `*_avant_fix_miroir.bag` (suppression = décision
  Nathan ; traj1-3 de `bag/` NON réécrits, toujours miroir) · aval : carte vert IDENTIQUE
  (NN 0.083), comblage bien placé, **fusion M1 0.296→0.205 m PASS** (⚠ doute ouvert :
  M2 p90 0.286→0.709 non investigué, M1 est le critère).
- **B′ (carte 2D dense, nouveau `analysis/carte_2d_dense.py`)** : détecteur RÉEL sur TOUS les
  pings × poses SLAM, GT-free, anti-bavure azimut (réponse aux « clouds circulaires » : bavure
  tangentielle passant le CFAR range-only). Run 160434 : **seuil 30 → 33 740 cellules**
  (24 725 persist. ≥2) vs carte keyframes 16 105 pts ; seuil 50 → 13 126. Quais/Γ/bateau lisibles.
  Lancement : voir docstring (podman exec ros1 … analysis/carte_2d_dense.py <run> [--seuil 30]).
- **Leçon traj5 (remarque Nathan VALIDÉE par le témoin 161938/traj3)** : l'errance du collègue
  (~6 m du quai OUEST, cap+z continus) sort 74 947 pts/2 quais en échelles avec la MÊME chaîne →
  **la trajectoire domine la qualité de la carte 2D**, le seuil est 2ᵉ. traj5 = errance naturelle
  + rapprochements des 2 quais + z continu, en gardant sweeps verticaux/approche bateau/phase A.

**Reste** : traj5 (ci-dessus) · décision suppression des `_avant_fix_miroir.bag` (2×11.5 Go) ·
doute M2 p90 · option pipeline A (threshold 50→30) si besoin de nourrir NSSM · fusion patchs
polaires (StereoFLS) · loops SC sur traj4.

## (clos midi) 2026-07-11 : traj4 CODÉE, bag court E1–E7 TOUT PASS — reste bag complet + runs

**Fait (vérifié par la commande nue `check_traj4.py`)** : `gen_bag_3d_v4.py` RÉÉCRIT → traj4
(guide §1) : 2ᵉ ImagingSonar VERTICAL avant (rotation [90,0,0], R_MOUNT_PROF, bras de levier
[0,0,+0.15], RangeMax 20 m/512 bins = 3.8 cm, 256 col. élévation), tilt+`/sonar_tilt`+profiler
SUPPRIMÉS, trajectoire par 60 segments (phase A C1 5.2 m/C2 360°/C3 ±3 m + transit + carré
2 tours z −4/−8, sweeps ±90° aux coins, approche bateau à 8 m au tour 2), durée 21.6 min.
Bag court `--test 150` (750 pings, 1.2 Go) : **E1–E7 TOUT PASS** — E1 std y 0.000 · E2 ±60.0° ·
E3 100 % · E4 dérive front 0.03 m · E5 0.0 ms · E6 Δr 0.02 m · E7 360° (160° sous l'eau).
Mesures + notes de terrain loggées au **guide §4** (entrée 2026-07-11).

**Découvertes de la reco monde (traj3 reprojeté, AUCUN lancement moteur — note mémoire
`pierharbor-geometrie-monde`)** : quais x=462/531.5 ; **mur interne « Γ » RÉEL que le ring
traj3 TRAVERSAIT** (→ carré traj4 rétréci x∈[490,522]) ; **bateau localisé (524,−680.5)**,
posé au fond le long du quai EST sud ; les blobs denses de /sonar_points dans la zone =
ARTEFACTS (le profiler voit le fond nu au travers).

**Pièges neufs (mesurés)** : ① le FOND ne renvoie RIEN à l'ImagingSonar (bruit pur, 50 images
moyennées) — seul le ProfilingSonar le voyait → C1 rapproché à 5.2 m, sinon E2 infaisable ;
② nappe d'échos surface à z≈+1.1 (bord +60°, ttes directions) → filtrer z<0 en aval ;
③ crash moteur mi-run possible (NVRM Xid 13, aléa GPU, 1×/3 runs) → relancer, ça reprend ;
④ python bufferise : toujours `python -u` pour un log en direct.

**✅ Bag COMPLET généré et VÉRIFIÉ (15:45)** : `BAG_files/holoocean_3d_traj4.bag` — 6486
pings, 1297 s, 10.3 Go, **E1–E7 TOUT PASS** (mesures identiques au bag court). ⚠ 2 crashs
GPU mi-run avant le fix (`NVRM Xid 13 Shader Program Header Error`, t=94/200 s, moteur
zombie + python suspendu + bag figé) → **cause = le VIEWPORT principal ; fix
`show_viewport=False`** dans holoocean.make (données sonar inchangées, E-checks ×2
identiques). Piège log : `python -u` obligatoire.

**✅ CHAÎNE AVAL COMPLÈTE VALIDÉE (16:04→16:50, run `160434` lancé par Nathan)** :
- **Run SLAM : ATE 0.04 m** (Umeyama, = DR, cap RMS 0.1°, S1-S3 ≤ 0.03, 758 KF).
  dt max 29.4 s = pause+C3 ascenseur+pause (z pur, invisible en Pose2) — PAS un drop ;
  trous 3.8-4.4 s = pauses bateau/coins. Chevauchement UE 16:11-16:19 sans effet.
- **carte_3d : chemin `/sonar_vert_points` VALIDÉ en réel du 1er coup** (le diff «125 lignes»
  était en fait déjà commité par Nathan, e2d122b) : /sonar_points plat → EXCLU ✓ ;
  vert détecté « fan VERTICAL avant, le + » (std z 6.80/y 0.00) ; carte structurelle
  25 586 pts, **NN méd 0.083 m** (p90 0.693 = bavure d'azimut du fan, limite assumée) ;
  comblage horizontal 9 632 pts. Couverture « phare » plus éparse que traj3 : assumé.
- **fusion_plus `--ap-hor 6 --ap-vert 6` : PASS** — 5220 paires, 637 avec fusion (recouvrement
  ±3° peuplé seulement quand une structure est droit devant : C1/C2/coins/bateau, attendu),
  14 252 pts fusionnés, **M1 0.296 m PASS**, **M2 0.066/0.286 m**. Hors-plans 11 % (vs 60 %
  traj2) = gates aux VRAIES ouvertures (6°) au lieu du défaut ±10°.
- Génération autonome pour la suite : **`./gen_traj4.sh [--test 150]`** (crash-retry ×3 +
  E-checks + notification bureau — commit 494de9f).

**🔎 17-20 h : « quais à peine visibles » DIAGNOSTIQUÉ (FABLE.md §9 + §9-bis)** — 3 causes
empilées, la carte ne montre RIEN de faux (blob bleu = mur Γ au mètre près ; tirets = pilotis ;
bassin réellement vide, fond muet en rasante) :
1. **Seuil calibré sur le mauvais bag** : nos images sont 15× plus faibles que test.bag du
   collègue (max méd 0.265 vs 0.776, jamais >0.49) or `filter.threshold: 50` a été calibré
   (07-03) sur SES images → coupe la bande 26-50 des pilotis (bruit p99 = 8-9 mono8 : seuil
   30 garde ×3 de marge ; émulation chaîne réelle : ×3.3 pts, quais lisibles, + de bavure).
2. **La carte du run = keyframes seulement** (758/6486 pings) : détecteur réel rejoué sur
   1 ping/3 → déjà ×3 plus dense au MÊME seuil 50.
3. **Bug générateur RÉEL : `/sonar_points` en MIROIR latéral** (sonar_to_points3d_msg,
   `y=−r·sin(a)` mais colonnes hautes = bâbord — confirmé 2× : émulation + arc Γ analytique).
   Fan VERTICAL net-correct (fond −19.4, E3/E4) → fix = flip du seul appel horizontal +
   réécriture offline des topics points. SLAM intact (n'utilise pas /sonar_points).
**DÉCISION NATHAN attendue (FABLE §9-bis)** : B′) script carte 2D dense offline (zéro
config, recommandé) · A) threshold 50→30 pipeline + re-run ×2 · D) fix miroir générateur
+ E8 anti-miroir au guide · C) 3D = couverture (traj5 sweeps/fusion), pas un seuil.
Preuves visuelles copiées dans le run : `fable_frames_polaires.png`, `fable_emul_50_vs_30.png`.

**Reste à faire (pistes ouvertes, plus rien de bloquant)** :
1. Regarder les html (carte_3d, fusion_plus) : coupes 3D du bateau (t sim 1140-1235) et des
   quais aux sweeps — juger visuellement si la couverture « phare » suffit ou si traj5 doit
   ajouter des sweeps.
2. Options qualité : fusion par images polaires (patchs, StereoFLS complet) pour désambiguïser
   les fantômes de bin ; densifier les sweeps ; loops SC sur traj4 (2 tours = revisites).
3. ⚠ pgrep se matche soi-même dans les wrappers bash : 2 guetteurs piégés aujourd'hui —
   filtrer par `ps -o comm=` (cf. gen_traj4.sh).

## (clos) 2026-07-11 (nuit) : HoloOcean local OPÉRATIONNEL — traj4 était l'étape suivante

**Fait (vérifié par le point d'entrée réel)** : `gen_bag_3d_v4.py --test 60` → PASS, bag
311 Mo, 8 topics conformes (1201 msg @20 Hz ×4, 300 @5 Hz ×4). Chaîne complète : clone
`SLAM/holoocean` (tag v1.0.0) + venv `SLAM/holoocean-venv` (py3.10, **numpy 1.26.4 obligatoire**)
+ package Ocean 3,85 Go (workaround `curl -C -` + `install(url=file://…)`, zip conservé dans
SLAM/) + les 3 fichiers du collègue commités (a5e9cd1). Détail des 6 pièges (proxy ADB, numpy 2,
timeout 60 s patché à 600 dans le venv, UE refuse root dans podman, SIGTERM ignoré → `kill -9`,
cache octree par résolution) → note mémoire `holoocean-install-local`.

**Saga crashs (4 morts SIGBUS/silencieuses puis PASS — cause jamais élucidée, NON bloquant)** :
hypothèses RÉFUTÉES par ablation : config du collègue (le scénario standard crashait aussi),
driver NVIDIA (crash identique forcé sur RADV), glibc 2.42 (crash identique dans ros1/glibc 2.31),
OOM kernel/systemd-oomd/THP (journaux vides, RAM stable, pic mesuré 2,8 Go). Les crashs ont cessé
après purge des caches octree partiels ; le build accumule ses fichiers entre runs → en cas de
récidive : relancer, ça reprend. Cache v4 = `Octrees/PierHarbor/min10_max640` (1008 fichiers,
505 Mo) désormais COMPLET → plus de phase à risque. ⚠ Fenêtre « ne répond pas » pendant un build
octree = NORMAL (thread principal bloqué). ⚠ Ne PAS lancer les scénarios de démo sonar
(PierHarbor-HoveringImagingSonar) : cache min2_max512 inutile de plusieurs Go.

**Reste à faire** : (1) modifier `gen_bag_3d_v4.py` → traj4 : remplacer tilt+profiler par le
2ᵉ sonar VERTICAL avant (`/sonar_vert`+`/sonar_vert_points` frame `auv0`, spec §1 du guide,
checks E1–E4 §2) ; (2) `--test 60` + E1–E4 ; (3) run 18 min complet (bag estimé ~5,6 Go) ;
(4) `carte_3d.py` : chemin `/sonar_vert_points` jamais exercé + diff 125 lignes NON commité
(voir mémoire `carte-3d-fan-vertical-prep`).

## (clos) 2026-07-10 (soir) : orientation 3D DÉCIDÉE, guide v3 écrit, fusion à coder

**Brainstorm Nathan** (`3D_BRAINSTORM.md`) : le « + » ne couvre pas les côtés (fan vertical fin en
azimut) → comment faire « de la 3D en toute circonstance » ? Discussion état de l'art menée.

**Décision Nathan : « + » + fusion type McConnell.** Pipeline Bruce 2D INTACT (pas de Pose3),
sonar vertical hors pipeline SLAM, et côté analyse une VRAIE fusion des 2 fans dans leur volume
de recouvrement (remplace le simple comblage).
- Craintes du brainstorm réfutées PAR NOS MESURES (pas d'opinion) : « loops rejetées à cause du
  Δz » — non (196/218 candidates à |Δz|<0.20 m échouent aussi ; verrou = ICP simu ; z=−depth
  absolu ne dérive jamais → aucune fermeture en z nécessaire) ; « il faut un SLAM 3D pour la
  carte 3D » — non (test pose GT parfaite : la carte bave pareil, réfuté 08-07).
- Problème réel identifié = **COUVERTURE volumétrique** (aucun montage fixe de fans 2D ne voit
  tout instantanément) → résolu par la TRAJECTOIRE (leçon Girona) : yaw-sweeps (le fan vertical
  balaie comme un phare) + variation z (strates du sonar SLAM sur les côtés).

**État de l'art (vérifié en ligne 07-10)** :
- ★ McConnell, Martin & Englot IROS 2020 *Fusing Concurrent Orthogonal Wide-aperture Sonar
  Images* (arXiv 2007.10407) = EXACTEMENT notre « + » ; **code `jake3991/StereoFLS`, MÊME AUTEUR
  que Bruce-SLAM**. Suite : *Large-Scale Dense 3D Mapping Using Submaps Derived From Orthogonal
  Imaging Sonars* (arXiv 2412.03760) — à lire AVANT de coder la fusion.
- Réserve : Predictive 3D Sonar Mapping ICRA 2021 (arXiv 2104.03203), inférence hors recouvrement.
- À lire par sections avec Nathan : Girona (Palomer/Ridao, sonar+laser, couverture par balayage).
- Écartés : Compact 3D Sonar (Water Linked 3D-15, arXiv 2510.18991) = autre capteur, pas de
  modèle HoloOcean ; Sonar-MASt3R ICRA 2026 = exige une caméra optique.

**Guide v3 écrit (07-10 soir)** : header « 2 changements » ; §1.1 réf. McConnell ; §1.2 stamps
appariés (E5) + ouvertures à déclarer ; **§1.5 traj4** = phase A calibration (C1 statique 10 s /
C2 yaw-sweep 360° / C3 ascenseur ±3 m) + phase B carré 2 tours z −4/−8 m, yaw-sweep ±90° aux
coins, approche bateau ±45° ; checks **E5 synchro / E6 recouvrement / E7 couverture** ; bag court
`--test 150` (contient la phase A).

**✅ 07-10 soir : `analysis/fusion_plus.py` CODÉ et VÉRIFIÉ sur traj2 (run 105410)** —
fusion à la StereoFLS adaptée aux topics de POINTS (pas d'images) : appariement des pings par
stamp (traj2 : |Δt| = 0 ms, appariement parfait), gates de recouvrement ±10°/±10° (défaut
StereoFLS ; `--ap-hor/--ap-vert` recevront les ouvertures déclarées du collègue), bins de
range 0.15 m, association gloutonne par |Δintensité| (critère FAIBLE assumé — incidences
différentes entre les 2 vues ; le bin de range fait l'essentiel). Mesures (déterministe,
×2 identiques, 9 s le run complet) :
- 2013 paires de pings (le run 105410 s'arrête à 404 s des 676 s du bag : normal),
  38 matches/ping, 76 567 pts fusionnés ;
- **60 % des points HORS des 2 plans centraux** (>2° en az ET en élév) = la vraie 3D de
  recouvrement que ni /sonar_points ni le fan vertical n'ont séparément ;
- **M1 (géométrie fusion, GT-composé vs échos bruts) : NN méd 0.138 m PASS** — p90 1.25 m
  = artefact de la référence (elle ne couvre que les 2 plans centraux ; un point fusionné
  correct hors plans en est loin par construction) ;
- **M2 (carte GT-free vs carte GT, Umeyama) : 0.024/0.038 m** ; rendu : pans de murs
  VERTICAUX nets le long de la spirale.
- ❌ `--unique` (1↔1 strict) mesuré : 23 pts, M1 0.72 m FAIL — les murs remplissent les
  bins à plusieurs échos, le strict ne garde que des échos isolés ≈ bruit. Glouton = défaut.
- Sorties : `results/run_holoocean_2026-07-07_105410/fusion_plus.{html,png,npy}`.
- Limites : fantômes possibles dans un bin (pas de patchs d'image pour désambiguïser —
  upgrade possible = images polaires comme StereoFLS) ; ouvertures réelles à caler (§4 guide).

**Reste à faire (ordre)** :
1. ✅ guide relu par Nathan, ENVOYÉ au collègue (07-10).
2. ✅ fusion codée + vérifiée (bloc ci-dessus).
3. NON VÉRIFIÉ toujours ouvert : chemin `/sonar_vert_points` en auv0 de carte_3d.py (diff
   non commité) — test = bag forgé depuis traj2. ⚠ fusion_plus a le même angle mort (testé
   via /profiler_points en map, pas /sonar_vert_points en auv0) : le bag forgé testera les 2.
4. À l'arrivée du bag : revérifier E1–E7 ici, run SLAM, carte_3d + fusion_plus
   (`--ap-hor/--ap-vert` = ouvertures déclarées au §4 du guide).

## 🔵 (précédent) 2026-07-10 matin : `carte_3d.py` préparé pour `/sonar_vert_points` (NON COMMITÉ)

**Contexte** : préparer la carte 3D « avant » pour qu'à l'arrivée du bag du collègue elle sorte
en UNE commande, sans debug. Validation possible dès maintenant car **traj2 a déjà la géométrie
x-z** (son `/profiler_points` est un fan vertical avant).

**Diff de 125 lignes sur `analysis/carte_3d.py`, sur le disque, NON commité.** Contenu :
- `/sonar_vert_points` ajouté à `TOPICS` ;
- détection du fan **x-z** en passe 1 (`std(y) < 0.05` et `std(z) > 0.5`), à côté du transverse ;
- priorité **vertical > transverse > sonar tilté** (le vertical est la source structurelle) ;
- `per_beam_max_xz()` : 1 retour fort par **élévation** `atan2(-z, x)`, bin 0.5° ;
- **GT lue AVANT la passe 1** pour dé-projeter les bags `frame_id=map` — voir le piège ci-dessous ;
- étiquette de carte fondée sur la géométrie retenue, plus sur le nom du topic (l'ancienne
  affichait « méthode grottes, 1 paroi/faisceau » alors que `per_beam_max` ne tournait pas,
  sa branche exigeant `frame_id=auv0`).

**Mesures observées** (sorties d'outil, cartes écrites dans le scratchpad, livrables non touchés) :

| | témoin (avant) | après |
|---|---|---|
| traj2 (run 105410) — NN carte-GT-free vs carte-GT | 0.057 / 0.101 m | **0.053 / 0.101 m** |
| traj2 — pts bruts → voxels | 126 811 → 17 968 | 81 191 → **14 601** |
| traj2 — colonnes verticales 0.5 m | 505 | **524** |
| traj2 — épaisseur locale des murs (ACP) | 3.6 cm (p90 6.9) | **3.4 cm (p90 6.3)** |
| traj3 (run 161938, figé) — NN, voxels | 0.057 / 0.097 m, 197 788 | **identiques** |

Enveloppe x/y/z de traj2 identique à 2 cm : `per_beam_max_xz` supprime les échos redondants **le
long du faisceau**, pas les murs. Non-régression stricte sur le chemin transverse.

**🚨 Piège de fond découvert (ce qui a coûté le plus)** : tester la géométrie d'un fan EXIGE le
**repère véhicule**. traj2 est une spirale de **720° de cap** ; sur ses points bruts (`frame_id=map`)
`std(y)` oscille de **0.00 à 8.41 m** selon le cap, alors qu'en repère véhicule il vaut **0.00
partout**. Le `std(x)=7.19 std(y)=0.00 std(z)=1.74` cité par le guide §1.1 et par ce fichier a été
mesuré sur les 60 premiers messages, au cap ≈ 0 : conclusion juste, preuve accidentelle. Le check
E1 du guide dit bien « en repère véhicule » → le collègue n'est pas induit en erreur.

**NON VÉRIFIÉ — reste à faire** : le chemin `/sonar_vert_points` en `auv0` n'a **jamais** été
exercé (traj2 fournit la géométrie, mais sous le nom `/profiler_points` et en `frame_id=map`).
Pour le prouver sans le bag du collègue : forger un bag depuis traj2 en renommant le topic et en
dé-projetant les points vers `auv0`, puis lancer `python3 analysis/carte_3d.py <run>` et vérifier
que la source retenue est bien `/sonar_vert_points`, géométrie « vertical », `/sonar_points` exclu.
Décider ensuite : commiter le diff, ou le remiser.

**Attendu du collègue** : bag avec `/sonar_vert` + `/sonar_vert_points` en **`frame_id=auv0`**
(le piège fatal est `map`, l'erreur exacte de traj2 : pose GT cuite dans les points → carte non
GT-free → inutilisable), E1–E4 tous PASS loggés au §4 du guide, bras de levier exact déclaré.

## 🔵 (précédent) — carte 3D structurelle GT-free livrée — état du 2026-07-09 au soir
**FAIT — carte 3D structurelle GT-free livrée.** Run `run_holoocean_2026-07-09_161938`
(bag traj3 FINAL corrigé, profiler boresight-bas §2.3ter). Chaîne complète, vérifiée :
- **Bag** : CHECK A/B/C revérifiés par moi (reproj pose GT) : A=0 % z>0, B=1.09M g/1.26M d,
  C=100 % sous robot → PASS. Mount `[90,90,90]`/`R_MOUNT_DOWN`.
- **Run SLAM** : 667 keyframes, **ATE 0.032 m** (RMSE 0.036), dt max 2.0 s (aucun drop),
  z peuplé −11.6→−2.3 → sain, = champion.
- **Carte** (`carte_3d.py`, méthode grottes) : profiler transverse détecté, sonar tilté exclu,
  230 589 pts, **NN carte-GT-free vs carte-GT = 5.7 cm méd / 9.8 cm p90 (Umeyama)**. Pilotis
  −19→−7 m et treillis en X NETS sur la VRAIE pose SLAM (pas GT) — vues x-z / y-z du .npy.
  Sortie : `results/run_holoocean_2026-07-09_161938/carte_3d.{html,png,npy}`.

**Méthode carte_3d.py** : (1) `per_beam_max()` = 1 retour fort/faisceau (bin azimut 0.5°) pour
`/profiler_points` transverse ; (2) détection transverse par géométrie std(x)≈0 & std(y) grand,
PAS std(z) (fond plat = z≈cst/ping → std(z) l'excluait à tort = carte VIDE, bug corrigé) ;
(3) profiler transverse présent → carte DEPUIS LUI SEUL, sonar tilté exclu (fans radiaux) ;
(4) **MIROIR y du profiler corrigé** `pts[:,1]=-pts[:,1]` ; (5) **COMBLAGE sonar horizontal** :
on ajoute les points de `pointcloud.csv` là où le profiler n'a rien à <4 m en 3D → le bateau
(profiler-aveugle) + le haut du quai apparaissent, colorés par z.

**Profiler vs sonar horizontal (complémentaires, à retenir)** : le profiler regarde vers le
bas (fan ±60°) → vraie 3D de ce que le ROV LONGE (fond + treillis des quais) mais AVEUGLE aux
structures peu profondes hors trajectoire (le bateau à 15 m/z−8..−11 est à ~73° de la verticale =
hors fan ; le profiler n'en voit que le fond dessous). Le sonar horizontal (`/sonar`, celui du
SLAM) = tranche à la profondeur du ROV, portée 40 m → voit le bateau mais en 2.5D. D'où le
comblage. Carte = profiler 3D (couleur z) + comblage sonar (bateau/tablier, 2.5D, couleur z) + traj.
Overlay orange plat d'avant = retiré (dédoublait les quais).

**🪞 Bug MIROIR y du profiler (commit 9c4bd6e) — trouvé sur retour Nathan** : les treillis
apparaissaient ENTRE les quais (or HoloOcean = rien au centre). Diagnostic : structures à
x≈+2/+38 vs quais vrais −10/+59 ; persiste sous pose GT (ni SLAM ni pose) ; un ping = mur 6 m
à DROITE du ROV alors que le quai est 6 m à GAUCHE = miroir ; négation de y → structures PILE
sur les quais. Cause = `/profiler_points` a l'axe y inversé vs convention véhicule (x avant,
y gauche). Fix analyse = négation y. Résultat : treillis SUR les quais (z −8..−18) coïncidant
en x avec le tablier orange = vraie fusion 3D, centre vide. **Cause RACINE = repère profiler
du bag (mount `[90,90,90]`) → à corriger côté générateur holoocean** (patch analyse en attendant ;
retirer si bag régénéré sans miroir). Même classe que le miroir cmd_vel (tourbillon).

**⚠ Ma faute de procédure (à ne pas refaire)** : j'ai commité 3bb9d00 + fait tourner des
analyses dans le conteneur ros1 PENDANT le run 161938 (lancé par Nathan à 16:19). Vérifié
a posteriori : aucun impact (ATE 0.032, 0 drop ; fichiers commités non chargés par le SLAM),
mais **AVANT tout commit/charge conteneur : `ps aux | grep roslaunch` pour vérifier qu'aucun
run n'est actif** (règle « un seul run à la fois »).

## 🔜 PROCHAINE ÉTAPE — 2ᵉ sonar VERTICAL avant (le « + »), spec écrite, EN ATTENTE du collègue

⚠ Le plan « A puis B » (élargir/faire tourner le profiler TRANSVERSE) est **ABANDONNÉ** : Nathan
ne veut pas cartographier ce qu'on longe ni le fond, mais **ce qu'on voit DEVANT**.

**Décision d'architecture 09-09** : ajouter un **2ᵉ sonar à l'avant, identique au sonar SLAM mais
tourné de 90° autour de l'axe d'avance** (`rotation=[90,0,0]`) → fan dans le plan **x-z** (haut/bas).
Vu de face : `−` (SLAM) + `|` (nouveau) = `+`. Son ambiguïté est en azimut (échos hors axe rabattus).
- Spec = **guide §1** (ex-§2.3quinquies, renuméroté) : topics `/sonar_vert` + `/sonar_vert_points` (**`auv0` obligatoire**) ;
  checks E1–E4 ; réalisme IRL (porteuses différentes 750 kHz/1.2 MHz → pings simultanés valides sans
  crosstalk ; bras de levier déclaré ~0.15 m ; tilt du sonar SLAM à 0) ; profiler transverse déclassé.
- **Déjà validé sur données existantes** : `holoocean_3d_traj2.bag` a cette géométrie
  (`std(y)=0.00` **en repère véhicule**, à tous les caps — les chiffres bruts `7.19/0.00/1.74`
  n'étaient vrais qu'au cap ≈ 0) → **murs verticaux nets z=−11..+6 m**. Rendu :
  `results/traj2_plus_3d.html`. Limite connue/assumée : bavure hors axe (ambiguïté d'azimut).
- 🚨 traj2 est en `frame_id=map` → **GT cuite dans les points**. Le nouveau bag DOIT être en `auv0`.
- Blocage : le collègue livre le bag. ⚠ **Périmé** : « rien à coder côté SLAM avant » — l'ajout à
  `carte_3d.py` est fait (non commité), voir §REPRISE ICI du 2026-07-10.

## ✅ 09-09 — hypothèse Δz sur les loops : RÉFUTÉE (mesure, pas opinion)

Nathan : « si on revient au même endroit à une hauteur différente, le sonar ne voit pas la même chose
→ l'ICP échoue ». Testé sur `loops_detected.csv` (runs 125954 / 111933) : **218 candidates retenues
par le gate SC, 0 acceptée** — mais **|Δz| médian 0.04 m, et 196/218 à |Δz| < 0.20 m**. Des candidates
à la MÊME profondeur échouent quand même à l'ICP. → **La profondeur n'est PAS la cause du 0/218** ;
le verrou reste l'ICP sur les features simulateur. Nuance : Δz max ici 2.76 m (pas les 9 m de traj3),
donc « grand Δz casse l'ICP » n'est pas testé — seulement « Δz≈0 ne le sauve pas ».
Rappel structurel : le SLAM est `gtsam.Pose2` (3 DOF x,y,yaw) et **`z = −depth`** (capteur de
pression) → **aucune fermeture de boucle en z, jamais**. On ne fait pas de SLAM 3D.

**Reste (autre)** : fix racine miroir y côté générateur (§2.3quater, CHECK D) → retirer le patch ;
loops SC sur traj3 ; éventuel traj3b (seed alt). ⚠ NE PAS re-tenter Pose3 (réfuté) ni filtres de
verticalité agressifs (sur-filtrage).

## ✅ 09-08 (Opus) : PRÉ-RUN traj3 transverse validé — prêt pour run + vraie carte 3D
- Bag `bag/holoocean_3d_traj3.bag` RÉGÉNÉRÉ (profiler transverse, §2.3bis). Vérifs pré-run :
  - profiler `/profiler_points` repère auv0 : **std x=0.00, y=7.83, z=15.55** → TRANSVERSE ✅
  - 8 topics présents (profiler_points, sonar_points, sonar_tilt, gt/imu/dvl/depth) ✅
  - 1 section (plan y-z, x=0.00) = profil cohérent : surface z≈12 + fond z≈−21 + mur diag ✅
  - /sonar 32FC1 max~0.05 → bridge ×255 en place (échos murs ~0.47→120 > CFAR 95) ✅
- **RUN à faire (Nathan)** : `BAG_HOLO=$PWD/bag/holoocean_3d_traj3.bag ./run_slam.sh holoocean`
- **Vraie carte 3D (phase suivante)** : adapter caves_3d.py au profiler HoloOcean transverse
  (PointCloud2, pas beams bruts) : par ping binner φ=atan2(z,y), max intensité/bin = paroi,
  projeter le long traj SLAM + overlay pointcloud_map. PAS carte_3d.py brut (mêle sonar_points
  bruités). Réutiliser caves_3d.py L68-73 (1/faisceau), L77-85 (projection transverse), L97-122 (rendu).

## 🎯 08-08 (Opus) : CAUSE RACINE carte 3D — profiler monté vers l'AVANT (pas transverse)
- Nathan pointe caves_3d.py (reconstruction propre grotte = SLAM 2D + profiler). J'y ai
  trouvé LA cause de tous mes échecs de rendu 3D sur traj3 :
  - caves SeaKing : point local (0, r·cosφ, r·sinφ) = plan TRANSVERSE y-z (⊥ mouvement)
    → sections propres qui enveloppent la traj.
  - profiler HoloOcean traj3 (mesuré) : std x=8.9 / y=0.00 / z=12.7 = plan x-z (AVANT) →
    regarde vers l'avant, voit le fond devant → FANS radiaux, pas de sections propres.
- Donc la méthode de Fable est CORRECTE ; il lui faut juste un profiler TRANSVERSE.
  Le profiler de traj3 est mal orienté → carte 3D propre IMPOSSIBLE depuis ce bag.
- Actions : demande collègue mise à jour (HOLOOCEAN_3D_GUIDE §2.3bis : profiler en y-z,
  critère std(x)≈0). Reconstruction propre = caves_3d.py directement, dès le prochain bag.
- Essais offline tracés dans _copy (démo) : extrusion (XY CFAR pointcloud_map + Z profiler,
  marche mais « brouillon » selon Nathan) ; per-beam strongest (méthode caves) réduit le
  bruit mais garde les fans du fond car profiler avant-orienté. Aucun ne remplace la vraie
  fix = capteur transverse. carte_3d.py laissé en nuage dense complet (état stable actuel).

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

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

## 14. HoloOcean : `/sonar_points` du générateur = MIROIR latéral (vécu : traj3-4, 07-11 — FIXÉ le soir même)

**FIX COMMITTÉ 07-11 soir** : `gen_bag_3d.py` corrigé (sin + R_MOUNT_PROF flippés ENSEMBLE,
projection du fan vertical prouvée identique à 0.0 près), check **E8 anti-miroir** ajouté à
`check_traj4.py` (validé : FAIL sur bag miroir, PASS sur bag corrigé), bags traj4 réécrits
offline — E1–E8 TOUT PASS, anciens conservés `*_avant_fix_miroir.bag`. ⚠ Les bags traj1-3
(`bag/`) ne sont PAS réécrits : toujours miroir. ⚠ Ne JAMAIS flipper le sin sans flipper
R_MOUNT_PROF (et inversement) : le vert n'est correct que par la compensation des DEUX.

Historique du bug : `sonar_to_points3d_msg` (gen_bag_3d.py) faisait `y = −r·sin(a)` en supposant colonnes
hautes = tribord ; MESURÉ (arc Γ à +37°/31 m, bâbord) : colonnes hautes = BÂBORD.
Tous les `/sonar_points` (horizontaux) des bags traj1→4 sont donc en miroir du cap.
- Le SLAM n'y touche pas (il CFAR-ise `/sonar` via le bridge, convention correcte) →
  cartes SLAM et ATE valides.
- Le fan VERTICAL (`/sonar_vert_points`) est net-CORRECT tel quel (fond −19.4, E3/E4
  PASS) : le signe erroné y est compensé par le montage → ne PAS « corriger » l'appel
  vertical sans re-passer E3/E4.
- Règle : ne JAMAIS conclure une géométrie depuis `/sonar_points` sans check anti-miroir
  LATÉRAL (E3 ne teste que l'élévation du fan vertical) ; une partie des « blobs
  artefacts » de traj3 était des structures miroir.
- Corollaire seuils : toute calibration aval (filter.threshold 50) faite sur test.bag
  du collègue (max méd 0.776, sature à 1.0) est invalide sur NOS bags (max méd 0.265,
  plafond 0.49) — comparer les dynamiques AVANT de calibrer (FABLE §9).

## 15. HoloOcean : l'ouverture HORS-PLAN des sonars est bien plus large qu'annoncée (vécu : traj5, 07-11)

Le SonarVert (« Elevation: 6 » = ±3° supposés hors du plan du fan) rend en réalité des échos
de structures à **~17-20° hors-plan** (mesuré : mur Γ rabattu en nappe fantôme z≈−13 sur
111 pings du corridor traj5 ; réfutation croisée par le profiler transverse traj3 = rien de
réel à cet endroit). Conséquences :
- Toute carte issue d'un fan fin SANS recoupement contient des fantômes rabattus de
  structures fortes hors-plan → filtre de cohérence 2D de `carte_3d.py` (opt-out `--brut`).
- ⚠ Une métrique NN carte-vs-GT est FLATTÉE par ces artefacts (nappes cohérentes qui se
  matchent entre elles) : 0.064 avec artefacts vs 0.107 sans, sur la MÊME carte traj5.
  Ne jamais comparer des NN à contenus différents.

## 16. HoloOcean : chaque NOUVELLE orientation de capteur a SA convention de montage (vécu : traj6, 2026-07-12)

Le mount véhicule du profiler TRANSVERSE 360° (`rotation [90,0,90]`) n'est PAS l'extension
analytique du montage vertical validé : le candidat Rz(90)@Rx(−90) (analogie de
R_MOUNT_PROF = Rx(−90)) mettait le FOND AU-DESSUS du robot. Mesuré au probe statique
((525,−662,−5) cap sud, confirmé ×2) : **R_MOUNT_TRANS = Rz(90)@Rx(+90)** — x_capteur→+y
(bâbord), y_capteur→+z (haut) ; mur quai EST à x_med 532.1 (attendu 531.5) et fond z_med
−19.7 (80.9 % dans [−21,−17]) ; flipY/flipZ échouent chacun sur leur discriminant.
- Règle : AUCUNE projection d'un nouveau capteur HoloOcean sans (1) probe statique en
  géométrie connue qui DÉTERMINE le mount (tester les 4 transformées du plan du fan :
  identité/flipY/flipZ/180°) puis (2) E-check permanent sur bag (E9 pour le transverse).
  Ne JAMAIS composer les conventions de rotation HoloOcean sur le papier.

## 17. Comparer 2 runs par INDEX de keyframe ment — toujours par TEMPS (vécu : loops SC traj6, 2026-07-12)

Deux runs du MÊME bag ne créent pas leurs keyframes aux mêmes stamps (jitter rosbag
temps réel : Δt 1-2 s au même index). Comparer les poses par index de KF a affiché
0.35 m d'écart entre deux trajectoires en réalité identiques à 2 mm (méd, interpolation
temporelle, runs 005329 vs 013055). Symptôme type : « les 2 runs divergent » alors que
l'ATE des deux est identique. Réflexe : `np.interp` sur les stamps avant tout Δ.

## 18. PierHarbor : le « bateau (524,−680.5) » n'existe PAS — structure inférée ≠ structure probée (vécu : traj7, 2026-07-12)

La reco monde (traj3 reprojeté) donnait une épave « posée au fond » (518→530, −684..−679),
devenue cible de traj4 et détour de traj5/6. Le double probe direct 2026-07-12 la RÉFUTE :
`probe_boat_traj7.py` (RangeFinderSensor, ray-cast exact — sanity fond 9.42 m ≈ 9.4 attendu,
quai O 27.5 ✓) ne voit RIEN au-dessus du fond sur toute l'empreinte, et `probe_boat_sonar.py`
(fan vertical = octree, ce qui fait foi pour le SLAM ; calibration signe sur fond connu)
mesure un fond PLAT −19.4/−19.8 sur 4 tranches, hauteur max 0.00 m. Rayons ET sonar
traversent jusqu'au quai EST (19.8 m depuis x=512 → 531.8 ≈ face 531.5). Le « bateau »
était un fantôme de reprojection (rabattus hors-plan ±20°, cf. piège 15 ; les « features
z −8..−11 » = z du ROBOT en Pose2, pas de la structure).
- Règle : une structure DÉDUITE (reprojection, features, blobs) est une HYPOTHÈSE, pas un
  fait de carte. Avant de concevoir une trajectoire autour/sous/vers elle : probe statique
  direct (RangeFinder pour la géométrie exacte + sonar pour l'octree). Conséquence traj7 :
  détour bateau supprimé, jambe quai EST droite x=529. (Le segment « bateau » de E8 dans
  check_traj4.py est inoffensif — score de prior, jamais discriminant seul.)

## 19. HoloOcean : SIGBUS moteur au premier démarrage après boot — relancer (vécu : probe traj7, 2026-07-12)

Symptôme : python suspendu, log du script VIDE, process `[Holodeck] <defunct>`, et dans
HolodeckLog.txt `SIGBUS ... UCommandCenter::GetCommandBuffer` 35 ms après l'init du serveur.
Mécanisme : course d'init sur /dev/shm — le client python crée `command_buffer` à 1 Mo
(O_TRUNC+ftruncate) pendant que le moteur mappe 8 388 608 octets ; un accès au-delà du
fichier rétréci → SIGBUS. Vu UNE fois sur ~12 démarrages, au premier lancement moteur
après reboot (timing d'init différent). Remède : tuer le python suspendu, `rm -f
/dev/shm/HOLODECK_MEM<uuid-du-run>*`, relancer — les gen_traj*.sh relancent déjà ×3 seuls.
Ne PAS « corriger » les tailles dans le venv : les runs normaux vivent très bien avec ce
mismatch (le JSON de commandes ne dépasse jamais 1 Mo).

## 20. Un test de SIGNE doit flipper LA MÊME population, jamais sélectionner APRÈS le flip (vécu : E9 traj7, 2026-07-12)

E9-fond appliquait flip-z PUIS `deep = q[:,2]<−8` : la variante « miroir » testait les
échos >8 m AU-DESSUS du robot (superstructure hors d'eau du port) au lieu de déplacer
les points du fond. Coïncidence géométrique (zw ∈ [2·z0+17, 2·z0+21]) → faux FAIL dès
que la trajectoire serre les quais (traj7 : flip 52 % ; réel : 0.0 %). Deux bags sains
plus tôt (traj6 : 0.3 %) n'avaient jamais déclenché le défaut.
- Règle : dans un test A-vs-transformé(A), figer la SÉLECTION sur les données
  originales, appliquer la transformation à cette sélection, comparer. Si la sélection
  dépend de la transformation, on compare deux populations différentes et le ratio ne
  mesure plus le signe. (Fix : check_traj4.py E9, re-validé sur traj6 ET traj7 complets.)

## 21. Un seuil d'INTENSITÉ ne se transfère pas d'un capteur/bag à l'autre — un descripteur peut être VIDE en silence (vécu : traj7r, 2026-07-13)

`sonar_context/intensity_threshold: 95` (calibré P900 Aracati) sur les bags HoloOcean traj7r
(max GLOBAL mesuré 75.5/255, max médian 16) → descripteur SONAR Context **identiquement nul sur
100 % des images** → dist cosinus exactement 1.0 partout (convention colonne vide) → **0 loop sur
toute la campagne**, en silence. Signature d'un descripteur vide : distance CONSTANTE au max
(1.0) ET shifts CONSTANTS égaux au premier de la boucle de recherche (−10,−5) — des valeurs
identiques sur toutes les paires = défaut structurel, jamais de la « sparsité ».
- Règle 1 : tout SEUIL absolu (descripteur, CFAR, carte) doit être re-calibré sur la DYNAMIQUE
  MESURÉE du bag courant (max/p99 par image), jamais hérité d'un autre capteur (cf. §14 corollaire).
- Règle 2 : au 1er run d'un descripteur sur un nouveau bag, vérifier qu'il est NON VIDE (le yaml
  le demandait — jamais fait). Un « échec de matching » doit d'abord exclure « l'entrée est vide ».
- Règle 3 : changer l'échelle du descripteur ⇒ recalibrer AUSSI son seuil de décision
  (dist_threshold 0.98, calibré sur descripteurs quasi-vides, retiendrait 100 % des fausses à
  seuil 5 — mesuré : vraies 0.11 / fausses 0.29, couper vers ~0.2).

## 22. ImagingSonar HoloOcean : AzimuthBins > ~512 STRIE l'image — sur-échantillonner un simu crée des trous, pas du détail (vécu : traj9 1024², 2026-07-15)

Passage 512×512 → 1024×1024 (« augmenter la résolution ») : E6 FAIL, first_echo nan 100 % sur
/sonar. Cause MESURÉE : à 1024 colonnes d'azimut le simulateur n'a pas assez de rayons →
**65 % de colonnes illuminées seulement, trous jusqu'à 8 colonnes** (témoin /sonar_vert 512×256 :
97 %, trou max 2). Les colonnes illuminées gardent l'intensité PLEINE (0.29 = niveau 512) —
la « dilution d'énergie ×4 » était une fausse piste (réfutée par mesure par colonne).
- Règle 1 : le gain « voir plus fin » passe par **RangeBins** (1024 → 2.0 cm/bin @20 m, pic
  range propre) ; l'azimut reste ≤512 (plein). Config validée : E1-E9 TOUT PASS, E6 |d|=0.00.
- Règle 2 : avant d'adopter une résolution de capteur simulé, mesurer la fraction de
  colonnes/lignes illuminées sur une cible connue (10 lignes numpy) — des stries décimeraient
  aussi les features CFAR du SLAM, pas seulement les E-checks.
- Règle 3 : deux mesures qui se contredisent (max fenêtre 0.053 vs pic profil 0.267) = investiguer
  par COLONNE avant tout verdict — ici les deux étaient justes, l'image était striée.

## 23. Ctrl-C sur un wrapper gen_traj*.sh NE tue PAS la génération — python orphelin + patch inopérant (vécu : ×2, 2026-07-15)

Le wrapper lance le python en `&` : Ctrl-C tue le wrapper, le python devient ORPHELIN et
continue d'écrire le bag (vu 12+ min après l'arrêt), avec moteur zombie + shm HOLODECK_* actifs
(→ BusyError au boot suivant). Après TOUT arrêt manuel : `pgrep -af gen_bag_3d` → kill, pkill
moteur, `rm /dev/shm/HOLODECK_*`, et SUPPRIMER le bag partiel (Writer non fermé = invalide ;
un script « skip si présent » le croirait bon).
- Corollaire 1 : éditer un module python PENDANT un gen est INOPÉRANT pour le process en cours
  (module importé en RAM) — un gen relancé avant un patch produit l'ancienne config en silence
  (vécu : 40 % de traj5 strié à jeter).
- Corollaire 2 : `pkill -9 -f <mot>` peut matcher TA propre ligne de commande (echos compris) →
  shell tué à mi-nettoyage. Motifs par concaténation : `P='Holo''deck'; pkill -f "$P"`.

## 24. Bruit image simulé : p99(bruit) doit rester SOUS le seuil d'intensité du détecteur, sinon INONDATION (vécu : noise round 2 ×5, 2026-07-16)

AddSigma 0.05 (×5) sur les 3 sonars → la queue du bruit dépasse `filter.threshold: 30` :
mesuré à la source (bags `_noise_test`) **6.4-6.8 % des pixels ≥ 30 (p99 = 40)** contre
0.09-0.11 % round 1 (p99 = 7.9, échos max ~65). Conséquence en cascade : nuage ×40-90
(1.6-2.2 M pts = blob illisible), nœud SLAM en retard → **KF perdus** (623/837 traj9 B),
ICP sur du bruit → 363 fausses contraintes, ATE origine 67 m. Les 4 runs ×5 : inexploitables
(archive 2 images/run : `results/noise_x5_archive/`).
- Règle 1 : avant de générer un bag bruité, calculer p99 ≈ 2.33·σ_add·255 et le comparer au
  seuil du détecteur aval (30). σ=0.02 → p99 ≈ 12 : OK. σ=0.05 → p99 ≈ 30 : inondation.
- Règle 2 : 10 lignes de numpy sur le bag test (% pixels ≥ seuil vs témoin round 1) AVANT
  les 2×25 Go et les 4 runs. C'est le même réflexe que PIEGES #22 règle 2.
- Règle 3 : « l'effet doit être visible sur le DR » ne passe PAS par L1 (l'image sonar ne
  touche pas le DR) — c'est L2/L3. L1 trop fort détruit l'outil de MESURE (la carte), pas
  la nav. ⚠ /sonar = 32FC1 (floats 0-1), seuil mono8 30 ≡ 0.118 — lire uint8 = mesure fausse.

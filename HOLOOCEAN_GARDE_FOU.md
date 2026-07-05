# HOLOOCEAN_GARDE_FOU.md — manuel de survie et de modification du pipeline HoloOcean

> **Pour qui** : Nathan + un assistant moins contextualisé (Opus) qui devra modifier ce
> pipeline sans avoir vécu les découvertes du stage. **Quand** : dès qu'il faut toucher
> au code holoocean — nouveau format de bag 3D, changement de source d'odométrie
> (imu/dvl → odométrie type Aracati, avec ou sans USBL), nouvelle méthode SLAM.
> **Règles d'or absolues** : lire `PIEGES.md` AVANT tout ; UN changement par run ;
> valider chaque étape avec les checklists de ce document ; ne jamais « corriger » un
> signe/repère sans passer par la checklist chiralité (§6.1) — c'est LE bug qui a coûté
> trois semaines sur Aracati.

---

## 1. Carte du pipeline (qui publie quoi, dans quel fichier)

Lancement : `./run_slam.sh holoocean [2D|3D] [Bruce|Bruce_Sonar_USBL]` (défauts 2D Bruce)
→ `bruce_slam/launch/holoocean.launch` avec `mode` et `method`.

```
bag test.bag ──/sonar (Image polaire)──► holoocean_sonar_bridge.py ──OculusPingUncompressed──►
     │                                        feature_extraction_node (callback POLAIRE :
     │                                        CFAR + [méthode BSU : descripteur SONAR Context])
     │                                              │ SONAR_FEATURE_TOPIC (PointCloud2 features)
     │                                              │ SONAR_DESCRIPTOR_TOPIC (si BSU)
     ├──/sonar_points (PointCloud2 3D)──► sonar_points_bridge.py  [mode 3D SEULEMENT :
     │                                    court-circuite image→CFAR, I≥intensity_min,
     │                                    voxel 0.5 m, packing (x, I, ±y) param ~flip_y]
     │
     ├──/dvl /imu /depth ──► dvl_imu_odom.py ──PoseStamped (z = −depth)──►
     │                       ODOM_BRIDGE_INPUT_TOPIC ──► odom_bridge_node.py ──►
     │                       LOCALIZATION_ODOM_TOPIC (nav_msgs/Odometry)
     │
     └──/ground_truth ──► gt_odom_to_pose.py ──► /pose_gt  [ÉVAL SEULEMENT : slam_ros
                          le bufferise → groundtruth.csv ; AUCUN calcul ne le lit]

slam_node.py (UN SEUL actif, gardé par if method=='...') :
  ApproximateTimeSync(features, odom) → keyframes → ICP séquentiel → [SSM] →
  NSSM (détection = covariance native OU SONAR Context selon la méthode) →
  shgo → ICP+cov → PCM → iSAM2 (Pose2) → export CSV au shutdown (required=true du rosbag)
```

Fichiers de config : `feature_holoocean.yaml` (CFAR + bloc sonar_context du descripteur),
`slam_holoocean.yaml` (keyframes, SSM/NSSM, bloc sonar_context du matching, usbl OFF).
Sorties : `results/run_holoocean_<date>/` → `./analyse.sh <run>` (figures AUTO-3D si
std(z) > 0.2 : bilan_run, traj_on_cloud colorent par z). Lexique des fichiers de sortie :
`TESTS.md` §2.5.

## 2. Les INVARIANTS — à ne JAMAIS casser, quelle que soit la modification

1. **Chiralité** (PIEGES §1) : le y latéral des features et le repère de l'odométrie
   doivent avoir la MÊME orientation (repère direct des deux côtés). Symptôme si cassé :
   carte « tourbillon » (arcs en virage), loops quasi nulles, TRAJECTOIRE POURTANT
   CORRECTE — c'est un bug silencieux. Test : §6.1.
2. **Les stamps doivent correspondre** : le SLAM associe features↔odom par
   ApproximateTimeSynchronizer (0.5 s) et features↔descripteur SC par égalité EXACTE de
   (sec, nsec). Tout bridge doit recopier `header.stamp` du message source — jamais
   `rospy.Time.now()` (le bag tourne en sim_time, souvent à rate ≠ 1).
3. **Les fenêtres NSSM comptent des KEYFRAMES, pas des mètres** (PIEGES §11) :
   `min_st_sep`, `pcm_queue_size`, `source_frames`. Si tu changes
   `keyframe_translation`, rescale ces trois fenêtres du même facteur — sinon
   auto-appariements court-terme validés comme loops (vécu : ATE 1.88 → 17.17 m).
4. **Jamais de détecteur de loops sans porte géométrique** (PIEGES §12) :
   tout candidat doit passer un gate de distance dans l'estimé AVANT shgo/ICP
   (vécu : 583 candidats non gatés → PCM pollué + CPU saturé).
5. **`/pose_gt` et `/ground_truth` ne servent QU'À l'évaluation** (TESTS.md §2.6).
   Toute nouvelle fonctionnalité qui les lit dans un chemin de CALCUL casse le
   statut GT-free du stage. Les seuls capteurs autorisés : sonar, dvl, imu, depth,
   (cmd_vel, usbl sur Aracati).
6. **Un seul nœud SLAM actif** : les groupes sont gardés par `if method=='...'` —
   deux nœuds `name="slam"` simultanés = comportement indéfini ROS.
7. **`required="true"` sur rosbag_play** : c'est LUI qui déclenche l'export CSV
   (rospy.on_shutdown). Si tu le retires, plus de CSV → run perdu (PIEGES §5).
8. **`SLAM_RESULTS_DIR`** est hérité de run_slam.sh : lancer roslaunch à la main sans
   lui écrit dans `results/` en vrac.

## 3. RECETTE A — le bag 3D du collègue arrive avec un format DIFFÉRENT

Le contrat attendu (HOLOOCEAN_3D_GUIDE.md) : `/sonar_points` = PointCloud2 champs
`x,y,z,intensity`, repère capteur x=avant/y=gauche/z=haut, `std(z) > 0.5 m`.
Si ce n'est pas ça, adapter DANS L'ORDRE :

### A.1 Topics renommés
`rosbag info le_bag.bag` → si les noms diffèrent, ne PAS renommer dans le code :
ajouter des `<remap>` dans holoocean.launch (ex.
`<remap from="/sonar_points" to="/oculus/points"/>` sur le nœud sonar_points_bridge).

### A.2 PointCloud2 avec d'autres champs / layout
Tout se joue dans `bruce_slam/scripts/sonar_points_bridge.py` :
```python
pts = np.array(list(pc2.read_points(msg, field_names=("x","y","z","intensity"), ...)))
```
- Diagnostiquer d'abord : `rostopic echo -n1 --noarr /sonar_points | head` et lister
  `msg.fields` (un print suffit). Champs sans intensité → remplacer par une constante
  255 (le SLAM veut un packing `[x, I, ±y]`, cf. ci-dessous). Champ `intensity` nommé
  autrement (`i`, `reflectivity`) → adapter `field_names`.
- **Le contrat de sortie du bridge, NON NÉGOCIABLE** : PointCloud2 publié sur
  SONAR_FEATURE_TOPIC, packé `[x_avant, INTENSITÉ, y_latéral_signé]` (le SLAM lit
  x et « −z » du message = y latéral — convention héritée d'Aracati). Le stamp = celui
  du message source (invariant §2.2).
- Régler `~intensity_min` (50 par défaut) et `~voxel` (0.5 m) selon la densité : le
  SLAM veut quelques CENTAINES de points significatifs par ping, pas 120 000.

### A.3 Le sonar n'est ni PointCloud2 ni Image (message custom)
Écrire un nouveau bridge sur le modèle de `sonar_points_bridge.py` (60 lignes).
Il suffit de produire le contrat A.2. Si le message est une image polaire d'un autre
format : viser plutôt `holoocean_sonar_bridge.py` (Image → OculusPingUncompressed) —
attention, OculusPingUncompressed n'a PAS de champ num_beams et les bearings sont en
**centi-degrés** (vécu : AttributeError au premier essai, cf. mémoire du 07-03).

### A.4 Conventions de repère différentes (NED, z vers le bas, y droite)
Symptômes → cause → fix :
- carte en miroir/tourbillon, loops mortes → chiralité y → `~flip_y` du bridge (§6.1) ;
- z négatif quand le robot monte → z NED → inverser au bridge (comme dvl_imu_odom
  fait `z = −depth`) ;
- `det(R) = −1` affiché par paper_eval → l'ÉVAL a absorbé une réflexion : la traj est
  évaluable mais le repère est indirect quelque part → remonter à la source (bridge),
  ne PAS “corriger” en aval ;
- cap qui tourne à l'envers (fit `s=−1` inattendu dans bilan_run) → convention
  compas/NED : lire le décodeur de conventions dans FABLE.md §1.

### A.5 Checklist de validation (après TOUTE adaptation de format)
1. `rostopic hz` sur SONAR_FEATURE_TOPIC pendant le run : ~cadence sonar, pas 0.
2. Fin de run : `python3 - <<'P'` → `std(z)` de pointcloud.csv **> 0.5 m** (sinon le
   bag n'est pas vraiment 3D — rester en 2D).
3. `./analyse.sh <run>` : les figures passent en 3D TOUTES SEULES (nuage coloré z).
   Figures plates = pas de relief = retour au point 2.
4. Aucune loop avant la première revisite RÉELLE de la trajectoire (regarder
   `nssm_constraints` dans trajectory.csv vs la géométrie du parcours — invariant §2.3).
5. Carte cohérente vs `/ground_truth` (cloud_vs_gt de paper_eval) : méd < 0.2 m en simu.
6. Référence 2D à retrouver AVANT de tester 3D : dvl, bag actuel → **ATE 0.13 m** ;
   si le 2D ne reproduit plus 0.13 ±0.02, la modification a cassé autre chose.

## 4. RECETTE B — passer d'imu/dvl à une ODOMÉTRIE type Aracati

### B.1 Sans USBL (odométrie seule)
Le SLAM ne demande QUE : `nav_msgs/Odometry` sur LOCALIZATION_ODOM_TOPIC, stamps du
bag, repère DIRECT, origine libre (l'éval aligne par Umeyama, cf. mini-papier §6.2a).
Deux chemins :
- le simulateur/bag fournit déjà une odométrie → un simple relay/bridge (modèle :
  `gt_odom_to_pose.py` à l'envers, ou publier PoseStamped sur ODOM_BRIDGE_INPUT_TOPIC
  et laisser `odom_bridge_node.py` faire l'Odometry) ;
- intégrer des consignes/vitesses (équivalent /cmd_vel) → PORTER
  `src/bruce_slam/cmd_vel_odom.py` de la branche Bruce (120 lignes, unicycle,
  cf. BRUCE_SLAM.md §3.1 pour la formule et le code). Seed (0,0,0) suffit.
Puis : nouveau `odom_source` dans holoocean.launch (dupliquer le groupe dvl) +
`ODOM_SOURCE=xxx` dans run_slam.sh. ⚠ Refaire la checklist chiralité §6.1 : une
odométrie qui tourne « à l'envers » du sonar reproduit EXACTEMENT le bug tourbillon.

### B.2 Avec USBL (ancrage absolu type Aracati)
Ce qu'il faut : un topic `geometry_msgs/PointStamped` de fixes de position absolue
dans un repère cohérent avec l'odométrie. Alors :
1. `slam_holoocean.yaml` : bloc `usbl` → `enable: True`, `sigma` à CALIBRER (voir 3),
   `max_dt: 1.0`, `max_speed` selon le véhicule ; le nom du topic est `/usbl_point`
   (remap sinon).
2. ⚠ PIEGES §2 : l'ancrage se fait au BACK-END uniquement — ne jamais activer en
   même temps une fusion USBL dans l'odométrie (double ancrage = zigzag, vécu
   1.45 → 4.66 m).
3. Calibrer sigma : la méthode complète est dans TESTS.md/ULTIME.md U6 — extraire les
   fixes + GT du bag (script type rosbags, cf. `traj_eval.odometrie_pure_depuis_bag`
   comme modèle de lecture), résidu fixes-vs-GT par fenêtre de 5 min, sigma ≈ médiane
   globale. Sur Aracati : bruit réel 0.87-3.09 m selon la zone, σ fixe médian gagne
   (le σ adaptatif MAD a été testé et REJETÉ — RU5, ATE 1.62 vs 1.47 ; le code existe
   sur BSU si on veut retenter avec `adaptive_min` ≥ σ champion).
4. Pas de capteur USBL dans le simulateur ? En fabriquer un pour tester la chaîne :
   nœud de 30 lignes qui écoute `/ground_truth`, publie position + bruit gaussien
   (σ 1-2 m) + outliers (1 % à 50 m) + dropouts (silences 30-120 s) à ~0.6 Hz sur
   `/usbl_point`. C'est un capteur SYNTHÉTIQUE : à déclarer comme tel, il ne rend pas
   le run « GT-free » (le vrai test attendra un capteur simulé par HoloOcean).
5. Leçon transversale à respecter (mesurée 5×, TESTS.md §2.0) : **le σ optimal dépend
   du pipeline** — raide (~bruit médian) avec loops SC, doux (×1.8) avec SSM/NSSM
   natifs. Ne pas copier-coller le 1.4 d'Aracati sans mesurer.

## 5. RECETTE C — ajouter/porter une MÉTHODE (le point d'extension `method`)

Modèle : le portage de `bruce_sonar_usbl` (07-05). Étapes exactes :
1. `holoocean.launch` : dupliquer le nœud SLAM avec
   `if="$(eval arg('method') == 'ma_methode')"` + ses `<param>` spécifiques
   (et, si la méthode a besoin d'un producteur en amont — ex. descripteur SC sur le
   nœud feature — un param conditionné par method).
2. `run_slam.sh` : ajouter le nom au `case "$METHOD"`.
3. La méthode DOIT respecter : la porte géométrique avant ICP (§2.4), le PCM en aval
   (ne jamais insérer une loop sans validation), le journal `loops_detected.csv`
   (colonnes source_key,target_key,…,retenu[,detector]) pour l'audit offline.
4. Valider : d'abord `method=bruce` INCHANGÉ (non-régression, ATE 0.13), puis la
   nouvelle méthode, puis comparer avec `paper_eval`.

## 6. Checklists de diagnostic

### 6.1 Checklist CHIRALITÉ (le bug à 3 semaines — 5 minutes pour l'écarter)
1. Run court, puis `./analyse.sh <run>` → regarder `traj_on_cloud.png` : les murs
   doivent être du BON côté de la trajectoire (comparer au parcours connu).
2. Symptômes positifs : structures en arcs « tourbillon » aux virages, NN cloud élevé,
   0-6 loops alors que la trajectoire revisite → inverser le y latéral AU BRIDGE
   (`~flip_y`) — jamais en aval — et re-runner.
3. Preuve formelle si doute : re-rendre le cloud aux poses GT (`paper_eval` le fait) —
   si le cloud GT-rendu est net et le nôtre en arcs, c'est la pose→scan ; si LES DEUX
   sont en arcs, c'est le scan lui-même (bridge).
   Rappel maths : rendre avec le mauvais signe = R(θ)M = MR(−θ) → scans peints au cap
   OPPOSÉ (BRUCE_SLAM.md §3.2).

### 6.2 Checklist « le run est-il valide ? »
- s'est arrêté SEUL (fin de bag) et a écrit les CSV ;
- `nssm_constraints` : 0 loop avant la 1ʳᵉ revisite géométrique réelle ;
- `bilan_run.png` : ATE dans l'attendu (2D dvl : 0.13 ±0.03 ; toute valeur > 0.5 sur
  ce bag = régression) ; cap méd ~0° en simu ;
- figures 3D ssi le bag a du relief ;
- comparer au run de référence archivé : `TESTS_image/run_holoocean_2026-07-05_010436_1`.

### 6.3 Où chercher quand ça casse
| Symptôme | Suspect n°1 | Voir |
|---|---|---|
| tourbillon / miroir | chiralité bridge | §6.1, PIEGES §1 |
| 0 feature au SLAM | stamps (Time.now au lieu du bag) ou sync 0.5 s | §2.2 |
| 0 loop en méthode BSU | descripteur pas publié (param feature) ou seuil SC inadapté | §7 |
| loops dès t=0 | fenêtres NSSM vs densité keyframes | PIEGES §11 |
| ATE explose avec USBL | double ancrage ou flip_y | PIEGES §2, §4.B.2 |
| crash OculusPing | num_beams/centi-degrés | §A.3 |
| CSV absents | required=true retiré ou kill -9 | §2.7 |

## 7. Calibration SONAR Context sur un NOUVEAU sonar (obligatoire)

Les seuils actuels (`dist_threshold: 0.70`, `gate_distance: 10`, `intensity_threshold:
95`) sont calibrés sur le **P900 d'Aracati** — ils ne se transfèrent PAS d'office.
Procédure (celle qui a donné 0.60→0.70 sur Aracati, cf. TESTS.md) :
1. Premier run méthode BSU avec les seuils actuels → `loops_detected.csv`.
2. Offline : pour chaque candidat, distance GT entre les deux keyframes (via
   groundtruth.csv) → vrais (< ~1/3 du gate) vs faux. Tracer sc_dist des vrais vs faux.
3. Choisir le seuil qui garde des retenus TOUS vrais (le PCM nettoie le reste) ;
   si le descripteur ne sépare pas (AUC ~0.5), régler `intensity_threshold` du
   descripteur (image sim trop propre/trop saturée → contexte uniforme, vécu sur P900).
4. Le gate : ~2× l'erreur d'odométrie attendue entre revisites, jamais 0.

## 8. La vraie 3D (Pose3) — GO/NO-GO et chemin balisé

- **NO-GO tant que** : `std(z)` des points sonar < 0.5 m (bag plat) OU le véhicule
  navigue à profondeur ~constante (le 2.5D actuel suffit : z = −depth porté partout).
- **GO** : trajectoire hélicoïdale du collègue + vrai balayage vertical. Chantier :
  `slam.py`/`slam_ros.py` Pose2→Pose3 (facteurs, ICP 3D, covariances 6×6, PCM en SE(3)),
  détaillé dans SLAM_3D_MIGRATION.md. Ordre conseillé : d'abord valider le mode 3d
  actuel (nuages 3D, graphe 2.5D), ensuite seulement ouvrir Pose3 — jamais les deux
  changements dans le même run (règle d'or n°2).

## 9. Références croisées

`PIEGES.md` (les 12 pièges vécus) · `TESTS.md` partie 2 (tous les chiffres de référence
+ lexique des sorties §2.5 + audit GT-free §2.6) · `SLAM_3D_MIGRATION.md` (pipeline
unifié 2D/3D + chantier Pose3) · `HOLOOCEAN_3D_GUIDE.md` (génération du bag 3D côté
collègue) · `BRUCE_SLAM.md` branche Bruce (avant/après code des modifications) ·
`Paper/MiniPapier/MINI_PAPIER.md` §6.2 (protocole d'évaluation expliqué).

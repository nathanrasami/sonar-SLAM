# DATASETS.md — au-delà d'Aracati2017 : quels datasets UV pour Bruce-SLAM ?

> Critère d'entrée NON NÉGOCIABLE : un **sonar imageur** (FLS multibeam ou MSIS à
> balayage) + une **source d'odométrie** (DVL/IMU ou équivalent), idéalement en ROS bag.
> Sans sonar, Bruce-SLAM n'a rien à manger. Audit du 2026-07-05.

## Verdicts sur les 3 candidats proposés

| Dataset | Capteurs | Format | Verdict |
|---|---|---|---|
| **CIRS Underwater Caves** (UdG) — https://cirs.udg.edu/caves-dataset/ | **MSIS Tritech Micron** (horizontal) + profiler SeaKing + DVL LinkQuest + 2 IMU + pression + caméra | **ROS bag 395 Mo** (topics STANDARDS : LaserScan/Imu/Odometry) + CSV | ✅ **GO — branche `caves` câblée** |
| Aqualoc (LIRMM) — https://www.lirmm.fr/aqualoc/ | caméra mono + IMU MEMS + pression — **AUCUN sonar** | ROS bag | ❌ dataset **visuel**-inertiel : rien pour Bruce |
| ACFR marine (Sydney) — http://marine.acfr.usyd.edu.au/datasets/ | stéréo + multibeam **bathymétrique** (DeltaT vers le bas) | images + ASCII/CSV, pas de bags | ❌ pas de sonar imageur ; portage énorme pour rien |

## Autres candidats repérés (si on veut élargir plus tard)

- **Bags des auteurs de Bruce-SLAM** (upstream jake3991/sonar-SLAM — le README pointe
  leurs bags BlueROV2/Oculus M750d) : compatibilité **native** (zéro bridge), parfait
  pour un sanity-check « le pipeline upstream tourne chez nous ». Effort ≈ 0.
- **Aracati2014** (même club nautique, BlueView P900-130, https://github.com/matheusbg8 —
  le prédécesseur d'Aracati2017) : notre chaîne aracati marcherait quasi telle quelle
  (mêmes conventions) — un « 2e dataset » à moindre coût si les topics matchent.
- Portoroz 2025 (ISOPoT, Oculus 750d) : séquences décrites dans le papier mais **non
  publiées** à ce jour. KRISO (Sonar Context) : non public.

## ✅ La branche `caves` (créée depuis `holoocean`)

**Pourquoi ce dataset est le bon** : grotte réelle (l'environnement le plus dur et le
plus intéressant), capteurs 100 % embarqués (GT-free par construction), topics ROS
STANDARDS (aucun package custom), 395 Mo, et un dataset de référence dans la littérature
MSIS (le papier de loop closure ULCDfMS de `Paper/Loop/` travaille dessus).

**Le câblage** (préparé, à valider au premier bag) :
- `bruce_slam/scripts/msis_scan_bridge.py` : le Micron publie **1 faisceau par message**
  (`/sonar_micron_ros`, LaserScan : angle dans angle_min, profil d'écho dans
  intensities[]) → assemblage en TOURS complets → image polaire (bins × beams) →
  `OculusPingUncompressed` → chaîne feature polaire existante. ⚠ v1 sans compensation
  de mouvement pendant le balayage (vitesse grotte ~0.2 m/s : tolérable ; sinon TODO n°1,
  compensation DVL façon ULCDfMS).
- Odométrie : le dead-reckoning DVL+IMU **fourni par le dataset** (`/odometry`,
  nav_msgs/Odometry) relayé tel quel. Alternative maison (intégrer /dvl_linkquest +
  /imu_adis_ros) : recette HOLOOCEAN_GARDE_FOU.md §4.B.1.
- `caves.launch` + `feature_caves.yaml` / `slam_caves.yaml` (base holoocean, seuils CFAR
  et keyframes à calibrer au 1er run) + cible `run_slam.sh caves` + case `analyse.sh`.
- Méthodes : `./run_slam.sh caves Bruce` ou `./run_slam.sh caves Bruce_Sonar_USBL`
  (loops par apparence ; seuils SC à recalibrer, GARDE_FOU §7).
- ⚠ **Évaluation** : la GT est ÉPARSE (cônes revus par la caméra à quelques instants) —
  pas d'ATE continu en v1. On juge d'abord la CARTE (traj_on_cloud, cohérence de la
  galerie, loops aux revisites) ; l'ATE aux cônes viendra des CSV du dataset
  (full_dataset.zip) si besoin.

## 📥 À TÉLÉCHARGER (Nathan) — et où le mettre

Depuis https://cirs.udg.edu/caves-dataset/ :
1. **`full_dataset.bag` (395 Mo)** — tous capteurs sauf caméra. → le renommer/poser en
   **`caves.bag` À LA RACINE du dépôt** (à côté d'ARACATI_2017_8bits_full.bag).
   Autre chemin possible : `BAG_CAVES=/chemin/xxx.bag ./run_slam.sh caves`.
   ⚠ PIEGES §10 : ne jamais suivre un bag dans git (les *.bag sont ignorés — vérifier).
2. **`full_dataset.zip` (38 Mo)** — les CSV texte (dont ce qui sert de GT aux cônes).
   → le dézipper dans **`caves_txt/` à la racine** (dossier à créer, ignoré par git).
3. PAS besoin des fichiers caméra (sparus_camera.bag 3.3 Go, frames*.zip) pour Bruce.

## 🎬 Procédure premier contact (dans l'ordre, cf. GARDE_FOU)

```bash
git checkout caves
python3 analysis/inspect_bag.py caves.bag          # 1) VALIDER le format supposé
#    → vérifier : /sonar_micron_ros = LaserScan 1 faisceau/msg ? combien de bins ?
#      range_max ? cadence ? /odometry présent ?
./run_slam.sh caves                                # 2) 1er run méthode Bruce
./analyse.sh 3D run_caves_<date>                   # 3) carte + carte 3D interactive
#    → checklist chiralité (GARDE_FOU §6.1) sur carte_finale/traj_on_cloud !
```

Si l'inspection contredit une hypothèse du bridge (format LaserScan, angles, bins),
tout est paramétré : `~scan_topic`, `~range_m`, `~min_coverage_deg`, `~intensity_scale`
dans `msis_scan_bridge.py` — et le GARDE_FOU §3 donne la marche à suivre.

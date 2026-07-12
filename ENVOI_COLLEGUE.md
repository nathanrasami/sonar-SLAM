# Envoi collègue — pilotage MANUEL de l'UV sur son HoloOcean (2026-07-12)

But : Nathan pilote le HoveringAUV au clavier sur le PC du collègue (HoloOcean
stable là-bas) et enregistre un bag 3 capteurs compatible avec notre chaîne SLAM.

## Fichiers à envoyer (7)

1. `gen_bag_3d.py`
2. `gen_bag_3d_v5.py`
3. `gen_bag_3d_v6.py`
4. `gen_bag_3d_v7.py`
5. `pierharbor_zone.json`   (requis : v5 le charge à l'import)
6. `check_traj4.py`
7. `HOLOOCEAN_3D_GUIDE.md`
8. `gen_bag_3d_v8.py`   (OPTIONNEL : modèle d'erreurs nav réaliste, voir prompt)

(+ ce document. Ne PAS envoyer de bag ni le venv — dépendances python :
holoocean 1.0.0, rosbags, numpy, scipy.)

## Prompt pour l'IA du collègue (copier-coller tel quel)

> Contexte : ce PC fait tourner HoloOcean 1.0.0 (monde PierHarbor, package
> Ocean) de façon stable. Les fichiers `gen_bag_3d*.py`, `pierharbor_zone.json`,
> `check_traj4.py` et `HOLOOCEAN_3D_GUIDE.md` viennent d'une chaîne SLAM validée
> bout en bout (bags « traj7 »).
>
> Objectif : écrire `manual_drive.py` pour que l'utilisateur PILOTE MANUELLEMENT
> le HoveringAUV (clavier : avance/recule, latéral, monte/descend, lacet) et
> ENREGISTRE un rosbag1 IDENTIQUE aux nôtres : topics /imu /dvl /depth
> /ground_truth /sonar /sonar_points /sonar_vert /sonar_vert_points /profiler
> /profiler_points, mêmes types/stamps (DT=0.05, sonar 5 Hz, profiler 2 Hz).
>
> À RÉUTILISER TEL QUEL (ne pas réécrire) : la config capteurs
> `make_cfg`/`make_cfg_v6` (v5/v6 ; RangeMax horizontal 20 m = variante v7 via
> `v5.RANGE_MAX = 20.0` comme dans gen_bag_3d_v7.py) et toute la boucle
> d'écriture du bag de `gen_bag_3d_v6.main` (writer rosbags, sérialisation,
> `sonar_to_points3d_msg` avec ses `R_MOUNT_PROF`/`R_MOUNT_TRANS`).
> SEUL changement : remplacer la pose planifiée (`v5.pose_at_v5` + `teleport`)
> par le contrôle natif (`control_scheme` du HoveringAUV + `env.act()`/`tick()`,
> clavier via pygame ou pynput). IMU/DVL/depth/GT doivent alors venir des
> CAPTEURS HoloOcean (ajouter IMUSensor/DVLSensor/DepthSensor/PoseSensor au cfg)
> et non de la trajectoire planifiée. `show_viewport=True` est OK sur ce PC.
>
> Validation OBLIGATOIRE à la fin :
> `python check_traj4.py <bag> --rmax-h 20` (ou sans le flag si RangeMax 40).
> Les checks E2/E6/E7 supposent une phase A de calibration au début : ~10 s
> statique face au mur EST à ~5.2 m depuis (526.3, −660, −9.5), puis 360° sur
> place, puis ±3 m vertical — soit la refaire en début de pilotage, soit ignorer
> ces 3 checks (E1/E3/E4/E5/E8/E9 restent valides).
>
> Option « navigation réaliste » (recommandée pour les bags destinés à tester
> les fermetures de boucle) : reprendre de `gen_bag_3d_v8.py` le modèle
> d'erreurs nav (`psi_err_series` : biais de cap 2° + marche aléatoire
> 0.15°/√s appliqués à l'ORIENTATION du message /imu ; `dvl_mismount` :
> échelle +0.5 % + désalignement 0.5° sur /dvl) — l'appliquer aux valeurs
> AVANT écriture dans le bag, GT intact. Sans cette option le dead-reckoning
> simulé est quasi parfait et le SLAM n'a rien à corriger.
>
> Pièges durement acquis — NE PAS « corriger » : les rotations capteurs
> ([0,0,0] / [90,0,0] / [90,0,90]) et les matrices R_MOUNT_* sont MESURÉES au
> probe, pas déduites ; `sonar_to_points3d_msg` contient déjà le fix du bug
> miroir latéral ; il n'y a PAS de bateau à (524,−680.5) dans PierHarbor (vérifié
> au double probe) ; l'ouverture hors-plan réelle des sonars est ~±20° (pas ±3°),
> des échos « fantômes » rabattus sont normaux ; RangeMax du bag ↔ conversion
> bins→mètres : toujours déclarer la valeur utilisée dans le nom du bag.

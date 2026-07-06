# Branche Bruce_Sonar_USBL — méthode du stage (champion)

- Config champion 1.2a FIGÉE dans les yaml : Sonar Context τ=0.70 + facteurs USBL σ=1.4, SSM off.
- Run standard : `./run_slam.sh` NU (aucune variable) — attendu ATE 1.5±0.1 m (répété 6×).
- NE PAS éditer slam_aracati.yaml pour un run standard ; surcharges ponctuelles par env :
  `USBL_SIGMA=x`, `LOOP_UNION=true`, `USBL_ADAPTIVE=true` (U6 testé et REJETÉ, cf. yaml).
- Carte livrable = pointcloud_compass.csv/png (rendu cap compas U1, sorti par ./analyse.sh).
- ⚠ jamais USBL=true (double ancrage front+back, PIEGES §2) ; flip_y=True seulement si DISO.
- Journal loops : loops_detected.csv (calibration τ) ; historique configs : CONFIGS.md, ULTIME.md.

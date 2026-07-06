# Branche holoocean — simulation (préparation 3D)

- Bags : test.bag (défaut) / test_2.bag (61 s) ; topics /sonar, /dvl, /imu, /depth, /ground_truth.
- Run : `./run_slam.sh holoocean` — odom_source=dvl (GT-free, ATE 0.13 m répétable) ;
  gt = debug seulement.
- Analyse dédiée : holoocean_report.py via ./analyse.sh (étiquettes DR IMU+DVL).
- 3D à venir (bag du collègue) : stratégies + garde-fous dans HOLOOCEAN_3D_GUIDE.md
  (⚠ test « 2.5D plaqué » : std(z) INTRA-message > 0.5 m) et GARDE_FOU si format différent.

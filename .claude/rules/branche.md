# Branche holoocean — simulation (préparation 3D)

- Bags : test.bag (défaut) / test_2.bag (61 s) ; topics /sonar, /dvl, /imu, /depth, /ground_truth.
- Run : `./run_slam.sh holoocean` — odom_source=dvl (GT-free) ; gt = debug seulement.
- Défauts figés 07-07 (7 runs) : **SSM=false** (seul → ATE 4.79 m : ICP séquentiel remplace
  le facteur DVL) · **NSSM=true + min_st_sep 25** (sep8 = fausses loops 0.96/2.52 m non
  répétables) → **0.13 m ×2**. Parité Bruce testable : `SSM=true ./run_slam.sh holoocean`.
- Analyse : `./analyse.sh <run>` = rapport + paper_eval + bilan (unifiés 07-07).
- Analyse dédiée : holoocean_report.py via ./analyse.sh (étiquettes DR IMU+DVL).
- 3D à venir (bag du collègue) : stratégies + garde-fous dans HOLOOCEAN_3D_GUIDE.md
  (⚠ test « 2.5D plaqué » : std(z) INTRA-message > 0.5 m) et GARDE_FOU si format différent.

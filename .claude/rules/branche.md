# Branche holoocean — simulation (préparation 3D)

- Bags : test.bag (défaut) / test_2.bag (61 s) ; topics /sonar, /dvl, /imu, /depth, /ground_truth.
- Run : `./run_slam.sh holoocean` — odom_source=dvl (GT-free) ; gt = debug seulement.
- Défauts figés 07-07 (7 runs) : **SSM=false** (seul → ATE 4.79 m : ICP séquentiel remplace
  le facteur DVL) · **NSSM=true + min_st_sep 25** (sep8 = fausses loops 0.96/2.52 m non
  répétables) → **0.13 m ×2**. Parité Bruce testable : `SSM=true ./run_slam.sh holoocean`.
- Analyse : `./analyse.sh <run>` = rapport + paper_eval + bilan (unifiés 07-07).
- Analyse dédiée : holoocean_report.py via ./analyse.sh (étiquettes DR IMU+DVL).
- 3D « + » (2ᵉ sonar VERTICAL avant, McConnell) : **génération autonome = `./gen_traj4.sh
  [--test 150]`** (lance + détecte crash/relance ×3 + checks E1–E7 + notification bureau).
  Générateur `gen_bag_3d_v4.py` (traj4, guide §1), checks `check_traj4.py` — bags court ET
  complet (10.3 Go) TOUT PASS 2026-07-11, mesures au guide §4. venv HÔTE (jamais le
  conteneur) ; `show_viewport=False` déjà dans le code (crashs GPU Xid 13 sinon).
  ⚠ /sonar_points traj4 = plat (tilt 0, NORMAL, guide §3) ; la 3D vient de /sonar_vert_points.
- traj6 « tout capter » = traj5 + profiler TRANSVERSE 360° : `./gen_traj6.sh [--test 150]`
  (gen_bag_3d_v6.py, checks E1–E9). Mount transverse MESURÉ Rz(90)@Rx(+90) (PIEGES #16 —
  ne pas le « corriger » sans re-probe). Run : `BAG_HOLO=$PWD/BAG_files/holoocean_3d_traj6.bag
  ./run_slam.sh holoocean` ; carte_3d fusionne vert+transverse (anti-résidus étendu).

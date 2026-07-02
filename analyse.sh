#!/bin/bash

echo "Analyse simulation"
CHEMIN="/home/nathanrasami/Documents/Polytech/Stage4A/SLAM/sonar-SLAM/results/$1"
echo "Chemin est $CHEMIN"
echo "Lancement des scipts pyhton"
# NB : c2_compass_offset.py retiré — le dé-swirl est fait inline dans la simu
# (cloud/use_compass_cap dans slam_aracati.yaml). filter_cloud.py filtre pointcloud.csv direct.
export SLAM_RESULTS_DIR="$CHEMIN" && for i in analyze_drift.py analyze_origine.py plot_trajectories.py filter_cloud.py; do python3 "$i" || break; done
# Bilan compact (1 image : traj+ATE, cloud+NN, erreur de cap dans le temps)
python3 bilan_run.py "$CHEMIN" || true
echo "Fin du script - fichiers disponibles"
exit 0



#!/bin/bash

echo "Analyse simulation"
CHEMIN="/home/nathanrasami/Documents/Polytech/Stage4A/SLAM/sonar-SLAM/results/$1"
echo "Chemin est $CHEMIN"
echo "Lancement des scipts pyhton"
export SLAM_RESULTS_DIR="$CHEMIN" && for i in analyze_drift.py analyze_origine.py plot_trajectories.py; do python3 "$i" || break; done
echo "Fin du script - fichiers disponibles"
exit 0



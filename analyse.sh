#!/bin/bash
# Analyse d'un run Bruce : trajectoires + ATE (Umeyama) à partir des CSV exportés.
# Usage : ./analyse.sh run_aracati_<date>

echo "Analyse simulation"
CHEMIN="/home/nathanrasami/Documents/Polytech/Stage4A/SLAM/sonar-SLAM/results/$1"
echo "Chemin : $CHEMIN"
export SLAM_RESULTS_DIR="$CHEMIN"
for i in analyze_drift.py analyze_origine.py plot_trajectories.py; do
    python3 "$i" || break
done
echo "Fin du script - fichiers disponibles dans $CHEMIN"
exit 0

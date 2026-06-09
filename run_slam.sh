#!/usr/bin/env bash
# Lance une simu SLAM dans un dossier de résultats DÉDIÉ et horodaté.
# Évite de mélanger les CSV de runs différents (chaque run = 1 dossier).
#
# Usage :
#   ./run_slam.sh aracati       # roslaunch bruce_slam aracati.launch
#   ./run_slam.sh diso          # roslaunch direct_sonar_odometry aracati2017.launch
#   ./run_slam.sh holoocean     # roslaunch bruce_slam holoocean.launch
#
# Les CSV (trajectory, groundtruth, odometry, pointcloud) sont écrits dans
# results/run_<type>_<date>/ . Pour analyser ce run :
#   SLAM_RESULTS_DIR=results/run_aracati_2026-... python3 analyze_drift.py

set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
TYPE="${1:-aracati}"
RUN_DIR="$HERE/results/run_${TYPE}_$(date +%Y-%m-%d_%H%M%S)"
mkdir -p "$RUN_DIR"
export SLAM_RESULTS_DIR="$RUN_DIR"
echo "[run_slam] Résultats dans : $RUN_DIR"

case "$TYPE" in
  aracati)   roslaunch bruce_slam aracati.launch ;;
  diso)      roslaunch direct_sonar_odometry aracati2017.launch ;;
  holoocean) roslaunch bruce_slam holoocean.launch ;;
  *) echo "Type inconnu: $TYPE (aracati|diso|holoocean)"; exit 1 ;;
esac

echo "[run_slam] Terminé. Analyse avec :"
echo "  SLAM_RESULTS_DIR=$RUN_DIR python3 analyze_drift.py"

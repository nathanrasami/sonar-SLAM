#!/usr/bin/env bash
# Lance une simu SLAM dans un dossier de résultats DÉDIÉ et horodaté.
# Évite de mélanger les CSV de runs différents (chaque run = 1 dossier).
#
# Usage :
#   ./run_slam.sh aracati                 # roslaunch bruce_slam aracati.launch (bag inclus)
#   ./run_slam.sh diso [chemin_bag]       # DISO standalone + rosbag play
#   ./run_slam.sh holoocean               # roslaunch bruce_slam holoocean.launch
#
# Le launch DISO ne joue PAS le bag → le wrapper le lance lui-même (mode diso).
# Les CSV (trajectory/diso_trajectory, groundtruth, odometry, pointcloud) sont
# écrits dans results/run_<type>_<date>/ . Pour analyser ce run :
#   SLAM_RESULTS_DIR=results/run_aracati_2026-... python3 analyze_drift.py

set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
TYPE="${1:-aracati}"
# bag par défaut = celui d'aracati.launch (surchageable en 2e argument)
BAG="${2:-/home/nathan/Aracati2017_DISO_backup/bags/ARACATI_2017_8bits_full.bag}"
RUN_DIR="$HERE/results/run_${TYPE}_$(date +%Y-%m-%d_%H%M%S)"
mkdir -p "$RUN_DIR"
export SLAM_RESULTS_DIR="$RUN_DIR"
echo "[run_slam] Résultats dans : $RUN_DIR"

case "$TYPE" in
  aracati)   roslaunch bruce_slam aracati.launch ;;
  holoocean) roslaunch bruce_slam holoocean.launch ;;
  diso)
    # le launch DISO ne joue pas le bag : on le lance, on attend que les nodes
    # soient prêts, puis on joue le bag ; CTRL+C arrête tout proprement.
    if [ ! -f "$BAG" ]; then
      echo "[run_slam] ERREUR: bag introuvable: $BAG"
      echo "           usage: ./run_slam.sh diso /chemin/vers/le.bag"
      exit 1
    fi
    roslaunch direct_sonar_odometry aracati2017.launch &
    LAUNCH_PID=$!
    trap "kill $LAUNCH_PID 2>/dev/null" EXIT
    echo "[run_slam] Démarrage DISO... (5 s pour initialiser les nodes + RViz)"
    sleep 5
    echo "[run_slam] Lecture du bag : $BAG"
    rosbag play "$BAG" --clock -r 1.0 -q
    echo "[run_slam] Bag terminé. CTRL+C pour fermer DISO et sauver les CSV."
    wait $LAUNCH_PID
    ;;
  *) echo "Type inconnu: $TYPE (aracati|diso|holoocean)"; exit 1 ;;
esac

echo "[run_slam] Terminé. Analyse avec :"
echo "  SLAM_RESULTS_DIR=$RUN_DIR python3 analyze_drift.py"

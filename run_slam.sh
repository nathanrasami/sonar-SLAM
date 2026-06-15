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

# Depuis Fedora : se re-lancer automatiquement dans le conteneur ros1.
# À l'intérieur du conteneur, CONTAINER_ID est positionné par distrobox.
if [ -z "$CONTAINER_ID" ]; then
    DISTROBOX="${DISTROBOX:-$(command -v distrobox 2>/dev/null || echo ~/.opt/bin/distrobox)}"
    exec "$DISTROBOX" enter ros1 -- bash "$0" "$@"
fi

# Source ROS (on est dans le conteneur)
source /opt/ros/noetic/setup.bash
[ -f "$HOME/ros1_ws/devel/setup.bash" ] && source "$HOME/ros1_ws/devel/setup.bash"

# ROS sur loopback : sinon les nœuds communiquent via l'IP Wi-Fi (hostname),
# et couper le Wi-Fi tue toutes les connexions ROS (gel constaté le 12/06).
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

HERE="$(cd "$(dirname "$0")" && pwd)"
TYPE="${1:-aracati}"
# bag par défaut : dans le dossier du repo (surchargeable en 2e argument)
BAG="${2:-$HERE/ARACATI_2017_8bits_full.bag}"
RUN_DIR="$HERE/results/run_${TYPE}_$(date +%Y-%m-%d_%H%M%S)"
mkdir -p "$RUN_DIR"
export SLAM_RESULTS_DIR="$RUN_DIR"
echo "[run_slam] Résultats dans : $RUN_DIR"

case "$TYPE" in
  aracati)   roslaunch bruce_slam aracati.launch bag_file:="$BAG" rate:="${RATE:-1.0}" ;;
  holoocean) roslaunch bruce_slam holoocean.launch ;;
  # NB : cette branche (Bruce base) n'a pas DISO. Pour l'odométrie sonar DISO,
  # utiliser la branche Bruce_DISO (./run_slam.sh diso y est défini).
  *) echo "Type inconnu: $TYPE (aracati|holoocean)"; exit 1 ;;
esac

echo "[run_slam] Terminé. Analyse avec :"
echo "  SLAM_RESULTS_DIR=$RUN_DIR python3 analyze_drift.py"

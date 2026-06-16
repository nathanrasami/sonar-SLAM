#!/usr/bin/env bash
# Lance une simu SLAM dans un dossier de résultats DÉDIÉ et horodaté.
# Le launch utilisé dépend de la branche git — pas besoin de passer un mode.
#
# Usage :
#   ./run_slam.sh              # aracati (défaut) — bag ARACATI_2017_8bits_full.bag
#   ./run_slam.sh holoocean    # roslaunch bruce_slam holoocean.launch
#
# Les CSV sont écrits dans results/run_aracati_<date>/. Pour analyser :
#   SLAM_RESULTS_DIR=results/run_aracati_2026-... python3 analyze_drift.py

set -e

# Depuis Fedora : se re-lancer automatiquement dans le conteneur ros1.
if [ -z "$CONTAINER_ID" ]; then
    DISTROBOX="${DISTROBOX:-$(command -v distrobox 2>/dev/null || echo ~/.opt/bin/distrobox)}"
    exec "$DISTROBOX" enter ros1 -- bash "$0" "$@"
fi

source /opt/ros/noetic/setup.bash
[ -f "$HOME/ros1_ws/devel/setup.bash" ] && source "$HOME/ros1_ws/devel/setup.bash"

# ROS sur loopback (couper le Wi-Fi ne tue plus les connexions ROS)
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

HERE="$(cd "$(dirname "$0")" && pwd)"
TYPE="${1:-aracati}"
BAG="${BAG:-$HERE/ARACATI_2017_8bits_full.bag}"
RUN_DIR="$HERE/results/run_${TYPE}_$(date +%Y-%m-%d_%H%M%S)"
mkdir -p "$RUN_DIR"
export SLAM_RESULTS_DIR="$RUN_DIR"
echo "[run_slam] Résultats dans : $RUN_DIR"

case "$TYPE" in
  aracati)   roslaunch bruce_slam aracati.launch bag_file:="$BAG" rate:="${RATE:-1.0}" ;;
  holoocean) roslaunch bruce_slam holoocean.launch ;;
  *) echo "Type inconnu: $TYPE (aracati|holoocean)"; exit 1 ;;
esac

echo "[run_slam] Terminé. Analyse avec :"
echo "  SLAM_RESULTS_DIR=$RUN_DIR python3 analyze_drift.py"
echo "  SLAM_RESULTS_DIR=$RUN_DIR python3 analyze_origine.py"
echo "  SLAM_RESULTS_DIR=$RUN_DIR python3 plot_trajectories.py"

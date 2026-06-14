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
# Filet de sécurité si l'env hôte n'a pas été transmis (lancement non-interactif).
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
  aracati)   roslaunch bruce_slam aracati.launch bag_file:="$BAG" ;;
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
  diso_nogt)
    # ABLATION GT : DISO lit une GT volontairement DÉRIVÉE (/pose_gt_drift) au lieu
    # de /pose_gt. Teste si DISO corrige la dérive par le sonar (vraie odométrie)
    # ou la suit (dépendant de la GT). Le nœud gt_drift est un simple script python
    # (pas de recompilation). DRIFT_LAT / DRIFT_YAW surchargent les défauts.
    if [ ! -f "$BAG" ]; then
      echo "[run_slam] ERREUR: bag introuvable: $BAG"; exit 1
    fi
    roslaunch direct_sonar_odometry aracati2017_drift.launch &
    LAUNCH_PID=$!
    sleep 3
    python3 "$HERE/DISO/scripts/gt_drift_node.py" \
        _lateral_drift_rate:="${DRIFT_LAT:-0.003}" _yaw_bias_deg:="${DRIFT_YAW:-3.0}" &
    DRIFT_PID=$!
    trap "kill $LAUNCH_PID $DRIFT_PID 2>/dev/null" EXIT
    echo "[run_slam] DISO + injection de dérive GT... (5 s d'init)"
    sleep 5
    echo "[run_slam] Lecture du bag : $BAG"
    rosbag play "$BAG" --clock -r "${RATE:-1.0}" -q
    echo "[run_slam] Bag terminé. CTRL+C pour fermer et sauver les CSV."
    wait $LAUNCH_PID
    ;;
  *) echo "Type inconnu: $TYPE (aracati|diso|diso_nogt|holoocean)"; exit 1 ;;
esac

echo "[run_slam] Terminé. Analyse avec :"
echo "  SLAM_RESULTS_DIR=$RUN_DIR python3 analyze_drift.py"

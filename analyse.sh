#!/bin/bash
# Analyse d'un run : ./analyse.sh <nom_du_run>
#   ex : ./analyse.sh run_aracati_2026-07-03_003823
#        ./analyse.sh run_holoocean_2026-07-03_015206
# Détecte le type de run (aracati / holoocean) et enchaîne les scripts de analysis/
# qui s'appliquent — tolérant : un script manquant ou en échec n'arrête pas la suite.
# Sorties : trajectoire, pointcloud (+filtré si intensité), erreur traj dans le temps,
# erreur de cap dans le temps + bilan 1 image (bilan_run.png).

HERE="$(cd "$(dirname "$0")" && pwd)"
RUN="$1"
[ -n "$RUN" ] || { echo "usage: ./analyse.sh <nom_du_run>"; exit 1; }
CHEMIN="$HERE/results/$RUN"
[ -d "$CHEMIN" ] || { echo "run introuvable : $CHEMIN"; exit 1; }
A="$HERE/analysis"
export SLAM_RESULTS_DIR="$CHEMIN"
echo "[analyse] $CHEMIN"

run_py() {  # lance un script s'il existe ; n'arrête jamais la chaîne
    if [ -f "$A/$1" ]; then
        echo "— $1"
        python3 "$A/$1" "${@:2}" || echo "  (échec $1 — on continue)"
    fi
}

# commun à tous les runs : trajectoires + erreur dans le temps
run_py analyze_drift.py        # erreur de trajectoire over time
run_py analyze_origine.py
run_py plot_trajectories.py

case "$RUN" in
  run_holoocean_*) run_py analyze_holoocean.py ;;
esac

# cloud filtré par intensité — seulement si la colonne existe dans pointcloud.csv
if head -1 "$CHEMIN/pointcloud.csv" 2>/dev/null | grep -q intensity; then
    run_py filter_cloud.py
fi

# bilan compact (1 image : traj+ATE Umeyama, cloud+NN, erreur de cap over time)
run_py bilan_run.py "$CHEMIN"

echo "[analyse] terminé — fichiers dans $CHEMIN"

#!/usr/bin/env bash
# Suite AUTONOME traj8 (nuit) : bag complet (si absent) -> 4 runs SLAM -> analyses.
# UN SEUL run à la fois (règle CLAUDE.md), enchaînement tolérant : un run en échec
# n'arrête pas les suivants ; seul un échec de GÉNÉRATION arrête tout.
#
# Usage : ./run_traj8_suite.sh        (laisser tourner ; ~4 h au total :
#         gen ~1 h 50 + 4 runs x ~27 min run+analyse)
#
# Runs enchaînés (tous : bag traj8, SONAR_RANGE=20, mode 2D) :
#   1. B      : method bruce,       NSSM=false  -> témoin DR (SLAM≡DR attendu,
#               ATE ≈ prédiction nav seed 8 : Umeyama ~1.1 m)
#   2. BS1    : method bruce_sonar (SC ; launch force nssm=true) -> LA mesure
#   3. BS2    : idem BS1 -> répétabilité (R3 : figer exige x2)
#   4. B_NSSM : method bruce, NSSM=true -> front-end proximité SANS descripteur
# (SSM volontairement absent : réfuté 07-07, ICP séquentiel 4.79 m.)
#
# Étiquetage PROGRAMMATIQUE (piège suffixe manuel, mémoire pipeline-non-determinisme) :
# le script renomme le dossier et écrit config_lancee.txt AVANT l'analyse — la
# vérité reste vérifiable dans le log roslaunch (« method:=... »).
#
# ⚠ Pré-requis : PC sans suspension ; ne RIEN lancer d'autre (ni commit) pendant.
# ⚠ analyse.sh 3D ouvre carte_3d.html à chaque run : des onglets peuvent s'empiler.
set -u
cd "$(dirname "$0")"
BAG="$PWD/BAG_files/holoocean_3d_traj8.bag"

notif() { command -v notify-send >/dev/null && notify-send "$1" "$2"; printf '\a'; }

# ── 0. bag complet + E1-E9 (STOP si échec : tout le reste en dépend) ──────────
if [ ! -f "$BAG" ]; then
    echo "=== bag complet absent -> ./gen_traj8.sh ($(date +%H:%M)) ==="
    ./gen_traj8.sh || { notif "traj8 suite : STOP ✗" "gen ou E-checks en échec — rien lancé"; exit 1; }
fi

do_run() {  # $1 = suffixe, $2 = méthode (bruce|bs), $3 = NSSM (true|false)
    echo "=== run $1 : method=$2 NSSM=$3 ($(date +%H:%M)) ==="
    BAG_HOLO="$BAG" SONAR_RANGE=20 NSSM="$3" ./run_slam.sh holoocean 2D "$2"
    rc=$?
    RUN=$(ls -td results/run_holoocean_* 2>/dev/null | head -1)
    [ -n "$RUN" ] || { notif "traj8 $1 : AUCUN run produit ✗" "on passe au suivant"; return 1; }
    NEW="${RUN}_$1"
    mv "$RUN" "$NEW"
    { echo "suffixe=$1 method=$2 NSSM=$3 SONAR_RANGE=20 mode=2D"
      echo "bag=$BAG"
      echo "lance_par=run_traj8_suite.sh rc_run=$rc $(date '+%Y-%m-%d %H:%M')"
    } > "$NEW/config_lancee.txt"
    if [ $rc -ne 0 ]; then
        notif "traj8 $1 : run en échec (rc=$rc) ⚠" "$(basename "$NEW") — suite continue"
        return 1
    fi
    ./analyse.sh 3D "$(basename "$NEW")" || notif "traj8 $1 : analyse en échec ⚠" "run conservé"
    notif "traj8 : run $1 terminé ✅" "$(basename "$NEW")"
}

do_run B      bruce false
do_run BS1    bs    false
do_run BS2    bs    false
do_run B_NSSM bruce true

notif "traj8 suite : TERMINÉE 🏁" "4 runs + analyses dans results/ — ouvrir une nouvelle discussion pour le verdict"
echo "=== suite terminée $(date '+%Y-%m-%d %H:%M') ==="

#!/usr/bin/env bash
# Génération traj7r AUTONOME (traj7 + navigation réaliste, gen_bag_3d_v8.py) : lance, surveille, relance si crash, vérifie, notifie.
# traj7 = traj6 (3 capteurs) avec quais serrés 2.5±0.5 m + RangeMax horizontal
# 20 m + détour bateau supprimé (bateau réfuté par probe 2026-07-12).
# Usage : ./gen_traj7.sh            -> bag complet (~35-60 min, ~12 Go)
#         ./gen_traj7.sh --test 150 -> bag court des checks (~10 min, ~1.3 Go)
#
# - Progression en direct dans le terminal (+ gen_traj7r.log).
# - Crash moteur (Xid 13 : bag figé, python suspendu) : détecté en ~3 min,
#   python tué, relance automatique (3 essais max). ⚠ vu aussi 2026-07-12 :
#   SIGBUS au premier démarrage après boot (course shm) — la relance couvre.
# - Fin : check_traj4.py --rmax-h 20 (E1–E9) + notification bureau + bip.
set -u
cd "$(dirname "$0")"
PY="$(cd .. && pwd)/holoocean-venv/bin/python"
LOG="gen_traj7r.log"
BAG="BAG_files/holoocean_3d_traj7r.bag"
case "$*" in *--test*) BAG="BAG_files/holoocean_3d_traj7r_test.bag";; esac

[ -x "$PY" ] || { echo "venv introuvable : $PY"; exit 2; }
# garde-fou : un VRAI générateur actif ? (comm=python*, pas les wrappers bash
# dont la ligne de commande contient juste le nom du script)
for p in $(pgrep -f "gen_bag_3d_v8.py"); do
    case "$(ps -o comm= -p "$p" 2>/dev/null)" in python*)
        echo "⚠ une génération tourne déjà (pid $p) — abandon."; exit 2;;
    esac
done

notifier() {  # $1 = titre, $2 = corps
    command -v notify-send >/dev/null && notify-send "$1" "$2"
    printf '\a'
}

for essai in 1 2 3; do
    echo "=== essai $essai/3 — $(date +%H:%M:%S) — log : $LOG ==="
    "$PY" -u gen_bag_3d_v8.py "$@" >"$LOG" 2>&1 &
    GENPID=$!
    tail --pid="$GENPID" -n +1 -f "$LOG" &   # affichage en direct, meurt avec le python
    TAILPID=$!

    prev=-1; fige=0
    while kill -0 "$GENPID" 2>/dev/null; do
        sleep 30
        cur=$(stat -c %s "$BAG" 2>/dev/null || echo 0)
        if [ "$cur" = "$prev" ]; then fige=$((fige+1)); else fige=0; fi
        prev=$cur
        if [ "$fige" -ge 6 ]; then          # 3 min sans croissance = moteur mort
            echo "✗ CRASH probable (bag figé 3 min) — kill du python et relance."
            kill -9 "$GENPID" 2>/dev/null
            break
        fi
    done
    wait "$GENPID" 2>/dev/null; kill "$TAILPID" 2>/dev/null

    if grep -q "bag ecrit" "$LOG"; then
        echo "=== génération OK, checks E1–E9 (--rmax-h 20)… ==="
        "$PY" check_traj4.py "$BAG" --rmax-h 20; rc=$?
        if [ $rc -eq 0 ]; then
            notifier "traj7r : TOUT PASS ✅" "$(tail -1 "$LOG") — E1–E9 PASS"
        else
            notifier "traj7r : bag écrit mais E-checks ÉCHEC ⚠" "voir la sortie de check_traj4.py"
        fi
        exit $rc
    fi
    notifier "traj7r : crash moteur (essai $essai/3)" "relance automatique…"
done
notifier "traj7r : ABANDON ✗" "3 crashs consécutifs — voir $LOG"
echo "✗ 3 échecs consécutifs — voir $LOG (journalctl : NVRM Xid ?)"
exit 1

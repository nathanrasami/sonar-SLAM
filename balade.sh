#!/usr/bin/env bash
# Balade libre PierHarbor (spawn zone 13) — voir balade_zone13.py.
# Le boot HoloOcean crashe ALÉATOIREMENT (Xid 13, même aléa que gen_traj8.sh) :
# le python sort avec rc=3 dans ce cas et on relance, jusqu'à 4 essais.
cd "$(dirname "$0")"
PY="$(cd .. && pwd)/holoocean-venv/bin/python"
[ -x "$PY" ] || { echo "venv introuvable : $PY"; exit 2; }
for essai in 1 2 3 4; do
    "$PY" balade_zone13.py "$@"
    rc=$?
    [ "$rc" -ne 3 ] && exit "$rc"
    echo "[balade.sh] boot raté (aléa Xid 13) — relance ($essai/4)"
    pkill -9 -f Holodeck 2>/dev/null
    sleep 3
done
echo "[balade.sh] 4 boots ratés d'affilée — vérifier : journalctl -k | grep Xid"
exit 3

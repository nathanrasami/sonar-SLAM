#!/usr/bin/env bash
# Balade libre PierHarbor (0 capteur, spawn zone 13) — voir balade_zone13.py.
cd "$(dirname "$0")"
PY="$(cd .. && pwd)/holoocean-venv/bin/python"
[ -x "$PY" ] || { echo "venv introuvable : $PY"; exit 2; }
exec "$PY" balade_zone13.py "$@"

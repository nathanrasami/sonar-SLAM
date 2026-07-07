#!/usr/bin/env bash
# Étapes suivantes AUTONOMES (07-08) : ablations sur le rendu PROPRE (bag traj1, 2 tours).
#   Run A : SSM=true  — l'ICP séquentiel marche-t-il sur murs propres ? (réf : 4.79 m sur ancien rendu)
#   Run B : méthode bs — loops Sonar Context sur bag long avec revisites (lire loops_detected.csv)
# Témoin (déjà mesuré) : défauts nus traj1 = ATE 0.03 m, 0 loop. 1 variable par run.
# Log : results/suite_next.log — commit+push auto à la fin.
set -u
cd "$(dirname "$0")"
exec >>results/suite_next.log 2>&1
echo "=== suite_next démarrée $(date) ==="
BAG="$PWD/bag/holoocean_3d_traj1.bag"

while pgrep -f "holoocean.launch" >/dev/null; do sleep 20; done

echo "--- RUN A : SSM=true (traj1, rendu propre)"
SSM=true BAG_HOLO="$BAG" ./run_slam.sh holoocean
RA=$(ls -td results/run_holoocean_* | head -1)
./analyse.sh "$(basename "$RA")"

echo "--- RUN B : méthode bs (SC loops, traj1)"
BAG_HOLO="$BAG" ./run_slam.sh holoocean 2D bs
RB=$(ls -td results/run_holoocean_* | head -1)
./analyse.sh "$(basename "$RB")"
echo "--- loops_detected RUN B :"
[ -f "$RB/loops_detected.csv" ] && head -20 "$RB/loops_detected.csv"

echo "=== BILAN (témoin défauts nus : 0.03 m, 0 loop) ==="
echo "RUN A (SSM=true) : $RA"; python3 analysis/holoocean_report.py "$RA" | head -1
echo "RUN B (bs/SC)    : $RB"; python3 analysis/holoocean_report.py "$RB" | head -1

git add suite_next.sh PROGRESS.md
git commit -m "🧪 suite_next : ablations rendu propre (A: SSM=true, B: SC/bs) sur traj1 — verdicts dans results/suite_next.log

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
git push origin holoocean
echo "=== suite_next terminée $(date) ==="

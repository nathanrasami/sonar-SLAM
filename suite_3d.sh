#!/usr/bin/env bash
# Suite 3D AUTONOME (07-07 soir, session Fable presque pleine) :
# attend la fin du run traj1 en cours → analyse → run traj2 → analyse →
# reconstruction /profiler_points → commit+push. Log : results/suite_3d.log
set -u
cd "$(dirname "$0")"
LOG=results/suite_3d.log
exec >>"$LOG" 2>&1
echo "=== suite_3d démarrée $(date) ==="

# 1. attendre la fin du run traj1 en cours
while pgrep -f "holoocean.launch" >/dev/null; do sleep 20; done
sleep 10
R1=$(ls -td results/run_holoocean_* | head -1)
echo "--- traj1 = $R1"
./analyse.sh "$(basename "$R1")"

# 2. run traj2 (mêmes topics + /profiler_points, SLAM identique)
BAG_HOLO="$PWD/bag/holoocean_3d_traj2.bag" ./run_slam.sh holoocean
R2=$(ls -td results/run_holoocean_* | head -1)
echo "--- traj2 = $R2"
./analyse.sh "$(basename "$R2")"

# 3. reconstruction vraie-3D du profiler (rosbag → conteneur ros1)
DISTROBOX="$(command -v distrobox 2>/dev/null || echo "$HOME/.opt/bin/distrobox")"
"$DISTROBOX" enter ros1 -- bash -lc "source /opt/ros/noetic/setup.bash; cd '$PWD'; python3 analysis/profiler_3d.py bag/holoocean_3d_traj2.bag '$R2/profiler_3d.png'"

# 4. bilan console
echo "=== BILAN ==="
python3 analysis/holoocean_report.py "$R1" | head -1
python3 analysis/holoocean_report.py "$R2" | head -1

# 5. commit du code/doc (PAS les results/, trop lourds)
git add analysis/profiler_3d.py suite_3d.sh PROGRESS.md bruce_slam/scripts/holoocean_sonar_bridge.py
git commit -m "🤿 bags 3D : bridge 32FC1→mono8 (fix CFAR aveugle) + suite_3d auto + profiler_3d.py

Runs traj1/traj2 exécutés par suite_3d.sh (autonome, session Fable close).
Résultats : voir results/suite_3d.log + les 2 derniers run_holoocean_*.
Bridge : /sonar des bags 3D = 32FC1 [0,1] (échos ≤0.47) → conversion
mono8 ×255 (applyColorMap crashait, CFAR seuil 95 aveugle sinon).
profiler_3d.py : nuage 3D monde de /profiler_points (z<0, sous-échant.).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
git push origin holoocean
echo "=== suite_3d terminée $(date) ==="

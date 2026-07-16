#!/usr/bin/env bash
# compare_prof.sh — 2 scénarios voulus par la prof × 2 méthodes SLAM.
# (Nathan 2026-07-15 · branche holoocean)
#
#   ① traj9  = « tour complet »               (gen_traj9.sh, z const, sonar 20)
#   ② traj5  = « z variable + contourne bateau » (gen_traj5.sh, z aléa [-12,-2], sonar 40)
#   × { Bruce , Bruce_Sonar }  →  4 runs SLAM, nssm=true (parité), ssm=false, 2D.
#
# Fait, dans l'ordre, SANS surveillance (lance et pars) :
#   1) régénère chaque bag MANQUANT via son wrapper (watchdog Xid13 + relance ×3
#      déjà dans gen_traj*.sh). Bag déjà présent = RÉUTILISÉ (FORCE_REGEN=1 force).
#   2) 4 runs SLAM SÉQUENTIELS (un seul à la fois — règle du repo) + analyse.sh.
#   Chaque résultat reçoit un scenario_label.txt (bag × méthode) pour lever
#   l'ambiguïté du nom horodaté (piège suffixe _B/_BS, mémoire pipeline-nondet).
#
# Usage :   ./compare_prof.sh            # réutilise traj9.bag, régénère traj5.bag
#           FORCE_REGEN=1 ./compare_prof.sh   # régénère AUSSI traj9.bag
# Logs   :  compare_prof_<horodatage>/{master.log, gen_*.log, run_*.log}
set -u
cd "$(dirname "$0")"; HERE="$(pwd)"
FORCE_REGEN="${FORCE_REGEN:-0}"
STAMP="$(date +%Y-%m-%d_%H%M%S)"
OUT="compare_prof_${STAMP}"; mkdir -p "$OUT"
MASTER="$OUT/master.log"

# nom | wrapper gen | bag | sonar_range | description
SPECS=(
  "traj9|gen_traj9.sh|BAG_files/holoocean_3d_traj9.bag|20|tour complet (z const)"
  "traj5|gen_traj5.sh|BAG_files/holoocean_3d_traj5.bag|40|z variable + contourne bateau"
)
METHODS=("Bruce" "Bruce_Sonar")

log()   { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$MASTER"; }
notif() { command -v notify-send >/dev/null 2>&1 && notify-send "compare_prof" "$*"; printf '\a'; }

log "===== compare_prof démarré → $OUT ====="

# --- garde-fous ---
BR="$(git branch --show-current 2>/dev/null || echo '?')"
if [ "$BR" != holoocean ]; then
  log "⚠ branche = '$BR' (attendu 'holoocean' : le launch SLAM dépend de la branche)."
  log "  → git checkout holoocean, puis relance. ABANDON."; exit 2
fi
if pgrep -f 'gen_bag_3d_v(5|10)\.py' >/dev/null || pgrep -f 'roslaunch bruce_slam' >/dev/null; then
  log "⚠ une génération ou un run SLAM tourne déjà — ABANDON (un seul à la fois)."; exit 2
fi

# ---------- 1) génération des bags manquants ----------
declare -A BAG_OK
for spec in "${SPECS[@]}"; do
  IFS='|' read -r name wrap bag sr desc <<<"$spec"
  if [ "$FORCE_REGEN" != 1 ] && [ -s "$bag" ]; then
    log "↷ $name : bag déjà présent ($(du -h "$bag" | cut -f1)) — régénération sautée."
    BAG_OK[$name]=1; continue
  fi
  log "GEN $name ($desc) via ./$wrap … log: $OUT/gen_${name}.log"
  if ./"$wrap" >"$OUT/gen_${name}.log" 2>&1 && [ -s "$bag" ]; then
    log "✓ $name généré ($(du -h "$bag" | cut -f1))"; BAG_OK[$name]=1
  else
    log "✗ $name : génération ou E-checks ÉCHEC — bag SAUTÉ (voir $OUT/gen_${name}.log)."
    BAG_OK[$name]=0; notif "gen $name : échec"
  fi
done

# ---------- 2) runs SLAM séquentiels (bags OK) ----------
n_ok=0; n_ko=0
for spec in "${SPECS[@]}"; do
  IFS='|' read -r name wrap bag sr desc <<<"$spec"
  if [ "${BAG_OK[$name]:-0}" != 1 ]; then
    log "— $name : bag KO, ses runs SLAM sont sautés."; continue
  fi
  for m in "${METHODS[@]}"; do
    rlog="$OUT/run_${name}_${m}.log"
    log "── RUN $name × $m  (sonar_range=$sr nssm=true ssm=false 2D) → $rlog ──"
    BAG_HOLO="$HERE/$bag" SONAR_RANGE="$sr" NSSM=true SSM=false \
      ./run_slam.sh holoocean 2D "$m" >"$rlog" 2>&1; rc=$?
    rundir="$(ls -dt results/run_holoocean_* 2>/dev/null | head -1)"
    if [ "$rc" = 0 ] && [ -n "$rundir" ]; then
      { echo "scenario : $name × $m"; echo "desc     : $desc";
        echo "bag      : $bag"; echo "params   : sonar_range=$sr nssm=true ssm=false mode=2D";
      } > "$rundir/scenario_label.txt"
      log "✓ $name × $m → $rundir"
      if ./analyse.sh "$(basename "$rundir")" >>"$rlog" 2>&1; then
        log "  analyse.sh OK"
      else
        log "  ⚠ analyse.sh a renvoyé une erreur (non bloquant, run gardé)"
      fi
      n_ok=$((n_ok+1))
    else
      log "✗ $name × $m : rc=$rc (voir $rlog / $rundir)"; notif "run $name×$m : échec"; n_ko=$((n_ko+1))
    fi
  done
done

log "===== TERMINÉ : $n_ok run(s) OK, $n_ko échec(s). Logs → $OUT/ · labels → results/*/scenario_label.txt ====="
notif "✅ terminé : $n_ok OK / $n_ko KO (traj9/traj5 × Bruce/Bruce_Sonar)"

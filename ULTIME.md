# ULTIME.md — branche `Bruce_Ultime` : fusionner le meilleur des deux mondes

> Phase lancée le 2026-07-04. Base : `Bruce_Sonar_USBL` (champion traj 1.2a, ATE 1.50 m).
> Objectif : y ramener l'avantage CARTE de la branche `Bruce` (B′ : méd 0.09 / p90 0.74),
> puis dépasser les deux. **Lire `PIEGES.md` avant tout.** Règles : 1 changement par run ;
> éval = `./analyse.sh <run>` + `python3 analysis/paper_eval.py <run>` (protocole papier).

## Cible chiffrée

| | traj (ATE um.) | carte méd/p90 vs vraie | référence |
|---|---|---|---|
| état de départ (1.2a `003823`) | **1.50 m** | 0.11 / 0.99 | mini-papier §6.3-6.4 |
| meilleur Bruce pur (B′ `120352-1`) | 1.88 m | 0.09 / 0.74 | ABLATION.md |
| borne assistée GT (`011733`) | 0.89 m | 0.11 / 0.40 | plancher pratique |
| **cible Ultime** | **≤ 1.4 m** | **≤ 0.09 / ≤ 0.45** | U1 y est déjà pour la carte |

## U1 — rendu de carte au cap compas ✅ VALIDÉ OFFLINE (07-04, 0 run)

**Idée** : le θ iSAM2 est optimisé pour la POSITION (USBL + loops) → transitoires jusqu'à
40° dans les virages → scans smeared. Le cap compas (dr_theta) est lisse et vrai. On re-rend
chaque scan à la **position optimisée** (ATE inchangée par construction) avec le **cap
compas recalé** : θ_rendu = dr_theta + δ, δ = moyenne circulaire de (θ_opt − dr_theta) —
100 % GT-free (fit interne au run, aucune GT).

**Mesuré sur 1.2a** (`analysis/render_compass_cloud.py results/... --imin 255 --eval`) :
NN auto 0.204 → **0.176** ; carte vs vraie méd 0.114 → **0.077**, p90 0.989 → **0.441**
= exactement la borne du rendu au cap GT (0.077/0.440). Le rendu compas donne à la traj BSU
une carte MEILLEURE que B′ (0.09/0.74) et que la réf GT-assistée en médiane.
Contre-vérif sur B′ : gain marginal (p90 0.74→0.64, NN dégradé) — l'avantage carte de Bruce
venait du cap local (SSM) ; U1 l'importe côté BSU sans SSM. Piège historique désamorcé :
l'ancien hack `use_compass_cap` (offset 162° figé, pré-fix chiralité) échouait ; la version
qui marche = offset auto-fit δ + post-fix.

## U2 — U1 intégré au flux standard ✅ FAIT (07-04, 0 run)

`analyse.sh` appelle désormais `render_compass_cloud.py` pour tout run aracati avec
`dr_theta` dans trajectory.csv (`--imin 255` si le cloud a l'intensité) →
`pointcloud_compass.csv/png` est une sortie standard de `./analyse.sh <run>`.
Option inline (RViz) : seulement si le offline devient limitant (reprendre l'infra
`cloud/use_compass_cap` avec δ auto-fit glissant au lieu de `cap_offset_deg` figé).

## U3 — σ USBL intermédiaire avec SC ✅ PRÊT À RUNNER (1 run)

Le trade-off mesuré (SC→1.4 raide, natif→2.5 doux) n'a jamais été échantillonné ENTRE.
**Câblé sans toucher au yaml** : env `USBL_SIGMA` → arg launch `usbl_sigma` → surcharge
`usbl/sigma` (vide = valeur du yaml, champion 1.4).
Succès si ATE < 1.50 sans dégrader la carte-U1 (p90 ≤ 0.45).

## U4 — union des détecteurs SC + NSSM natif ✅ CODÉ, PRÊT À RUNNER (1 run)

SC REMPLAÇAIT la présélection covariance du NSSM ; les deux voient des candidats
différents (apparence vs géométrie). Implémenté le 07-04 :

- `slam.py` : `add_nonsequential_scan_matching` boucle sur les détecteurs (`sc` puis
  `native`, dédupliqués s'ils sont d'accord) et soumet chaque candidat au corps
  historique INCHANGÉ, extrait dans `_process_nssm_candidate` (shgo/ICP/garde-fous/PCM).
  `initialize_nonsequential_scan_matching(detector=...)` force le chemin voulu.
- Journal : `loops_detected.csv` gagne une colonne `detector` (sc|nssm) ; les candidats
  du gating natif sont tracés avec `sc_dist=-1`.
- Pilotage : param `sonar_context/union` (défaut **False** = champion 1.2a intact),
  surchargé par le launch ← env `LOOP_UNION`.
- Coût : jusqu'à 2 shgo/keyframe quand les détecteurs divergent → `RATE=0.5` si le CPU
  sature.

Attendu : constraints 116 → 150+, S2/S3 raffermies, 0 faux (le PCM commun tranche).

## 🎬 SÉQUENCE DE RUNS À ENCHAÎNER (chacun ~45 min, arrêt auto fin de bag)

> Après CHAQUE run : `./analyse.sh <run>` (inclut la carte compas U1) puis
> `python3 analysis/paper_eval.py results/<run>` et reporter ici (Journal).
> Baseline de comparaison = 1.2a `003823` (ATE 1.50, carte compas 0.077/0.441).

| # | Commande (branche Bruce_Ultime) | Teste | Verdict si |
|---|---|---|---|
| RU1 | `USBL_SIGMA=1.8 ./run_slam.sh` | U3 σ intermédiaire | garde si ATE<1.50 et p90≤0.45 |
| RU2 | `USBL_SIGMA=2.0 ./run_slam.sh` | U3 (seulement si RU1 > 1.2a) | idem ; sinon retour σ1.4 |
| RU3 | `LOOP_UNION=true ./run_slam.sh` | U4 union (σ = meilleur connu via USBL_SIGMA) | garde si constraints↑ ET ATE≤RU-précédent |
| RU4 | *(branche Bruce)* `./run_slam.sh` | U5 keyframes 1.0 (yaml déjà modifié) | B″ remplace B′ si ATE<1.88 |

Si RU1-RU3 donnent un gagnant : figer ses réglages dans le yaml Ultime (défauts = config
gagnante, plus aucune variable d'env) + 1 run final de confirmation = livrable.

## U5 — keyframes 1.0 m sur la branche `Bruce` (1 run, indépendant)

Réponse à « la trajectooire Bruce est géométrique » : `keyframe_translation: 3.0` (upstream)
→ 256 KF espacées de ~3 m. Passer à `1.0` (comme BSU, dans la plage upstream « 1-4 m ») :
trajectoire lissée, ~665 KF, coût CPU ×2.6 (ok). Comparer à B′ avec paper_eval. Si gain,
B″ devient le nouveau champion Bruce du papier BRUCE_SLAM.md.

## U6 — σ USBL adaptatif par fix (moyen)

Covariance du facteur USBL par fix (qualité/résidu, correntropie maximale — papier
INS/USBL/DVL FGO, Sensors 2023). Supprime le réglage manuel raide/doux (leçon 3× mesurée).
À faire après U3 (qui dira si le σ fixe optimal suffit).

## U7 — réserve (si le goulot se déplace)

- SONIC offline sur les paires candidates (CONFIGS.md#sonic-offline) : 40/122 candidats non
  convertis par l'ICP restent la perte principale côté loops.
- MCFAR (CONFIGS.md#32-mcfar) : densité/qualité des détections si la carte redevient le
  goulot après U1.

## Journal

- 07-04 : branche créée depuis `Bruce_Sonar_USBL` (cc96ab3+). U1 validé offline (chiffres
  ci-dessus), script `analysis/render_compass_cloud.py` commité.
- 07-04 (soir) : **configuration complète** — U2 fait (analyse.sh) ; U3 câblé
  (`USBL_SIGMA`) ; U4 codé (`LOOP_UNION`, colonne `detector` dans loops_detected.csv) ;
  U5 appliqué côté branche Bruce (keyframe_translation 1.0, protocole B″ dans
  ABLATION.md). **Prochain pas : Nathan enchaîne RU1→RU4** (tableau ci-dessus).
- RU1 : _(à remplir)_
- RU2 : _(à remplir)_
- RU3 : _(à remplir)_
- RU4 (B″, branche Bruce) : _(à remplir)_

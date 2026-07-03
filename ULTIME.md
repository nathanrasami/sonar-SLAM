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

**Reste à faire (U2)** : intégrer au flux standard.

## U2 — intégrer U1 au pipeline (offline d'abord)

1. `analyse.sh` : appeler `render_compass_cloud.py` après `filter_cloud` pour tout run
   aracati cmd_vel → `pointcloud_compass.csv/png` devient une sortie standard. (Simple,
   sans risque, à faire en premier sur cette branche.)
2. Option inline plus tard (rendu RViz/CSV directement au cap compas) : reprendre l'infra
   `cloud/use_compass_cap` de slam_ros mais avec δ auto-fit glissant au lieu de
   `cap_offset_deg` figé. À ne faire QUE si le offline devient limitant.

## U3 — σ USBL intermédiaire avec SC (1 run)

Le trade-off mesuré (SC→1.4 raide, natif→2.5 doux) n'a jamais été échantillonné ENTRE :
`usbl/sigma: 1.8` (puis 2.0 si tendance bonne), tout le reste = config 1.2a.
Succès si ATE < 1.50 sans dégrader la carte-U1 (p90 ≤ 0.45).

## U4 — union des détecteurs de loops : SC + NSSM natif (code léger, 1 run)

Aujourd'hui SC REMPLACE la présélection covariance du NSSM. Les deux voient des candidats
différents (apparence vs géométrie). Union : soumettre à shgo/ICP/PCM les candidats des
DEUX détecteurs (dédupliqués), PCM commun tranche. Attendu : constraints 116 → 150+,
sections S2/S3 raffermies. Fichier : slam.py (détection), garder le journal
loops_detected.csv avec une colonne `source` (sc|nssm).

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
  ci-dessus), script `analysis/render_compass_cloud.py` commité. Prochain pas : U2.1
  (analyse.sh) puis U3.

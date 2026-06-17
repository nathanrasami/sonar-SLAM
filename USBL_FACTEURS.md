# Fusion USBL par facteurs dans le graphe SLAM

Document de méthode — branche `Bruce_Sonar_USBL`. À lire pour valider l'approche
avant le run complet.

---

## 1. Le problème

L'odométrie **cmd_vel** (intégration des vitesses commandées) est indépendante de
la GT, mais elle **dérive en cap** (pas de compas/IMU dans le bag) → la trajectoire
se tord et l'erreur grandit sans borne (ATE brut ~47 m sur les 400 premières s,
~11.5 m Umeyama sur le run complet).

On veut **ancrer** cette dérive avec une mesure absolue **sans utiliser la GT**.

## 2. La ressource : USBL

**USBL = Ultra-Short BaseLine**, positionnement **acoustique** sous-marin. Une station
mesure la position du véhicule par temps de vol + déphasage acoustique. C'est l'équivalent
sous-marin d'un GPS bruité et lent. Dans le bag : topic `/usbl_point` (PointStamped, repère
local mètres, déjà aligné sur le repère GT).

Caractérisation mesurée (vs GT, `usbl_sim.py`) :

| Propriété | Valeur |
|---|---|
| Nombre de fixes | 977 (vs 14436 poses) |
| Erreur médiane / GT | **1.39 m** |
| Outliers (multipath) | jusqu'à **73 m** (~2 %) |
| Cadence | ~1 fix / 1.6 s, trou max 138 s |
| Indépendance GT | ✅ capteur distinct (le DGPS, lui, EST la GT → exclu) |

## 3. Pourquoi PAS au niveau du nœud d'odométrie (l'erreur initiale)

Première tentative : corriger la position publiée par un **filtre complémentaire**
(`pos += K·(usbl − pos)` à chaque fix). **Échec prouvé** (`usbl_sim.py`) :

| Méthode | ATE | Ratio trajet (zigzag) |
|---|---|---|
| Dead-reckoning seul | 46.8 m | 1.10 (lisse) |
| Filtre complémentaire K=0.1 | 7.7 m | 2.27 (zigzag) |
| Filtre complémentaire K=0.5 | 4.6 m | 3.06 (zigzag) |
| **Pose-graph (facteurs)** | **3.05 m** | **1.05 (lisse)** |

Le filtre **tire la position par à-coups** vers chaque fix bruité → la trajectoire
zigzague (trajet 2-3× trop long) et le SLAM, qui lit l'odométrie comme du mouvement
relatif lisse, interprète les sauts comme du déplacement réel. **Mauvais étage.**

## 4. La méthode retenue : facteurs USBL dans le graphe gtsam

L'odométrie cmd_vel reste **pure et lisse**. Chaque fix USBL devient un **facteur de
position absolue** (prior unaire) sur le keyframe le plus proche en temps. L'optimiseur
gtsam fusionne le tout au sens des moindres carrés : il **moyenne le bruit** USBL sur
toute la trajectoire et **corrige le cap** conjointement (avec les loop closures).

### Formulation

Pour un keyframe `k` à l'instant `t_k`, on cherche le fix USBL `(t_u, x_u, y_u)` tel que
`|t_u − t_k| < max_dt`. On ajoute :

```
facteur USBL = PriorFactorPose2( X(k), Pose2(x_u, y_u, 0), bruit )
bruit = Robust( Cauchy, Diagonal([σ_xy, σ_xy, σ_θ]) )
       avec σ_xy = 1.4 m   (bruit USBL mesuré)
            σ_θ  = 1e6      (ÉNORME → le cap n'est PAS contraint, seul x,y l'est)
```

Trois ingrédients clés :
1. **Position seulement** : `σ_θ = 1e6` rend le résidu de cap négligeable → USBL n'impose
   que x,y, le cap reste géré par l'odométrie (l'USBL ne mesure pas l'orientation).
2. **Noyau robuste Cauchy** : pondère à la baisse les gros résidus → les outliers 73 m
   sont absorbés sans déformer la trajectoire (en plus du gate en amont).
3. **Le graphe corrige le cap indirectement** : en ancrant les positions à plusieurs
   instants, l'optimiseur doit « tourner » les segments entre ancres → il répare la
   dérive de cap, ce qu'un filtre position ne pouvait pas faire.

### Rejet d'outliers en amont (gate vitesse)

Avant même le graphe, le callback rejette les glitches : un fix dont la **vitesse**
depuis le dernier fix accepté dépasse `max_speed` (3 m/s) est impossible physiquement
(ROV à ~0.24 m/s) → ignoré. Gate **indépendant de la dérive** de l'odométrie (il compare
USBL à USBL, pas à l'estimé courant).

## 5. Où c'est implémenté

| Fichier | Rôle |
|---|---|
| `bruce_slam/src/bruce_slam/slam.py` | `add_usbl(keyframe)` : trouve le fix proche, ajoute le facteur robuste position-only. Params `usbl_*` + `usbl_buffer`. |
| `bruce_slam/src/bruce_slam/slam_ros.py` | `_usbl_callback` : gate outliers, remplit `usbl_buffer`. Lit les rosparams. Appelle `add_usbl(frame)` avant `update_factor_graph`. |
| `bruce_slam/src/bruce_slam/utils/topics.py` | `USBL_TOPIC = "/usbl_point"` |
| `bruce_slam/config/slam_aracati.yaml` | section `usbl:` (enable, sigma, max_dt, max_speed) |
| `usbl_sim.py` | sandbox hors-ligne qui a validé l'approche (sans lancer le SLAM) |

Point d'insertion dans le flux SLAM (`slam_ros.py`) :
```
frame keyframe ? → add_prior / add_sequential_scan_matching   (facteur odométrie)
                 → add_usbl(frame)                            (facteur position USBL)  ← NOUVEAU
                 → update_factor_graph(frame)                 (optimisation ISAM2)
                 → loop closures (NSSM / Sonar Context)
```

## 6. Comment lancer

```bash
# active les facteurs USBL via le YAML (usbl.enable: True), puis :
./run_slam.sh
```
`odom_source` reste `cmd_vel` (l'USBL ancre cette odométrie). La GT ne sert qu'à l'ATE.

## 7. Tableau d'ablation visé

| # | Config | ATE attendu |
|---|---|---|
| 1 | cmd_vel seul | ~11.5 m |
| 2 | cmd_vel + loops | ~10.6 m |
| 3 | cmd_vel + USBL (facteurs) | **~2-4 m** (sandbox : 3 m) |
| 4 | cmd_vel + USBL + loops | ≤ 3 |

## 8. Limites connues

- **Trou USBL de 138 s** : pendant ce trou, seule l'odométrie porte → dérive locale
  temporaire, rattrapée au retour des fixes.
- **σ_θ = 1e6** est un hack propre pour un prior position-only avec `PriorFactorPose2`
  (gtsam n'a pas de facteur position 2D natif simple ici). Équivalent à un facteur
  type GPS.
- **Réglage** : `sigma` (confiance USBL) vs `odom_sigmas` (confiance odométrie) déterminent
  l'équilibre lissage/ancrage. Valeurs initiales issues de la mesure ; à affiner sur le run.

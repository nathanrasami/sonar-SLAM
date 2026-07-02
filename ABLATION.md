# ABLATION.md — Bruce pur vs bricolage (branche `Bruce`) — guide autoportant

> **But** : répondre à la question « Bruce-SLAM seul (ses modules natifs SSM + NSSM,
> réparés par le fix miroir) peut-il égaler le “bricolage” `Bruce_Sonar_USBL`
> (cmd_vel + USBL back-end + Sonar Context) ? » — avec 2 runs que TU lances.
> Tout est déjà en place sur cette branche. Contexte complet : `FABLE.md` §1 et §3
> (branche `Bruce_Sonar_USBL`).

## Pourquoi refaire ces tests (le point clé)

Les verdicts historiques « SSM diverge » (05-27, ATE 14 m) et « les loops NSSM natives
sont fausses » (06-10, ATE 11.3 m) datent d'**avant le fix miroir** : les features
étaient livrées au SLAM avec le y latéral **en miroir du cap** (bug de chiralité,
résolu le 02-07). SSM et NSSM font de l'ICP initialisé par l'odométrie : avec des
scans miroités face à une odométrie propre, l'init était systématiquement à
contre-sens en rotation → leurs échecs étaient **structurels, pas algorithmiques**.
Le fix (`flip_bearing: True` dans `bruce_slam/config/feature_aracati.yaml`, porté sur
cette branche) rend ces tests à nouveau signifiants.

## Référence à battre (run C, branche `Bruce_Sonar_USBL`)

| Run | ATE Umeyama | Cloud NN | Cap méd |
|---|---|---|---|
| **C** = `run_aracati_2026-07-02_141223` (bricolage post-fix) | **1.53 m** | **0.203** | 3.4° |
| Meilleur DR-pur historique (repère bas) | 3.7 m | — | — |

## Run A — Bruce pur (SSM + NSSM natifs, ZÉRO USBL)

```bash
git checkout Bruce
SSM=true NSSM=true USBL=false ./run_slam.sh
```

- Durée ≈ 45 min (bag complet). **Le run s'arrête TOUT SEUL** à la fin du bag
  (`required="true"` sur rosbag_play) et écrit les CSV dans
  `results/run_aracati_<date>/`. Pas de Ctrl-C à faire.
- `USBL=false` → pas de seed ni de correction USBL : départ (0,0,0). C'est voulu —
  l'ATE Umeyama aligne de toute façon (le repère global n'informe pas le SLAM).

### Évaluation (1 commande, 1 image)

```bash
python3 bilan_run.py results/run_aracati_<date>
```

→ `bilan_run.png` (trajectoire+ATE, pointcloud+NN, erreur de cap dans le temps)
+ 1 ligne de chiffres en console. Reporter la ligne dans le tableau ci-dessous.

## Run B — A + ancre USBL back-end (« le code B » = 3 variables d'env)

Rien à éditer : B s'applique sur A **uniquement par l'environnement** :

```bash
SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true ./run_slam.sh
```

- `USBL=true USBL_GAIN=0` : l'USBL **seede** le repère (position 1er fix + cap
  course-over-ground, GT-free) mais ne corrige pas l'odométrie en route.
- `USBL_BACKEND=true` : chaque fix `/usbl_point` devient un facteur gtsam
  (prior unaire Cauchy sur x,y) → ancrage global UNIQUE au back-end.
- ⚠ **Ne jamais** mettre `USBL_GAIN=0.4` avec `USBL_BACKEND=true` : double ancrage
  → zigzag (ATE 1.45→4.66 m constaté sur l'autre branche).

## Tableau à remplir

| Run | Config | ATE (m) | NN cloud | Cap méd (°) | Loops |
|---|---|---|---|---|---|
| A `194559` | SSM+NSSM, sans USBL | **1.95** | **0.204** | **2.3** | n/e* |
| B `204329` | A + USBL back-end | 2.03 | 0.218 | 2.9 | n/e* (≫ A en RViz) |
| C `141223` | bricolage (réf) | **1.53** | 0.203† | 3.4 | 82 |

*n/e = non exporté à l'époque du run (colonne `nssm_constraints` ajoutée depuis).
†cloud C = filtré I≥255 ; A/B = seuil 65 non filtré — NN comparables A↔B, pas avec C.

## ✅ VERDICT (2026-07-02)

- **A bat B sur TOUT** (ATE, cloud, cap) : l'ancre USBL telle que branchée ici
  (`usbl/sigma: 1.0`, raide) TIRE les poses hors de la solution scan-cohérente —
  murs doublés visibles dans B, +loops mais moins bonnes. Trade-off mesuré :
  ancre absolue (ATE global) vs cohérence scan (cloud/cap).
- **Champion branche Bruce = A : 1.95 m, 100 % GT-free PUR (zéro capteur absolu).**
  DR seul = 10.55 m → SSM+NSSM réparés corrigent ÷5.4. Erreur PLATE dans le temps.
- **C (bricolage) garde 0.42 m d'avance ATE** → Sonar Context justifié pour l'ATE ;
  mais A gagne le cap (2.3° vs 3.4°) et son cloud est au plafond de ses poses
  (0.204 → 0.190 avec poses parfaites, 7 % de marge).
- **Option non testée (1 run, si temps)** : B' = USBL back-end à sigma RELÂCHÉ
  (2.5–3.0) — ancre douce qui pourrait donner l'ATE de C sans casser la cohérence
  de A. Cf. mémoire « trade-off » + CONFIGS.md §1.5.

## Lecture des résultats (grille d'origine)

- **B ≈ C (≤ ~1.6 m)** → « Bruce réparé suffit » : NON ATTEINT (B=2.03).
- **C garde > 0.3 m d'avance** → OUI : le bricolage (Sonar Context) est justifié par les chiffres.
- **A** mesure la valeur propre de SSM+NSSM réparés : tout ATE < 3.7 m (meilleur DR-pur)
  = gain réel des modules natifs. A >> B est attendu (pas d'ancre globale).

## Pièges / dépannage

- `flip_bearing` doit rester `True` ici (odométrie cmd_vel). Le mettre à `False`
  UNIQUEMENT si un jour cette branche utilise l'odométrie DISO (repère réfléchi).
- Si un run doit être interrompu avant la fin : `Ctrl-C` dans le terminal du run
  suffit (roslaunch propage SIGINT → les CSV sont quand même écrits au shutdown).
- `groundtruth.csv` contient maintenant `theta` (cap compas NED) → `bilan_run.py`
  marche sans 2e argument. Pour un vieux run sans theta :
  `python3 bilan_run.py results/<run> results/<run_récent_avec_theta>`.
- Les CSV vont dans le dossier affiché au lancement (`[run_slam] Résultats dans : …`).

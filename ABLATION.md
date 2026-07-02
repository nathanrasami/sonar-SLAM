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
| A | SSM+NSSM, sans USBL | | | | |
| B | A + USBL back-end | | | | |
| C | bricolage (réf) | 1.53 | 0.203 | 3.4 | 82 |

## Lecture des résultats

- **B ≈ C (≤ ~1.6 m)** → « Bruce réparé suffit » : le récit du stage se simplifie
  (fix de chiralité + USBL back-end, sans Sonar Context). Vérifier aussi le cloud (NN).
- **C garde > 0.3 m d'avance** → le bricolage (Sonar Context) est justifié par les chiffres.
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

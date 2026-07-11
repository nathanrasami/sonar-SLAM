# TRAJ6_ANALYSE — mode d'emploi + interprétation (handoff Fable → Opus, 2026-07-12)

> Pour l'assistant qui analyse le run traj6 SANS avoir vécu les sessions précédentes.
> Contexte long : FABLE §10, §10-bis, §11. Pièges : PIEGES #14/#15/#16 (OBLIGATOIRE avant
> tout debug). L'état courant est dans PROGRESS.md.

## 1. Contexte en 5 lignes
- traj6 = MÊME trajectoire que traj5 (errance validée, run réf `222233`) + 3ᵉ capteur :
  profiler TRANSVERSE 360° → `/profiler_points`. Le SLAM ne lit QUE /sonar /dvl /imu /depth
  (identiques traj5/traj6) → l'ATE ne doit PAS bouger ; tout l'enjeu est la CARTE 3D.
- `carte_3d.py` FUSIONNE désormais vert+transverse (anti-résidus étendu).
- Mount transverse MESURÉ ×2 au probe et verrouillé par E9 : `Rz(90)@Rx(+90)` (PIEGES #16).
- Bag test : **E1–E9 TOUT PASS** (E9 : latéral 55.8 % vs 0.0 % miroir ; fond 95.4 % vs 0.0 %).
- Attendu de la fusion : fond hors nadir + flancs continus EN PLUS de ce que traj5 voyait.

## 2. Étapes exactes (dans l'ordre, UNE à la fois)
```bash
# 0. le bag complet doit exister et être validé (généré nuit du 12, notification
#    « traj6 : TOUT PASS ✅ » ; en cas de doute, re-vérifier — ~5 min :)
../holoocean-venv/bin/python check_traj4.py BAG_files/holoocean_3d_traj6.bag

# 1. run SLAM (un seul à la fois ; ne rien committer/modifier pendant)
BAG_HOLO=$PWD/BAG_files/holoocean_3d_traj6.bag ./run_slam.sh holoocean

# 2. analyses standard (bilan + rapport + carte_3d fusion, ouvre le html)
./analyse.sh 3D <nom_du_run>

# 3. comparaison chiffrée vs traj5 (lit les CSV/NPY, pas le bag)
podman exec ros1 bash -lc 'source /opt/ros/noetic/setup.bash; cd ~/…/sonar-SLAM; \
  python3 analysis/compare_traj6.py results/<run>'      # ou venv hôte, idem

# 4. facultatif (métriques M1/M2 et carte 2D dense, conteneur) :
#    python3 analysis/fusion_plus.py results/<run>
#    python3 analysis/carte_2d_dense.py results/<run> --seuil 30
```

## 3. Références chiffrées (à quoi comparer)
| métrique | traj4 `160434` | traj5 `222233` (RÉF) | traj6 attendu |
|---|---|---|---|
| ATE Umeyama / cap RMS | 0.04 m / 0.1° | 0.048 m / 0.07° | **≈ idem traj5 (|Δ|≤0.02)** |
| carte_3d filtrée | — | 18 126 pts (NN-GT 0.107/0.733) | **beaucoup + de pts** (transverse) |
| répartition carte | — | fond 24.8 % / structures 70.5 % | fond ↑ (fond continu hors nadir) |
| fusion M1 méd/p90 | 0.205/0.402 | 0.122/0.268 | ≈ traj5 (même vert, même traj) |
| fusion M2 méd/p90 | 0.052/0.709 | 0.034/0.112 | ≈ traj5 |
| carte_2d_dense s30 | 33 740 cell. (bavure) | 25 455 cell. | ≈ traj5 (même /sonar) |

## 4. Interprétation (verdicts de compare_traj6.py)
- **[1] |ΔATE| ≤ 0.02 m = attendu.** Au-delà : VRAIE anomalie (les entrées SLAM sont
  identiques) → R2 complet, ne pas « lisser ». Vérifier d'abord SSM=false/NSSM défauts.
- **[2] couverture NN(réf→run) méd ≤ 0.3 m = la fusion contient traj5.** Sinon :
  soit désalignement (regarder [1]), soit le VERT a disparu de la fusion → relire la
  sortie carte_3d : les 2 topics doivent être listés « retenus », libellé FUSION.
- **[3] apport ≥ 15 % = succès traj6** (contenu nouveau réel). ~0 % : le transverse
  n'est pas entré (détection « transverse » = std(x)≈0 ; relire la passe 1 de carte_3d).
- **NN carte-vs-GT du titre carte_3d : NE PAS le comparer au 0.107/0.733 de traj5.**
  Contenus différents = NN non comparables (PIEGES #15, leçon chiffrée §10-bis).
- Visuel carte_3d.html attendu : quais en échelles (vert) + fond continu sous la
  trajectoire + flancs/pilotis au passage (transverse), PAS de nappe au-dessus de la
  surface ni de « coin » fantôme (l'anti-résidus les coupe ; --brut pour les voir).

## 5. Interdits / pièges (résumé ; détail PIEGES.md)
- **Mount transverse FIGÉ** : une carte transverse qui « semble en miroir » ne se corrige
  JAMAIS en éditant R_MOUNT_TRANS sans re-probe (#16 : probe statique + E9, ×2).
- Un SEUL run/génération à la fois ; `ps aux | grep -E "roslaunch|gen_bag"` avant de
  committer ou de lancer quoi que ce soit.
- Ne PAS lancer avec USBL=true ; défauts holoocean = SSM=false, NSSM=true (branche.md).
- Bags traj1-3 et traj4 SUPPRIMÉS (nettoyage 2026-07-12) : ne pas chercher à re-analyser
  les runs anciens qui les référencent (160434 : artefacts archivés seulement).
- venv HÔTE pour générer/checker les bags ; conteneur ros1 pour SLAM/analyses.

## 6. STOP / escalade (R7)
STOP et question à Nathan si : E-checks FAIL sur le bag complet · ΔATE > 0.02 m non
expliqué après R2 · couverture [2] > 0.3 m avec [1] OK · crash SLAM. 2 échecs sur la
même piste = réécrire les hypothèses, pas insister.

## 7. À consigner en fin d'analyse
- PROGRESS.md : nouvelle section datée ISO en tête (chiffres du tableau §3 remplis).
- FABLE.md : §11-bis « traj6 run complet » (verdicts [1][2][3], hypothèses rejetées).
- Commit + push (message chiffré), APRÈS `ps aux | grep roslaunch`.
- Si succès : proposer à Nathan la suite (loops SC traj5 · fusion patchs polaires
  StereoFLS · threshold 50→30) — et une NOUVELLE discussion.

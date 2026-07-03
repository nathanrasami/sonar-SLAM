# PROGRESS — état au 2026-07-03

> Docs : **FABLE.md** (investigations) · **CONFIGS.md** (recettes) · **PIEGES.md** (à ne
> jamais faire) · **ABLATION.md** (A/B, branche Bruce). Branches : main, Bruce,
> Bruce_Sonar_USBL, holoocean (les autres = tags `archive/*`).

## Aracati — état des runs

| Run | Config | ATE | NN | Cap méd | Loops |
|---|---|---|---|---|---|
| A `194559` (+A-bis `214846`) | champion Bruce pur (SSM+NSSM, 0 USBL) | 1.95–2.04 | 0.204* | 2.3–2.9° | 124 natives |
| B `204329` | A + USBL sigma 1.0 (raide) | 2.03 | 0.218* | 2.9° | — |
| **1.2a `003823`** | **champion New** (SC seuil 0.70 + USBL 1.4) | **1.50** | 0.204 | **2.6°** | 230 retenus / 116 |
| Réf GT `011733` | DISO+GT (pas GT-free) | 0.89 | 0.199 | 1.7° | — |

*seuil 65 non filtré (≠ seuil 255 des runs BSU) — comparer au même seuil via filter_cloud.

- Découvertes clés : fix miroir → PCM 6→82→116 constraints ; ancre raide dégrade la
  cohérence scan (B) ; ZÉRO constraint à t=13-16.5 min = fenêtre du décrochage 5.5 m de A.
- **PRÊTS À LANCER** (un à la fois, arrêt auto partout) :
  - **1.3** (Bruce_Sonar_USBL) : SSM on + export cloud complet → `./run_slam.sh`
  - **B′** (Bruce) : sigma 2.5 → `SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true ./run_slam.sh`
- Ensuite : 1.4 combo → champion New figé ; loterie 3.1 DISO wz inversé ; **comparaison
  finale champion vs champion → mini-papier** (FABLE §7).

## HoloOcean (branche holoocean, bag `test.bag` — ex test, renommé)

- **2D validé** (carrés visibles après assouplissement extraction 50/5) ; **2.5D en place**
  (colonne z dans les 3 CSV ; z = /depth en mode dvl, z GT en mode gt).
- ⚠ **ATE en mode gt = circulaire (~0 par construction : l'odométrie EST la GT).**
  L'ATE qui compte = `ODOM_SOURCE=dvl ./run_slam.sh holoocean` (0.13 m au smoke).
- **Arcs dans l'image sonar BRUTE = artefact simulateur** (pas notre pipeline) → à signaler
  au collègue avec la spec vraie-3D (SLAM_3D_MIGRATION.md §proposition, point 4).
- `/sonar_points` du bag : z=0 partout → vraie 3D = bag collègue requis.
- Opt-in loop closure : `NSSM=true ./run_slam.sh holoocean` (la boucle carrée revient au
  départ → 1-2 vraies boucles possibles).
- `test.bag` (714 Mo) désindexé de git (limite GitHub 100 Mo) — le fichier reste sur disque.

## Papier à présenter

**SONIC** (CMU/Kaess) — `Paper/Sonar/SONIC.md`. Réserve : INS/USBL/DVL FGO (Paper/Factor Graph/).

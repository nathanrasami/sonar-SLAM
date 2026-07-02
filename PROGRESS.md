# PROGRESS — état au 2026-07-02 (soir), branche `Bruce_Sonar_USBL`

> Doc d'investigation détaillé : **FABLE.md** (v2). Historique : STAGE.md (journal 07-02).

## ✅ Fait aujourd'hui — le tourbillon est RÉSOLU et VALIDÉ

- **Cause racine** : bug de miroir — features `y latéral = +droite` (repère réfléchi « même
  signe que DISO ») vs odométrie cmd_vel propre → chaque scan peint en miroir de son cap.
- **Fix** : `cartesian/flip_bearing` (feature_extraction.py, dans le packing APRÈS la relecture
  d'intensité) + `flip_bearing: True` (feature_aracati.yaml ; **False si odom_source=diso**).
- **Run de validation `run_aracati_2026-07-02_141223`** (100 % GT-free, natif) :
  **NN 0.203** (vs 0.365), aire ÷2.2, **quai en T visible**, ATE 1.53 m, cap 3.4° méd/5.8° RMS,
  82 loops (vs ~122 — à investiguer, cf. FABLE §4.2). Figure : `bilan_run.png` du run.
- Run `111934` = INVALIDE (1er essai du flip avant la relecture d'intensité → 4 325 pts).
- Post-traitement des anciens runs : `fix_mirror_cloud.py` (150034 : 0.365→0.208 ;
  150959 : 0.469→0.271). ⚠ Pas pour 111746 (rendu use_compass_cap).
- Audit GT-free ✅ (nuance : wz de /cmd_vel = dérivé compas, cf. README aracati — assumé).
- Nouveaux outils : `bilan_run.py` (1 image bilan/run, branché dans `analyse.sh`).
- Ménage : PROMPT_FABLE.md supprimé ; branche `slam3-d` → `holoocean` ; retour sur
  `Bruce_Sonar_USBL` (modifs portées, RIEN de commité). FABLE.md réécrit (v2) ;
  SLAM_3D_MIGRATION.md et STAGE.md à jour.
- NB : CODE.md / IMU_ISSUE_REPORT.md / PISTE.md apparaissent supprimés dans l'arbre de
  travail (pas par moi — ménage utilisateur présumé).

## 🎯 Prochaines étapes — suivre **FABLE.md §4** (matrice de configs, ordre imposé)

- **Phase 1** (cette branche) : 1.1 diagnostic loops (offline) → 1.2 loops renforcés →
  1.3 SSM on → 1.4 combo → 1.5 USBL sigma. Un changement par run + `bilan_run.py`.
- **Phase 2** (branche `Bruce`) : ablation **A puis B** — tout est prêt, suivre
  **`ABLATION.md`** sur la branche `Bruce` (runs lancés par Nathan, arrêt automatique).
- **Phase 3** (à créer) : 3.1 DISO wz inversé, 3.2 MCFAR. Cible : **ATE < 1 m** + quai en T,
  puis **mini-papier** (FABLE §7).
- **HoloOcean** : branche `holoocean` reconstruite, config 2D prête :
  `./run_slam.sh holoocean` (bag test_2.bag, 61 s, odom GT par défaut, `ODOM_SOURCE=dvl` sinon).

## Chiffres de référence (à jour)

| Config | ATE | Cloud NN | Note |
|---|---|---|---|
| GT-free post-fix (141223) | 1.53 m | **0.203** | LE run courant |
| GT-free pré-fix (150034) | 1.46 m | 0.365 (0.208 dé-miroité) | |
| Réf DISO+GT (011733) | 0.89 m | 0.199 | pas GT-free (cible visuelle) |
| Plafond poses GT (mêmes détections) | — | 0.138 | marge restante via les poses |

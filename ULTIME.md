# ULTIME.md — phase ULTIME (CLOSE) — document d'historique

> **07-05 : branche `Bruce_Ultime` FUSIONNÉE dans `Bruce_Sonar_USBL` puis archivée**
> (tag `archive/Bruce_Ultime`). Raison : la répétabilité a montré σ1.4 ≈ σ1.8 (BSU méd
> 1.50 vs Ultime 1.54) et tout le reste (rendu compas, analyse) était déjà partagé —
> aucun avantage propre. Ce qui a été porté ici : le code des expériences U4/U6
> (union, σ adaptatif — REJETÉES, off par défaut) et les boutons d'env
> USBL_SIGMA / LOOP_UNION / USBL_ADAPTIVE. Le champion de CETTE branche reste σ1.4.

# (historique) branche `Bruce_Ultime` : fusionner le meilleur des deux mondes

> Phase lancée le 2026-07-04. Base : `Bruce_Sonar_USBL` (champion traj 1.2a, ATE 1.50 m).
> Objectif : y ramener l'avantage CARTE de la branche `Bruce` (B′ : méd 0.09 / p90 0.74),
> puis dépasser les deux. **Lire `PIEGES.md` avant tout.** Règles : 1 changement par run ;
> éval = `./analyse.sh <run>` + `python3 analysis/paper_eval.py <run>` (protocole papier).

## Cible chiffrée

| | traj (ATE um.) | carte méd/p90 vs vraie | référence |
|---|---|---|---|
| état de départ (1.2a `003823`) | 1.50 m | 0.11 / 0.99 | mini-papier §6.3-6.4 |
| **🏆 CHAMPION ULTIME : RU1 σ1.8** (`125434-RU1`) | **1.47 m** | **0.075 / 0.413** (rendu compas) | config FIGÉE dans le yaml |
| meilleur Bruce pur (B′ `120352-1`) | 1.88 m | 0.09 / 0.74 | ABLATION.md |
| borne assistée GT (`011733`) | 0.89 m | 0.11 / 0.40 | plancher pratique |
| cible Ultime | ≤ 1.4 m (reste 0.07) | ≤ 0.09 / ≤ 0.45 ✅ ATTEINTE | carte : mieux que la réf GT en méd |

RU1 gagne TOUT côté carte (NN compas 0.172 = meilleur cloud du stage, p90 0.413 ≈ borne
GT 0.40) et bat 1.2a en traj (1.47, S3 1.20, RE 5.25 %, 125 constraints). Cap méd 3.5°
(vs 2.6 pour 1.2a) : l'ancre plus raide de RU1 bouge un peu le θ optimisé — sans effet
carte puisque le rendu compas s'en affranchit (c'est exactement la division du travail
U1 : position ← USBL/loops, cap de rendu ← compas).

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

## U3 — σ USBL intermédiaire avec SC ✅ VERDICT (RU1/RU2, 07-04) : σ1.8 CHAMPION

Échantillonnage (loops SC, sans SSM) : **1.4 → 1.50** (1.2a) ; **1.8 → 1.47** (RU1) ;
**2.0 → 1.60** (RU2). L'optimum est ~1.8 — cohérent avec la médiane du bruit USBL réel
mesurée offline (1.37 m global, mais variable, cf. U6). **σ1.8 FIGÉ dans le yaml** :
`./run_slam.sh` nu reproduit le champion. Env `USBL_SIGMA` conservé pour rejouer un σ.

## U4 — union des détecteurs SC + NSSM natif ❌ REJETÉ (RU3, 07-04)

**RU3 (`145924-RU3`) : ATE 2.91 m** (vs 1.47 RU1), cap 5.0°, carte 0.15/1.14, 615 KF
seulement (CPU saturé → frames sonar perdues). Diagnostic (loops_detected.csv, colonne
`detector`) : le gating natif a proposé **583 candidats** (vs 353 SC) **sans aucune porte
géométrique** → shgo/ICP inondés ET le PCM a laissé passer de mauvaises contraintes
corrélées (210 constraints dont des fausses → S3 3.70 m). C'est EXACTEMENT la voie de
faux positifs que SC+gate avait éliminée : l'union la réintroduit. Le code reste en place
(défaut False) ; une U4-bis (porte géométrique 10 m appliquée AUSSI aux candidats natifs)
serait possible mais l'intérêt est faible : RU1 montre que les loops SC suffisent.

## 🎬 SÉQUENCE RU1-RU4 : FAITE (07-04) — verdicts

| # | Run | Teste | Verdict |
|---|---|---|---|
| RU1 | `125434-RU1` | σ1.8 | **🏆 CHAMPION : ATE 1.47, carte compas 0.075/0.413, NN 0.172** |
| RU2 | `134157-RU2` | σ2.0 | 1.60 → optimum confirmé ~1.8 |
| RU3 | `145924-RU3` | union SC+natif | ❌ 2.91 (faux positifs natifs + CPU, cf. U4) |
| RU4 | `114439-RU4` | B″ keyframes 1.0 (Bruce) | ❌ 17.17 (fausses loops court-terme, cf. U5) |

**σ1.8 figé dans le yaml** → `./run_slam.sh` nu = champion (RU1 en est le run de
validation). Prochain run utile : **RU5** ci-dessous (U6).

| # | Run | Teste | Verdict |
|---|---|---|---|
| RU5 | `161907-RU5` | U6 σ adaptatif | ❌ 1.62 — le σ fixe 1.8 reste champion (cf. U6) |

## 🏁 PHASE ULTIME CLOSE (07-04) — LIVRABLE

**Champion final : RU1 `125434-RU1`** — reproductible par `./run_slam.sh` nu (branche
`Bruce_Ultime`, tout est figé dans les yaml). Livrables carte : `pointcloud_compass.csv/png`
(sortie standard d'`./analyse.sh`). Bilan face aux références :

| | ATE | carte méd/p90 | NN |
|---|---|---|---|
| **Ultime (RU1 + rendu compas)** | **1.47 m** | **0.075 / 0.413** | **0.172** |
| meilleur Bruce pur (B′) | 1.88 m | 0.09 / 0.74 | 0.205 |
| borne assistée GT | 0.89 m | 0.11 / 0.40 | 0.199 |

Sur l'épaisseur du nuage RViz en fin de run : c'est attendu — RViz superpose les scans
aux poses optimisées, et les passages successifs gardent un drift résiduel de l'ordre de
la métrique carte (méd ~8 cm, queue ~40 cm) → murs « épais ». La carte fine est le
produit OFFLINE (`pointcloud_compass` + filtre intensité) ; un republish RViz du rendu
compas (U2-inline) est possible si besoin de démo live, sinon inutile.

## U5 — keyframes 1.0 m sur `Bruce` ❌ ÉCHEC INSTRUCTIF (RU4, 07-04) — ROLLBACK fait

**RU4 (`114439-RU4`) : ATE 17.17 m, cap 10°, carte détruite.** La densification a bien
donné 666 KF, mais **les fenêtres NSSM sont en KEYFRAMES, pas en mètres** : `min_st_sep: 8`
ne représentait plus que ~8 m/16 s d'exclusion (au lieu de 24 m) → auto-appariements
court-terme pris pour des loops. Preuve : 1re « loop » à **t=3.0 min** et 47 contraintes
avant 8 min, alors qu'aucune vraie revisite n'existe (B′ : 1re à 12.5 min, 0 avant 8).
415 contraintes majoritairement fausses → graphe détruit. **Rollback appliqué**
(`keyframe_translation: 3.0`, B′ reste champion Bruce). Piège documenté : PIEGES §11.
**B″-bis possible** (1 run, si envie) : 1.0 m AVEC les fenêtres rescalées ×3
(`min_st_sep: 24`, `pcm_queue_size: 15`, `source_frames: 15`) — recette dans ABLATION.md.

## U6 — σ USBL adaptatif par fix ❌ REJETÉ (RU5, 07-04) — σ fixe 1.8 reste champion

**RU5 (`161907-RU5`, USBL_ADAPTIVE=true) : ATE 1.62 m** (vs 1.47 RU1), carte compas
0.078/0.484 (vs 0.075/0.413), 120 constraints (vs 125). Le proxy estime bien le bruit
(corr. 0.64) mais l'imperfection résiduelle se paie : raidir à 0.9 dans les fenêtres
« propres » sur-contraint par endroits (même mécanisme que B σ1.0 sur la branche Bruce),
et le graphe moyenne déjà naturellement les fixes — moduler la confiance ajoute du bruit
de confiance sans bénéfice net ici. Le code reste (défaut False) ; variante non testée
si on y revient : `adaptive_min: 1.4` (ne jamais être PLUS raide que le champion).
Décision : ne pas sur-optimiser à ~0.1 m près — **la phase Ultime est close sur RU1**.

### (archive) justification et implémentation U6

**Test interne (07-04, bag vs GT, offline)** : le bruit USBL réel VARIE ×3.5 le long de
la mission — méd 0.87 m (t=10-15 min) à 3.09 m (t=25-30, fenêtre de dropouts jusqu'à
138 s entre fixes) ; 1 % d'outliers >20 m. Un σ fixe est prouvé sous-optimal.
**Proxy GT-free validé** : dispersion locale des fixes (écart à la médiane ±4 voisins,
MAD glissant ~21 fixes) → corrélation **0.64** (Spearman) avec le vrai résidu, retrouve
les fenêtres propre/sale sans GT. Loi : σ_i = clip(3.4·MAD, 0.9, 3.5) — médiane 1.8
(= champion RU1), p10 0.90, p90 3.50.
**Implémenté** : `slam.py::_usbl_adaptive_sigma` (+ params `usbl/adaptive*`, env
`USBL_ADAPTIVE`, sigma loggé par facteur). Défaut False = champion intact. → **RU5**.

## U7 — réserve + verdict ISOPoT / SONIC (07-04)

- **ISOPoT : NON faisable dans le stage.** Code non publié ; le cœur est un tracker de
  points vidéo lourd (TAPNext) à réimplémenter et adapter au sonar — hors de portée.
  On garde ses idées déjà intégrées : protocole d'éval (sections, RE) et le constat
  « détecteurs locaux épars faibles sur Aracati » (cohérent avec notre pivot cmd_vel).
- **SONIC : faisable en TEST OFFLINE seulement** (code + poids publiés, rpl-cmu/sonic) :
  rejouer nos paires candidates (CONFIGS.md#sonic-offline) et comparer aux transforms
  ICP. Risque élevé de domain gap (entraîné HoloOcean M1200d/10 m vs P900/48 m réel,
  et SONIC échoue déjà sur Aracati dans la table d'ISOPoT : ATE 36-113 m sonar-seul).
  Effort ~1 journée env PyTorch + extraction d'images. À ne tenter QUE si le goulot
  redevient la conversion des candidats (40/122) — pas prioritaire : RU1 a 125
  constraints et la carte est à la borne GT.
- MCFAR (CONFIGS.md#32-mcfar) : en réserve si la carte redevient le goulot.

## Journal

- 07-04 : branche créée depuis `Bruce_Sonar_USBL` (cc96ab3+). U1 validé offline (chiffres
  ci-dessus), script `analysis/render_compass_cloud.py` commité.
- 07-04 (soir) : **configuration complète** — U2 fait (analyse.sh) ; U3 câblé
  (`USBL_SIGMA`) ; U4 codé (`LOOP_UNION`, colonne `detector` dans loops_detected.csv) ;
  U5 appliqué côté branche Bruce (keyframe_translation 1.0, protocole B″ dans
  ABLATION.md). Nathan enchaîne RU1→RU4.
- RU1 `125434-RU1` (σ1.8) : **🏆 1.47 m**, cap 3.5°, 125 constraints, NN compas 0.172,
  carte compas 0.075/0.413 (≈ borne GT 0.40). → σ1.8 FIGÉ dans le yaml.
- RU2 `134157-RU2` (σ2.0) : 1.60 m → optimum ~1.8 confirmé (1.4:1.50 / 1.8:1.47 / 2.0:1.60).
- RU3 `145924-RU3` (union) : ❌ 2.91 m — 583 candidats natifs sans porte → PCM pollué +
  CPU saturé (615 KF). U4 rejeté, code laissé en place (défaut off).
- RU4 `114439-RU4` (B″ KF 1.0, branche Bruce) : ❌ 17.17 m — min_st_sep en KEYFRAMES →
  fausses loops court-terme (1re à t=3 min). Rollback 3.0 fait, PIEGES §11, B″-bis
  optionnelle dans ABLATION.md.
- 07-04 (nuit) : **U6 codé** (σ adaptatif par fix, proxy MAD validé offline corr. 0.64,
  loi clip(3.4·MAD, 0.9, 3.5)) → RU5. Verdict U7 :
  ISOPoT infaisable (code non publié) ; SONIC = test offline en réserve seulement.
- RU5 `161907-RU5` (σ adaptatif) : ❌ 1.62 m, carte 0.078/0.484, 120 constraints →
  U6 rejeté, σ fixe 1.8 champion. **PHASE ULTIME CLOSE sur RU1** (section 🏁 ci-dessus).
- 07-05 : **runs finaux de répétabilité** (TESTS.md §2.4) — Ultime σ1.8 : 1.54 / 1.60 ;
  BSU σ1.4 : 1.45 / 1.52. Verdict honnête : σ1.4 ≈ σ1.8 dans la variance ICP (±0.1) ;
  livrable robuste = ATE 1.5 ± 0.1, carte compas 0.075/0.43 (les cartes compas des 4 runs
  sont interchangeables). Audit GT-free vérifié code (TESTS.md §2.6). Phase DÉFINITIVEMENT
  close.

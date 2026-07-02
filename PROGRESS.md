# PROGRESS — état au 2026-07-03

> Docs : **FABLE.md** (investigations, v2) · **CONFIGS.md** (recettes par piste) ·
> **PIEGES.md** (à ne jamais faire) · **ABLATION.md** (verdict A/B, branche Bruce) ·
> STAGE.md (journal). Les 4 branches (main, Bruce, Bruce_Sonar_USBL, holoocean)
> partagent docs + outils d'analyse ; les autres branches sont archivées en tags `archive/*`.

## État des runs (tout au 07-02 soir)

| Run | Config | ATE | NN | Cap méd | Loops |
|---|---|---|---|---|---|
| **A** `194559` | **champion Bruce pur** (SSM+NSSM, 0 USBL), branche Bruce | **1.95 m** | 0.204* | **2.3°** | n/e |
| B `204329` | A + USBL back-end sigma 1.0 | 2.03 m | 0.218* | 2.9° | n/e |
| **C** `141223` | **champion actuel** (USBL+SC), branche Bruce_Sonar_USBL | **1.53 m** | 0.203 | 3.4° | 82 |
| Réf GT `011733` | DISO+GT (PAS GT-free, cible visuelle) | 0.89 m | 0.199 | 1.7° | — |

*seuil 65 non filtré — comparable A↔B seulement. n/e = loops non exportées à l'époque
(corrigé : la branche Bruce exporte désormais `nssm_constraints`).

## Acquis majeurs

1. **Tourbillon RÉSOLU** (bug de chiralité, fix `flip_bearing`) — quai en T GT-free,
   loops PCM 6→82. Cf. FABLE §1, PIEGES §1.
2. **Ablation FAITE** : Bruce pur réparé = 1.95 m sans AUCUN capteur absolu (DR 10.55 ÷5.4) ;
   l'ancre USBL raide dégrade (B) ; le bricolage garde 0.42 m d'avance → SC justifié.
3. Anatomie du résidu de A : warp uniforme ~1.35 m + UN décrochage local 5.5 m (t≈14-15 min),
   pas un manque de loops. Cloud limité par les détections, plus par les poses.

## 🎯 Prochaines étapes (plan resserré — plus de tests tous azimuts)

1. **Run 1.2a** (branche Bruce_Sonar_USBL, PRÉPARÉ — dist_threshold 0.70 commité) :
   `git checkout Bruce_Sonar_USBL && ./run_slam.sh` ; arrêt : Ctrl-C à la fin du bag
   (PIEGES §5) ; éval `python3 bilan_run.py results/<run>`. Attendu : constraints 82→150+,
   ATE <1.4. [CONFIGS.md#12a]
2. **Run 1.3** (SSM on) puis **1.4** combo → **champion Bruce_New figé** (~1.2 m visé).
3. Option Bruce pur : **B′ sigma relâché 2.5** (FABLE §3 post-ablation) — 1 run.
4. Loterie **3.1 DISO wz inversé** — 1 run borné. [CONFIGS.md#31]
5. **Comparaison finale champion Bruce vs champion Bruce_New** → mini-papier (FABLE §7),
   fin de la contribution Aracati. Puis HoloOcean (2D prêt : `./run_slam.sh holoocean`).

## Papier à présenter

**SONIC** (CMU/Kaess, arXiv 2310.15023) — résumé : `Paper/Sonar/SONIC.md`. Attaque notre
goulot (association aux revisites), entraîné sur HoloOcean, code public. Réserve phase
1.5/USBL : « INS/USBL/DVL factor graph » (Paper/Factor Graph/).

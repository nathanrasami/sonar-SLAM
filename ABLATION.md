# ABLATION.md — branche `Bruce` : état et PROCHAIN RUN (B′)

> A et B sont FAITS (verdict ci-dessous). **Le prochain run de cette branche est B′**
> (ancre USBL douce). Contexte : `FABLE.md` §3, recettes : `CONFIGS.md`, pièges : `PIEGES.md`.

## Résultats acquis (ne pas refaire)

| Run | Config | ATE (m) | NN cloud* | Cap méd | Loops |
|---|---|---|---|---|---|
| A `194559` / A-bis `214846` | SSM+NSSM, sans USBL | **1.95** / 2.04 | 0.204 / 0.205 | **2.3°** / 2.9° | 124 natives |
| B `204329` | A + USBL back-end **sigma 1.0** | 2.03 | 0.218 | 2.9° | n/e |
| C `003823` (réf, branche Bruce_Sonar_USBL) | SC 0.70 + USBL 1.4 | **1.50** | 0.204 | 2.6° | 230/116 |

*clouds seuil 65 non filtrés (≠ seuil 255) — comparer au même seuil via `filter_cloud.py`.

**Verdict** : l'ancre raide (sigma 1.0) DÉGRADE la solution scan-cohérente (murs doublés).
A varie ±0.1 m run-à-run (aléas ICP non seedés). Découverte : **zéro constraint dans
t=13-16.5 min** (fenêtre du décrochage 5.5 m de A, pourtant riche en revisites).

## ▶ PROCHAIN RUN : B′ — ancre douce (sigma 2.5, déjà commité dans le yaml)

```bash
git checkout Bruce
SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true ./run_slam.sh
```

- S'arrête TOUT SEUL à la fin du bag (~45 min). Un seul run à la fois (PIEGES §4).
- `USBL_GAIN=0` OBLIGATOIRE (sinon double ancrage, PIEGES §2). Le sigma 2.5 est
  DÉJÀ dans `slam_aracati.yaml` (ne rien éditer).
- Évaluation : `python3 analysis/bilan_run.py results/run_aracati_<date>`

**Succès si** : ATE < 1.8 m ET NN ≤ 0.205 (la cohérence de A préservée) — l'ancre douce
doit rabattre le warp uniforme ~1.35 m et plafonner l'excursion t≈14-15 min.
**Si ATE ≥ 1.9** : l'ancre n'apporte rien à ce pipeline → champion Bruce = A, on passe
à la comparaison finale. **Si NN > 0.21** : sigma encore trop raide → 3.5 (dernier essai).

## Après B′

Champion `Bruce` figé (A ou B′) → **comparaison finale vs champion `Bruce_Sonar_USBL`**
(1.2a=1.50, 1.3 en cours) → mini-papier (FABLE §7). Loterie DISO : branche
`Bruce_DISO_wz` (préparée, cf. CONFIGS.md #31), à lancer après les perfectionnements.

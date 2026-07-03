# ABLATION.md — branche `Bruce` : état et PROCHAIN RUN (B″, keyframes 1.0)

> A, B et B′ sont FAITS (verdicts ci-dessous). **Le prochain run de cette branche est B″**
> (U5 : keyframes densifiées — voir section dédiée). Contexte : `FABLE.md` §3/§8,
> recettes : `CONFIGS.md`, pièges : `PIEGES.md`, papier : `BRUCE_SLAM.md`.

## 🎬 PROCHAIN RUN : B″ — keyframes 3.0 → 1.0 m (U5, RU4 de la séquence ULTIME)

**Pourquoi** : la trajectoire B′ est « géométrique » — 256 keyframes espacées de ~3 m
(`keyframe_translation: 3.0`, réglage upstream), segments droits entre. À 1.0 m (comme
BSU) : ~665 KF, trajectoire lissée, nuage rendu depuis plus de poses. Coût ~×2.6 en
ICP/NSSM (ok). Le yaml est DÉJÀ MODIFIÉ (07-04) — ne rien éditer.

```bash
git checkout Bruce
SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true ./run_slam.sh   # = B′ + KF 1.0
./analyse.sh run_aracati_<date>                    # inclut désormais la carte compas (U1)
python3 analysis/paper_eval.py results/run_aracati_<date>
```

**Verdict** : B″ remplace B′ comme champion si ATE < 1.88 m (à cohérence égale, NN ≤ 0.21
au seuil 65). Si ATE ≥ 1.88 : rollback `keyframe_translation: 3.0`, B′ reste champion —
et la densification n'aura servi qu'au lissage visuel (documenter dans BRUCE_SLAM.md §6.1).
⚠ Si le CPU sature (3× plus de keyframes → SSM/NSSM plus fréquents) : `RATE=0.5`.

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

## ✅ B′ FAIT (run `120352-1`) — CHAMPION de la branche Bruce

**ATE 1.88 m, cap 2.6° méd/3.8° RMS, NN 0.205, 130 constraints natives (record), DR 10.58.**
L'ancre douce (σ2.5) améliore A (1.95→1.88) SANS casser la cohérence (NN 0.205 ≈ A 0.204) :
succès exactement selon le critère (<1.8 raté de 0.08 m, mais meilleur des runs Bruce).
Config CHAMPION FIGÉE dans le yaml (σ2.5 + SSM/NSSM via envs). Verdict de la comparaison
finale : champion New (1.2a, 1.50 m) garde **0.38 m d'avance** → Sonar Context justifié.
Commande de reproduction :
`SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true ./run_slam.sh`

## (Archive) protocole B′ d'origine

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

# ABLATION.md — branche `Bruce` : CLOSE (champion B′) — B″ testé et rejeté

> A, B, B′ et B″ sont FAITS. **Champion de la branche : B′ (1.88 m)**, yaml revenu à sa
> config (rollback keyframes fait le 07-04). Contexte : `FABLE.md` §3/§8, pièges :
> `PIEGES.md` (§11 est né ici), papier : `BRUCE_SLAM.md`.

## ❌ B″ FAIT (run `114439-RU4`) — ÉCHEC INSTRUCTIF : ATE 17.17 m, ROLLBACK appliqué

**Ce qui s'est passé** : la densification a fonctionné (666 KF) mais `min_st_sep: 8`,
`pcm_queue_size` et `source_frames` comptent des KEYFRAMES, pas des mètres. À 1.0 m/KF,
l'exclusion de revisite est tombée de ~24 m à ~8 m → le NSSM a validé des
auto-appariements court-terme comme loops : **1re « loop » à t=3.0 min, 47 contraintes
avant 8 min** (B′ : 1re à 12.5 min, 0 avant 8) → 415 contraintes majoritairement fausses,
cap 10°, carte détruite (0.19/2.92). Piège générique documenté : **PIEGES §11**.

**Rollback fait** : `keyframe_translation: 3.0` (champion B′ reproductible tel quel).

**B″-bis (optionnelle, 1 run)** — la densification RESTE bonne pour lisser la trajectoire,
à condition de rescaler les fenêtres du même facteur ×3 :
```yaml
keyframe_translation: 1.0   # slam_aracati.yaml
nssm:
  min_st_sep: 24            # 8 → 24  (≈ 24 m d'exclusion, comme avant)
  source_frames: 15         # 5 → 15  (même contexte métrique de submap)
pcm_queue_size: 15          # 5 → 15  (même fenêtre métrique de cohérence)
```
puis `SSM=true NSSM=true USBL=true USBL_GAIN=0 USBL_BACKEND=true RATE=0.5 ./run_slam.sh`
(RATE 0.5 : 2.6× plus de keyframes = plus de charge). Verdict : remplace B′ si ATE < 1.88
à NN ≤ 0.21 (seuil 65). Sans ça, la trajectoire « géométrique » reste un artefact
d'affichage assumé (BRUCE_SLAM.md §6.1).

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

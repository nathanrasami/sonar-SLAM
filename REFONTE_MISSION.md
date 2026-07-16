# REFONTE_MISSION.md — 4 méthodes propres, 1 état de code (session « Ultracode »)

> Décisions Nathan 2026-07-17 (fin de discussion papier). Contexte : les configs
> historiques mélangent seed USBL dans l'odométrie (−31° mesuré), branches au code
> divergent, et 3 conventions d'évaluation successives. On repart PROPRE.
> **Interdit de coder quoi que ce soit qui contredit les décisions ⛔ ci-dessous.**

## ⛔ Décisions FIGÉES (ne pas re-débattre en session de code)

1. **Métrique unique « origine »** = TRANSLATION PURE : GT et estimé épinglés à
   (0,0), AUCUNE rotation/réflexion ajustée, nulle part, jamais. Sections S1/S2/S3 =
   tiers temporels ré-épinglés translation. Déjà implémentée :
   `analysis/paper_figs_origine.py` + `analysis/analyze_origine.py` (alignés 17-07).
   Historique interdit : corde 15 m, fit 15 % des poses, Umeyama (comparaison
   uniquement en interne, jamais dans le papier).
2. **Odométrie SACRÉE** : le nœud cmd_vel→odom ne s'abonne JAMAIS à l'USBL.
   Ni seed, ni correction, ni gain. Même profil d'odométrie dans les 4 méthodes.
   **Gate de vérification : les traces dr des 4 runs doivent être identiques**
   (rigid-check ×4 : rotation 0.00°, résidu ~0 — script 10 lignes, pattern du
   17-07). Un run qui échoue ce gate est invalide.
3. **USBL = facteurs unaires back-end UNIQUEMENT** (position seule, Cauchy,
   σ champion 1.4 pour BSU / 2.5 pour Bruce_U — à re-vérifier sur 1 run chacun).
   Jamais de front-end. (PIEGES #2 double ancrage, PIEGES #25 seed.)
4. **4 méthodes, 1 seul état de code, 2 interrupteurs** (SC × USBL) :
   | Nom | Sonar Context | USBL back-end |
   |---|---|---|
   | Bruce | off (NSSM natif) | off |
   | Bruce_U | off (NSSM natif) | on |
   | Bruce_Sonar | on | off |
   | BSU | on | on |
   **1 run chacun** (décision Nathan ; ⚠ variance ICP ±0.1 m connue — si un chiffre
   est surprenant, ×2 avant verdict, R3).
5. **Seed initial : GT-free strict** (jamais /pose_gt, même pas t0). Position :
   (0,0) (la métrique translation-pure l'ignore de toute façon). **Cap : à trancher
   en ÉTAPE 0 par mesure** (voir ci-dessous) — objectif ATE origine < 2 m ou proche
   pour BSU, et ordre attendu : BSU ≤ Bruce_Sonar/Bruce_U < Bruce.
   Si BS/BSU ressort PIRE que Bruce : STOP, investigation R2, pas de rapport.

## ÉTAPE 0 (avant tout code) — trancher le cap de seed par MESURE

Deux candidats GT-free, à évaluer offline sur le bag (10-20 lignes, conteneur ros1) :
- (a) **cap fixe 0** : sur CE dataset il tombe ~juste (β mesuré 90.3° nominal ±0-4°
  sur les runs sans USBL) — simple mais « chanceux », à assumer comme tel ;
- (b) **route-fond USBL longue base** : direction du déplacement sur les premiers
  ≥ 20-30 m de piste USBL (fit robuste sur N fixes, PAS 1 m : bruit fix 1.4 m,
  cf. PIEGES #25). Attendu ~2-3° d'erreur → ~0.3-0.5 m d'ATE. Défendable
  (équivalent embarqué d'un /initialpose).
Mesure : calculer (a) et (b) offline, comparer l'angle à la route-fond DGPS des
30 premiers mètres (GT utilisée pour VALIDER le choix a posteriori, pas pour
seeder). Retenir la règle, la déclarer dans le papier, ne plus y toucher.

## ✅ ÉTAPE 0 — TRANCHÉ par MESURE (2026-07-16, `analysis/etape0_seed_cap.py`, ×2 identiques)

- θ0 vrai à t0 (fit rigide cmd_vel→DGPS) : **−1.1° / +2.5°** (bases 20/30 m) →
  **(a) cap fixe 0 RETENU** (erreur 1.1-2.5°, la croissance à 30-40 m = dérive cmd_vel).
- (b) route-fond USBL CORRECTE = fit de FORME Kabsch (trajectoire cmd_vel θ0=0 → fixes
  USBL) : 2.2-4.0° d'erreur — pas mieux que (a), plus complexe → rejeté.
- ⚠ (b) NAÏF (atan2/course de la piste USBL, même en longue base) : **45-78° d'erreur** —
  le ROV tourne de −88° pendant les 30 premiers mètres, la course n'est PAS le cap.
  PIEGES #25 aggravé : la longue base ne suffit pas, il faut un fit de forme.
- Règle déclarée (papier) : seed = (0,0), cap 0, codé EN DUR dans cmd_vel_odom.py
  (zéro paramètre). Ne plus y toucher.

## Chantier code (session Ultracode — branche NEUVE)

1. **Nouvelle branche** (proposition : `refonte`, basée sur `Bruce_Sonar_USBL` qui
   a SC + USBL + les derniers fix) — Nathan tranche le nom/la base au lancement.
2. `cmd_vel_odom.py` : SUPPRIMER tout le code USBL (seed + filtre + gain + params).
   Intégration pure, départ (0,0, cap seed étape 0). Publier theta COHÉRENT avec
   xy (⚠ incohérence mesurée 17-07 : branche Bruce logge dr_theta sans l'offset
   de seed alors que dr_x/y le portent).
3. Launch : un seul `aracati.launch` avec 2 args (`sc`, `usbl_backend`) → les
   4 méthodes = 4 lignes de commande. Supprimer les args morts (usbl, usbl_gain,
   gt_free_seed…) pour que la mauvaise config soit INTAPABLE.
4. `run_slam.sh` : presets `bruce | bruce_u | bruce_sonar | bsu` + label
   programmatique du dossier de run (jamais de suffixe manuel, piège connu).
5. Gates post-runs (script unique, pattern run_noise2.sh) : ① dr identiques ×4
   ② ATE origine translation-pure + sections ③ ordre BSU ≤ BS/BU < B ④ carte
   (cloud_vs_gt méd/p90). Verdict imprimé, rien d'affaibli en silence.
6. Papier : mettre à jour tableaux/figures EN DERNIER, après verdict des gates
   (chap 1 = Bruce + Bruce_U ; chap 2 = Bruce_Sonar + BSU + comparaison publiés).

## Chiffres de référence (état 17-07, convention translation pure, runs ANCIENS)

| Méthode (ancienne config) | ATE global | S1/S2/S3 | remarque |
|---|---|---|---|
| Odométrie (repère cap 0) | 19.57 | 6.00/12.62/8.44 | ≈ ligne ISOPoT 5.8/12.5/6.5 ✓ |
| Bruce sans USBL | 2.58 · 3.56 | 2.57/5.61/2.80 | seed = cap 0 |
| Bruce + USBL (seed 1 m) | 8.09 · 10.90 | — | repère tourné −31° (PIEGES #25) |
| BSU (seed route-fond) | 1.94 · 2.00 | 2.22/2.46/2.65 | loops SC redressent le repère |
| Publiés (ISOPoT Table I, assisté) | — | 3.2/3.5/4.6 | leur toolbox, sections reset mag |

Runs anciens conservés : 223959 (Bruce), 233119/001730 (Bruce_U), 201541/210733
(BSU). Ils restent la référence de comparaison avant/après refonte.

## 🔧 Amendements en session Ultracode (Nathan, 2026-07-16, mesures à l'appui)

1. **⛔4 amendé — 2 yamls champions FIGÉS** (commit 2db18eb, PIEGES #26) : le NSSM
   natif sur le yaml BSU = 31 fausses loops, +190°, ATE 80.7 m (run 192435). Les 2
   champions historiques n'ont jamais partagé un yaml. `sc=false` →
   `slam_aracati_native.yaml` (copie branche Bruce : SSM on, kf 3 m, σ_odom 0.2,
   sep 8, min_pcm 4, σ_USBL 2.5) ; `sc=true` → `slam_aracati.yaml` (BSU, σ 1.4).
   « 1 état de CODE » inchangé. Gate ① validé en réel : dr bit-identiques (0.0000°/0.00 mm).
2. **Gate ③ amendé — Bruce_Sonar hors verdict, reporté comme FINDING** : SC seul
   détecte 154 vraies revisites (sc_dist 0.60) mais 0 acceptée — sans ancre absolue
   la correction nécessaire à la revisite (méd 9.5 m, 69 % > 8 m) dépasse le
   garde-fou `max_translation` et affame le PCM ; avec USBL : méd 1.2 m, 65 loops,
   ATE 2.02. → complémentarité SC↔USBL, résultat honnête au papier (run 205255,
   19.24 ≈ odométrie). Ordre du verdict : BSU ≤ Bruce_U < Bruce.
3. Premier chiffre refonte validé : **BSU 2.02 m origine / 1.38 m Umeyama**
   (meilleur Umeyama du stage, seed 100 % GT-free cap 0), run 213715.

## Interdits (rappel)

- /pose_gt hors évaluation (seed compris). Umeyama hors debug interne.
- Toute rotation « ajustée » dans une figure ou un tableau du papier.
- Modifier l'odométrie pour améliorer un chiffre (le gate dr-identiques le détecte).
- Committer pendant un run ; suffixes de dossier manuels ; affaiblir un gate.
